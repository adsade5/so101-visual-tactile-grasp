from __future__ import annotations

import datetime as dt
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from so101_kinematics.urdf_fk import (
    ARM_JOINT_NAMES,
    UrdfForwardKinematics,
)

from verify_so101_position_ik import (
    BASE_LINK,
    POSITION_TOLERANCE_MM,
    TIP_LINK,
    URDF_PATH,
    PositionIKSolver,
    AttemptResult,
    finite_array,
    format_float_list,
    joint_limits_from_fk,
    minimum_joint_limit_margin,
    q_to_dict,
    sha256_file,
    validate_transform,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]

REPORT_PATH = (
    PROJECT_ROOT
    / "data"
    / "verification"
    / "stage_2b2_report.json"
)

LOG_PATH = (
    PROJECT_ROOT
    / "data"
    / "verification"
    / "stage_2b2_verification.log"
)

REFERENCE_Q_DEG = np.asarray(
    [
        0.0,
        -30.0,
        45.0,
        -20.0,
        0.0,
    ],
    dtype=np.float64,
)

MINIMUM_LIMIT_MARGIN_RAD = 0.05
MAXIMUM_ADJACENT_DELTA_RAD = 0.15
MAXIMUM_RETURN_DELTA_RAD = 0.15
MAXIMUM_WRIST_ROLL_DEVIATION_RAD = 0.50

LOCAL_PERTURBATIONS_RAD = (
    0.05,
    0.10,
)

LOCAL_RANDOM_STARTS = 16
LOCAL_RANDOM_STD_RAD = 0.08

PATH_STEP_M = 0.002
PATH_STEPS_PER_SEGMENT = 5

RANDOM_SEED = 20260804


def timestamp() -> str:
    return dt.datetime.now(
        dt.timezone.utc
    ).isoformat()


def ensure_output_directory() -> None:
    REPORT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )


def log(message: str) -> None:
    ensure_output_directory()

    line = f"{timestamp()} {message}"

    print(
        line,
        flush=True,
    )

    with LOG_PATH.open(
        "a",
        encoding="utf-8",
    ) as file:
        file.write(line + "\n")


@dataclass
class SafeCandidate:
    attempt: AttemptResult
    minimum_limit_margin_rad: float
    normalized_reference_distance: float
    normalized_center_distance: float
    maximum_reference_delta_rad: float
    score: float

    def to_report(self) -> dict[str, Any]:
        value = self.attempt.to_report()

        value.update(
            {
                "minimum_limit_margin_rad": (
                    self.minimum_limit_margin_rad
                ),
                "normalized_reference_distance": (
                    self.normalized_reference_distance
                ),
                "normalized_center_distance": (
                    self.normalized_center_distance
                ),
                "maximum_reference_delta_rad": (
                    self.maximum_reference_delta_rad
                ),
                "selection_score": self.score,
            }
        )

        return value


def normalized_distance(
    first: np.ndarray,
    second: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> float:
    joint_ranges = upper - lower

    if np.any(joint_ranges <= 0.0):
        raise ValueError(
            "Invalid joint range"
        )

    normalized_difference = (
        first - second
    ) / joint_ranges

    return float(
        np.sqrt(
            np.mean(
                normalized_difference**2
            )
        )
    )


def build_local_starts(
    reference_q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    rng: np.random.Generator,
) -> list[tuple[str, np.ndarray]]:
    starts: list[
        tuple[str, np.ndarray]
    ] = [
        (
            "reference",
            reference_q.copy(),
        )
    ]

    for joint_index, joint_name in enumerate(
        ARM_JOINT_NAMES
    ):
        for magnitude in (
            LOCAL_PERTURBATIONS_RAD
        ):
            for sign in (-1.0, 1.0):
                candidate = (
                    reference_q.copy()
                )

                candidate[joint_index] += (
                    sign * magnitude
                )

                candidate = np.clip(
                    candidate,
                    lower,
                    upper,
                )

                starts.append(
                    (
                        (
                            f"{joint_name}_"
                            f"{sign:+.0f}_"
                            f"{magnitude:.2f}"
                        ),
                        candidate,
                    )
                )

    for index in range(
        LOCAL_RANDOM_STARTS
    ):
        perturbation = rng.normal(
            loc=0.0,
            scale=LOCAL_RANDOM_STD_RAD,
            size=len(ARM_JOINT_NAMES),
        )

        candidate = np.clip(
            reference_q + perturbation,
            lower,
            upper,
        )

        starts.append(
            (
                f"local_random_{index:02d}",
                candidate,
            )
        )

    return starts


def classify_candidate(
    attempt: AttemptResult,
    reference_q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> SafeCandidate:
    midpoint = 0.5 * (
        lower + upper
    )

    margin = minimum_joint_limit_margin(
        attempt.q,
        lower,
        upper,
    )

    reference_distance = (
        normalized_distance(
            attempt.q,
            reference_q,
            lower,
            upper,
        )
    )

    center_distance = (
        normalized_distance(
            attempt.q,
            midpoint,
            lower,
            upper,
        )
    )

    maximum_reference_delta = float(
        np.max(
            np.abs(
                attempt.q - reference_q
            )
        )
    )

    # 位置误差只要达到1 mm要求即可。
    # 选解时主要考虑与当前姿态的连续性，
    # 其次考虑远离关节限位和关节中心。
    score = (
        reference_distance
        + 0.10 * center_distance
        + 0.001
        * attempt.position_error_mm
    )

    return SafeCandidate(
        attempt=attempt,
        minimum_limit_margin_rad=margin,
        normalized_reference_distance=(
            reference_distance
        ),
        normalized_center_distance=(
            center_distance
        ),
        maximum_reference_delta_rad=(
            maximum_reference_delta
        ),
        score=score,
    )


def solve_safe_position(
    solver: PositionIKSolver,
    target_position: np.ndarray,
    reference_q: np.ndarray,
    rng: np.random.Generator,
) -> tuple[
    SafeCandidate | None,
    list[SafeCandidate],
]:
    finite_array(
        target_position,
        "target_position",
    )

    finite_array(
        reference_q,
        "reference_q",
    )

    attempts: list[
        AttemptResult
    ] = []

    for seed_name, start_q in (
        build_local_starts(
            reference_q,
            solver.lower,
            solver.upper,
            rng,
        )
    ):
        attempts.append(
            solver.solve_one(
                target=target_position,
                q0=start_q,
                seed=seed_name,
            )
        )

    candidates = [
        classify_candidate(
            attempt,
            reference_q,
            solver.lower,
            solver.upper,
        )
        for attempt in attempts
    ]

    safe_candidates = [
        candidate
        for candidate in candidates
        if (
            candidate.attempt.success
            and candidate.attempt.within_joint_limits
            and candidate.minimum_limit_margin_rad
            >= MINIMUM_LIMIT_MARGIN_RAD
            and np.all(
                np.isfinite(
                    candidate.attempt.q
                )
            )
        )
    ]

    safe_candidates.sort(
        key=lambda candidate: (
            candidate.score,
            candidate.attempt.position_error_mm,
        )
    )

    if not safe_candidates:
        return None, candidates

    return (
        safe_candidates[0],
        candidates,
    )


def build_closed_cartesian_path() -> list[
    dict[str, Any]
]:
    points: list[
        dict[str, Any]
    ] = [
        {
            "name": "center_start",
            "offset_m": np.zeros(
                3,
                dtype=np.float64,
            ),
        }
    ]

    offset = np.zeros(
        3,
        dtype=np.float64,
    )

    segments = [
        ("x_positive", 0, 1.0),
        ("y_positive", 1, 1.0),
        ("z_positive", 2, 1.0),
        ("z_negative", 2, -1.0),
        ("y_negative", 1, -1.0),
        ("x_negative", 0, -1.0),
    ]

    point_index = 1

    for segment_name, axis, sign in segments:
        for _ in range(
            PATH_STEPS_PER_SEGMENT
        ):
            offset = offset.copy()

            offset[axis] += (
                sign * PATH_STEP_M
            )

            points.append(
                {
                    "name": (
                        f"{point_index:02d}_"
                        f"{segment_name}"
                    ),
                    "offset_m": offset.copy(),
                }
            )

            point_index += 1

    return points


def main() -> int:
    ensure_output_directory()

    if LOG_PATH.exists():
        LOG_PATH.unlink()

    log(
        "Stage 2B-2 safe continuous "
        "position IK verification started."
    )

    fk = UrdfForwardKinematics(
        urdf_path=URDF_PATH.resolve(),
        base_link=BASE_LINK,
        tip_link=TIP_LINK,
    )

    fk.validate_expected_chain()

    lower, upper, joint_limits = (
        joint_limits_from_fk(fk)
    )

    solver = PositionIKSolver(
        fk=fk,
        lower=lower,
        upper=upper,
    )

    reference_q = np.radians(
        REFERENCE_Q_DEG
    )

    finite_array(
        reference_q,
        "reference_q",
    )

    if not solver.within_limits(
        reference_q
    ):
        raise ValueError(
            "Reference pose violates "
            "URDF limits"
        )

    reference_margin = (
        minimum_joint_limit_margin(
            reference_q,
            lower,
            upper,
        )
    )

    if (
        reference_margin
        < MINIMUM_LIMIT_MARGIN_RAD
    ):
        raise ValueError(
            "Reference pose is too close "
            "to a joint limit"
        )

    center_transform = fk.compute(
        q_to_dict(reference_q)
    )

    validate_transform(
        center_transform
    )

    center_position = np.asarray(
        center_transform[:3, 3],
        dtype=np.float64,
    )

    path = build_closed_cartesian_path()

    rng = np.random.default_rng(
        RANDOM_SEED
    )

    previous_q = reference_q.copy()

    path_results: list[
        dict[str, Any]
    ] = []

    failures: list[
        dict[str, Any]
    ] = []

    maximum_adjacent_delta_rad = 0.0
    maximum_position_error_mm = 0.0
    minimum_observed_margin_rad = math.inf
    maximum_wrist_roll_deviation_rad = 0.0

    wrist_roll_index = (
        ARM_JOINT_NAMES.index(
            "wrist_roll"
        )
    )

    for point_index, point in enumerate(
        path
    ):
        target_position = (
            center_position
            + point["offset_m"]
        )

        selected, all_candidates = (
            solve_safe_position(
                solver,
                target_position,
                previous_q,
                rng,
            )
        )

        if selected is None:
            best_raw = min(
                all_candidates,
                key=lambda candidate: (
                    candidate.attempt
                    .position_error_mm
                ),
            )

            failures.append(
                {
                    "point_index": point_index,
                    "point_name": point["name"],
                    "reason": (
                        "No candidate satisfied "
                        "position, joint-limit "
                        "and safety-margin rules"
                    ),
                    "best_raw_error_mm": (
                        best_raw.attempt
                        .position_error_mm
                    ),
                    "best_raw_q_deg": (
                        format_float_list(
                            np.degrees(
                                best_raw.attempt.q
                            )
                        )
                    ),
                }
            )

            break

        selected_q = (
            selected.attempt.q
        )

        adjacent_delta = float(
            np.max(
                np.abs(
                    selected_q
                    - previous_q
                )
            )
        )

        wrist_roll_deviation = abs(
            float(
                selected_q[
                    wrist_roll_index
                ]
                - reference_q[
                    wrist_roll_index
                ]
            )
        )

        maximum_adjacent_delta_rad = max(
            maximum_adjacent_delta_rad,
            adjacent_delta,
        )

        maximum_position_error_mm = max(
            maximum_position_error_mm,
            selected.attempt.position_error_mm,
        )

        minimum_observed_margin_rad = min(
            minimum_observed_margin_rad,
            selected.minimum_limit_margin_rad,
        )

        maximum_wrist_roll_deviation_rad = max(
            maximum_wrist_roll_deviation_rad,
            wrist_roll_deviation,
        )

        point_passed = bool(
            selected.attempt.position_error_mm
            <= POSITION_TOLERANCE_MM
            and selected.minimum_limit_margin_rad
            >= MINIMUM_LIMIT_MARGIN_RAD
            and adjacent_delta
            <= MAXIMUM_ADJACENT_DELTA_RAD
            and wrist_roll_deviation
            <= MAXIMUM_WRIST_ROLL_DEVIATION_RAD
        )

        if not point_passed:
            failures.append(
                {
                    "point_index": point_index,
                    "point_name": point["name"],
                    "reason": (
                        "Selected candidate failed "
                        "a continuity or safety rule"
                    ),
                    "position_error_mm": (
                        selected.attempt
                        .position_error_mm
                    ),
                    "minimum_limit_margin_rad": (
                        selected
                        .minimum_limit_margin_rad
                    ),
                    "adjacent_delta_rad": (
                        adjacent_delta
                    ),
                    "wrist_roll_deviation_rad": (
                        wrist_roll_deviation
                    ),
                }
            )

        path_results.append(
            {
                "point_index": point_index,
                "point_name": point["name"],
                "offset_m": format_float_list(
                    point["offset_m"]
                ),
                "target_position_m": (
                    format_float_list(
                        target_position
                    )
                ),
                "selected_q_rad": (
                    format_float_list(
                        selected_q
                    )
                ),
                "selected_q_deg": (
                    format_float_list(
                        np.degrees(
                            selected_q
                        )
                    )
                ),
                "achieved_position_m": (
                    format_float_list(
                        selected.attempt
                        .achieved_position
                    )
                ),
                "position_error_mm": (
                    selected.attempt
                    .position_error_mm
                ),
                "minimum_limit_margin_rad": (
                    selected
                    .minimum_limit_margin_rad
                ),
                "adjacent_max_joint_delta_rad": (
                    adjacent_delta
                ),
                "adjacent_max_joint_delta_deg": (
                    math.degrees(
                        adjacent_delta
                    )
                ),
                "wrist_roll_deviation_rad": (
                    wrist_roll_deviation
                ),
                "selection_score": (
                    selected.score
                ),
                "selected_seed": (
                    selected.attempt.seed
                ),
                "candidate_count": len(
                    all_candidates
                ),
                "passed": point_passed,
            }
        )

        previous_q = selected_q.copy()

        log(
            f"{point_index:02d} "
            f"{point['name']} | "
            f"error="
            f"{selected.attempt.position_error_mm:.4f} mm | "
            f"margin="
            f"{selected.minimum_limit_margin_rad:.4f} rad | "
            f"delta="
            f"{adjacent_delta:.4f} rad | "
            f"wrist_roll="
            f"{math.degrees(selected_q[wrist_roll_index]):.2f} deg"
        )

    return_delta_rad = float(
        np.max(
            np.abs(
                previous_q - reference_q
            )
        )
    )

    if (
        return_delta_rad
        > MAXIMUM_RETURN_DELTA_RAD
    ):
        failures.append(
            {
                "test": "closed_path_return",
                "reason": (
                    "Final joint pose did not "
                    "return close enough to the "
                    "initial reference pose"
                ),
                "return_delta_rad": (
                    return_delta_rad
                ),
                "threshold_rad": (
                    MAXIMUM_RETURN_DELTA_RAD
                ),
            }
        )

    unreachable_target = np.asarray(
        [0.80, 0.00, 0.80],
        dtype=np.float64,
    )

    unreachable_selected, (
        unreachable_candidates
    ) = solve_safe_position(
        solver,
        unreachable_target,
        reference_q,
        np.random.default_rng(
            RANDOM_SEED + 1000
        ),
    )

    unreachable_rejected = (
        unreachable_selected is None
    )

    if not unreachable_rejected:
        failures.append(
            {
                "test": "unreachable_target",
                "reason": (
                    "Unreachable target was "
                    "incorrectly accepted"
                ),
                "selected_error_mm": (
                    unreachable_selected
                    .attempt.position_error_mm
                ),
            }
        )

    repeat_rng = np.random.default_rng(
        RANDOM_SEED
    )

    repeat_first, _ = (
        solve_safe_position(
            solver,
            center_position,
            reference_q,
            repeat_rng,
        )
    )

    deterministic_repeat = bool(
        repeat_first is not None
        and path_results
        and np.linalg.norm(
            repeat_first.attempt.q
            - np.asarray(
                path_results[0][
                    "selected_q_rad"
                ],
                dtype=np.float64,
            )
        )
        <= 1.0e-12
    )

    if not deterministic_repeat:
        failures.append(
            {
                "test": (
                    "deterministic_repeat"
                ),
                "reason": (
                    "Fixed-seed repeated solve "
                    "was not numerically "
                    "equivalent"
                ),
            }
        )

    status = (
        "PASS"
        if not failures
        else "FAIL"
    )

    report = {
        "stage": "2B-2",
        "status": status,
        "timestamp": timestamp(),
        "description": (
            "Safe and continuous "
            "position-only IK selection"
        ),
        "model": {
            "urdf": str(URDF_PATH),
            "urdf_sha256": (
                sha256_file(URDF_PATH)
            ),
            "base_link": BASE_LINK,
            "tip_link": TIP_LINK,
            "joint_names": (
                ARM_JOINT_NAMES
            ),
            "joint_limits_rad": (
                joint_limits
            ),
        },
        "configuration": {
            "position_tolerance_mm": (
                POSITION_TOLERANCE_MM
            ),
            "minimum_limit_margin_rad": (
                MINIMUM_LIMIT_MARGIN_RAD
            ),
            "maximum_adjacent_delta_rad": (
                MAXIMUM_ADJACENT_DELTA_RAD
            ),
            "maximum_return_delta_rad": (
                MAXIMUM_RETURN_DELTA_RAD
            ),
            (
                "maximum_wrist_roll_"
                "deviation_rad"
            ): (
                MAXIMUM_WRIST_ROLL_DEVIATION_RAD
            ),
            "reference_q_deg": (
                format_float_list(
                    REFERENCE_Q_DEG
                )
            ),
            "path_step_m": PATH_STEP_M,
            "path_steps_per_segment": (
                PATH_STEPS_PER_SEGMENT
            ),
            "local_random_starts": (
                LOCAL_RANDOM_STARTS
            ),
            "random_seed": RANDOM_SEED,
        },
        "reference": {
            "q_rad": format_float_list(
                reference_q
            ),
            "q_deg": format_float_list(
                REFERENCE_Q_DEG
            ),
            "center_position_m": (
                format_float_list(
                    center_position
                )
            ),
            "minimum_limit_margin_rad": (
                reference_margin
            ),
        },
        "summary": {
            "path_point_count": len(
                path
            ),
            "solved_point_count": len(
                path_results
            ),
            "maximum_position_error_mm": (
                maximum_position_error_mm
            ),
            "minimum_observed_margin_rad": (
                minimum_observed_margin_rad
            ),
            "maximum_adjacent_delta_rad": (
                maximum_adjacent_delta_rad
            ),
            "maximum_adjacent_delta_deg": (
                math.degrees(
                    maximum_adjacent_delta_rad
                )
            ),
            (
                "maximum_wrist_roll_"
                "deviation_rad"
            ): (
                maximum_wrist_roll_deviation_rad
            ),
            (
                "maximum_wrist_roll_"
                "deviation_deg"
            ): math.degrees(
                maximum_wrist_roll_deviation_rad
            ),
            "closed_path_return_delta_rad": (
                return_delta_rad
            ),
            "closed_path_return_delta_deg": (
                math.degrees(
                    return_delta_rad
                )
            ),
            "unreachable_rejected": (
                unreachable_rejected
            ),
            "deterministic_repeat": (
                deterministic_repeat
            ),
        },
        "path_results": path_results,
        "unreachable_case": {
            "target_position_m": (
                format_float_list(
                    unreachable_target
                )
            ),
            "rejected": (
                unreachable_rejected
            ),
            "candidate_count": len(
                unreachable_candidates
            ),
            "best_raw_error_mm": min(
                candidate.attempt
                .position_error_mm
                for candidate
                in unreachable_candidates
            ),
        },
        "safety": {
            "opened_com_ports": False,
            "started_hardware_services": (
                False
            ),
            "published_robot_commands": (
                False
            ),
            "used_offline_computation_only": (
                True
            ),
        },
        "failures": failures,
        "logs": {
            "verification_log": str(
                LOG_PATH
            ),
        },
    }

    REPORT_PATH.write_text(
        json.dumps(
            report,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        ),
        encoding="utf-8",
    )

    log(
        f"Report written: {REPORT_PATH}"
    )

    log(
        f"Stage 2B-2 status: {status}"
    )

    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())