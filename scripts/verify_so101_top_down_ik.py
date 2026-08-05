from __future__ import annotations

import datetime as dt
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_SRC = PROJECT_ROOT / "ros2_ws" / "src" / "so101_kinematics"
if str(ROS2_SRC) not in sys.path:
    sys.path.insert(0, str(ROS2_SRC))

from so101_kinematics.top_down_ik import (  # noqa: E402
    APPROACH_TOLERANCE_DEG,
    BASE_LINK,
    EXPECTED_URDF_SHA256,
    MAXIMUM_ADJACENT_DELTA_RAD,
    MINIMUM_LIMIT_MARGIN_RAD,
    POSITION_TOLERANCE_MM,
    RANDOM_SEED,
    REFERENCE_Q_DEG,
    TARGET_APPROACH_AXIS_BASE,
    TIP_LINK,
    TOOL_APPROACH_AXIS_LOCAL,
    create_default_solver,
    format_float_list,
    generate_vertical_joint_path,
    minimum_joint_limit_margin,
    pose_data,
    sha256_file,
    solve_safe_top_down_position,
)
from so101_kinematics.urdf_fk import ARM_JOINT_NAMES  # noqa: E402


REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_2b3_report.json"
LOG_PATH = PROJECT_ROOT / "data" / "verification" / "stage_2b3_verification.log"
URDF_PATH = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)

MAXIMUM_WRIST_ROLL_DEVIATION_RAD = 0.50
MAXIMUM_PREGRASP_RETURN_DELTA_RAD = 0.15


def timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def log(message: str) -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    line = f"{timestamp()} {message}"
    print(line, flush=True)
    with LOG_PATH.open("a", encoding="utf-8") as file:
        file.write(line + "\n")


def build_vertical_path_offsets() -> list[float]:
    return [
        0.000,
        0.005,
        0.010,
        0.015,
        0.020,
        0.025,
        0.020,
        0.015,
        0.010,
        0.005,
        0.000,
        -0.005,
        -0.010,
        -0.015,
        -0.020,
        -0.015,
        -0.010,
        -0.005,
        0.000,
        0.005,
        0.010,
        0.015,
        0.020,
        0.025,
    ]


def add_failure(
    failures: list[dict[str, Any]],
    test: str,
    detail: str,
    **values: Any,
) -> None:
    failure = {
        "test": test,
        "detail": detail,
    }
    failure.update(values)
    failures.append(failure)


def main() -> int:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    if LOG_PATH.exists():
        LOG_PATH.unlink()

    log("Stage 2B-3 top-down IK verification started.")

    solver, model = create_default_solver(PROJECT_ROOT)
    failures: list[dict[str, Any]] = []

    urdf_hash = sha256_file(URDF_PATH)
    if urdf_hash != EXPECTED_URDF_SHA256:
        add_failure(
            failures,
            "urdf_sha256",
            "Frozen URDF hash changed.",
            measured=urdf_hash,
            expected=EXPECTED_URDF_SHA256,
        )

    reference_q = np.radians(REFERENCE_Q_DEG)
    if not solver.within_limits(reference_q):
        add_failure(
            failures,
            "reference_limits",
            "Reference pose violates URDF limits.",
        )

    reference_position, _, reference_approach = pose_data(solver.fk, reference_q)
    reference_approach_error_deg = float(
        math.degrees(
            math.acos(
                float(
                    np.clip(
                        np.dot(reference_approach, TARGET_APPROACH_AXIS_BASE),
                        -1.0,
                        1.0,
                    )
                )
            )
        )
    )
    reference_margin = minimum_joint_limit_margin(
        reference_q,
        solver.lower,
        solver.upper,
    )

    if reference_approach_error_deg > APPROACH_TOLERANCE_DEG:
        add_failure(
            failures,
            "reference_approach",
            "Reference approach is not top-down enough.",
            measured_deg=reference_approach_error_deg,
            threshold_deg=APPROACH_TOLERANCE_DEG,
        )
    if reference_margin < MINIMUM_LIMIT_MARGIN_RAD:
        add_failure(
            failures,
            "reference_margin",
            "Reference pose has insufficient joint-limit margin.",
            measured_rad=reference_margin,
            threshold_rad=MINIMUM_LIMIT_MARGIN_RAD,
        )

    offsets = build_vertical_path_offsets()
    targets = [
        reference_position + np.asarray([0.0, 0.0, offset], dtype=np.float64)
        for offset in offsets
    ]
    path = generate_vertical_joint_path(
        solver=solver,
        target_positions_m=targets,
        reference_q_rad=reference_q,
        position_tolerance_mm=POSITION_TOLERANCE_MM,
        approach_tolerance_deg=APPROACH_TOLERANCE_DEG,
        minimum_limit_margin_rad=MINIMUM_LIMIT_MARGIN_RAD,
        maximum_adjacent_delta_rad=MAXIMUM_ADJACENT_DELTA_RAD,
        rng=np.random.default_rng(RANDOM_SEED),
    )

    wrist_roll_index = ARM_JOINT_NAMES.index("wrist_roll")
    max_wrist_roll_deviation = 0.0
    path_results: list[dict[str, Any]] = []
    for index, point in enumerate(path.points):
        result = point.result
        wrist_roll_deviation = None
        if result.q_rad is not None:
            wrist_roll_deviation = abs(
                float(result.q_rad[wrist_roll_index] - reference_q[wrist_roll_index])
            )
            max_wrist_roll_deviation = max(max_wrist_roll_deviation, wrist_roll_deviation)
        point_passed = bool(
            result.success
            and result.position_error_mm is not None
            and result.position_error_mm <= POSITION_TOLERANCE_MM
            and result.approach_error_deg is not None
            and result.approach_error_deg <= APPROACH_TOLERANCE_DEG
            and result.minimum_joint_limit_margin_rad is not None
            and result.minimum_joint_limit_margin_rad >= MINIMUM_LIMIT_MARGIN_RAD
            and point.adjacent_max_joint_delta_rad is not None
            and point.adjacent_max_joint_delta_rad <= MAXIMUM_ADJACENT_DELTA_RAD
            and wrist_roll_deviation is not None
            and wrist_roll_deviation <= MAXIMUM_WRIST_ROLL_DEVIATION_RAD
        )
        point_report = point.to_report()
        point_report.update(
            {
                "z_offset_m": offsets[index],
                "wrist_roll_deviation_rad": wrist_roll_deviation,
                "wrist_roll_deviation_deg": (
                    None
                    if wrist_roll_deviation is None
                    else math.degrees(wrist_roll_deviation)
                ),
                "passed": point_passed,
            }
        )
        path_results.append(point_report)
        if not point_passed:
            add_failure(
                failures,
                "path_point",
                "Top-down vertical path waypoint failed.",
                point_index=index,
                failure_reason=result.failure_reason,
                position_error_mm=result.position_error_mm,
                approach_error_deg=result.approach_error_deg,
                minimum_limit_margin_rad=result.minimum_joint_limit_margin_rad,
                adjacent_delta_rad=point.adjacent_max_joint_delta_rad,
                wrist_roll_deviation_rad=wrist_roll_deviation,
            )
            break
        log(
            f"point={index:02d} z_offset={offsets[index]:+.3f} m | "
            f"position_error={result.position_error_mm:.4f} mm | "
            f"approach_error={result.approach_error_deg:.4f} deg | "
            f"margin={result.minimum_joint_limit_margin_rad:.4f} rad | "
            f"delta={point.adjacent_max_joint_delta_rad:.4f} rad"
        )

    pregrasp_return_delta_rad: float | None = None
    if len(path.q_path_rad) == len(offsets):
        pregrasp_return_delta_rad = float(
            np.max(np.abs(path.q_path_rad[-1] - path.q_path_rad[5]))
        )
        if pregrasp_return_delta_rad > MAXIMUM_PREGRASP_RETURN_DELTA_RAD:
            add_failure(
                failures,
                "pregrasp_return",
                "Return-to-pregrasp joint delta is too large.",
                measured_rad=pregrasp_return_delta_rad,
                threshold_rad=MAXIMUM_PREGRASP_RETURN_DELTA_RAD,
            )
    else:
        add_failure(
            failures,
            "pregrasp_return",
            "Path did not reach both pregrasp endpoints.",
        )

    unreachable_target = np.asarray([0.80, 0.00, 0.80], dtype=np.float64)
    unreachable = solve_safe_top_down_position(
        solver=solver,
        target_position_m=unreachable_target,
        reference_q_rad=reference_q,
        rng=np.random.default_rng(RANDOM_SEED + 1000),
    )
    if unreachable.success:
        add_failure(
            failures,
            "unreachable_target",
            "Unreachable target was accepted.",
            position_error_mm=unreachable.position_error_mm,
            approach_error_deg=unreachable.approach_error_deg,
        )

    status = "PASS" if not failures else "FAIL"
    report = {
        "stage": "2B-3",
        "status": status,
        "timestamp": timestamp(),
        "description": (
            "Position IK with top-down tool approach-axis constraint, "
            "implemented in so101_kinematics.top_down_ik."
        ),
        "model": model,
        "frame_convention": {
            "tool_frame": TIP_LINK,
            "tool_approach_axis_local": format_float_list(TOOL_APPROACH_AXIS_LOCAL),
            "target_approach_axis_base": format_float_list(TARGET_APPROACH_AXIS_BASE),
            "note": (
                "Only the approach direction is constrained; rotation about "
                "that axis is not independently commanded."
            ),
        },
        "configuration": {
            "position_tolerance_mm": POSITION_TOLERANCE_MM,
            "approach_tolerance_deg": APPROACH_TOLERANCE_DEG,
            "minimum_limit_margin_rad": MINIMUM_LIMIT_MARGIN_RAD,
            "maximum_adjacent_delta_rad": MAXIMUM_ADJACENT_DELTA_RAD,
            "maximum_wrist_roll_deviation_rad": MAXIMUM_WRIST_ROLL_DEVIATION_RAD,
            "maximum_pregrasp_return_delta_rad": MAXIMUM_PREGRASP_RETURN_DELTA_RAD,
            "reference_q_deg": format_float_list(REFERENCE_Q_DEG),
            "vertical_offsets_m": offsets,
            "random_seed": RANDOM_SEED,
        },
        "reference": {
            "q_rad": format_float_list(reference_q),
            "q_deg": format_float_list(REFERENCE_Q_DEG),
            "position_m": format_float_list(reference_position),
            "approach_axis_base": format_float_list(reference_approach),
            "approach_error_deg": reference_approach_error_deg,
            "minimum_limit_margin_rad": reference_margin,
        },
        "summary": {
            "path_point_count": len(offsets),
            "solved_point_count": len(path.q_path_rad),
            "maximum_position_error_mm": path.max_position_error_mm,
            "maximum_approach_error_deg": path.max_approach_error_deg,
            "minimum_observed_margin_rad": path.min_margin_rad,
            "maximum_adjacent_delta_rad": path.max_adjacent_delta_rad,
            "maximum_adjacent_delta_deg": (
                None
                if path.max_adjacent_delta_rad is None
                else math.degrees(path.max_adjacent_delta_rad)
            ),
            "maximum_wrist_roll_deviation_rad": max_wrist_roll_deviation,
            "maximum_wrist_roll_deviation_deg": math.degrees(max_wrist_roll_deviation),
            "pregrasp_return_delta_rad": pregrasp_return_delta_rad,
            "pregrasp_return_delta_deg": (
                None
                if pregrasp_return_delta_rad is None
                else math.degrees(pregrasp_return_delta_rad)
            ),
            "unreachable_rejected": not unreachable.success,
        },
        "path_results": path_results,
        "unreachable_case": {
            "target_position_m": format_float_list(unreachable_target),
            "rejected": not unreachable.success,
            "candidate_count": len(unreachable.attempts),
            "best_raw_position_error_mm": unreachable.position_error_mm,
            "best_raw_approach_error_deg": unreachable.approach_error_deg,
        },
        "safety": {
            "opened_com_ports": False,
            "started_hardware_services": False,
            "published_robot_commands": False,
            "used_offline_computation_only": True,
        },
        "limitations": [
            "This stage constrains position and gripper approach direction only.",
            "It does not independently enforce gripper roll/yaw about the approach axis.",
            (
                "It is not a time-parameterized trajectory and contains no velocity, "
                "acceleration, collision or hardware-control validation."
            ),
        ],
        "failures": failures,
        "logs": {
            "verification_log": str(LOG_PATH),
        },
    }
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    log(f"Report written: {REPORT_PATH}")
    log(f"Stage 2B-3 status: {status}")
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
