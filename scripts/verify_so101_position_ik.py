from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from so101_kinematics.urdf_fk import (
    ARM_JOINT_NAMES,
    UrdfForwardKinematics,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)
REPORT_PATH = (
    PROJECT_ROOT
    / "data"
    / "verification"
    / "stage_2b1_report.json"
)
LOG_PATH = (
    PROJECT_ROOT
    / "data"
    / "verification"
    / "stage_2b1_verification.log"
)

BASE_LINK = "base_link"
TIP_LINK = "gripper_frame_link"

POSITION_TOLERANCE_MM = 1.0
UNREACHABLE_MIN_ERROR_MM = 50.0
MAXIMUM_ITERATIONS = 400
RANDOM_RESTARTS = 24
MAXIMUM_JOINT_STEP_RAD = 0.20
FINITE_DIFFERENCE_STEP_RAD = 1.0e-5
INITIAL_DAMPING = 0.02
RANDOM_SEED = 20260804


def timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def ensure_report_dir() -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)


def log(message: str) -> None:
    ensure_report_dir()
    line = f"{timestamp()} {message}"
    print(line, flush=True)
    with LOG_PATH.open("a", encoding="utf-8") as file:
        file.write(line + "\n")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_array(values: np.ndarray, name: str) -> None:
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} contains NaN or Inf")


def q_to_dict(q: np.ndarray) -> dict[str, float]:
    return {
        name: float(value)
        for name, value in zip(ARM_JOINT_NAMES, q.tolist(), strict=True)
    }


def validate_transform(transform: np.ndarray) -> None:
    if transform.shape != (4, 4):
        raise ValueError(f"FK transform shape is {transform.shape}, expected (4, 4)")
    finite_array(transform, "FK transform")
    bottom_row_error = float(
        np.linalg.norm(transform[3] - np.asarray([0.0, 0.0, 0.0, 1.0]))
    )
    if bottom_row_error > 1.0e-12:
        raise ValueError(f"Invalid homogeneous transform bottom row: {bottom_row_error}")


def format_float_list(values: np.ndarray | list[float]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def minimum_joint_limit_margin(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> float:
    lower_margin = q - lower
    upper_margin = upper - q
    return float(np.min(np.minimum(lower_margin, upper_margin)))


@dataclass
class AttemptResult:
    seed: str
    iterations: int
    success: bool
    q: np.ndarray
    achieved_position: np.ndarray
    position_error_mm: float
    within_joint_limits: bool
    limit_clipped: bool
    line_search_backtracks: int
    final_damping: float
    jacobian_singular_values: list[float]
    jacobian_condition: float | None

    def to_report(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "iterations": self.iterations,
            "success": self.success,
            "q_rad": format_float_list(self.q),
            "q_deg": format_float_list(np.degrees(self.q)),
            "achieved_position_m": format_float_list(self.achieved_position),
            "position_error_mm": float(self.position_error_mm),
            "within_joint_limits": self.within_joint_limits,
            "minimum_joint_limit_margin_rad": None,
            "limit_clipped": self.limit_clipped,
            "line_search_backtracks": self.line_search_backtracks,
            "final_damping": self.final_damping,
            "jacobian_singular_values": self.jacobian_singular_values,
            "jacobian_condition": self.jacobian_condition,
        }


class PositionIKSolver:
    def __init__(
        self,
        fk: UrdfForwardKinematics,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> None:
        self.fk = fk
        self.lower = lower.astype(np.float64)
        self.upper = upper.astype(np.float64)

    def clamp(self, q: np.ndarray) -> tuple[np.ndarray, bool]:
        clamped = np.clip(q, self.lower, self.upper)
        return clamped, bool(np.any(np.abs(clamped - q) > 1.0e-12))

    def within_limits(self, q: np.ndarray) -> bool:
        return bool(
            q.shape == (len(ARM_JOINT_NAMES),)
            and np.all(q >= self.lower - 1.0e-10)
            and np.all(q <= self.upper + 1.0e-10)
        )

    def position(self, q: np.ndarray) -> np.ndarray:
        if q.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(f"q shape is {q.shape}, expected (5,)")
        finite_array(q, "q")
        q, _ = self.clamp(q)
        transform = self.fk.compute(q_to_dict(q))
        validate_transform(transform)
        position = transform[:3, 3].astype(np.float64)
        finite_array(position, "FK position")
        return position

    def numerical_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, bool]:
        columns = []
        used_one_sided = False
        for index in range(len(ARM_JOINT_NAMES)):
            minus_room = float(q[index] - self.lower[index])
            plus_room = float(self.upper[index] - q[index])
            h = min(FINITE_DIFFERENCE_STEP_RAD, minus_room, plus_room)
            if h >= 1.0e-8:
                q_plus = q.copy()
                q_minus = q.copy()
                q_plus[index] += h
                q_minus[index] -= h
                column = (self.position(q_plus) - self.position(q_minus)) / (2.0 * h)
            elif plus_room >= FINITE_DIFFERENCE_STEP_RAD:
                used_one_sided = True
                q_plus = q.copy()
                q_plus[index] += FINITE_DIFFERENCE_STEP_RAD
                column = (self.position(q_plus) - self.position(q)) / FINITE_DIFFERENCE_STEP_RAD
            elif minus_room >= FINITE_DIFFERENCE_STEP_RAD:
                used_one_sided = True
                q_minus = q.copy()
                q_minus[index] -= FINITE_DIFFERENCE_STEP_RAD
                column = (self.position(q) - self.position(q_minus)) / FINITE_DIFFERENCE_STEP_RAD
            else:
                used_one_sided = True
                column = np.zeros(3, dtype=np.float64)
            columns.append(column)
        jacobian = np.column_stack(columns).astype(np.float64)
        finite_array(jacobian, "Jacobian")
        return jacobian, used_one_sided

    def solve_one(self, target: np.ndarray, q0: np.ndarray, seed: str) -> AttemptResult:
        q, clipped = self.clamp(q0.astype(np.float64))
        finite_array(q, "initial q")
        damping = INITIAL_DAMPING
        backtracks = 0
        used_one_sided = False
        singular_values: list[float] = []
        condition: float | None = None

        current_position = self.position(q)
        current_error = float(np.linalg.norm(target - current_position))

        for iteration in range(1, MAXIMUM_ITERATIONS + 1):
            if current_error * 1000.0 <= POSITION_TOLERANCE_MM:
                break

            jacobian, one_sided = self.numerical_jacobian(q)
            used_one_sided = used_one_sided or one_sided
            singular = np.linalg.svd(jacobian, compute_uv=False)
            singular_values = [float(value) for value in singular.tolist()]
            if singular.size and float(np.min(singular)) > 1.0e-12:
                condition = float(np.max(singular) / np.min(singular))
            else:
                condition = None

            error_vector = target - current_position
            lhs = jacobian @ jacobian.T + (damping * damping) * np.eye(3)
            try:
                dq = jacobian.T @ np.linalg.solve(lhs, error_vector)
            except np.linalg.LinAlgError:
                damping *= 2.0
                continue

            finite_array(dq, "joint update")
            step_norm = float(np.linalg.norm(dq, ord=np.inf))
            if step_norm > MAXIMUM_JOINT_STEP_RAD:
                dq *= MAXIMUM_JOINT_STEP_RAD / step_norm

            accepted = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                candidate, candidate_clipped = self.clamp(q + alpha * dq)
                candidate_position = self.position(candidate)
                candidate_error = float(np.linalg.norm(target - candidate_position))
                if candidate_error < current_error:
                    q = candidate
                    current_position = candidate_position
                    current_error = candidate_error
                    clipped = clipped or candidate_clipped
                    if alpha < 1.0:
                        backtracks += 1
                    accepted = True
                    damping = max(0.002, damping * 0.95)
                    break
                backtracks += 1

            if not accepted:
                damping = min(0.5, damping * 2.0)

        achieved_position = self.position(q)
        error_mm = float(np.linalg.norm(target - achieved_position) * 1000.0)
        within = self.within_limits(q)
        success = bool(error_mm <= POSITION_TOLERANCE_MM and within)
        return AttemptResult(
            seed=seed,
            iterations=iteration if "iteration" in locals() else 0,
            success=success,
            q=q,
            achieved_position=achieved_position,
            position_error_mm=error_mm,
            within_joint_limits=within,
            limit_clipped=clipped or used_one_sided,
            line_search_backtracks=backtracks,
            final_damping=float(damping),
            jacobian_singular_values=singular_values,
            jacobian_condition=condition,
        )

    def make_starts(self, rng: np.random.Generator) -> list[tuple[str, np.ndarray]]:
        starts: list[tuple[str, np.ndarray]] = []
        middle = 0.5 * (self.lower + self.upper)
        starts.append(("zero", np.zeros(len(ARM_JOINT_NAMES), dtype=np.float64)))
        starts.append(("joint_limit_middle", middle))
        starts.append(("folded_positive", np.asarray([0.35, -0.45, 0.65, -0.35, 0.15])))
        starts.append(("folded_negative", np.asarray([-0.35, 0.35, -0.55, 0.45, -0.15])))
        for index in range(RANDOM_RESTARTS):
            q = rng.uniform(self.lower, self.upper)
            starts.append((f"random_{index:02d}", q.astype(np.float64)))
        return [(name, self.clamp(q)[0]) for name, q in starts]

    def solve(self, target: np.ndarray, seed_offset: int = 0) -> tuple[AttemptResult, list[AttemptResult]]:
        target = target.astype(np.float64)
        if target.shape != (3,):
            raise ValueError(f"target shape is {target.shape}, expected (3,)")
        finite_array(target, "target")
        rng = np.random.default_rng(RANDOM_SEED + seed_offset)
        attempts = [
            self.solve_one(target, q0, seed)
            for seed, q0 in self.make_starts(rng)
        ]
        attempts.sort(key=lambda item: item.position_error_mm)
        return attempts[0], attempts


def joint_limits_from_fk(fk: UrdfForwardKinematics) -> tuple[np.ndarray, np.ndarray, dict[str, dict[str, float]]]:
    limits: dict[str, dict[str, float]] = {}
    lower_values = []
    upper_values = []
    joints_by_name = {joint.name: joint for joint in fk.chain}
    for name in ARM_JOINT_NAMES:
        joint = joints_by_name[name]
        if joint.lower is None or joint.upper is None:
            raise ValueError(f"Missing URDF joint limits for {name}")
        lower = float(joint.lower)
        upper = float(joint.upper)
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"Invalid URDF joint limits for {name}: {lower}, {upper}")
        limits[name] = {"lower": lower, "upper": upper}
        lower_values.append(lower)
        upper_values.append(upper)
    return (
        np.asarray(lower_values, dtype=np.float64),
        np.asarray(upper_values, dtype=np.float64),
        limits,
    )


def reachable_case_report(
    solver: PositionIKSolver,
    fk: UrdfForwardKinematics,
    name: str,
    reference_q_deg: list[float],
    seed_offset: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    reference_q = np.radians(np.asarray(reference_q_deg, dtype=np.float64))
    finite_array(reference_q, f"{name} reference_q")
    target = solver.position(reference_q)
    best, attempts = solver.solve(target, seed_offset=seed_offset)
    repeat_best, _ = solver.solve(target, seed_offset=seed_offset)
    repeat_equivalent = bool(
        abs(best.position_error_mm - repeat_best.position_error_mm) <= 1.0e-9
        and np.linalg.norm(best.achieved_position - repeat_best.achieved_position) <= 1.0e-12
    )

    transform = fk.compute(q_to_dict(best.q))
    validate_transform(transform)
    successful_attempts = sum(1 for attempt in attempts if attempt.success)
    report = {
        "name": name,
        "reference_q_deg": reference_q_deg,
        "target_position_m": format_float_list(target),
        "success": best.success,
        "solution_q_deg": format_float_list(np.degrees(best.q)),
        "solution_q_rad": format_float_list(best.q),
        "achieved_position_m": format_float_list(best.achieved_position),
        "position_error_mm": best.position_error_mm,
        "within_joint_limits": best.within_joint_limits,
        "minimum_joint_limit_margin_rad": minimum_joint_limit_margin(
            best.q, solver.lower, solver.upper
        ),
        "attempt_count": len(attempts),
        "successful_attempt_count": successful_attempts,
        "iterations": best.iterations,
        "best_seed": best.seed,
        "deterministic_repeat_equivalent": repeat_equivalent,
        "final_damping": best.final_damping,
        "jacobian_singular_values": best.jacobian_singular_values,
        "jacobian_condition": best.jacobian_condition,
        "line_search_backtracks": best.line_search_backtracks,
        "limit_clipped": best.limit_clipped,
    }
    return report, [attempt.to_report() for attempt in attempts]


def unreachable_case_report(
    solver: PositionIKSolver,
    target: np.ndarray,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    best, attempts = solver.solve(target, seed_offset=1000)
    report = {
        "target_position_m": format_float_list(target),
        "success": best.success,
        "rejected_as_unreachable": not best.success
        and best.position_error_mm > POSITION_TOLERANCE_MM,
        "best_position_error_mm": best.position_error_mm,
        "best_q_deg": format_float_list(np.degrees(best.q)),
        "best_q_rad": format_float_list(best.q),
        "best_achieved_position_m": format_float_list(best.achieved_position),
        "within_joint_limits": best.within_joint_limits,
        "attempt_count": len(attempts),
        "successful_attempt_count": sum(1 for attempt in attempts if attempt.success),
        "minimum_expected_error_mm": UNREACHABLE_MIN_ERROR_MM,
    }
    return report, [attempt.to_report() for attempt in attempts]


def add_failure(
    failures: list[dict[str, Any]],
    name: str,
    measured: Any,
    threshold: Any,
    detail: str,
    best_q_deg: list[float] | None = None,
) -> None:
    failures.append(
        {
            "test": name,
            "measured": measured,
            "threshold": threshold,
            "best_q_deg": best_q_deg or [],
            "detail": detail,
            "log": str(LOG_PATH),
        }
    )


def run_verification() -> int:
    ensure_report_dir()
    if LOG_PATH.exists():
        LOG_PATH.unlink()
    log("Stage 2B-1 offline position IK verification started.")
    log(f"Python executable: {sys.executable}")

    fk = UrdfForwardKinematics(
        urdf_path=URDF_PATH.resolve(),
        base_link=BASE_LINK,
        tip_link=TIP_LINK,
    )
    fk.validate_expected_chain()
    lower, upper, joint_limits = joint_limits_from_fk(fk)
    ik_solver = PositionIKSolver(fk=fk, lower=lower, upper=upper)

    urdf_hash = sha256_file(URDF_PATH)
    failures: list[dict[str, Any]] = []
    reachable_cases: list[dict[str, Any]] = []
    attempts_by_case: dict[str, list[dict[str, Any]]] = {}

    test_cases = [
        ("zero_pose", [0.0, 0.0, 0.0, 0.0, 0.0]),
        ("pose_a", [30.0, -30.0, 45.0, -20.0, 15.0]),
        ("pose_b", [-30.0, 20.0, -40.0, 30.0, -15.0]),
    ]

    for index, (name, reference_q_deg) in enumerate(test_cases):
        log(f"Solving reachable case {name}.")
        case, attempts = reachable_case_report(
            ik_solver,
            fk,
            name,
            reference_q_deg,
            seed_offset=index * 100,
        )
        reachable_cases.append(case)
        attempts_by_case[name] = attempts
        if not case["success"]:
            add_failure(
                failures,
                name,
                case["position_error_mm"],
                POSITION_TOLERANCE_MM,
                "Reachable target did not solve within position tolerance.",
                case["solution_q_deg"],
            )
        if not case["within_joint_limits"]:
            add_failure(
                failures,
                name,
                case["within_joint_limits"],
                True,
                "IK solution violates URDF joint limits.",
                case["solution_q_deg"],
            )
        if not case["deterministic_repeat_equivalent"]:
            add_failure(
                failures,
                name,
                False,
                True,
                "Fixed-seed repeated solve was not numerically equivalent.",
                case["solution_q_deg"],
            )

    log("Solving unreachable case.")
    unreachable_target = np.asarray([0.80, 0.0, 0.80], dtype=np.float64)
    unreachable_case, unreachable_attempts = unreachable_case_report(
        ik_solver,
        unreachable_target,
    )
    attempts_by_case["unreachable"] = unreachable_attempts
    if unreachable_case["success"]:
        add_failure(
            failures,
            "unreachable_case",
            True,
            False,
            "Unreachable target was incorrectly reported as success.",
            unreachable_case["best_q_deg"],
        )
    if unreachable_case["best_position_error_mm"] <= UNREACHABLE_MIN_ERROR_MM:
        add_failure(
            failures,
            "unreachable_case",
            unreachable_case["best_position_error_mm"],
            f">{UNREACHABLE_MIN_ERROR_MM}",
            "Unreachable target best error was not clearly beyond reachable tolerance.",
            unreachable_case["best_q_deg"],
        )
    if not unreachable_case["within_joint_limits"]:
        add_failure(
            failures,
            "unreachable_case",
            False,
            True,
            "Best unreachable attempt violates URDF joint limits.",
            unreachable_case["best_q_deg"],
        )

    report = {
        "stage": "2B-1",
        "status": "PASS" if not failures else "FAIL",
        "timestamp": timestamp(),
        "solver": {
            "type": "finite-difference damped least squares",
            "position_tolerance_mm": POSITION_TOLERANCE_MM,
            "maximum_iterations": MAXIMUM_ITERATIONS,
            "random_restarts": RANDOM_RESTARTS,
            "total_attempts_per_case": RANDOM_RESTARTS + 4,
            "maximum_joint_step_rad": MAXIMUM_JOINT_STEP_RAD,
            "finite_difference_step_rad": FINITE_DIFFERENCE_STEP_RAD,
            "initial_damping": INITIAL_DAMPING,
            "random_seed": RANDOM_SEED,
        },
        "model": {
            "urdf": str(URDF_PATH),
            "urdf_sha256": urdf_hash,
            "base_link": BASE_LINK,
            "tip_link": TIP_LINK,
            "joint_names": ARM_JOINT_NAMES,
            "joint_limits_rad": joint_limits,
        },
        "reachable_cases": reachable_cases,
        "unreachable_case": unreachable_case,
        "attempts": attempts_by_case,
        "safety": {
            "opened_com_ports": False,
            "started_hardware_services": False,
            "published_robot_commands": False,
            "used_offline_computation_only": True,
        },
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
    log(f"Stage 2B-1 status: {report['status']}")
    return 0 if report["status"] == "PASS" else 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline SO-101 3D position IK verification."
    )
    parser.parse_args()
    try:
        return run_verification()
    except Exception as exc:
        ensure_report_dir()
        log(f"ERROR: {exc!r}")
        failure_report = {
            "stage": "2B-1",
            "status": "FAIL",
            "timestamp": timestamp(),
            "failures": [
                {
                    "test": "verification_exception",
                    "measured": repr(exc),
                    "threshold": "no exception",
                    "best_q_deg": [],
                    "detail": "Verifier raised an exception.",
                    "log": str(LOG_PATH),
                }
            ],
            "safety": {
                "opened_com_ports": False,
                "started_hardware_services": False,
                "published_robot_commands": False,
                "used_offline_computation_only": True,
            },
        }
        REPORT_PATH.write_text(
            json.dumps(failure_report, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
