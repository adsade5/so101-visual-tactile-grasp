from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .urdf_fk import ARM_JOINT_NAMES, UrdfForwardKinematics


DEFAULT_PROJECT_ROOT = Path(
    "E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp"
)
DEFAULT_URDF_PATH = (
    DEFAULT_PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)

BASE_LINK = "base_link"
TIP_LINK = "gripper_frame_link"
EXPECTED_URDF_SHA256 = (
    "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
)

REFERENCE_Q_DEG = np.asarray([0.0, -20.0, 30.0, 80.0, 0.0], dtype=np.float64)
TOOL_APPROACH_AXIS_LOCAL = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
TARGET_APPROACH_AXIS_BASE = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)

POSITION_TOLERANCE_MM = 1.0
APPROACH_TOLERANCE_DEG = 2.0
MINIMUM_LIMIT_MARGIN_RAD = 0.05
MAXIMUM_ADJACENT_DELTA_RAD = 0.15
ORIENTATION_LENGTH_SCALE_M = 0.08
FINITE_DIFFERENCE_STEP_RAD = 1.0e-5
MAXIMUM_ITERATIONS = 400
MAXIMUM_JOINT_STEP_RAD = 0.12
INITIAL_DAMPING = 0.02
LOCAL_RANDOM_STARTS = 12
LOCAL_RANDOM_STD_RAD = 0.06
RANDOM_SEED = 20260804


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_array(values: np.ndarray, name: str) -> None:
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} contains NaN or Inf")


def format_float_list(values: np.ndarray | list[float]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def q_to_dict(q: np.ndarray) -> dict[str, float]:
    return {
        name: float(value)
        for name, value in zip(ARM_JOINT_NAMES, q.tolist(), strict=True)
    }


def small_dot(left: np.ndarray, right: np.ndarray) -> float:
    return float(sum(float(a) * float(b) for a, b in zip(left, right, strict=True)))


def small_matvec(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
    rows, cols = matrix.shape
    result = np.zeros(rows, dtype=np.float64)
    for row in range(rows):
        total = 0.0
        for col in range(cols):
            total += float(matrix[row, col]) * float(vector[col])
        result[row] = total
    return result


def small_gram_rows(matrix: np.ndarray) -> np.ndarray:
    rows, cols = matrix.shape
    result = np.zeros((rows, rows), dtype=np.float64)
    for row in range(rows):
        for col in range(rows):
            total = 0.0
            for index in range(cols):
                total += float(matrix[row, index]) * float(matrix[col, index])
            result[row, col] = total
    return result


def small_transpose_matvec(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
    rows, cols = matrix.shape
    result = np.zeros(cols, dtype=np.float64)
    for col in range(cols):
        total = 0.0
        for row in range(rows):
            total += float(matrix[row, col]) * float(vector[row])
        result[col] = total
    return result


def small_solve(matrix: np.ndarray, vector: np.ndarray) -> np.ndarray:
    size = int(matrix.shape[0])
    if matrix.shape != (size, size) or vector.shape != (size,):
        raise np.linalg.LinAlgError("small_solve shape mismatch")
    augmented = [
        [float(matrix[row, col]) for col in range(size)] + [float(vector[row])]
        for row in range(size)
    ]
    for col in range(size):
        pivot = max(range(col, size), key=lambda row: abs(augmented[row][col]))
        pivot_value = augmented[pivot][col]
        if abs(pivot_value) <= 1.0e-15:
            raise np.linalg.LinAlgError("singular matrix")
        if pivot != col:
            augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        inv_pivot = 1.0 / augmented[col][col]
        for index in range(col, size + 1):
            augmented[col][index] *= inv_pivot
        for row in range(size):
            if row == col:
                continue
            factor = augmented[row][col]
            if factor == 0.0:
                continue
            for index in range(col, size + 1):
                augmented[row][index] -= factor * augmented[col][index]
    return np.asarray([augmented[row][size] for row in range(size)], dtype=np.float64)


def validate_transform(transform: np.ndarray) -> None:
    if transform.shape != (4, 4):
        raise ValueError(f"FK transform shape is {transform.shape}, expected (4, 4)")
    finite_array(transform, "FK transform")
    bottom_row_error = float(
        np.linalg.norm(transform[3] - np.asarray([0.0, 0.0, 0.0, 1.0]))
    )
    if bottom_row_error > 1.0e-12:
        raise ValueError(f"Invalid homogeneous transform bottom row: {bottom_row_error}")


def joint_limits_from_fk(
    fk: UrdfForwardKinematics,
) -> tuple[np.ndarray, np.ndarray, dict[str, dict[str, float]]]:
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


def minimum_joint_limit_margin(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> float:
    lower_margin = q - lower
    upper_margin = upper - q
    return float(np.min(np.minimum(lower_margin, upper_margin)))


def normalize(vector: np.ndarray, name: str) -> np.ndarray:
    finite_array(vector, name)
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} has zero norm")
    return vector / norm


def pose_data(
    fk: UrdfForwardKinematics,
    q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    transform = fk.compute(q_to_dict(q))
    validate_transform(transform)
    position = np.asarray(transform[:3, 3], dtype=np.float64)
    rotation = np.asarray(transform[:3, :3], dtype=np.float64)
    approach = normalize(small_matvec(rotation, TOOL_APPROACH_AXIS_LOCAL), "tool approach axis")
    return position, rotation, approach


def angle_degrees(first: np.ndarray, second: np.ndarray) -> float:
    first_normalized = normalize(first, "first axis")
    second_normalized = normalize(second, "second axis")
    dot_product = float(np.clip(small_dot(first_normalized, second_normalized), -1.0, 1.0))
    return math.degrees(math.acos(dot_product))


@dataclass
class ConstrainedAttempt:
    seed: str
    iterations: int
    q: np.ndarray
    position: np.ndarray
    approach: np.ndarray
    position_error_mm: float
    approach_error_deg: float
    within_limits: bool
    minimum_limit_margin_rad: float
    success: bool
    line_search_backtracks: int
    final_damping: float

    def to_report(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "iterations": self.iterations,
            "q_rad": format_float_list(self.q),
            "q_deg": format_float_list(np.degrees(self.q)),
            "achieved_position_m": format_float_list(self.position),
            "achieved_approach_axis_base": format_float_list(self.approach),
            "position_error_mm": self.position_error_mm,
            "approach_error_deg": self.approach_error_deg,
            "within_joint_limits": self.within_limits,
            "minimum_joint_limit_margin_rad": self.minimum_limit_margin_rad,
            "success": self.success,
            "line_search_backtracks": self.line_search_backtracks,
            "final_damping": self.final_damping,
        }


@dataclass
class SafeTopDownResult:
    success: bool
    q_rad: np.ndarray | None
    achieved_position_m: np.ndarray | None
    achieved_approach_axis_base: np.ndarray | None
    position_error_mm: float | None
    approach_error_deg: float | None
    minimum_joint_limit_margin_rad: float | None
    failure_reason: str
    selected_seed: str | None
    attempts: list[ConstrainedAttempt]

    def to_report(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "q_rad": None if self.q_rad is None else format_float_list(self.q_rad),
            "q_deg": None if self.q_rad is None else format_float_list(np.degrees(self.q_rad)),
            "achieved_position_m": (
                None
                if self.achieved_position_m is None
                else format_float_list(self.achieved_position_m)
            ),
            "achieved_approach_axis_base": (
                None
                if self.achieved_approach_axis_base is None
                else format_float_list(self.achieved_approach_axis_base)
            ),
            "position_error_mm": self.position_error_mm,
            "approach_error_deg": self.approach_error_deg,
            "minimum_joint_limit_margin_rad": self.minimum_joint_limit_margin_rad,
            "failure_reason": self.failure_reason,
            "selected_seed": self.selected_seed,
            "attempt_count": len(self.attempts),
        }


@dataclass
class VerticalPathPoint:
    index: int
    target_position_m: np.ndarray
    result: SafeTopDownResult
    adjacent_max_joint_delta_rad: float | None

    def to_report(self) -> dict[str, Any]:
        report = {
            "point_index": self.index,
            "target_position_m": format_float_list(self.target_position_m),
            "adjacent_max_joint_delta_rad": self.adjacent_max_joint_delta_rad,
        }
        report.update(self.result.to_report())
        return report


@dataclass
class VerticalPathResult:
    success: bool
    points: list[VerticalPathPoint]
    q_path_rad: list[np.ndarray]
    max_position_error_mm: float | None
    max_approach_error_deg: float | None
    min_margin_rad: float | None
    max_adjacent_delta_rad: float | None
    failure_reason: str

    def to_report(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "waypoint_count": len(self.points),
            "max_position_error_mm": self.max_position_error_mm,
            "max_approach_error_deg": self.max_approach_error_deg,
            "min_margin_rad": self.min_margin_rad,
            "max_adjacent_delta_rad": self.max_adjacent_delta_rad,
            "failure_reason": self.failure_reason,
            "points": [point.to_report() for point in self.points],
        }


class TopDownIKSolver:
    def __init__(
        self,
        fk: UrdfForwardKinematics,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> None:
        self.fk = fk
        self.lower = np.asarray(lower, dtype=np.float64)
        self.upper = np.asarray(upper, dtype=np.float64)
        self.target_axis = normalize(TARGET_APPROACH_AXIS_BASE, "target approach axis")

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper)

    def within_limits(self, q: np.ndarray) -> bool:
        return bool(
            q.shape == (len(ARM_JOINT_NAMES),)
            and np.all(q >= self.lower - 1.0e-10)
            and np.all(q <= self.upper + 1.0e-10)
        )

    def residual(self, q: np.ndarray, target_position: np.ndarray) -> np.ndarray:
        position, _, approach = pose_data(self.fk, q)
        position_residual = target_position - position
        direction_residual = ORIENTATION_LENGTH_SCALE_M * np.cross(
            approach,
            self.target_axis,
        )
        residual = np.concatenate([position_residual, direction_residual])
        finite_array(residual, "task residual")
        return residual

    def numerical_residual_jacobian(
        self,
        q: np.ndarray,
        target_position: np.ndarray,
    ) -> np.ndarray:
        baseline = self.residual(q, target_position)
        columns: list[np.ndarray] = []
        for index in range(len(ARM_JOINT_NAMES)):
            minus_room = float(q[index] - self.lower[index])
            plus_room = float(self.upper[index] - q[index])
            h_minus = min(FINITE_DIFFERENCE_STEP_RAD, minus_room)
            h_plus = min(FINITE_DIFFERENCE_STEP_RAD, plus_room)
            if h_minus >= 1.0e-8 and h_plus >= 1.0e-8:
                q_plus = q.copy()
                q_minus = q.copy()
                q_plus[index] += h_plus
                q_minus[index] -= h_minus
                column = (
                    self.residual(q_plus, target_position)
                    - self.residual(q_minus, target_position)
                ) / (h_plus + h_minus)
            elif h_plus >= 1.0e-8:
                q_plus = q.copy()
                q_plus[index] += h_plus
                column = (self.residual(q_plus, target_position) - baseline) / h_plus
            elif h_minus >= 1.0e-8:
                q_minus = q.copy()
                q_minus[index] -= h_minus
                column = (baseline - self.residual(q_minus, target_position)) / h_minus
            else:
                column = np.zeros(6, dtype=np.float64)
            columns.append(column)
        jacobian = np.column_stack(columns).astype(np.float64)
        finite_array(jacobian, "task Jacobian")
        return jacobian

    def solve_one(
        self,
        target_position: np.ndarray,
        initial_q: np.ndarray,
        seed: str,
    ) -> ConstrainedAttempt:
        target_position = np.asarray(target_position, dtype=np.float64)
        finite_array(target_position, "target position")
        if target_position.shape != (3,):
            raise ValueError(
                f"target position shape is {target_position.shape}, expected (3,)"
            )
        q = self.clamp(initial_q)
        damping = INITIAL_DAMPING
        backtracks = 0
        iterations = 0
        for iteration in range(1, MAXIMUM_ITERATIONS + 1):
            iterations = iteration
            position, _, approach = pose_data(self.fk, q)
            position_error_mm = float(np.linalg.norm(target_position - position) * 1000.0)
            approach_error_deg = angle_degrees(approach, self.target_axis)
            if (
                position_error_mm <= POSITION_TOLERANCE_MM
                and approach_error_deg <= APPROACH_TOLERANCE_DEG
            ):
                break
            residual = self.residual(q, target_position)
            current_norm = float(np.linalg.norm(residual))
            jacobian = self.numerical_residual_jacobian(q, target_position)
            lhs = small_gram_rows(jacobian) + (damping * damping) * np.eye(6)
            try:
                joint_step = -small_transpose_matvec(jacobian, small_solve(lhs, residual))
            except np.linalg.LinAlgError:
                damping = min(0.5, damping * 2.0)
                continue
            finite_array(joint_step, "joint step")
            largest_step = float(np.max(np.abs(joint_step)))
            if largest_step > MAXIMUM_JOINT_STEP_RAD:
                joint_step *= MAXIMUM_JOINT_STEP_RAD / largest_step
            accepted = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                candidate = self.clamp(q + alpha * joint_step)
                _, _, candidate_approach = pose_data(self.fk, candidate)
                if small_dot(candidate_approach, self.target_axis) <= 0.0:
                    backtracks += 1
                    continue
                candidate_norm = float(
                    np.linalg.norm(self.residual(candidate, target_position))
                )
                if candidate_norm < current_norm:
                    q = candidate
                    damping = max(0.001, damping * 0.8)
                    accepted = True
                    if alpha < 1.0:
                        backtracks += 1
                    break
                backtracks += 1
            if not accepted:
                damping = min(0.5, damping * 2.0)
        position, _, approach = pose_data(self.fk, q)
        position_error_mm = float(np.linalg.norm(target_position - position) * 1000.0)
        approach_error_deg = angle_degrees(approach, self.target_axis)
        within = self.within_limits(q)
        margin = minimum_joint_limit_margin(q, self.lower, self.upper)
        success = bool(
            position_error_mm <= POSITION_TOLERANCE_MM
            and approach_error_deg <= APPROACH_TOLERANCE_DEG
            and within
        )
        return ConstrainedAttempt(
            seed=seed,
            iterations=iterations,
            q=q,
            position=position,
            approach=approach,
            position_error_mm=position_error_mm,
            approach_error_deg=approach_error_deg,
            within_limits=within,
            minimum_limit_margin_rad=margin,
            success=success,
            line_search_backtracks=backtracks,
            final_damping=float(damping),
        )


def build_local_starts(
    reference_q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    rng: np.random.Generator,
) -> list[tuple[str, np.ndarray]]:
    starts: list[tuple[str, np.ndarray]] = [("reference", reference_q.copy())]
    for joint_index, joint_name in enumerate(ARM_JOINT_NAMES):
        for magnitude in (0.04, 0.08):
            for sign in (-1.0, 1.0):
                candidate = reference_q.copy()
                candidate[joint_index] += sign * magnitude
                starts.append(
                    (
                        f"{joint_name}_{sign:+.0f}_{magnitude:.2f}",
                        np.clip(candidate, lower, upper),
                    )
                )
    for index in range(LOCAL_RANDOM_STARTS):
        candidate = reference_q + rng.normal(
            0.0,
            LOCAL_RANDOM_STD_RAD,
            size=len(ARM_JOINT_NAMES),
        )
        starts.append((f"local_random_{index:02d}", np.clip(candidate, lower, upper)))
    return starts


def normalized_joint_distance(
    first: np.ndarray,
    second: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> float:
    normalized_difference = (first - second) / (upper - lower)
    return float(np.sqrt(np.mean(normalized_difference**2)))


def create_default_solver(
    project_root: Path | str = DEFAULT_PROJECT_ROOT,
) -> tuple[TopDownIKSolver, dict[str, Any]]:
    project_root = Path(project_root).resolve()
    urdf_path = (
        project_root
        / "data"
        / "robot_model"
        / "so101"
        / "so101_new_calib.urdf"
    )
    fk = UrdfForwardKinematics(
        urdf_path=urdf_path,
        base_link=BASE_LINK,
        tip_link=TIP_LINK,
    )
    fk.validate_expected_chain()
    lower, upper, joint_limits = joint_limits_from_fk(fk)
    solver = TopDownIKSolver(fk=fk, lower=lower, upper=upper)
    metadata = {
        "urdf": str(urdf_path),
        "urdf_sha256": sha256_file(urdf_path),
        "base_link": BASE_LINK,
        "tip_link": TIP_LINK,
        "joint_names": ARM_JOINT_NAMES,
        "joint_limits_rad": joint_limits,
    }
    return solver, metadata


def solve_safe_top_down_position(
    solver: TopDownIKSolver,
    target_position_m: np.ndarray | list[float],
    reference_q_rad: np.ndarray | list[float],
    position_tolerance_mm: float = POSITION_TOLERANCE_MM,
    approach_tolerance_deg: float = APPROACH_TOLERANCE_DEG,
    minimum_limit_margin_rad: float = MINIMUM_LIMIT_MARGIN_RAD,
    rng: np.random.Generator | None = None,
) -> SafeTopDownResult:
    target_position = np.asarray(target_position_m, dtype=np.float64)
    reference_q = np.asarray(reference_q_rad, dtype=np.float64)
    if target_position.shape != (3,):
        raise ValueError(f"target_position_m shape is {target_position.shape}, expected (3,)")
    if reference_q.shape != (len(ARM_JOINT_NAMES),):
        raise ValueError(f"reference_q_rad shape is {reference_q.shape}, expected (5,)")
    finite_array(target_position, "target_position_m")
    finite_array(reference_q, "reference_q_rad")
    rng = rng or np.random.default_rng(RANDOM_SEED)
    attempts = [
        solver.solve_one(target_position, start_q, seed_name)
        for seed_name, start_q in build_local_starts(
            reference_q,
            solver.lower,
            solver.upper,
            rng,
        )
    ]
    safe = [
        attempt
        for attempt in attempts
        if (
            attempt.success
            and attempt.position_error_mm <= position_tolerance_mm
            and attempt.approach_error_deg <= approach_tolerance_deg
            and attempt.minimum_limit_margin_rad >= minimum_limit_margin_rad
            and np.all(np.isfinite(attempt.q))
        )
    ]
    safe.sort(
        key=lambda attempt: (
            normalized_joint_distance(
                attempt.q,
                reference_q,
                solver.lower,
                solver.upper,
            )
            + 0.001 * attempt.position_error_mm
            + 0.001 * attempt.approach_error_deg,
            attempt.position_error_mm,
        )
    )
    if not safe:
        best = min(
            attempts,
            key=lambda item: item.position_error_mm + item.approach_error_deg,
        )
        return SafeTopDownResult(
            success=False,
            q_rad=None,
            achieved_position_m=best.position,
            achieved_approach_axis_base=best.approach,
            position_error_mm=best.position_error_mm,
            approach_error_deg=best.approach_error_deg,
            minimum_joint_limit_margin_rad=best.minimum_limit_margin_rad,
            failure_reason=(
                "no safe top-down IK solution: "
                f"best_position_error_mm={best.position_error_mm:.6f}, "
                f"best_approach_error_deg={best.approach_error_deg:.6f}, "
                f"best_margin_rad={best.minimum_limit_margin_rad:.6f}"
            ),
            selected_seed=None,
            attempts=attempts,
        )
    selected = safe[0]
    return SafeTopDownResult(
        success=True,
        q_rad=selected.q,
        achieved_position_m=selected.position,
        achieved_approach_axis_base=selected.approach,
        position_error_mm=selected.position_error_mm,
        approach_error_deg=selected.approach_error_deg,
        minimum_joint_limit_margin_rad=selected.minimum_limit_margin_rad,
        failure_reason="",
        selected_seed=selected.seed,
        attempts=attempts,
    )


def generate_vertical_joint_path(
    solver: TopDownIKSolver,
    target_positions_m: list[np.ndarray | list[float]],
    reference_q_rad: np.ndarray | list[float],
    position_tolerance_mm: float = POSITION_TOLERANCE_MM,
    approach_tolerance_deg: float = APPROACH_TOLERANCE_DEG,
    minimum_limit_margin_rad: float = MINIMUM_LIMIT_MARGIN_RAD,
    maximum_adjacent_delta_rad: float = MAXIMUM_ADJACENT_DELTA_RAD,
    rng: np.random.Generator | None = None,
) -> VerticalPathResult:
    rng = rng or np.random.default_rng(RANDOM_SEED)
    previous_q = np.asarray(reference_q_rad, dtype=np.float64)
    points: list[VerticalPathPoint] = []
    q_path: list[np.ndarray] = []
    max_position = 0.0
    max_approach = 0.0
    min_margin = math.inf
    max_delta = 0.0
    for index, target in enumerate(target_positions_m):
        target_array = np.asarray(target, dtype=np.float64)
        result = solve_safe_top_down_position(
            solver=solver,
            target_position_m=target_array,
            reference_q_rad=previous_q,
            position_tolerance_mm=position_tolerance_mm,
            approach_tolerance_deg=approach_tolerance_deg,
            minimum_limit_margin_rad=minimum_limit_margin_rad,
            rng=rng,
        )
        adjacent_delta = None
        if result.success and result.q_rad is not None:
            adjacent_delta = float(np.max(np.abs(result.q_rad - previous_q)))
            if adjacent_delta > maximum_adjacent_delta_rad:
                result.success = False
                result.failure_reason = (
                    "adjacent joint delta exceeds limit: "
                    f"{adjacent_delta:.6f} > {maximum_adjacent_delta_rad:.6f} rad"
                )
        points.append(
            VerticalPathPoint(
                index=index,
                target_position_m=target_array,
                result=result,
                adjacent_max_joint_delta_rad=adjacent_delta,
            )
        )
        if not result.success or result.q_rad is None:
            return VerticalPathResult(
                success=False,
                points=points,
                q_path_rad=q_path,
                max_position_error_mm=None if not q_path else max_position,
                max_approach_error_deg=None if not q_path else max_approach,
                min_margin_rad=None if not q_path else min_margin,
                max_adjacent_delta_rad=None if not q_path else max_delta,
                failure_reason=result.failure_reason or f"waypoint {index} failed",
            )
        q_path.append(result.q_rad.copy())
        previous_q = result.q_rad.copy()
        max_position = max(max_position, float(result.position_error_mm or 0.0))
        max_approach = max(max_approach, float(result.approach_error_deg or 0.0))
        min_margin = min(min_margin, float(result.minimum_joint_limit_margin_rad or 0.0))
        max_delta = max(max_delta, float(adjacent_delta or 0.0))
    return VerticalPathResult(
        success=True,
        points=points,
        q_path_rad=q_path,
        max_position_error_mm=max_position,
        max_approach_error_deg=max_approach,
        min_margin_rad=min_margin,
        max_adjacent_delta_rad=max_delta,
        failure_reason="",
    )
