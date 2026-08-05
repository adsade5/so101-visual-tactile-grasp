from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np

from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.ik import solve_ik
from so101_mvp_kinematics.joint_limits import clamp_to_limits, joints_within_limits
from so101_mvp_kinematics.model import JOINT_NAMES, So101KinematicModel
from so101_mvp_kinematics.transforms import normalize_vector, rotation_angle_error


ARM_JOINT_NAMES = tuple(JOINT_NAMES)
REFERENCE_SEED_RAD = np.asarray([0.0, -0.35, 0.35, 1.22, 0.0], dtype=np.float64)
TOOL_APPROACH_AXIS_LOCAL = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
DESIRED_APPROACH_BASE = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
DEFAULT_POSITION_TOLERANCE_M = 0.002
DEFAULT_APPROACH_TOLERANCE_DEG = 5.0
MAX_SEED_COUNT = 7


@dataclass(frozen=True)
class PoseSnapshot:
    frame_id: str
    position_m: np.ndarray
    received_monotonic_s: float


@dataclass(frozen=True)
class JointStateSnapshot:
    names: tuple[str, ...]
    positions_rad: np.ndarray
    received_monotonic_s: float


@dataclass(frozen=True)
class SeedCandidate:
    source: str
    q_rad: np.ndarray


@dataclass(frozen=True)
class IkAttempt:
    attempt_index: int
    seed_source: str
    seed_rad: np.ndarray
    solver_success: bool
    solver_reason: str | None
    iterations: int | None
    position_error_m: float | None
    approach_error_deg: float | None
    joint_limit_valid: bool
    solution_rad: np.ndarray | None
    fk_valid: bool
    final_reason: str

    def to_dict(self) -> dict[str, object]:
        return {
            "attempt_index": self.attempt_index,
            "seed_source": self.seed_source,
            "seed_rad": listf(self.seed_rad),
            "solver_success": self.solver_success,
            "solver_reason": self.solver_reason,
            "iterations": self.iterations,
            "position_error_m": self.position_error_m,
            "approach_error_deg": self.approach_error_deg,
            "joint_limit_valid": self.joint_limit_valid,
            "solution_rad": None if self.solution_rad is None else listf(self.solution_rad),
            "fk_valid": self.fk_valid,
            "final_reason": self.final_reason,
        }


@dataclass(frozen=True)
class PregraspPlan:
    success: bool
    reason: str
    object_position_m: np.ndarray | None = None
    pregrasp_position_m: np.ndarray | None = None
    joint_positions_rad: np.ndarray | None = None
    seed_source: str | None = None
    selected_attempt_index: int | None = None
    position_error_m: float | None = None
    approach_error_deg: float | None = None
    final_position_m: np.ndarray | None = None
    object_pose_age_s: float | None = None
    joint_state_seed_available: bool = False
    target_radius_xy_m: float | None = None
    target_distance_3d_m: float | None = None
    approx_max_reach_m: float | None = None
    seeds_attempted: list[SeedCandidate] = field(default_factory=list)
    attempt_results: list[IkAttempt] = field(default_factory=list)

    def best_position_error_m(self) -> float | None:
        values = [
            attempt.position_error_m
            for attempt in self.attempt_results
            if attempt.position_error_m is not None
        ]
        return None if not values else float(min(values))

    def best_approach_error_deg(self) -> float | None:
        values = [
            attempt.approach_error_deg
            for attempt in self.attempt_results
            if attempt.approach_error_deg is not None
        ]
        return None if not values else float(min(values))

    def last_solver_reason(self) -> str | None:
        if not self.attempt_results:
            return None
        return self.attempt_results[-1].solver_reason

    def diagnostic_dict(self, timestamp_s: float | None = None) -> dict[str, object]:
        return {
            "timestamp": timestamp_s,
            "object_pose_base": None
            if self.object_position_m is None
            else listf(self.object_position_m),
            "pregrasp_pose_base": None
            if self.pregrasp_position_m is None
            else listf(self.pregrasp_position_m),
            "object_pose_age_s": self.object_pose_age_s,
            "target_radius_xy_m": self.target_radius_xy_m,
            "target_distance_3d_m": self.target_distance_3d_m,
            "approx_max_reach_m": self.approx_max_reach_m,
            "desired_approach": listf(DESIRED_APPROACH_BASE),
            "joint_state_seed_available": self.joint_state_seed_available,
            "seeds_attempted": [
                {"source": seed.source, "seed_rad": listf(seed.q_rad)}
                for seed in self.seeds_attempted
            ],
            "attempt_results": [attempt.to_dict() for attempt in self.attempt_results],
            "selected_attempt": self.selected_attempt_index,
            "selected_solution_rad": None
            if self.joint_positions_rad is None
            else listf(self.joint_positions_rad),
            "final_reason": self.reason,
        }


def default_urdf_path(project_root: str | Path) -> Path:
    return (
        Path(project_root)
        / "data"
        / "robot_model"
        / "so101"
        / "so101_new_calib.urdf"
    )


def create_model(project_root: str | Path) -> So101KinematicModel:
    return So101KinematicModel(default_urdf_path(project_root))


def top_down_quaternion_xyzw() -> tuple[float, float, float, float]:
    return (0.0, 1.0, 0.0, 0.0)


def finite_position(position_m: np.ndarray) -> bool:
    values = np.asarray(position_m, dtype=np.float64)
    return bool(values.shape == (3,) and np.all(np.isfinite(values)))


def listf(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def fmt_xyz(values: np.ndarray | None) -> str:
    if values is None:
        return "null"
    return "[" + ", ".join(f"{float(value):.6f}" for value in values.tolist()) + "]"


def optional_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def optional_int(value: object) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def approx_max_reach_from_urdf(model: So101KinematicModel) -> float:
    total = 0.0
    for joint in model.chain:
        total += float(np.linalg.norm(joint.origin_xyz))
    return total


def fresh_joint_state_seed(
    joint_state: JointStateSnapshot | None,
    *,
    now_monotonic_s: float,
    max_age_s: float,
) -> np.ndarray | None:
    if joint_state is None:
        return None
    if now_monotonic_s - joint_state.received_monotonic_s > max_age_s:
        return None
    if tuple(joint_state.names) != ARM_JOINT_NAMES:
        return None
    values = np.asarray(joint_state.positions_rad, dtype=np.float64)
    if values.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(values)):
        return None
    return values.copy()


def clean_seed(model: So101KinematicModel, q_rad: np.ndarray) -> np.ndarray | None:
    values = np.asarray(q_rad, dtype=np.float64)
    if values.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(values)):
        return None
    values = clamp_to_limits(model, values)
    if not joints_within_limits(model, values):
        return None
    return values


def build_seed_candidates(
    model: So101KinematicModel,
    pregrasp_position_m: np.ndarray,
    joint_state: JointStateSnapshot | None,
    *,
    now_monotonic_s: float,
    use_joint_state_seed: bool,
    max_joint_state_age_s: float,
) -> tuple[list[SeedCandidate], bool]:
    seeds: list[SeedCandidate] = []
    seen: set[tuple[float, ...]] = set()

    def add(source: str, q_rad: np.ndarray) -> None:
        if len(seeds) >= MAX_SEED_COUNT:
            return
        clean = clean_seed(model, q_rad)
        if clean is None:
            return
        key = tuple(round(float(value), 12) for value in clean.tolist())
        if key in seen:
            return
        seen.add(key)
        seeds.append(SeedCandidate(source=source, q_rad=clean))

    joint_seed = (
        fresh_joint_state_seed(
            joint_state,
            now_monotonic_s=now_monotonic_s,
            max_age_s=max_joint_state_age_s,
        )
        if use_joint_state_seed
        else None
    )
    joint_state_seed_available = joint_seed is not None
    if joint_seed is not None:
        add("joint_state", joint_seed)

    base_yaw = math.atan2(float(pregrasp_position_m[1]), float(pregrasp_position_m[0]))
    target_yaw_seed = REFERENCE_SEED_RAD.copy()
    target_yaw_seed[0] = base_yaw
    add("target_yaw", target_yaw_seed)
    add("reference", REFERENCE_SEED_RAD)

    yaw_plus = target_yaw_seed.copy()
    yaw_plus[0] = base_yaw + math.radians(15.0)
    add("target_yaw_plus_15deg", yaw_plus)

    yaw_minus = target_yaw_seed.copy()
    yaw_minus[0] = base_yaw - math.radians(15.0)
    add("target_yaw_minus_15deg", yaw_minus)

    add(
        "elbow_high",
        np.asarray([base_yaw, -0.55, 0.55, 1.10, 0.0], dtype=np.float64),
    )
    add(
        "elbow_low",
        np.asarray([base_yaw, -0.20, 0.20, 1.35, 0.0], dtype=np.float64),
    )
    return seeds, joint_state_seed_available


def validate_fk_result(
    model: So101KinematicModel,
    q_rad: np.ndarray,
    target_position_m: np.ndarray,
    *,
    position_tolerance_m: float,
    approach_tolerance_deg: float,
) -> tuple[bool, float, float, np.ndarray]:
    fk = forward_kinematics(model, q_rad)
    position = np.asarray(fk["position_m"], dtype=np.float64)
    rotation = np.asarray(fk["rotation_matrix"], dtype=np.float64)
    current_approach = normalize_vector(
        rotation @ TOOL_APPROACH_AXIS_LOCAL,
        "current approach",
    )
    position_error_m = float(np.linalg.norm(target_position_m - position))
    approach_error_deg = math.degrees(
        rotation_angle_error(current_approach, DESIRED_APPROACH_BASE)
    )
    ok = bool(
        position_error_m <= position_tolerance_m
        and approach_error_deg <= approach_tolerance_deg
    )
    return ok, position_error_m, approach_error_deg, position


def make_failure_message(plan: PregraspPlan) -> str:
    parts = [
        plan.reason,
        f"object_xyz_m={fmt_xyz(plan.object_position_m)}",
        f"pregrasp_xyz_m={fmt_xyz(plan.pregrasp_position_m)}",
        f"attempt_count={len(plan.attempt_results)}",
        f"best_position_error_m={_fmt_optional(plan.best_position_error_m(), 6)}",
        f"best_approach_error_deg={_fmt_optional(plan.best_approach_error_deg(), 3)}",
        f"last_solver_reason={plan.last_solver_reason()}",
    ]
    return " ".join(parts)[:480]


def make_success_message(plan: PregraspPlan) -> str:
    assert plan.pregrasp_position_m is not None
    return (
        "pregrasp_ready "
        f"seed_source={plan.seed_source} "
        f"attempt_index={plan.selected_attempt_index} "
        f"x={plan.pregrasp_position_m[0]:.6f} "
        f"y={plan.pregrasp_position_m[1]:.6f} "
        f"z={plan.pregrasp_position_m[2]:.6f} "
        f"position_error_m={plan.position_error_m:.6f} "
        f"approach_error_deg={plan.approach_error_deg:.3f}"
    )


def _fmt_optional(value: float | None, digits: int) -> str:
    return "null" if value is None else f"{float(value):.{digits}f}"


def compute_pregrasp_plan(
    *,
    model: So101KinematicModel,
    object_pose: PoseSnapshot | None,
    joint_state: JointStateSnapshot | None,
    base_frame: str,
    now_monotonic_s: float,
    max_object_pose_age_s: float,
    pregrasp_height_m: float,
    use_joint_state_seed: bool,
    max_joint_state_age_s: float,
    position_tolerance_m: float = DEFAULT_POSITION_TOLERANCE_M,
    approach_tolerance_deg: float = DEFAULT_APPROACH_TOLERANCE_DEG,
    ik_solver: Callable[..., dict[str, object]] = solve_ik,
) -> PregraspPlan:
    approx_reach = approx_max_reach_from_urdf(model)
    if object_pose is None:
        return PregraspPlan(False, "no_object_pose", approx_max_reach_m=approx_reach)

    object_pose_age_s = now_monotonic_s - object_pose.received_monotonic_s
    if object_pose_age_s > max_object_pose_age_s:
        return PregraspPlan(
            False,
            "object_pose_stale",
            object_position_m=np.asarray(object_pose.position_m, dtype=np.float64),
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
        )

    if object_pose.frame_id != base_frame:
        return PregraspPlan(
            False,
            "invalid_frame",
            object_position_m=np.asarray(object_pose.position_m, dtype=np.float64),
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
        )

    object_position = np.asarray(object_pose.position_m, dtype=np.float64)
    if not finite_position(object_position):
        return PregraspPlan(
            False,
            "non_finite_object_pose",
            object_position_m=object_position,
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
        )

    pregrasp_position = object_position + np.asarray(
        [0.0, 0.0, float(pregrasp_height_m)],
        dtype=np.float64,
    )
    if not finite_position(pregrasp_position):
        return PregraspPlan(
            False,
            "non_finite_object_pose",
            object_position_m=object_position,
            pregrasp_position_m=pregrasp_position,
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
        )

    target_radius_xy = float(np.linalg.norm(pregrasp_position[:2]))
    target_distance_3d = float(np.linalg.norm(pregrasp_position))
    seeds, joint_state_seed_available = build_seed_candidates(
        model,
        pregrasp_position,
        joint_state,
        now_monotonic_s=now_monotonic_s,
        use_joint_state_seed=use_joint_state_seed,
        max_joint_state_age_s=max_joint_state_age_s,
    )

    attempt_results: list[IkAttempt] = []
    final_failure_reason = "ik_failed_all_seeds"
    for index, seed in enumerate(seeds, start=1):
        ik_result = ik_solver(
            model,
            pregrasp_position,
            seed.q_rad,
            DESIRED_APPROACH_BASE,
            TOOL_APPROACH_AXIS_LOCAL,
            position_tolerance_m=position_tolerance_m,
            approach_tolerance_deg=approach_tolerance_deg,
        )
        solver_success = bool(ik_result.get("success"))
        solver_reason = None
        if "reason" in ik_result:
            solver_reason = str(ik_result.get("reason"))
        iterations = optional_int(ik_result.get("iterations"))
        position_error_m = optional_float(ik_result.get("position_error_m"))
        approach_error_deg = optional_float(ik_result.get("approach_error_deg"))

        solution = None
        raw_solution = ik_result.get("joint_positions_rad")
        if raw_solution is not None:
            solution = np.asarray(raw_solution, dtype=np.float64)
            if solution.shape != (len(ARM_JOINT_NAMES),):
                solution = None

        joint_limit_valid = bool(solution is not None and joints_within_limits(model, solution))
        fk_valid = False
        final_position = None
        final_reason = "solver_failed"
        if solver_success and solution is not None and np.all(np.isfinite(solution)):
            if not joint_limit_valid:
                final_reason = "joint_limit_failed"
                final_failure_reason = "joint_limit_failed"
            else:
                try:
                    (
                        fk_valid,
                        position_error_m,
                        approach_error_deg,
                        final_position,
                    ) = validate_fk_result(
                        model,
                        solution,
                        pregrasp_position,
                        position_tolerance_m=position_tolerance_m,
                        approach_tolerance_deg=approach_tolerance_deg,
                    )
                except (ValueError, np.linalg.LinAlgError):
                    fk_valid = False
                if not fk_valid:
                    final_reason = "fk_validation_failed"
                    final_failure_reason = "fk_validation_failed"
                else:
                    final_reason = "pregrasp_ready"

        attempt = IkAttempt(
            attempt_index=index,
            seed_source=seed.source,
            seed_rad=seed.q_rad,
            solver_success=solver_success,
            solver_reason=solver_reason,
            iterations=iterations,
            position_error_m=position_error_m,
            approach_error_deg=approach_error_deg,
            joint_limit_valid=joint_limit_valid,
            solution_rad=solution,
            fk_valid=fk_valid,
            final_reason=final_reason,
        )
        attempt_results.append(attempt)

        if final_reason == "pregrasp_ready":
            return PregraspPlan(
                True,
                "pregrasp_ready",
                object_position_m=object_position,
                pregrasp_position_m=pregrasp_position,
                joint_positions_rad=solution,
                seed_source=seed.source,
                selected_attempt_index=index,
                position_error_m=position_error_m,
                approach_error_deg=approach_error_deg,
                final_position_m=final_position,
                object_pose_age_s=object_pose_age_s,
                joint_state_seed_available=joint_state_seed_available,
                target_radius_xy_m=target_radius_xy,
                target_distance_3d_m=target_distance_3d,
                approx_max_reach_m=approx_reach,
                seeds_attempted=seeds,
                attempt_results=attempt_results,
            )

    return PregraspPlan(
        False,
        final_failure_reason,
        object_position_m=object_position,
        pregrasp_position_m=pregrasp_position,
        object_pose_age_s=object_pose_age_s,
        joint_state_seed_available=joint_state_seed_available,
        target_radius_xy_m=target_radius_xy,
        target_distance_3d_m=target_distance_3d,
        approx_max_reach_m=approx_reach,
        seeds_attempted=seeds,
        attempt_results=attempt_results,
    )
