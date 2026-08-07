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
DEFAULT_PREGRASP_POSITION_TOLERANCE_M = 0.010
DEFAULT_PREGRASP_APPROACH_TOLERANCE_DEG = 5.0
MAX_SEED_COUNT = 7
CANDIDATE_OFFSETS_M = tuple(
    np.asarray(values, dtype=np.float64)
    for values in (
        (0.000, 0.000, 0.000),
        (0.000, 0.000, 0.005),
        (0.000, 0.000, 0.010),
        (0.005, 0.000, 0.000),
        (-0.005, 0.000, 0.000),
        (0.000, 0.005, 0.000),
        (0.000, -0.005, 0.000),
    )
)


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
    solution_type: str | None = None

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
            "solution_type": self.solution_type,
        }


@dataclass(frozen=True)
class CandidateResult:
    candidate_index: int
    offset_m: np.ndarray
    requested_pregrasp_position_m: np.ndarray
    selected_pregrasp_position_m: np.ndarray
    attempt_results: list[IkAttempt]
    success: bool
    selected_attempt_index: int | None = None
    seed_source: str | None = None
    solution_type: str | None = None
    solution_rad: np.ndarray | None = None
    position_error_m: float | None = None
    approach_error_deg: float | None = None
    final_position_m: np.ndarray | None = None
    cost: float | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "candidate_index": self.candidate_index,
            "offset_m": listf(self.offset_m),
            "requested_pregrasp_xyz_m": listf(self.requested_pregrasp_position_m),
            "selected_pregrasp_xyz_m": listf(self.selected_pregrasp_position_m),
            "success": self.success,
            "selected_attempt_index": self.selected_attempt_index,
            "seed_source": self.seed_source,
            "solution_type": self.solution_type,
            "solution_rad": None if self.solution_rad is None else listf(self.solution_rad),
            "position_error_m": self.position_error_m,
            "approach_error_deg": self.approach_error_deg,
            "cost": self.cost,
            "attempt_results": [attempt.to_dict() for attempt in self.attempt_results],
        }


@dataclass(frozen=True)
class PregraspPlan:
    success: bool
    reason: str
    object_position_m: np.ndarray | None = None
    requested_pregrasp_position_m: np.ndarray | None = None
    pregrasp_position_m: np.ndarray | None = None
    joint_positions_rad: np.ndarray | None = None
    seed_source: str | None = None
    selected_attempt_index: int | None = None
    selected_candidate_index: int | None = None
    selected_offset_m: np.ndarray | None = None
    solution_type: str | None = None
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
    candidate_offsets_m: tuple[np.ndarray, ...] = CANDIDATE_OFFSETS_M
    candidate_results: list[CandidateResult] = field(default_factory=list)
    acceptance_tolerances: dict[str, float] = field(default_factory=dict)
    best_q_within_joint_limits: bool | None = None

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
            "requested_pregrasp_xyz_m": None
            if self.requested_pregrasp_position_m is None
            else listf(self.requested_pregrasp_position_m),
            "candidate_offsets_m": [listf(offset) for offset in self.candidate_offsets_m],
            "candidate_results": [candidate.to_dict() for candidate in self.candidate_results],
            "selected_candidate_index": self.selected_candidate_index,
            "selected_offset_m": None
            if self.selected_offset_m is None
            else listf(self.selected_offset_m),
            "selected_pregrasp_xyz_m": None
            if self.pregrasp_position_m is None
            else listf(self.pregrasp_position_m),
            "solution_type": self.solution_type,
            "acceptance_tolerances": self.acceptance_tolerances,
            "best_q_within_joint_limits": self.best_q_within_joint_limits,
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


def candidate_cost(
    *,
    position_error_m: float,
    approach_error_deg: float,
    offset_m: np.ndarray,
) -> float:
    return (
        float(position_error_m)
        + 0.001 * float(approach_error_deg)
        + 2.0 * float(np.linalg.norm(offset_m))
    )


def evaluate_pregrasp_ik_result(
    *,
    model: So101KinematicModel,
    ik_result: dict[str, object],
    target_position_m: np.ndarray,
    strict_position_tolerance_m: float,
    strict_approach_tolerance_deg: float,
    pregrasp_position_tolerance_m: float,
    pregrasp_approach_tolerance_deg: float,
) -> tuple[
    bool,
    str,
    str | None,
    np.ndarray | None,
    float | None,
    float | None,
    np.ndarray | None,
    bool,
    bool,
]:
    solver_success = bool(ik_result.get("success"))
    solver_reason = None if "reason" not in ik_result else str(ik_result.get("reason"))
    raw_solution = ik_result.get("joint_positions_rad")
    if raw_solution is None:
        return (
            False,
            "solver_failed",
            None,
            None,
            optional_float(ik_result.get("position_error_m")),
            optional_float(ik_result.get("approach_error_deg")),
            None,
            False,
            False,
        )

    try:
        solution = np.asarray(raw_solution, dtype=np.float64)
    except (TypeError, ValueError):
        return (False, "solver_failed", None, None, None, None, None, False, False)

    if solution.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(solution)):
        return (False, "solver_failed", None, solution, None, None, None, False, False)

    joint_limit_valid = joints_within_limits(model, solution)
    if not joint_limit_valid:
        return (
            False,
            "joint_limit_failed",
            None,
            solution,
            optional_float(ik_result.get("position_error_m")),
            optional_float(ik_result.get("approach_error_deg")),
            None,
            False,
            False,
        )

    try:
        (
            strict_fk_valid,
            position_error_m,
            approach_error_deg,
            final_position_m,
        ) = validate_fk_result(
            model,
            solution,
            target_position_m,
            position_tolerance_m=strict_position_tolerance_m,
            approach_tolerance_deg=strict_approach_tolerance_deg,
        )
    except (ValueError, np.linalg.LinAlgError):
        return (False, "fk_validation_failed", None, solution, None, None, None, True, False)

    if solver_success and strict_fk_valid:
        return (
            True,
            "pregrasp_ready_exact",
            "exact_solution",
            solution,
            position_error_m,
            approach_error_deg,
            final_position_m,
            True,
            True,
        )

    near_allowed = (not solver_success) or solver_reason == "max_iterations"
    near_valid = bool(
        near_allowed
        and position_error_m <= pregrasp_position_tolerance_m
        and approach_error_deg <= pregrasp_approach_tolerance_deg
    )
    if near_valid:
        return (
            True,
            "pregrasp_ready_near",
            "accepted_near_solution",
            solution,
            position_error_m,
            approach_error_deg,
            final_position_m,
            True,
            False,
        )

    return (
        False,
        "fk_validation_failed",
        None,
        solution,
        position_error_m,
        approach_error_deg,
        final_position_m,
        True,
        False,
    )


def solve_candidate(
    *,
    model: So101KinematicModel,
    requested_pregrasp_position_m: np.ndarray,
    candidate_index: int,
    offset_m: np.ndarray,
    joint_state: JointStateSnapshot | None,
    now_monotonic_s: float,
    use_joint_state_seed: bool,
    max_joint_state_age_s: float,
    strict_position_tolerance_m: float,
    strict_approach_tolerance_deg: float,
    pregrasp_position_tolerance_m: float,
    pregrasp_approach_tolerance_deg: float,
    ik_solver: Callable[..., dict[str, object]],
    attempt_start_index: int = 1,
) -> tuple[CandidateResult, list[SeedCandidate], bool]:
    target = requested_pregrasp_position_m + offset_m
    seeds, joint_state_seed_available = build_seed_candidates(
        model,
        target,
        joint_state,
        now_monotonic_s=now_monotonic_s,
        use_joint_state_seed=use_joint_state_seed,
        max_joint_state_age_s=max_joint_state_age_s,
    )
    attempts: list[IkAttempt] = []

    for local_index, seed in enumerate(seeds):
        attempt_index = attempt_start_index + local_index
        ik_result = ik_solver(
            model,
            target,
            seed.q_rad,
            DESIRED_APPROACH_BASE,
            TOOL_APPROACH_AXIS_LOCAL,
            position_tolerance_m=strict_position_tolerance_m,
            approach_tolerance_deg=strict_approach_tolerance_deg,
        )
        (
            accepted,
            final_reason,
            solution_type,
            solution,
            position_error_m,
            approach_error_deg,
            final_position,
            joint_limit_valid,
            strict_fk_valid,
        ) = evaluate_pregrasp_ik_result(
            model=model,
            ik_result=ik_result,
            target_position_m=target,
            strict_position_tolerance_m=strict_position_tolerance_m,
            strict_approach_tolerance_deg=strict_approach_tolerance_deg,
            pregrasp_position_tolerance_m=pregrasp_position_tolerance_m,
            pregrasp_approach_tolerance_deg=pregrasp_approach_tolerance_deg,
        )
        attempt = IkAttempt(
            attempt_index=attempt_index,
            seed_source=seed.source,
            seed_rad=seed.q_rad,
            solver_success=bool(ik_result.get("success")),
            solver_reason=None if "reason" not in ik_result else str(ik_result.get("reason")),
            iterations=optional_int(ik_result.get("iterations")),
            position_error_m=position_error_m,
            approach_error_deg=approach_error_deg,
            joint_limit_valid=joint_limit_valid,
            solution_rad=solution,
            fk_valid=strict_fk_valid,
            final_reason=final_reason,
            solution_type=solution_type,
        )
        attempts.append(attempt)
        if accepted:
            cost = candidate_cost(
                position_error_m=float(position_error_m),
                approach_error_deg=float(approach_error_deg),
                offset_m=offset_m,
            )
            return (
                CandidateResult(
                    candidate_index=candidate_index,
                    offset_m=offset_m,
                    requested_pregrasp_position_m=requested_pregrasp_position_m,
                    selected_pregrasp_position_m=target,
                    attempt_results=attempts,
                    success=True,
                    selected_attempt_index=attempt_index,
                    seed_source=seed.source,
                    solution_type=solution_type,
                    solution_rad=solution,
                    position_error_m=position_error_m,
                    approach_error_deg=approach_error_deg,
                    final_position_m=final_position,
                    cost=cost,
                ),
                seeds,
                joint_state_seed_available,
            )

    return (
        CandidateResult(
            candidate_index=candidate_index,
            offset_m=offset_m,
            requested_pregrasp_position_m=requested_pregrasp_position_m,
            selected_pregrasp_position_m=target,
            attempt_results=attempts,
            success=False,
        ),
        seeds,
        joint_state_seed_available,
    )


def make_failure_message(plan: PregraspPlan) -> str:
    parts = [
        plan.reason,
        f"object_xyz_m={fmt_xyz(plan.object_position_m)}",
        f"requested_xyz={fmt_xyz(plan.requested_pregrasp_position_m)}",
        f"selected_xyz={fmt_xyz(plan.pregrasp_position_m)}",
        f"offset_m={fmt_xyz(plan.selected_offset_m)}",
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
        f"solution_type={plan.solution_type} "
        f"seed_source={plan.seed_source} "
        f"attempt_index={plan.selected_attempt_index} "
        f"selected_candidate_index={plan.selected_candidate_index} "
        f"requested_xyz={fmt_xyz(plan.requested_pregrasp_position_m)} "
        f"selected_xyz={fmt_xyz(plan.pregrasp_position_m)} "
        f"offset_m={fmt_xyz(plan.selected_offset_m)} "
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
    pregrasp_position_tolerance_m: float = DEFAULT_PREGRASP_POSITION_TOLERANCE_M,
    pregrasp_approach_tolerance_deg: float = DEFAULT_PREGRASP_APPROACH_TOLERANCE_DEG,
    ik_solver: Callable[..., dict[str, object]] = solve_ik,
) -> PregraspPlan:
    approx_reach = approx_max_reach_from_urdf(model)
    acceptance_tolerances = {
        "strict_position_tolerance_m": float(position_tolerance_m),
        "strict_approach_tolerance_deg": float(approach_tolerance_deg),
        "pregrasp_position_tolerance_m": float(pregrasp_position_tolerance_m),
        "pregrasp_approach_tolerance_deg": float(pregrasp_approach_tolerance_deg),
    }
    if object_pose is None:
        return PregraspPlan(
            False,
            "no_object_pose",
            approx_max_reach_m=approx_reach,
            acceptance_tolerances=acceptance_tolerances,
        )

    object_pose_age_s = now_monotonic_s - object_pose.received_monotonic_s
    if object_pose_age_s > max_object_pose_age_s:
        return PregraspPlan(
            False,
            "object_pose_stale",
            object_position_m=np.asarray(object_pose.position_m, dtype=np.float64),
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
            acceptance_tolerances=acceptance_tolerances,
        )

    if object_pose.frame_id != base_frame:
        return PregraspPlan(
            False,
            "invalid_frame",
            object_position_m=np.asarray(object_pose.position_m, dtype=np.float64),
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
            acceptance_tolerances=acceptance_tolerances,
        )

    object_position = np.asarray(object_pose.position_m, dtype=np.float64)
    if not finite_position(object_position):
        return PregraspPlan(
            False,
            "non_finite_object_pose",
            object_position_m=object_position,
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
            acceptance_tolerances=acceptance_tolerances,
        )

    requested_pregrasp_position = object_position + np.asarray(
        [0.0, 0.0, float(pregrasp_height_m)],
        dtype=np.float64,
    )
    if not finite_position(requested_pregrasp_position):
        return PregraspPlan(
            False,
            "non_finite_object_pose",
            object_position_m=object_position,
            requested_pregrasp_position_m=requested_pregrasp_position,
            pregrasp_position_m=requested_pregrasp_position,
            object_pose_age_s=object_pose_age_s,
            approx_max_reach_m=approx_reach,
            acceptance_tolerances=acceptance_tolerances,
        )

    target_radius_xy = float(np.linalg.norm(requested_pregrasp_position[:2]))
    target_distance_3d = float(np.linalg.norm(requested_pregrasp_position))
    candidate_results: list[CandidateResult] = []
    all_attempts: list[IkAttempt] = []
    all_seeds: list[SeedCandidate] = []
    joint_state_seed_available = False
    attempt_start_index = 1

    original_candidate, original_seeds, original_joint_seed_available = solve_candidate(
        model=model,
        requested_pregrasp_position_m=requested_pregrasp_position,
        candidate_index=0,
        offset_m=CANDIDATE_OFFSETS_M[0],
        joint_state=joint_state,
        now_monotonic_s=now_monotonic_s,
        use_joint_state_seed=use_joint_state_seed,
        max_joint_state_age_s=max_joint_state_age_s,
        strict_position_tolerance_m=position_tolerance_m,
        strict_approach_tolerance_deg=approach_tolerance_deg,
        pregrasp_position_tolerance_m=pregrasp_position_tolerance_m,
        pregrasp_approach_tolerance_deg=pregrasp_approach_tolerance_deg,
        ik_solver=ik_solver,
        attempt_start_index=attempt_start_index,
    )
    candidate_results.append(original_candidate)
    all_attempts.extend(original_candidate.attempt_results)
    all_seeds = original_seeds
    joint_state_seed_available = original_joint_seed_available
    attempt_start_index += len(original_candidate.attempt_results)

    successful_candidates: list[CandidateResult] = []
    if original_candidate.success:
        successful_candidates.append(original_candidate)
    else:
        for candidate_index, offset_m in enumerate(CANDIDATE_OFFSETS_M[1:], start=1):
            candidate, _, candidate_joint_seed_available = solve_candidate(
                model=model,
                requested_pregrasp_position_m=requested_pregrasp_position,
                candidate_index=candidate_index,
                offset_m=offset_m,
                joint_state=joint_state,
                now_monotonic_s=now_monotonic_s,
                use_joint_state_seed=use_joint_state_seed,
                max_joint_state_age_s=max_joint_state_age_s,
                strict_position_tolerance_m=position_tolerance_m,
                strict_approach_tolerance_deg=approach_tolerance_deg,
                pregrasp_position_tolerance_m=pregrasp_position_tolerance_m,
                pregrasp_approach_tolerance_deg=pregrasp_approach_tolerance_deg,
                ik_solver=ik_solver,
                attempt_start_index=attempt_start_index,
            )
            candidate_results.append(candidate)
            all_attempts.extend(candidate.attempt_results)
            attempt_start_index += len(candidate.attempt_results)
            joint_state_seed_available = (
                joint_state_seed_available or candidate_joint_seed_available
            )
            if candidate.success:
                successful_candidates.append(candidate)

    if successful_candidates:
        selected = sorted(
            successful_candidates,
            key=lambda item: (
                0 if item.candidate_index == 0 else 1,
                float("inf") if item.cost is None else float(item.cost),
                float(np.linalg.norm(item.offset_m)),
                float("inf") if item.position_error_m is None else float(item.position_error_m),
                float("inf") if item.approach_error_deg is None else float(item.approach_error_deg),
            ),
        )[0]
        status = (
            "pregrasp_ready_offset"
            if selected.candidate_index != 0
            else (
                "pregrasp_ready_exact"
                if selected.solution_type == "exact_solution"
                else "pregrasp_ready_near"
            )
        )
        return PregraspPlan(
            True,
            status,
            object_position_m=object_position,
            requested_pregrasp_position_m=requested_pregrasp_position,
            pregrasp_position_m=selected.selected_pregrasp_position_m,
            joint_positions_rad=selected.solution_rad,
            seed_source=selected.seed_source,
            selected_attempt_index=selected.selected_attempt_index,
            selected_candidate_index=selected.candidate_index,
            selected_offset_m=selected.offset_m,
            solution_type=selected.solution_type,
            position_error_m=selected.position_error_m,
            approach_error_deg=selected.approach_error_deg,
            final_position_m=selected.final_position_m,
            object_pose_age_s=object_pose_age_s,
            joint_state_seed_available=joint_state_seed_available,
            target_radius_xy_m=target_radius_xy,
            target_distance_3d_m=target_distance_3d,
            approx_max_reach_m=approx_reach,
            seeds_attempted=all_seeds,
            attempt_results=all_attempts,
            candidate_results=candidate_results,
            acceptance_tolerances=acceptance_tolerances,
            best_q_within_joint_limits=any(
                attempt.joint_limit_valid for attempt in all_attempts
            ),
        )

    final_failure_reason = "ik_failed_all_candidates"
    if any(attempt.final_reason == "joint_limit_failed" for attempt in all_attempts):
        final_failure_reason = "joint_limit_failed"
    elif any(attempt.final_reason == "fk_validation_failed" for attempt in all_attempts):
        final_failure_reason = "ik_failed_all_candidates"

    return PregraspPlan(
        False,
        final_failure_reason,
        object_position_m=object_position,
        requested_pregrasp_position_m=requested_pregrasp_position,
        pregrasp_position_m=requested_pregrasp_position,
        selected_candidate_index=None,
        selected_offset_m=None,
        object_pose_age_s=object_pose_age_s,
        joint_state_seed_available=joint_state_seed_available,
        target_radius_xy_m=target_radius_xy,
        target_distance_3d_m=target_distance_3d,
        approx_max_reach_m=approx_reach,
        seeds_attempted=all_seeds,
        attempt_results=all_attempts,
        candidate_results=candidate_results,
        acceptance_tolerances=acceptance_tolerances,
        best_q_within_joint_limits=any(attempt.joint_limit_valid for attempt in all_attempts),
    )
