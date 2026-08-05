from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.ik import solve_ik
from so101_mvp_kinematics.joint_limits import joints_within_limits
from so101_mvp_kinematics.model import JOINT_NAMES, So101KinematicModel
from so101_mvp_kinematics.transforms import normalize_vector, rotation_angle_error


ARM_JOINT_NAMES = tuple(JOINT_NAMES)
REFERENCE_SEED_RAD = np.asarray([0.0, -0.35, 0.35, 1.22, 0.0], dtype=np.float64)
TOOL_APPROACH_AXIS_LOCAL = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
DESIRED_APPROACH_BASE = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
DEFAULT_POSITION_TOLERANCE_M = 0.002
DEFAULT_APPROACH_TOLERANCE_DEG = 5.0


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
class PregraspPlan:
    success: bool
    reason: str
    pregrasp_position_m: np.ndarray | None = None
    joint_positions_rad: np.ndarray | None = None
    seed_source: str | None = None
    position_error_m: float | None = None
    approach_error_deg: float | None = None
    final_position_m: np.ndarray | None = None


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


def joint_state_seed_or_reference(
    joint_state: JointStateSnapshot | None,
    *,
    now_monotonic_s: float,
    max_age_s: float,
) -> tuple[np.ndarray, str]:
    if joint_state is None:
        return REFERENCE_SEED_RAD.copy(), "reference"

    if now_monotonic_s - joint_state.received_monotonic_s > max_age_s:
        return REFERENCE_SEED_RAD.copy(), "reference"

    by_name = {
        name: float(position)
        for name, position in zip(
            joint_state.names,
            joint_state.positions_rad.tolist(),
            strict=False,
        )
    }
    try:
        seed = np.asarray([by_name[name] for name in ARM_JOINT_NAMES], dtype=np.float64)
    except KeyError:
        return REFERENCE_SEED_RAD.copy(), "reference"

    if seed.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(seed)):
        return REFERENCE_SEED_RAD.copy(), "reference"

    return seed, "joint_state"


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
    if object_pose is None:
        return PregraspPlan(False, "no_object_pose")

    if now_monotonic_s - object_pose.received_monotonic_s > max_object_pose_age_s:
        return PregraspPlan(False, "object_pose_stale")

    if object_pose.frame_id != base_frame:
        return PregraspPlan(False, "invalid_frame")

    object_position = np.asarray(object_pose.position_m, dtype=np.float64)
    if not finite_position(object_position):
        return PregraspPlan(False, "non_finite_object_pose")

    pregrasp_position = object_position + np.asarray(
        [0.0, 0.0, float(pregrasp_height_m)],
        dtype=np.float64,
    )
    if not finite_position(pregrasp_position):
        return PregraspPlan(False, "non_finite_object_pose")

    seed, seed_source = (
        joint_state_seed_or_reference(
            joint_state,
            now_monotonic_s=now_monotonic_s,
            max_age_s=max_joint_state_age_s,
        )
        if use_joint_state_seed
        else (REFERENCE_SEED_RAD.copy(), "reference")
    )

    ik_result = ik_solver(
        model,
        pregrasp_position,
        seed,
        DESIRED_APPROACH_BASE,
        TOOL_APPROACH_AXIS_LOCAL,
        position_tolerance_m=position_tolerance_m,
        approach_tolerance_deg=approach_tolerance_deg,
    )
    if not bool(ik_result.get("success")):
        return PregraspPlan(
            False,
            "ik_failed",
            pregrasp_position_m=pregrasp_position,
            seed_source=seed_source,
            position_error_m=_optional_float(ik_result.get("position_error_m")),
            approach_error_deg=_optional_float(ik_result.get("approach_error_deg")),
        )

    q_rad = np.asarray(ik_result.get("joint_positions_rad"), dtype=np.float64)
    if q_rad.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(q_rad)):
        return PregraspPlan(False, "ik_failed", pregrasp_position_m=pregrasp_position)

    if not joints_within_limits(model, q_rad):
        return PregraspPlan(
            False,
            "joint_limit_failed",
            pregrasp_position_m=pregrasp_position,
            joint_positions_rad=q_rad,
            seed_source=seed_source,
        )

    try:
        fk_ok, position_error_m, approach_error_deg, final_position = validate_fk_result(
            model,
            q_rad,
            pregrasp_position,
            position_tolerance_m=position_tolerance_m,
            approach_tolerance_deg=approach_tolerance_deg,
        )
    except (ValueError, np.linalg.LinAlgError):
        return PregraspPlan(
            False,
            "fk_validation_failed",
            pregrasp_position_m=pregrasp_position,
            joint_positions_rad=q_rad,
            seed_source=seed_source,
        )

    if not fk_ok:
        return PregraspPlan(
            False,
            "fk_validation_failed",
            pregrasp_position_m=pregrasp_position,
            joint_positions_rad=q_rad,
            seed_source=seed_source,
            position_error_m=position_error_m,
            approach_error_deg=approach_error_deg,
            final_position_m=final_position,
        )

    return PregraspPlan(
        True,
        "pregrasp_ready",
        pregrasp_position_m=pregrasp_position,
        joint_positions_rad=q_rad,
        seed_source=seed_source,
        position_error_m=position_error_m,
        approach_error_deg=approach_error_deg,
        final_position_m=final_position,
    )


def _optional_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number
