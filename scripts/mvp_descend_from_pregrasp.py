from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
for package_path in (
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.ik import solve_ik
from so101_mvp_kinematics.joint_limits import clamp_to_limits, joints_within_limits
from so101_mvp_kinematics.model import So101KinematicModel
from so101_mvp_kinematics.transforms import normalize_vector, rotation_angle_error


ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
CONFIRM_PHRASE = "DESCEND_3CM"
REFERENCE_SEED_RAD = np.asarray([0.0, -0.35, 0.35, 1.22, 0.0], dtype=np.float64)
TOOL_APPROACH_AXIS_LOCAL = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
DESIRED_APPROACH_BASE = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
ACCEPTED_PREGRASP_STATUS = {
    "pregrasp_ready",
    "pregrasp_ready_exact",
    "pregrasp_ready_near",
    "pregrasp_ready_offset",
}


@dataclass(frozen=True)
class DescentConfig:
    waypoint_drop_m: tuple[float, ...] = (0.01, 0.02, 0.03)
    start_pregrasp_joint_tolerance_rad: float = 0.10
    max_abs_joint_delta_per_waypoint_rad: float = 0.25
    position_tolerance_m: float = 0.008
    approach_tolerance_deg: float = 5.0
    max_xy_error_from_waypoint_m: float = 0.010
    minimum_actual_z_drop_per_waypoint_m: float = 0.004
    minimum_total_actual_z_drop_m: float = 0.020
    final_joint_tolerance_rad: float = 0.035
    speed_rad_s: float = 0.06
    max_speed_rad_s: float = 0.08
    execute_service_timeout_s: float = 120.0
    inter_waypoint_hold_s: float = 0.5
    joint_state_max_age_s: float = 1.0
    pregrasp_target_max_age_s: float = 2.0


@dataclass(frozen=True)
class StampedJointState:
    names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    received_monotonic_s: float


@dataclass(frozen=True)
class StampedPose:
    frame_id: str
    xyz_m: tuple[float, float, float]
    received_monotonic_s: float


@dataclass(frozen=True)
class FrozenPregrasp:
    object_pose_base: list[float] | None
    pregrasp_pose_base: list[float]
    pregrasp_joint_target_rad: list[float]
    solution_type: str | None
    selected_offset_m: list[float] | None
    position_error_m: float | None
    approach_error_deg: float | None


@dataclass(frozen=True)
class WaypointPlan:
    index: int
    requested_xyz_m: list[float]
    actual_fk_xyz_m: list[float]
    selected_joint_target_rad: list[float]
    joint_delta_from_previous_rad: list[float]
    maximum_abs_joint_delta_rad: float
    position_error_m: float
    approach_error_deg: float
    xy_error_m: float
    actual_z_drop_from_previous_m: float
    joint_limits_valid: bool
    solution_type: str
    seed_source: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "requested_xyz_m": self.requested_xyz_m,
            "actual_fk_xyz_m": self.actual_fk_xyz_m,
            "selected_joint_target_rad": self.selected_joint_target_rad,
            "joint_delta_from_previous_rad": self.joint_delta_from_previous_rad,
            "maximum_abs_joint_delta_rad": self.maximum_abs_joint_delta_rad,
            "position_error_m": self.position_error_m,
            "approach_error_deg": self.approach_error_deg,
            "xy_error_m": self.xy_error_m,
            "actual_z_drop_from_previous_m": self.actual_z_drop_from_previous_m,
            "joint_limits_valid": self.joint_limits_valid,
            "solution_type": self.solution_type,
            "seed_source": self.seed_source,
        }


@dataclass(frozen=True)
class DescentPlan:
    success: bool
    reason: str
    frozen: FrozenPregrasp | None = None
    current_joint_positions_rad: list[float] | None = None
    start_pregrasp_joint_error_rad: list[float] | None = None
    start_pregrasp_max_error_rad: float | None = None
    pregrasp_fk_xyz_m: list[float] | None = None
    waypoints: list[WaypointPlan] = field(default_factory=list)
    total_requested_z_drop_m: float = 0.03
    total_actual_z_drop_m: float | None = None

    def to_summary(self, mode: str, config: DescentConfig, hardware_command_sent: bool) -> dict[str, Any]:
        return {
            "success": self.success,
            "reason": self.reason,
            "mode": mode,
            "object_pose_base": None if self.frozen is None else self.frozen.object_pose_base,
            "pregrasp_pose_base": None if self.frozen is None else self.frozen.pregrasp_pose_base,
            "current_joint_positions_rad": self.current_joint_positions_rad,
            "pregrasp_joint_target_rad": None
            if self.frozen is None
            else self.frozen.pregrasp_joint_target_rad,
            "start_pregrasp_joint_error_rad": self.start_pregrasp_joint_error_rad,
            "start_pregrasp_max_error_rad": self.start_pregrasp_max_error_rad,
            "start_pregrasp_tolerance_rad": config.start_pregrasp_joint_tolerance_rad,
            "waypoint_count": len(self.waypoints),
            "waypoints": [waypoint.to_dict() for waypoint in self.waypoints],
            "total_requested_z_drop_m": self.total_requested_z_drop_m,
            "total_actual_z_drop_m": self.total_actual_z_drop_m,
            "speed_rad_s": config.speed_rad_s,
            "estimated_total_motion_duration_s": estimated_total_duration_s(
                self.current_joint_positions_rad or [],
                [waypoint.selected_joint_target_rad for waypoint in self.waypoints],
                config.speed_rad_s,
            ),
            "all_waypoints_valid": bool(self.success and len(self.waypoints) == len(config.waypoint_drop_m)),
            "hardware_command_sent": bool(hardware_command_sent),
        }


def listf(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def optional_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_hardware_max_speed() -> float:
    path = PROJECT_ROOT / "config" / "mvp_hardware.json"
    if not path.is_file():
        return 0.08
    data = json.loads(path.read_text(encoding="utf-8"))
    return float(data.get("maximum_speed_rad_s", 0.08))


def load_config(path: Path | None = None) -> DescentConfig:
    config_path = path or PROJECT_ROOT / "config" / "mvp_descent.yaml"
    if not config_path.is_file():
        return DescentConfig(max_speed_rad_s=load_hardware_max_speed())
    values: dict[str, float | list[float]] = {}
    current_list_key: str | None = None
    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.endswith(":"):
            current_list_key = stripped[:-1].strip()
            values[current_list_key] = []
            continue
        if stripped.startswith("-") and current_list_key is not None:
            item = stripped[1:].strip()
            existing = values.get(current_list_key)
            if isinstance(existing, list):
                existing.append(float(item))
            continue
        current_list_key = None
        if ":" not in stripped:
            continue
        key, raw_value = stripped.split(":", 1)
        try:
            values[key.strip()] = float(raw_value.strip())
        except ValueError:
            continue
    drops = values.get("waypoint_drop_m", [0.01, 0.02, 0.03])
    return DescentConfig(
        waypoint_drop_m=tuple(float(value) for value in drops) if isinstance(drops, list) else (0.01, 0.02, 0.03),
        start_pregrasp_joint_tolerance_rad=float(values.get("start_pregrasp_joint_tolerance_rad", 0.10)),
        max_abs_joint_delta_per_waypoint_rad=float(values.get("max_abs_joint_delta_per_waypoint_rad", 0.25)),
        position_tolerance_m=float(values.get("position_tolerance_m", 0.008)),
        approach_tolerance_deg=float(values.get("approach_tolerance_deg", 5.0)),
        max_xy_error_from_waypoint_m=float(values.get("max_xy_error_from_waypoint_m", 0.010)),
        minimum_actual_z_drop_per_waypoint_m=float(values.get("minimum_actual_z_drop_per_waypoint_m", 0.004)),
        minimum_total_actual_z_drop_m=float(values.get("minimum_total_actual_z_drop_m", 0.020)),
        final_joint_tolerance_rad=float(values.get("final_joint_tolerance_rad", 0.035)),
        speed_rad_s=float(values.get("speed_rad_s", 0.06)),
        max_speed_rad_s=load_hardware_max_speed(),
        execute_service_timeout_s=float(values.get("execute_service_timeout_s", 120.0)),
        inter_waypoint_hold_s=float(values.get("inter_waypoint_hold_s", 0.5)),
        joint_state_max_age_s=float(values.get("joint_state_max_age_s", 1.0)),
        pregrasp_target_max_age_s=float(values.get("pregrasp_target_max_age_s", 2.0)),
    )


def create_model() -> So101KinematicModel:
    return So101KinematicModel(
        PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
    )


def validate_joint_contract(names: list[str] | tuple[str, ...], positions: list[float] | tuple[float, ...]) -> tuple[bool, str]:
    if tuple(names) != ARM_JOINT_NAMES:
        return False, "joint_name_order_invalid"
    if len(positions) != len(ARM_JOINT_NAMES):
        return False, "joint_position_count_invalid"
    try:
        values = [float(value) for value in positions]
    except (TypeError, ValueError):
        return False, "joint_position_non_finite"
    if not all(math.isfinite(value) for value in values):
        return False, "joint_position_non_finite"
    return True, "ok"


def validate_fresh_joint_state(
    state: StampedJointState | None,
    *,
    now_monotonic_s: float,
    max_age_s: float,
) -> tuple[bool, str]:
    if state is None:
        return False, "missing_joint_state"
    valid, reason = validate_joint_contract(state.names, state.positions_rad)
    if not valid:
        return False, reason
    if now_monotonic_s - state.received_monotonic_s > max_age_s:
        return False, "joint_state_stale"
    return True, "ok"


def status_is_accepted(status: str, compute_message: str) -> bool:
    return str(status) in ACCEPTED_PREGRASP_STATUS or "solution_type=accepted_near_solution" in str(compute_message)


def parse_compute_message(message: str) -> tuple[dict[str, Any], list[str]]:
    from mvp_move_to_pregrasp import parse_compute_message as parse_pregrasp_message

    return parse_pregrasp_message(message)


def build_waypoint_xyz(pregrasp_xyz_m: list[float], drops_m: tuple[float, ...]) -> list[list[float]]:
    x, y, z = [float(value) for value in pregrasp_xyz_m]
    return [[x, y, z - float(drop)] for drop in drops_m]


def joint_delta(previous_rad: list[float], target_rad: list[float]) -> tuple[list[float], float]:
    delta = [float(t) - float(p) for p, t in zip(previous_rad, target_rad, strict=True)]
    maximum = max(abs(value) for value in delta) if delta else math.inf
    return delta, float(maximum)


def final_joint_error(final_rad: list[float], target_rad: list[float], tolerance_rad: float) -> dict[str, Any]:
    errors = [abs(float(f) - float(t)) for f, t in zip(final_rad, target_rad, strict=True)]
    maximum = max(errors) if errors else math.inf
    return {
        "final_joint_error_rad": errors,
        "maximum_final_joint_error_rad": maximum,
        "final_target_reached": bool(maximum <= tolerance_rad),
    }


def start_pregrasp_error(current_rad: list[float], pregrasp_target_rad: list[float]) -> tuple[list[float], float]:
    errors = [abs(float(c) - float(t)) for c, t in zip(current_rad, pregrasp_target_rad, strict=True)]
    return errors, max(errors) if errors else math.inf


def estimated_total_duration_s(start_rad: list[float], waypoint_targets: list[list[float]], speed_rad_s: float) -> float:
    if speed_rad_s <= 0.0 or not start_rad:
        return math.inf
    total = 0.0
    previous = list(start_rad)
    for target in waypoint_targets:
        total += sum(abs(float(t) - float(p)) for p, t in zip(previous, target, strict=True))
        previous = list(target)
    return float(total / speed_rad_s)


def fk_metrics(
    *,
    model: So101KinematicModel,
    q_rad: np.ndarray,
    requested_xyz_m: np.ndarray,
    previous_actual_xyz_m: np.ndarray,
    fk_func: Callable[[So101KinematicModel, np.ndarray], dict[str, object]],
) -> tuple[float, float, float, float, np.ndarray]:
    fk = fk_func(model, q_rad)
    actual = np.asarray(fk["position_m"], dtype=np.float64)
    rotation = np.asarray(fk["rotation_matrix"], dtype=np.float64)
    current_approach = normalize_vector(rotation @ TOOL_APPROACH_AXIS_LOCAL, "current approach")
    position_error_m = float(np.linalg.norm(requested_xyz_m - actual))
    approach_error_deg = math.degrees(rotation_angle_error(current_approach, DESIRED_APPROACH_BASE))
    xy_error_m = float(np.linalg.norm(requested_xyz_m[:2] - actual[:2]))
    actual_z_drop = float(previous_actual_xyz_m[2] - actual[2])
    return position_error_m, approach_error_deg, xy_error_m, actual_z_drop, actual


def clean_seed(model: So101KinematicModel, q_rad: np.ndarray) -> np.ndarray | None:
    values = np.asarray(q_rad, dtype=np.float64)
    if values.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(values)):
        return None
    clamped = clamp_to_limits(model, values)
    return clamped if joints_within_limits(model, clamped) else None


def build_descent_seeds(
    model: So101KinematicModel,
    target_xyz_m: np.ndarray,
    first_seed_rad: np.ndarray,
    first_source: str,
) -> list[tuple[str, np.ndarray]]:
    seeds: list[tuple[str, np.ndarray]] = []
    seen: set[tuple[float, ...]] = set()

    def add(source: str, q_rad: np.ndarray) -> None:
        clean = clean_seed(model, q_rad)
        if clean is None:
            return
        key = tuple(round(float(value), 12) for value in clean.tolist())
        if key in seen:
            return
        seen.add(key)
        seeds.append((source, clean))

    add(first_source, first_seed_rad)
    base_yaw = math.atan2(float(target_xyz_m[1]), float(target_xyz_m[0]))
    yaw_seed = REFERENCE_SEED_RAD.copy()
    yaw_seed[0] = base_yaw
    add("target_yaw", yaw_seed)
    add("reference", REFERENCE_SEED_RAD)
    plus = yaw_seed.copy()
    plus[0] = base_yaw + math.radians(15.0)
    add("target_yaw_plus_15deg", plus)
    minus = yaw_seed.copy()
    minus[0] = base_yaw - math.radians(15.0)
    add("target_yaw_minus_15deg", minus)
    add("elbow_high", np.asarray([base_yaw, -0.55, 0.55, 1.10, 0.0], dtype=np.float64))
    add("elbow_low", np.asarray([base_yaw, -0.20, 0.20, 1.35, 0.0], dtype=np.float64))
    return seeds[:7]


def validate_waypoint_candidate(
    *,
    model: So101KinematicModel,
    q_rad: np.ndarray,
    requested_xyz_m: np.ndarray,
    previous_joint_rad: list[float],
    previous_actual_xyz_m: np.ndarray,
    config: DescentConfig,
    fk_func: Callable[[So101KinematicModel, np.ndarray], dict[str, object]],
    joint_limits_checker: Callable[[So101KinematicModel, np.ndarray], bool],
) -> tuple[bool, str, dict[str, Any]]:
    q = np.asarray(q_rad, dtype=np.float64)
    if q.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(q)):
        return False, "non_finite_joint_solution", {}
    joint_limits_valid = bool(joint_limits_checker(model, q))
    if not joint_limits_valid:
        return False, "joint_limit_failed", {"joint_limits_valid": False}
    delta, max_delta = joint_delta(previous_joint_rad, listf(q))
    if max_delta > config.max_abs_joint_delta_per_waypoint_rad:
        return False, "descent_joint_delta_exceeded", {"maximum_abs_joint_delta_rad": max_delta}
    try:
        position_error, approach_error, xy_error, z_drop, actual = fk_metrics(
            model=model,
            q_rad=q,
            requested_xyz_m=requested_xyz_m,
            previous_actual_xyz_m=previous_actual_xyz_m,
            fk_func=fk_func,
        )
    except (ValueError, np.linalg.LinAlgError, KeyError):
        return False, "fk_validation_failed", {}
    details = {
        "joint_delta_from_previous_rad": delta,
        "maximum_abs_joint_delta_rad": max_delta,
        "position_error_m": position_error,
        "approach_error_deg": approach_error,
        "xy_error_m": xy_error,
        "actual_z_drop_from_previous_m": z_drop,
        "actual_fk_xyz_m": listf(actual),
        "joint_limits_valid": joint_limits_valid,
    }
    if xy_error > config.max_xy_error_from_waypoint_m:
        return False, "descent_xy_error_exceeded", details
    if position_error > config.position_tolerance_m:
        return False, "fk_position_validation_failed", details
    if approach_error > config.approach_tolerance_deg:
        return False, "fk_approach_validation_failed", details
    if z_drop < config.minimum_actual_z_drop_per_waypoint_m:
        return False, "non_monotonic_descent", details
    return True, "ok", details


def plan_segmented_descent(
    *,
    model: So101KinematicModel,
    frozen: FrozenPregrasp,
    current_joint_positions_rad: list[float],
    config: DescentConfig,
    ik_solver: Callable[..., dict[str, object]] = solve_ik,
    fk_func: Callable[[So101KinematicModel, np.ndarray], dict[str, object]] = forward_kinematics,
    joint_limits_checker: Callable[[So101KinematicModel, np.ndarray], bool] = joints_within_limits,
) -> DescentPlan:
    start_errors, start_max_error = start_pregrasp_error(
        current_joint_positions_rad,
        frozen.pregrasp_joint_target_rad,
    )
    pregrasp_target = np.asarray(frozen.pregrasp_joint_target_rad, dtype=np.float64)
    try:
        pregrasp_fk = np.asarray(fk_func(model, pregrasp_target)["position_m"], dtype=np.float64)
    except (ValueError, np.linalg.LinAlgError, KeyError):
        pregrasp_fk = np.asarray(frozen.pregrasp_pose_base, dtype=np.float64)
    base_plan_args = {
        "frozen": frozen,
        "current_joint_positions_rad": current_joint_positions_rad,
        "start_pregrasp_joint_error_rad": start_errors,
        "start_pregrasp_max_error_rad": start_max_error,
        "pregrasp_fk_xyz_m": listf(pregrasp_fk),
    }
    if start_max_error > config.start_pregrasp_joint_tolerance_rad:
        return DescentPlan(False, "not_at_pregrasp", **base_plan_args)

    waypoints_xyz = build_waypoint_xyz(frozen.pregrasp_pose_base, config.waypoint_drop_m)
    if len(waypoints_xyz) != 3:
        return DescentPlan(False, "invalid_waypoint_count", **base_plan_args)

    previous_joint = list(current_joint_positions_rad)
    previous_actual = pregrasp_fk
    first_seed = np.asarray(current_joint_positions_rad, dtype=np.float64)
    planned: list[WaypointPlan] = []
    failure_reason = "ik_failed"
    for index, requested in enumerate(waypoints_xyz, start=1):
        requested_xyz = np.asarray(requested, dtype=np.float64)
        seed_source = "current_joint_state" if index == 1 else f"previous_waypoint_{index - 1}"
        seeds = build_descent_seeds(model, requested_xyz, first_seed, seed_source)
        waypoint: WaypointPlan | None = None
        for source, seed in seeds:
            result = ik_solver(
                model,
                requested_xyz,
                seed,
                DESIRED_APPROACH_BASE,
                TOOL_APPROACH_AXIS_LOCAL,
                position_tolerance_m=config.position_tolerance_m,
                approach_tolerance_deg=config.approach_tolerance_deg,
            )
            raw_q = result.get("joint_positions_rad")
            if raw_q is None:
                failure_reason = str(result.get("reason", "ik_failed"))
                continue
            q = np.asarray(raw_q, dtype=np.float64)
            valid, reason, details = validate_waypoint_candidate(
                model=model,
                q_rad=q,
                requested_xyz_m=requested_xyz,
                previous_joint_rad=previous_joint,
                previous_actual_xyz_m=previous_actual,
                config=config,
                fk_func=fk_func,
                joint_limits_checker=joint_limits_checker,
            )
            if not valid:
                failure_reason = reason
                continue
            waypoint = WaypointPlan(
                index=index,
                requested_xyz_m=listf(requested_xyz),
                actual_fk_xyz_m=details["actual_fk_xyz_m"],
                selected_joint_target_rad=listf(q),
                joint_delta_from_previous_rad=details["joint_delta_from_previous_rad"],
                maximum_abs_joint_delta_rad=float(details["maximum_abs_joint_delta_rad"]),
                position_error_m=float(details["position_error_m"]),
                approach_error_deg=float(details["approach_error_deg"]),
                xy_error_m=float(details["xy_error_m"]),
                actual_z_drop_from_previous_m=float(details["actual_z_drop_from_previous_m"]),
                joint_limits_valid=bool(details["joint_limits_valid"]),
                solution_type="exact_solution" if bool(result.get("success")) else "accepted_near_solution",
                seed_source=source,
            )
            break
        if waypoint is None:
            return DescentPlan(False, failure_reason, waypoints=planned, **base_plan_args)
        planned.append(waypoint)
        previous_joint = waypoint.selected_joint_target_rad
        previous_actual = np.asarray(waypoint.actual_fk_xyz_m, dtype=np.float64)
        first_seed = np.asarray(waypoint.selected_joint_target_rad, dtype=np.float64)

    total_actual_drop = float(pregrasp_fk[2] - np.asarray(planned[-1].actual_fk_xyz_m, dtype=np.float64)[2])
    if total_actual_drop < config.minimum_total_actual_z_drop_m:
        return DescentPlan(
            False,
            "total_z_drop_too_small",
            waypoints=planned,
            total_actual_z_drop_m=total_actual_drop,
            **base_plan_args,
        )
    return DescentPlan(
        True,
        "segmented_descent_ready",
        waypoints=planned,
        total_actual_z_drop_m=total_actual_drop,
        **base_plan_args,
    )


def make_frozen_pregrasp(
    *,
    object_pose_base: list[float] | None,
    pregrasp_pose_base: list[float],
    pregrasp_joint_target_rad: list[float],
    compute_message: str,
) -> FrozenPregrasp:
    fields, _ = parse_compute_message(compute_message)
    return FrozenPregrasp(
        object_pose_base=object_pose_base,
        pregrasp_pose_base=pregrasp_pose_base,
        pregrasp_joint_target_rad=pregrasp_joint_target_rad,
        solution_type=fields.get("solution_type"),
        selected_offset_m=fields.get("offset_m"),
        position_error_m=optional_float(fields.get("position_error_m")),
        approach_error_deg=optional_float(fields.get("approach_error_deg")),
    )


def pose_to_xyz(msg: Any) -> list[float]:
    return [
        float(msg.pose.position.x),
        float(msg.pose.position.y),
        float(msg.pose.position.z),
    ]


def execute_preconditions(*, execute: bool, confirm: str) -> tuple[bool, str]:
    if not execute:
        return True, "ok"
    if confirm != CONFIRM_PHRASE:
        return False, "wrong_confirmation"
    return True, "ok"


def json_print(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False))


class DescentNode:
    def __init__(self, config: DescentConfig) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.JointState = JointState
        self.Trigger = Trigger
        self.node = _Node("mvp_descend_from_pregrasp")
        self.config = config
        self.latest_joint_state: StampedJointState | None = None
        self.latest_pregrasp_target: StampedJointState | None = None
        self.latest_pregrasp_pose: StampedPose | None = None
        self.latest_object_pose: StampedPose | None = None
        self.pregrasp_valid = False
        self.pregrasp_status = ""
        self.tcp_connected = False
        self.tcp_status = "unknown"

        self.node.create_subscription(JointState, "/mvp/joint_states", self._joint_state_cb, 10)
        self.node.create_subscription(JointState, "/mvp/pregrasp_joint_target", self._pregrasp_target_cb, 10)
        self.node.create_subscription(PoseStamped, "/mvp/pregrasp_pose", self._pregrasp_pose_cb, 10)
        self.node.create_subscription(PoseStamped, "/object_pose_base", self._object_pose_cb, 10)
        self.node.create_subscription(Bool, "/mvp/pregrasp_valid", self._pregrasp_valid_cb, 10)
        self.node.create_subscription(String, "/mvp/pregrasp_status", self._pregrasp_status_cb, 10)
        self.node.create_subscription(Bool, "/mvp/tcp_connected", self._tcp_connected_cb, 10)
        self.node.create_subscription(String, "/mvp/tcp_status", self._tcp_status_cb, 10)
        self.target_pub = self.node.create_publisher(JointState, "/mvp/joint_target", 10)
        self.compute_client = self.node.create_client(Trigger, "/mvp/compute_pregrasp")
        self.execute_client = self.node.create_client(Trigger, "/mvp/execute_target")

    def _joint_state_cb(self, msg: Any) -> None:
        self.latest_joint_state = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )

    def _pregrasp_target_cb(self, msg: Any) -> None:
        self.latest_pregrasp_target = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )

    def _pregrasp_pose_cb(self, msg: Any) -> None:
        self.latest_pregrasp_pose = StampedPose(str(msg.header.frame_id), tuple(pose_to_xyz(msg)), time.monotonic())

    def _object_pose_cb(self, msg: Any) -> None:
        self.latest_object_pose = StampedPose(str(msg.header.frame_id), tuple(pose_to_xyz(msg)), time.monotonic())

    def _pregrasp_valid_cb(self, msg: Any) -> None:
        self.pregrasp_valid = bool(msg.data)

    def _pregrasp_status_cb(self, msg: Any) -> None:
        self.pregrasp_status = str(msg.data)

    def _tcp_connected_cb(self, msg: Any) -> None:
        self.tcp_connected = bool(msg.data)

    def _tcp_status_cb(self, msg: Any) -> None:
        self.tcp_status = str(msg.data)

    def spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call_trigger(self, client: Any, timeout_s: float) -> tuple[bool, str, bool]:
        if not client.wait_for_service(timeout_sec=3.0):
            return False, "service_unavailable", False
        future = client.call_async(self.Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, "execute_service_timeout" if timeout_s >= 100.0 else "service_timeout", True
        response = future.result()
        return bool(response.success), str(response.message), True

    def publish_target_once(self, target_rad: list[float]) -> None:
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(value) for value in target_rad]
        self.target_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def destroy(self) -> None:
        self.node.destroy_node()


def run(args: argparse.Namespace) -> int:
    allowed, reason = execute_preconditions(execute=bool(args.execute), confirm=str(args.confirm))
    if not allowed:
        json_print({"success": False, "reason": reason, "required_confirm": CONFIRM_PHRASE})
        return 2

    import rclpy

    config = load_config()
    model = create_model()
    rclpy.init()
    node = DescentNode(config)
    try:
        if not node.spin_until(lambda: node.tcp_connected and node.tcp_status == "connected", 5.0):
            json_print({"success": False, "reason": "tcp_not_connected"})
            return 3
        if not node.spin_until(
            lambda: validate_fresh_joint_state(
                node.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=config.joint_state_max_age_s,
            )[0],
            10.0,
        ):
            json_print({"success": False, "reason": "joint_state_unavailable_or_stale"})
            return 4
        assert node.latest_joint_state is not None
        current = [float(value) for value in node.latest_joint_state.positions_rad]
        if (
            node.latest_object_pose is None
            or time.monotonic() - node.latest_object_pose.received_monotonic_s
            > config.pregrasp_target_max_age_s
        ):
            json_print({"success": False, "reason": "object_pose_unavailable_or_stale"})
            return 5

        compute_started = time.monotonic()
        compute_success, compute_message, _ = node.call_trigger(node.compute_client, 10.0)
        if not compute_success:
            json_print({"success": False, "reason": "compute_pregrasp_failed", "message": compute_message})
            return 6
        if not node.spin_until(
            lambda: node.latest_pregrasp_target is not None
            and node.latest_pregrasp_target.received_monotonic_s >= compute_started
            and node.latest_pregrasp_pose is not None
            and node.latest_pregrasp_pose.received_monotonic_s >= compute_started,
            config.pregrasp_target_max_age_s,
        ):
            json_print({"success": False, "reason": "pregrasp_target_unavailable_or_stale"})
            return 7
        assert node.latest_pregrasp_target is not None
        assert node.latest_pregrasp_pose is not None
        valid_target, target_reason = validate_joint_contract(
            node.latest_pregrasp_target.names,
            node.latest_pregrasp_target.positions_rad,
        )
        if not valid_target:
            json_print({"success": False, "reason": target_reason})
            return 8
        if not node.pregrasp_valid:
            json_print({"success": False, "reason": "pregrasp_invalid"})
            return 9
        if not status_is_accepted(node.pregrasp_status, compute_message):
            json_print({"success": False, "reason": "pregrasp_status_not_ready", "status": node.pregrasp_status})
            return 10

        frozen = make_frozen_pregrasp(
            object_pose_base=None if node.latest_object_pose is None else list(node.latest_object_pose.xyz_m),
            pregrasp_pose_base=list(node.latest_pregrasp_pose.xyz_m),
            pregrasp_joint_target_rad=[float(value) for value in node.latest_pregrasp_target.positions_rad],
            compute_message=compute_message,
        )
        plan = plan_segmented_descent(
            model=model,
            frozen=frozen,
            current_joint_positions_rad=current,
            config=config,
        )
        summary = plan.to_summary("execute" if args.execute else "plan_only", config, False)
        summary["compute_response_message"] = compute_message
        summary["tcp_connected"] = node.tcp_connected
        summary["tcp_status"] = node.tcp_status
        if not plan.success:
            json_print(summary)
            return 11
        if not args.execute:
            json_print(summary)
            return 0

        publish_count = 0
        execute_count = 0
        completed = 0
        final_positions: list[float] | None = None
        final_errors: dict[str, Any] | None = None
        for waypoint in plan.waypoints:
            node.publish_target_once(waypoint.selected_joint_target_rad)
            publish_count += 1
            execute_success, execute_message, _ = node.call_trigger(
                node.execute_client,
                config.execute_service_timeout_s,
            )
            execute_count += 1
            if not execute_success:
                summary.update(
                    {
                        "success": False,
                        "reason": f"motion_result_unknown: {execute_message}"
                        if "timeout" in execute_message
                        else execute_message,
                        "hardware_command_sent": True,
                        "completed_waypoint_count": completed,
                        "joint_target_publish_count": publish_count,
                        "execute_call_count": execute_count,
                    }
                )
                json_print(summary)
                return 12
            if not node.spin_until(
                lambda: validate_fresh_joint_state(
                    node.latest_joint_state,
                    now_monotonic_s=time.monotonic(),
                    max_age_s=config.joint_state_max_age_s,
                )[0],
                3.0,
            ):
                summary.update(
                    {
                        "success": False,
                        "reason": "joint_state_unavailable_after_waypoint",
                        "hardware_command_sent": True,
                        "completed_waypoint_count": completed,
                        "joint_target_publish_count": publish_count,
                        "execute_call_count": execute_count,
                    }
                )
                json_print(summary)
                return 13
            assert node.latest_joint_state is not None
            final_positions = [float(value) for value in node.latest_joint_state.positions_rad]
            final_errors = final_joint_error(
                final_positions,
                waypoint.selected_joint_target_rad,
                config.final_joint_tolerance_rad,
            )
            if not final_errors["final_target_reached"]:
                summary.update(
                    {
                        "success": False,
                        "reason": "final_joint_tolerance_failed",
                        "hardware_command_sent": True,
                        "completed_waypoint_count": completed,
                        "joint_target_publish_count": publish_count,
                        "execute_call_count": execute_count,
                        **final_errors,
                    }
                )
                json_print(summary)
                return 14
            completed += 1
            time.sleep(config.inter_waypoint_hold_s)

        assert final_positions is not None
        assert final_errors is not None
        final_fk = forward_kinematics(model, np.asarray(final_positions, dtype=np.float64))
        final_fk_xyz = np.asarray(final_fk["position_m"], dtype=np.float64)
        pregrasp_fk_xyz = np.asarray(plan.pregrasp_fk_xyz_m, dtype=np.float64)
        final_rotation = np.asarray(final_fk["rotation_matrix"], dtype=np.float64)
        final_approach = normalize_vector(final_rotation @ TOOL_APPROACH_AXIS_LOCAL, "final approach")
        final_approach_error_deg = math.degrees(rotation_angle_error(final_approach, DESIRED_APPROACH_BASE))
        final_xy_shift = float(np.linalg.norm(final_fk_xyz[:2] - pregrasp_fk_xyz[:2]))
        total_actual_z_drop = float(pregrasp_fk_xyz[2] - final_fk_xyz[2])
        final_ok = bool(
            final_errors["maximum_final_joint_error_rad"] <= config.final_joint_tolerance_rad
            and total_actual_z_drop >= config.minimum_total_actual_z_drop_m
            and final_xy_shift <= 0.015
            and final_approach_error_deg <= config.approach_tolerance_deg
            and node.tcp_connected
        )
        summary.update(
            {
                "success": final_ok,
                "reason": "ok" if final_ok else "final_descent_validation_failed",
                "hardware_command_sent": True,
                "completed_waypoint_count": completed,
                "joint_target_publish_count": publish_count,
                "execute_call_count": execute_count,
                "final_joint_positions_rad": final_positions,
                **final_errors,
                "final_fk_xyz_m": listf(final_fk_xyz),
                "pregrasp_fk_xyz_m": listf(pregrasp_fk_xyz),
                "total_actual_z_drop_m": total_actual_z_drop,
                "final_xy_shift_from_pregrasp_m": final_xy_shift,
                "final_approach_error_deg": final_approach_error_deg,
                "tcp_connected_after_motion": node.tcp_connected,
                "tcp_status_after_motion": node.tcp_status,
            }
        )
        json_print(summary)
        return 0 if final_ok else 15
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plan or execute a frozen three-segment MVP-4C descent from pregrasp.")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.execute and args.plan_only:
        json_print({"success": False, "reason": "plan_only_and_execute_are_mutually_exclusive"})
        return 2
    if not args.execute:
        args.plan_only = True
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
