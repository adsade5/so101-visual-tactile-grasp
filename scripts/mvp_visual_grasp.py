from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
HARDWARE_CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_hardware.json"
GRASP_CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_grasp.yaml"
INTEGRATED_SNAPSHOT_PATH = PROJECT_ROOT / "data" / "runtime" / "mvp_last_integrated_grasp_snapshot.json"
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
for package_path in (
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_descend_from_pregrasp import (  # noqa: E402
    ARM_JOINT_NAMES,
    DESIRED_APPROACH_BASE,
    DescentConfig,
    FrozenPregrasp,
    TOOL_APPROACH_AXIS_LOCAL,
    build_descent_seeds,
    create_model,
    estimated_total_duration_s,
    forward_kinematics,
    joints_within_limits,
    listf,
    plan_segmented_descent,
    solve_ik,
    validate_fresh_joint_state,
)
from so101_mvp_kinematics.transforms import normalize_vector, rotation_angle_error  # noqa: E402
from mvp_move_to_pregrasp import (  # noqa: E402
    MoveConfig,
    SNAPSHOT_PATH,
    atomic_write_json,
    estimated_duration_s,
    final_joint_error,
    joint_delta,
    make_pregrasp_snapshot,
    status_is_accepted,
    target_within_urdf_limits,
    validate_joint_contract,
)


CONFIRM_PHRASE = "VISUAL_GRASP"
GRIPPER_LOGICAL_KEY = "gripper.pos"
GRIPPER_LOGICAL_NAME = "gripper"
WRIST_ROLL_LOGICAL_KEY = "wrist_roll.pos"


@dataclass(frozen=True)
class GraspConfig:
    total_descent_m: float = 0.07
    descent_waypoint_drop_m: tuple[float, ...] = (0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07)
    pregrasp_max_abs_joint_delta_rad: float = 1.00
    max_abs_joint_delta_per_descent_waypoint_rad: float = 0.25
    position_tolerance_m: float = 0.008
    approach_tolerance_deg: float = 5.0
    max_xy_error_from_waypoint_m: float = 0.010
    minimum_actual_z_drop_per_waypoint_m: float = 0.004
    minimum_total_actual_z_drop_m: float = 0.050
    arm_final_joint_tolerance_rad: float = 0.035
    speed_rad_s: float = 0.06
    max_speed_rad_s: float = 0.08
    execute_service_timeout_s: float = 120.0
    tcp_ready_timeout_s: float = 8.0
    tcp_status_max_age_s: float = 1.0
    inter_waypoint_hold_s: float = 0.3
    joint_state_max_age_s: float = 1.0
    object_pose_max_age_s: float = 2.0
    pregrasp_target_max_age_s: float = 2.0
    gripper_state_max_age_s: float = 1.0
    gripper_target_max_age_s: float = 2.0
    gripper_open_delta: float = 10.0
    gripper_close_target_source: str = "safe_close_limit"
    gripper_close_hold_s: float = 1.0
    gripper_interpolation_enabled: bool = True
    gripper_only_motion_duration_s: float = 2.0
    gripper_close_step: float = 2.0
    gripper_safe_close_limit: float = 5.0
    gripper_close_incremental: bool = True
    gripper_close_timeout_s: float = 30.0
    gripper_close_stall_threshold: float = 0.3
    gripper_close_stall_steps: int = 3
    gripper_open_ramp_fraction: tuple[float, ...] = (0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.00)
    tactile_stop_enabled: bool = True
    tactile_state_max_age_s: float = 0.25
    tactile_require_clear_before_grasp: bool = True
    tactile_static_test_timeout_s: float = 30.0
    lift_enabled: bool = True
    lift_total_m: float = 0.03
    lift_waypoint_rise_m: tuple[float, ...] = (0.01, 0.02, 0.03)
    max_abs_joint_delta_per_lift_waypoint_rad: float = 0.25
    lift_position_tolerance_m: float = 0.008
    lift_approach_tolerance_deg: float = 5.0
    lift_max_xy_error_m: float = 0.010
    minimum_actual_lift_per_waypoint_m: float = 0.004
    minimum_total_actual_lift_m: float = 0.020
    lift_speed_rad_s: float = 0.06
    inter_lift_waypoint_hold_s: float = 0.3


@dataclass(frozen=True)
class StampedGripperState:
    position: float
    received_monotonic_s: float


@dataclass(frozen=True)
class StampedTactileState:
    ready: bool
    contact_detected: bool
    contact_score: float
    status: str
    received_monotonic_s: float
    source: str = ""
    port: str = ""
    state_age_s: float | None = None
    error: str | None = None
    frame_count: int = 0


def log_event(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def parse_yaml_scalar(value: str) -> float | bool | None | str:
    text = value.strip()
    if text.lower() == "null":
        return None
    if text.lower() == "true":
        return True
    if text.lower() == "false":
        return False
    try:
        return float(text)
    except ValueError:
        return text


def load_grasp_config(path: Path | None = None) -> GraspConfig:
    config_path = path or GRASP_CONFIG_PATH
    if not config_path.is_file():
        return GraspConfig()
    values: dict[str, Any] = {}
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
            item = parse_yaml_scalar(stripped[1:].strip())
            values[current_list_key].append(item)
            continue
        current_list_key = None
        if ":" not in stripped:
            continue
        key, raw_value = stripped.split(":", 1)
        values[key.strip()] = parse_yaml_scalar(raw_value)
    return GraspConfig(
        total_descent_m=float(values.get("total_descent_m", 0.07)),
        descent_waypoint_drop_m=tuple(float(v) for v in values.get("descent_waypoint_drop_m", [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07])),
        pregrasp_max_abs_joint_delta_rad=float(values.get("pregrasp_max_abs_joint_delta_rad", 1.00)),
        max_abs_joint_delta_per_descent_waypoint_rad=float(values.get("max_abs_joint_delta_per_descent_waypoint_rad", 0.25)),
        position_tolerance_m=float(values.get("position_tolerance_m", 0.008)),
        approach_tolerance_deg=float(values.get("approach_tolerance_deg", 5.0)),
        max_xy_error_from_waypoint_m=float(values.get("max_xy_error_from_waypoint_m", 0.010)),
        minimum_actual_z_drop_per_waypoint_m=float(values.get("minimum_actual_z_drop_per_waypoint_m", 0.004)),
        minimum_total_actual_z_drop_m=float(values.get("minimum_total_actual_z_drop_m", 0.050)),
        arm_final_joint_tolerance_rad=float(values.get("arm_final_joint_tolerance_rad", 0.035)),
        speed_rad_s=float(values.get("speed_rad_s", 0.06)),
        max_speed_rad_s=float(values.get("max_speed_rad_s", 0.08)),
        execute_service_timeout_s=float(values.get("execute_service_timeout_s", 120.0)),
        tcp_ready_timeout_s=float(values.get("tcp_ready_timeout_s", 8.0)),
        tcp_status_max_age_s=float(values.get("tcp_status_max_age_s", 1.0)),
        inter_waypoint_hold_s=float(values.get("inter_waypoint_hold_s", 0.3)),
        joint_state_max_age_s=float(values.get("joint_state_max_age_s", 1.0)),
        object_pose_max_age_s=float(values.get("object_pose_max_age_s", 2.0)),
        pregrasp_target_max_age_s=float(values.get("pregrasp_target_max_age_s", 2.0)),
        gripper_state_max_age_s=float(values.get("gripper_state_max_age_s", 1.0)),
        gripper_target_max_age_s=float(values.get("gripper_target_max_age_s", 2.0)),
        gripper_open_delta=float(values.get("gripper_open_delta", 10.0)),
        gripper_close_target_source=str(values.get("gripper_close_target_source", "safe_close_limit")),
        gripper_close_hold_s=float(values.get("gripper_close_hold_s", 1.0)),
        gripper_interpolation_enabled=bool(values.get("gripper_interpolation_enabled", True)),
        gripper_only_motion_duration_s=float(values.get("gripper_only_motion_duration_s", 2.0)),
        gripper_close_step=float(values.get("gripper_close_step", 2.0)),
        gripper_safe_close_limit=float(values.get("gripper_safe_close_limit", 5.0)),
        gripper_close_incremental=bool(values.get("gripper_close_incremental", True)),
        gripper_close_timeout_s=float(values.get("gripper_close_timeout_s", 30.0)),
        gripper_close_stall_threshold=float(values.get("gripper_close_stall_threshold", 0.3)),
        gripper_close_stall_steps=int(values.get("gripper_close_stall_steps", 3)),
        gripper_open_ramp_fraction=tuple(float(v) for v in values.get("gripper_open_ramp_fraction", [0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.00])),
        tactile_stop_enabled=bool(values.get("tactile_stop_enabled", True)),
        tactile_state_max_age_s=float(values.get("tactile_state_max_age_s", 0.25)),
        tactile_require_clear_before_grasp=bool(values.get("tactile_require_clear_before_grasp", True)),
        tactile_static_test_timeout_s=float(values.get("tactile_static_test_timeout_s", 30.0)),
        lift_enabled=bool(values.get("lift_enabled", True)),
        lift_total_m=float(values.get("lift_total_m", 0.03)),
        lift_waypoint_rise_m=tuple(float(v) for v in values.get("lift_waypoint_rise_m", [0.01, 0.02, 0.03])),
        max_abs_joint_delta_per_lift_waypoint_rad=float(values.get("max_abs_joint_delta_per_lift_waypoint_rad", 0.25)),
        lift_position_tolerance_m=float(values.get("lift_position_tolerance_m", 0.008)),
        lift_approach_tolerance_deg=float(values.get("lift_approach_tolerance_deg", 5.0)),
        lift_max_xy_error_m=float(values.get("lift_max_xy_error_m", 0.010)),
        minimum_actual_lift_per_waypoint_m=float(values.get("minimum_actual_lift_per_waypoint_m", 0.004)),
        minimum_total_actual_lift_m=float(values.get("minimum_total_actual_lift_m", 0.020)),
        lift_speed_rad_s=float(values.get("lift_speed_rad_s", 0.06)),
        inter_lift_waypoint_hold_s=float(values.get("inter_lift_waypoint_hold_s", 0.3)),
    )


def load_motor_mapping(
    hardware_config_path: Path = HARDWARE_CONFIG_PATH,
) -> dict[str, Any]:
    hardware = json.loads(hardware_config_path.read_text(encoding="utf-8"))
    calibration_path = Path(str(hardware["calibration_path"]))
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    wrist_roll_id = int(calibration["wrist_roll"]["id"])
    gripper_id = int(calibration["gripper"]["id"])
    return {
        "calibration_path": str(calibration_path),
        "id_5_name": next((name for name, entry in calibration.items() if int(entry["id"]) == 5), None),
        "id_6_name": next((name for name, entry in calibration.items() if int(entry["id"]) == 6), None),
        "gripper_hardware_id": gripper_id,
        "wrist_roll_hardware_id": wrist_roll_id,
        "gripper_logical_key": GRIPPER_LOGICAL_KEY,
        "wrist_roll_logical_key": WRIST_ROLL_LOGICAL_KEY,
        "motor_mapping_verified": wrist_roll_id == 5 and gripper_id == 6,
        "gripper_calibration_range": [0.0, 100.0],
    }


def gripper_target_in_range(value: float | None) -> bool:
    return value is not None and math.isfinite(float(value)) and 0.0 <= float(value) <= 100.0


def validate_runtime_gripper_targets(
    initial_gripper_position: float,
    open_delta: float,
) -> tuple[bool, str, float | None]:
    if not gripper_target_in_range(initial_gripper_position):
        return False, "invalid_initial_gripper_position", None
    if not math.isfinite(float(open_delta)):
        return False, "invalid_gripper_open_delta", None
    open_target = float(initial_gripper_position + open_delta)
    if not gripper_target_in_range(open_target):
        return False, "gripper_open_delta_out_of_calibration_range", open_target
    return True, "ok", open_target


def build_gripper_ramp_targets(
    initial_gripper_position: float,
    open_delta: float,
    fractions: tuple[float, ...],
) -> list[float]:
    return [
        float(initial_gripper_position + fraction * float(open_delta))
        for fraction in fractions
    ]


def gripper_delta_from_initial(
    initial_gripper_position: float,
    targets: list[float],
) -> list[float]:
    return [float(value - initial_gripper_position) for value in targets]


def validate_gripper_ramp_targets(targets: list[float]) -> bool:
    return all(gripper_target_in_range(value) for value in targets)


def make_descent_config(config: GraspConfig) -> DescentConfig:
    return DescentConfig(
        waypoint_drop_m=config.descent_waypoint_drop_m,
        start_pregrasp_joint_tolerance_rad=0.10,
        max_abs_joint_delta_per_waypoint_rad=config.max_abs_joint_delta_per_descent_waypoint_rad,
        position_tolerance_m=config.position_tolerance_m,
        approach_tolerance_deg=config.approach_tolerance_deg,
        max_xy_error_from_waypoint_m=config.max_xy_error_from_waypoint_m,
        minimum_actual_z_drop_per_waypoint_m=config.minimum_actual_z_drop_per_waypoint_m,
        minimum_total_actual_z_drop_m=config.minimum_total_actual_z_drop_m,
        final_joint_tolerance_rad=config.arm_final_joint_tolerance_rad,
        speed_rad_s=config.speed_rad_s,
        max_speed_rad_s=config.max_speed_rad_s,
        execute_service_timeout_s=config.execute_service_timeout_s,
        inter_waypoint_hold_s=config.inter_waypoint_hold_s,
        joint_state_max_age_s=config.joint_state_max_age_s,
        pregrasp_target_max_age_s=config.pregrasp_target_max_age_s,
    )


def validate_fresh_tactile_state(
    state: StampedTactileState | None,
    *,
    now_monotonic_s: float,
    max_age_s: float,
    require_clear: bool,
) -> tuple[bool, str]:
    if state is None:
        return False, "tactile_state_unavailable_or_stale"
    if now_monotonic_s - state.received_monotonic_s > max_age_s:
        return False, "tactile_state_unavailable_or_stale"
    if not state.ready:
        return False, "tactile_not_ready"
    if require_clear and state.contact_detected:
        return False, "tactile_contact_already_active"
    return True, "ok"


def build_lift_waypoint_xyz(start_xyz_m: list[float], rises_m: tuple[float, ...]) -> list[list[float]]:
    x, y, z = [float(value) for value in start_xyz_m]
    return [[x, y, z + float(rise)] for rise in rises_m]


def lift_fk_metrics(
    *,
    model: Any,
    q_rad: np.ndarray,
    requested_xyz_m: np.ndarray,
    previous_actual_xyz_m: np.ndarray,
) -> tuple[float, float, float, float, np.ndarray]:
    fk = forward_kinematics(model, q_rad)
    actual = np.asarray(fk["position_m"], dtype=np.float64)
    rotation = np.asarray(fk["rotation_matrix"], dtype=np.float64)
    current_approach = normalize_vector(rotation @ TOOL_APPROACH_AXIS_LOCAL, "lift approach")
    position_error_m = float(np.linalg.norm(requested_xyz_m - actual))
    approach_error_deg = math.degrees(rotation_angle_error(current_approach, DESIRED_APPROACH_BASE))
    xy_error_m = float(np.linalg.norm(requested_xyz_m[:2] - actual[:2]))
    actual_lift_m = float(actual[2] - previous_actual_xyz_m[2])
    return position_error_m, approach_error_deg, xy_error_m, actual_lift_m, actual


def plan_lift_waypoints(
    *,
    model: Any,
    start_joint_positions_rad: list[float],
    start_xyz_m: list[float],
    config: GraspConfig,
) -> dict[str, Any]:
    if not config.lift_enabled:
        return {"success": True, "reason": "lift_disabled", "lift_waypoints": [], "total_actual_lift_m": 0.0}
    waypoints_xyz = build_lift_waypoint_xyz(start_xyz_m, config.lift_waypoint_rise_m)
    previous_joint = list(start_joint_positions_rad)
    previous_actual = np.asarray(start_xyz_m, dtype=np.float64)
    first_seed = np.asarray(start_joint_positions_rad, dtype=np.float64)
    planned: list[dict[str, Any]] = []
    failure_reason = "lift_ik_failed"
    for index, requested in enumerate(waypoints_xyz, start=1):
        requested_xyz = np.asarray(requested, dtype=np.float64)
        waypoint: dict[str, Any] | None = None
        seed_source = "contact_hold_joint_state" if index == 1 else f"previous_lift_waypoint_{index - 1}"
        for source, seed in build_descent_seeds(model, requested_xyz, first_seed, seed_source):
            result = solve_ik(
                model,
                requested_xyz,
                seed,
                DESIRED_APPROACH_BASE,
                TOOL_APPROACH_AXIS_LOCAL,
                position_tolerance_m=config.lift_position_tolerance_m,
                approach_tolerance_deg=config.lift_approach_tolerance_deg,
            )
            raw_q = result.get("joint_positions_rad")
            if raw_q is None:
                failure_reason = str(result.get("reason", "lift_ik_failed"))
                continue
            q = np.asarray(raw_q, dtype=np.float64)
            if q.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(q)):
                failure_reason = "lift_non_finite_joint_solution"
                continue
            if not joints_within_limits(model, q):
                failure_reason = "lift_joint_limit_failed"
                continue
            delta_values = [float(t) - float(p) for p, t in zip(previous_joint, listf(q), strict=True)]
            max_delta = max(abs(value) for value in delta_values)
            if max_delta > config.max_abs_joint_delta_per_lift_waypoint_rad:
                failure_reason = "lift_joint_delta_exceeded"
                continue
            try:
                pos_error, approach_error, xy_error, actual_lift, actual = lift_fk_metrics(
                    model=model,
                    q_rad=q,
                    requested_xyz_m=requested_xyz,
                    previous_actual_xyz_m=previous_actual,
                )
            except (ValueError, np.linalg.LinAlgError, KeyError):
                failure_reason = "lift_fk_validation_failed"
                continue
            if xy_error > config.lift_max_xy_error_m:
                failure_reason = "lift_xy_error_exceeded"
                continue
            if pos_error > config.lift_position_tolerance_m:
                failure_reason = "lift_fk_position_validation_failed"
                continue
            if approach_error > config.lift_approach_tolerance_deg:
                failure_reason = "lift_fk_approach_validation_failed"
                continue
            if actual_lift < config.minimum_actual_lift_per_waypoint_m:
                failure_reason = "non_monotonic_lift"
                continue
            waypoint = {
                "index": index,
                "requested_xyz_m": listf(requested_xyz),
                "actual_fk_xyz_m": listf(actual),
                "arm_joint_target_rad": listf(q),
                "arm_joint_delta_rad": delta_values,
                "maximum_abs_arm_joint_delta_rad": float(max_delta),
                "position_error_m": float(pos_error),
                "approach_error_deg": float(approach_error),
                "xy_error_m": float(xy_error),
                "actual_lift_from_previous_m": float(actual_lift),
                "solution_type": "exact_solution" if bool(result.get("success")) else "accepted_near_solution",
                "seed_source": source,
            }
            break
        if waypoint is None:
            return {
                "success": False,
                "reason": failure_reason,
                "lift_waypoints": planned,
                "total_requested_lift_m": max(config.lift_waypoint_rise_m),
                "total_actual_lift_m": None,
            }
        planned.append(waypoint)
        previous_joint = waypoint["arm_joint_target_rad"]
        previous_actual = np.asarray(waypoint["actual_fk_xyz_m"], dtype=np.float64)
        first_seed = np.asarray(waypoint["arm_joint_target_rad"], dtype=np.float64)
    total_actual_lift = float(np.asarray(planned[-1]["actual_fk_xyz_m"], dtype=np.float64)[2] - float(start_xyz_m[2]))
    if total_actual_lift < config.minimum_total_actual_lift_m:
        return {
            "success": False,
            "reason": "total_lift_too_small",
            "lift_waypoints": planned,
            "total_requested_lift_m": max(config.lift_waypoint_rise_m),
            "total_actual_lift_m": total_actual_lift,
        }
    return {
        "success": True,
        "reason": "lift_plan_ready",
        "lift_waypoints": planned,
        "total_requested_lift_m": max(config.lift_waypoint_rise_m),
        "total_actual_lift_m": total_actual_lift,
    }


def build_integrated_plan_summary(
    *,
    mode: str,
    config: GraspConfig,
    motor_mapping: dict[str, Any],
    object_pose_base: list[float],
    pregrasp_pose_base: list[float],
    current_joint_positions_rad: list[float],
    pregrasp_joint_target_rad: list[float],
    initial_gripper_position: float,
    compute_message: str,
    tactile_state: StampedTactileState | None = None,
    object_x_raw: float = 0.0,
    grasp_x_offset_m: float = 0.0,
) -> dict[str, Any]:
    model = create_model()
    frozen = FrozenPregrasp(
        object_pose_base=object_pose_base,
        pregrasp_pose_base=pregrasp_pose_base,
        pregrasp_joint_target_rad=pregrasp_joint_target_rad,
        solution_type=None,
        selected_offset_m=None,
        position_error_m=None,
        approach_error_deg=None,
    )
    pregrasp_delta = joint_delta(current_joint_positions_rad, pregrasp_joint_target_rad)
    descent_config = make_descent_config(config)
    descent_plan = plan_segmented_descent(
        model=model,
        frozen=frozen,
        current_joint_positions_rad=pregrasp_joint_target_rad,
        config=descent_config,
    )
    ramp_targets = build_gripper_ramp_targets(
        initial_gripper_position,
        config.gripper_open_delta,
        config.gripper_open_ramp_fraction,
    )
    ramp_deltas = gripper_delta_from_initial(initial_gripper_position, ramp_targets)
    all_gripper_targets_valid = validate_gripper_ramp_targets(ramp_targets)
    open_valid, open_reason, open_target = validate_runtime_gripper_targets(
        initial_gripper_position,
        config.gripper_open_delta,
    )
    lift_plan = {"success": False, "reason": "descent_plan_unavailable", "lift_waypoints": []}
    if descent_plan.success and descent_plan.waypoints:
        lift_plan = plan_lift_waypoints(
            model=model,
            start_joint_positions_rad=descent_plan.waypoints[-1].selected_joint_target_rad,
            start_xyz_m=descent_plan.waypoints[-1].actual_fk_xyz_m,
            config=config,
        )
    tactile_ok, tactile_reason = validate_fresh_tactile_state(
        tactile_state,
        now_monotonic_s=time.monotonic(),
        max_age_s=config.tactile_state_max_age_s,
        require_clear=config.tactile_require_clear_before_grasp,
    )
    waypoints = []
    for index, waypoint in enumerate(descent_plan.waypoints):
        waypoints.append(
            {
                "index": waypoint.index,
                "requested_xyz_m": waypoint.requested_xyz_m,
                "actual_fk_xyz_m": waypoint.actual_fk_xyz_m,
                "arm_joint_target_rad": waypoint.selected_joint_target_rad,
                "arm_joint_delta_rad": waypoint.joint_delta_from_previous_rad,
                "maximum_abs_arm_joint_delta_rad": waypoint.maximum_abs_joint_delta_rad,
                "position_error_m": waypoint.position_error_m,
                "approach_error_deg": waypoint.approach_error_deg,
                "xy_error_m": waypoint.xy_error_m,
                "actual_z_drop_from_previous_m": waypoint.actual_z_drop_from_previous_m,
                "gripper_ramp_fraction": config.gripper_open_ramp_fraction[index],
                "gripper_delta_from_initial": ramp_deltas[index],
                "gripper_target_position": ramp_targets[index],
            }
        )
    estimated_pregrasp = estimated_duration_s(
        current_joint_positions_rad,
        pregrasp_joint_target_rad,
        config.speed_rad_s,
    )
    estimated_descent = 0.0
    previous = list(pregrasp_joint_target_rad)
    for waypoint in descent_plan.waypoints:
        estimated_descent += estimated_duration_s(previous, waypoint.selected_joint_target_rad, config.speed_rad_s)
        previous = waypoint.selected_joint_target_rad
    reason = (
        "integrated_visual_grasp_plan_ready"
        if descent_plan.success and all_gripper_targets_valid and open_valid and lift_plan["success"] and tactile_ok
        else (
            tactile_reason
            if not tactile_ok
            else open_reason
            if not open_valid
            else lift_plan["reason"]
            if not lift_plan["success"]
            else descent_plan.reason
        )
    )
    all_motion_waypoints = 1 + len(waypoints) + len(lift_plan.get("lift_waypoints", [])) + 1
    estimated_lift = estimated_total_duration_s(
        descent_plan.waypoints[-1].selected_joint_target_rad if descent_plan.waypoints else [],
        [waypoint["arm_joint_target_rad"] for waypoint in lift_plan.get("lift_waypoints", [])],
        config.lift_speed_rad_s,
    )
    return {
        "success": bool(descent_plan.success and all_gripper_targets_valid and open_valid and lift_plan["success"] and tactile_ok),
        "reason": reason,
        "mode": mode,
        "object_pose_base": object_pose_base,
        "object_x_raw": object_x_raw,
        "grasp_x_offset_m": grasp_x_offset_m,
        "object_x_corrected": float(object_pose_base[0]),
        "pregrasp_pose_base": pregrasp_pose_base,
        "current_joint_positions_rad": current_joint_positions_rad,
        "pregrasp_joint_target_rad": pregrasp_joint_target_rad,
        "pregrasp_max_abs_joint_delta_rad": float(pregrasp_delta["maximum_abs_joint_delta_rad"]),
        "pregrasp_limit_rad": config.pregrasp_max_abs_joint_delta_rad,
        "compute_response_message": compute_message,
        "initial_gripper_position": float(initial_gripper_position),
        "gripper_motor_logical_name": GRIPPER_LOGICAL_NAME,
        "gripper_motor_hardware_id": motor_mapping["gripper_hardware_id"],
        "wrist_roll_motor_hardware_id": motor_mapping["wrist_roll_hardware_id"],
        "gripper_open_delta": float(config.gripper_open_delta),
        "gripper_open_target_position": open_target,
        "gripper_close_target_source": config.gripper_close_target_source,
        "gripper_close_target_position": float(config.gripper_safe_close_limit),
        "gripper_close_target_equals_initial": False,
        "gripper_close_reference_g0": float(initial_gripper_position),
        "gripper_safe_close_limit": float(config.gripper_safe_close_limit),
        "gripper_close_step": float(config.gripper_close_step),
        "gripper_close_incremental": bool(config.gripper_close_incremental),
        "gripper_close_mode": "incremental_until_tactile_contact",
        "stop_gripper_on_tactile_contact": True,
        "lift_requires_tactile_contact": True,
        "tactile_stop_enabled": bool(config.tactile_stop_enabled),
        "tactile_source": None if tactile_state is None else tactile_state.source,
        "tactile_port": None if tactile_state is None else tactile_state.port,
        "tactile_ready_before_motion": None if tactile_state is None else bool(tactile_state.ready),
        "tactile_contact_before_motion": None if tactile_state is None else bool(tactile_state.contact_detected),
        "tactile_contact_score_before_motion": None if tactile_state is None else float(tactile_state.contact_score),
        "tactile_state_age_s_before_motion": None if tactile_state is None else tactile_state.state_age_s,
        "tactile_error_before_motion": None if tactile_state is None else tactile_state.error,
        "tactile_frame_count_before_motion": None if tactile_state is None else tactile_state.frame_count,
        "tactile_status_before_motion": None if tactile_state is None else tactile_state.status,
        "tactile_pre_motion_check": tactile_reason,
        "gripper_contact_preload_offset": 0.0,
        "waypoint_count": len(waypoints),
        "descent_waypoints": waypoints,
        "lift_enabled": bool(config.lift_enabled),
        "lift_waypoint_count": len(lift_plan.get("lift_waypoints", [])),
        "lift_waypoints": lift_plan.get("lift_waypoints", []),
        "total_requested_lift_m": lift_plan.get("total_requested_lift_m"),
        "total_actual_lift_m": lift_plan.get("total_actual_lift_m"),
        "all_lift_waypoints_valid": bool(lift_plan["success"]),
        "all_motion_waypoint_count": all_motion_waypoints,
        "total_requested_z_drop_m": config.total_descent_m,
        "total_actual_z_drop_m": descent_plan.total_actual_z_drop_m,
        "all_arm_waypoints_valid": bool(descent_plan.success),
        "all_gripper_targets_valid": bool(all_gripper_targets_valid),
        "estimated_pregrasp_duration_s": estimated_pregrasp,
        "estimated_descent_duration_s": estimated_descent,
        "estimated_gripper_close_duration_s": config.gripper_only_motion_duration_s,
        "estimated_lift_duration_s": estimated_lift,
        "estimated_total_duration_s": estimated_pregrasp + estimated_descent + config.gripper_only_motion_duration_s + estimated_lift,
        "live_visual_used_before_motion": True,
        "live_visual_required_after_motion": False,
        "all_motion_planned_before_execute": True,
        "hardware_command_sent": False,
    }


def pose_to_list(msg: Any) -> list[float]:
    return [
        float(msg.pose.position.x),
        float(msg.pose.position.y),
        float(msg.pose.position.z),
    ]


def json_print(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False))


def parse_tactile_status_fields(status: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for part in str(status).split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key.strip()] = value.strip()
    return fields


def optional_status_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def optional_status_int(value: str | None) -> int:
    if value is None or value == "":
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def tcp_status_is_connected(status: str) -> bool:
    base = str(status).split(";", 1)[0].strip().lower()
    return base == "connected"


class VisualGraspNode:
    def __init__(self, config: GraspConfig) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, String
        from std_srvs.srv import Trigger

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.JointState = JointState
        self.Float64 = Float64
        self.Trigger = Trigger
        self.node = _Node("mvp_visual_grasp")
        self.config = config
        self.latest_joint_state = None
        self.latest_gripper_state: StampedGripperState | None = None
        self.latest_object_pose: PoseStamped | None = None
        self.latest_object_pose_time = 0.0
        self.latest_pregrasp_target = None
        self.latest_pregrasp_target_time = 0.0
        self.latest_pregrasp_pose: PoseStamped | None = None
        self.pregrasp_valid = False
        self.pregrasp_status = ""
        self.tcp_connected = False
        self.tcp_status = "unknown"
        self.tcp_connected_received_monotonic_s: float | None = None
        self.tcp_status_received_monotonic_s: float | None = None
        self._last_logged_tcp_connected: bool | None = None
        self._last_logged_tcp_status: str | None = None
        self.latest_tactile_ready = False
        self.latest_tactile_contact = False
        self.latest_tactile_score = 0.0
        self.latest_tactile_status = "unknown"
        self.latest_tactile_state: StampedTactileState | None = None
        self.node.create_subscription(JointState, "/mvp/joint_states", self._joint_state_cb, 10)
        self.node.create_subscription(Float64, "/mvp/gripper_state", self._gripper_state_cb, 10)
        self.node.create_subscription(PoseStamped, "/object_pose_base", self._object_pose_cb, 10)
        self.node.create_subscription(JointState, "/mvp/pregrasp_joint_target", self._pregrasp_target_cb, 10)
        self.node.create_subscription(PoseStamped, "/mvp/pregrasp_pose", self._pregrasp_pose_cb, 10)
        self.node.create_subscription(Bool, "/mvp/pregrasp_valid", self._pregrasp_valid_cb, 10)
        self.node.create_subscription(String, "/mvp/pregrasp_status", self._pregrasp_status_cb, 10)
        state_qos = self._state_qos_profile()
        self.node.create_subscription(Bool, "/mvp/tcp_connected", self._tcp_connected_cb, state_qos)
        self.node.create_subscription(String, "/mvp/tcp_status", self._tcp_status_cb, state_qos)
        self.node.create_subscription(Bool, "/mvp/tactile_ready", self._tactile_ready_cb, state_qos)
        self.node.create_subscription(Bool, "/mvp/tactile_contact", self._tactile_contact_cb, state_qos)
        self.node.create_subscription(Float64, "/mvp/tactile_score", self._tactile_score_cb, state_qos)
        self.node.create_subscription(String, "/mvp/tactile_status", self._tactile_status_cb, state_qos)
        self.arm_target_pub = self.node.create_publisher(JointState, "/mvp/joint_target", 10)
        self.gripper_target_pub = self.node.create_publisher(Float64, "/mvp/gripper_target", 10)
        self.stop_gripper_on_tactile_pub = self.node.create_publisher(Bool, "/mvp/stop_gripper_on_tactile_contact", 10)
        self.compute_client = self.node.create_client(Trigger, "/mvp/compute_pregrasp")
        self.execute_client = self.node.create_client(Trigger, "/mvp/execute_target")

    def _state_qos_profile(self) -> Any:
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _joint_state_cb(self, msg: Any) -> None:
        from mvp_descend_from_pregrasp import StampedJointState

        self.latest_joint_state = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )

    def _gripper_state_cb(self, msg: Any) -> None:
        self.latest_gripper_state = StampedGripperState(float(msg.data), time.monotonic())

    def _object_pose_cb(self, msg: Any) -> None:
        self.latest_object_pose = msg
        self.latest_object_pose_time = time.monotonic()

    def _pregrasp_target_cb(self, msg: Any) -> None:
        from mvp_descend_from_pregrasp import StampedJointState

        self.latest_pregrasp_target = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )
        self.latest_pregrasp_target_time = self.latest_pregrasp_target.received_monotonic_s

    def _pregrasp_pose_cb(self, msg: Any) -> None:
        self.latest_pregrasp_pose = msg

    def _pregrasp_valid_cb(self, msg: Any) -> None:
        self.pregrasp_valid = bool(msg.data)

    def _pregrasp_status_cb(self, msg: Any) -> None:
        self.pregrasp_status = str(msg.data)

    def _tcp_connected_cb(self, msg: Any) -> None:
        self.tcp_connected = bool(msg.data)
        self.tcp_connected_received_monotonic_s = time.monotonic()
        if self._last_logged_tcp_connected is None or self._last_logged_tcp_connected != self.tcp_connected:
            self._last_logged_tcp_connected = self.tcp_connected
            log_event(
                "VISUAL_TCP_STATUS_RECEIVED "
                f"connected={str(self.tcp_connected).lower()} age_s=0.000 status={self.tcp_status}"
            )

    def _tcp_status_cb(self, msg: Any) -> None:
        self.tcp_status = str(msg.data)
        self.tcp_status_received_monotonic_s = time.monotonic()
        if self._last_logged_tcp_status != self.tcp_status:
            self._last_logged_tcp_status = self.tcp_status
            log_event(
                "VISUAL_TCP_STATUS_RECEIVED "
                f"connected={str(self.tcp_connected).lower()} age_s=0.000 status={self.tcp_status}"
            )

    def _refresh_tactile_state(self) -> None:
        fields = parse_tactile_status_fields(self.latest_tactile_status)
        self.latest_tactile_state = StampedTactileState(
            ready=bool(self.latest_tactile_ready),
            contact_detected=bool(self.latest_tactile_contact),
            contact_score=float(self.latest_tactile_score),
            status=str(self.latest_tactile_status),
            received_monotonic_s=time.monotonic(),
            source=fields.get("source", ""),
            port=fields.get("port", ""),
            state_age_s=optional_status_float(fields.get("age_s")),
            error=fields.get("error") or None,
            frame_count=optional_status_int(fields.get("frame_count")),
        )

    def _tactile_ready_cb(self, msg: Any) -> None:
        self.latest_tactile_ready = bool(msg.data)
        self._refresh_tactile_state()

    def _tactile_contact_cb(self, msg: Any) -> None:
        self.latest_tactile_contact = bool(msg.data)
        self._refresh_tactile_state()

    def _tactile_score_cb(self, msg: Any) -> None:
        self.latest_tactile_score = float(msg.data)
        self._refresh_tactile_state()

    def _tactile_status_cb(self, msg: Any) -> None:
        self.latest_tactile_status = str(msg.data)
        self._refresh_tactile_state()

    def spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def runtime_ready_diagnostics(self, *, execute_mode: bool) -> dict[str, Any]:
        now = time.monotonic()
        tcp_connected_age = (
            None
            if self.tcp_connected_received_monotonic_s is None
            else now - self.tcp_connected_received_monotonic_s
        )
        tcp_status_age = (
            None
            if self.tcp_status_received_monotonic_s is None
            else now - self.tcp_status_received_monotonic_s
        )
        joint_state_age = (
            None
            if self.latest_joint_state is None
            else now - self.latest_joint_state.received_monotonic_s
        )
        gripper_state_age = (
            None
            if self.latest_gripper_state is None
            else now - self.latest_gripper_state.received_monotonic_s
        )
        tactile_state_age = (
            None
            if self.latest_tactile_state is None
            else now - self.latest_tactile_state.received_monotonic_s
        )
        joint_ok, joint_reason = validate_fresh_joint_state(
            self.latest_joint_state,
            now_monotonic_s=now,
            max_age_s=self.config.joint_state_max_age_s,
        )
        tactile_ok = True
        tactile_reason = "not_required"
        if execute_mode:
            tactile_ok, tactile_reason = validate_fresh_tactile_state(
                self.latest_tactile_state,
                now_monotonic_s=now,
                max_age_s=self.config.tactile_state_max_age_s,
                require_clear=self.config.tactile_require_clear_before_grasp,
            )
        tcp_connected_fresh = (
            tcp_connected_age is not None
            and tcp_connected_age <= self.config.tcp_status_max_age_s
        )
        tcp_status_fresh = (
            tcp_status_age is not None
            and tcp_status_age <= self.config.tcp_status_max_age_s
        )
        gripper_ok = (
            self.latest_gripper_state is not None
            and gripper_state_age is not None
            and gripper_state_age <= self.config.gripper_state_max_age_s
        )
        checks = {
            "tcp_connected_seen": self.tcp_connected_received_monotonic_s is not None,
            "tcp_connected": bool(self.tcp_connected),
            "tcp_connected_age_s": tcp_connected_age,
            "tcp_status_seen": self.tcp_status_received_monotonic_s is not None,
            "tcp_status": self.tcp_status,
            "tcp_status_connected": tcp_status_is_connected(self.tcp_status),
            "tcp_status_age_s": tcp_status_age,
            "joint_state_seen": self.latest_joint_state is not None,
            "joint_state_age_s": joint_state_age,
            "joint_state_valid": bool(joint_ok),
            "joint_state_reason": joint_reason,
            "gripper_state_seen": self.latest_gripper_state is not None,
            "gripper_state_age_s": gripper_state_age,
            "gripper_state_valid": bool(gripper_ok),
            "tactile_ready_seen": self.latest_tactile_state is not None,
            "tactile_state_age_s": tactile_state_age,
            "tactile_ready": None if self.latest_tactile_state is None else self.latest_tactile_state.ready,
            "tactile_contact_detected": None
            if self.latest_tactile_state is None
            else self.latest_tactile_state.contact_detected,
            "tactile_state_valid": bool(tactile_ok),
            "tactile_reason": tactile_reason,
        }
        ready = (
            checks["tcp_connected_seen"]
            and checks["tcp_connected"]
            and tcp_connected_fresh
            and checks["tcp_status_seen"]
            and checks["tcp_status_connected"]
            and tcp_status_fresh
            and joint_ok
            and gripper_ok
            and tactile_ok
        )
        reason = "ready"
        if not checks["tcp_connected_seen"]:
            reason = "tcp_connected_state_not_received"
        elif not checks["tcp_connected"]:
            reason = "tcp_connected_false"
        elif not tcp_connected_fresh:
            reason = "tcp_connected_state_stale"
        elif not checks["tcp_status_seen"]:
            reason = "tcp_status_not_received"
        elif not checks["tcp_status_connected"]:
            reason = "tcp_status_not_connected"
        elif not tcp_status_fresh:
            reason = "tcp_status_stale"
        elif not joint_ok:
            reason = joint_reason
        elif not gripper_ok:
            reason = "gripper_state_unavailable_or_stale"
        elif not tactile_ok:
            reason = tactile_reason
        checks["runtime_ready"] = bool(ready)
        checks["reason"] = reason
        return checks
    def call_trigger(self, client: Any, timeout_s: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=3.0):
            return False, "service_unavailable"
        future = client.call_async(self.Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, "execute_service_timeout" if timeout_s >= 100.0 else "service_timeout"
        response = future.result()
        return bool(response.success), str(response.message)

    def publish_arm_target_once(self, target_rad: list[float]) -> None:
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(value) for value in target_rad]
        self.arm_target_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def publish_gripper_target_once(self, target_pos: float) -> None:
        msg = self.Float64()
        msg.data = float(target_pos)
        self.gripper_target_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def publish_stop_gripper_on_tactile_once(self, enabled: bool) -> None:
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = bool(enabled)
        self.stop_gripper_on_tactile_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def destroy(self) -> None:
        self.node.destroy_node()


def wait_for_mvp_runtime_ready(
    node: VisualGraspNode,
    config: GraspConfig,
    *,
    execute_mode: bool,
) -> tuple[bool, dict[str, Any]]:
    log_event(f"VISUAL_TCP_WAIT_STARTED timeout_s={config.tcp_ready_timeout_s}")
    deadline = time.monotonic() + config.tcp_ready_timeout_s
    diagnostics: dict[str, Any] = {}
    while node.rclpy.ok() and time.monotonic() < deadline:
        node.rclpy.spin_once(node.node, timeout_sec=0.05)
        diagnostics = node.runtime_ready_diagnostics(execute_mode=execute_mode)
        if diagnostics["runtime_ready"]:
            log_event("VISUAL_TCP_READY true")
            return True, diagnostics
    diagnostics = node.runtime_ready_diagnostics(execute_mode=execute_mode)
    log_event("VISUAL_TCP_READY false")
    log_event(f"reason={diagnostics['reason']}")
    log_event(
        "tcp_connected_seen={tcp_connected_seen} tcp_status_seen={tcp_status_seen} "
        "tcp_status_age_s={tcp_status_age_s} joint_state_seen={joint_state_seen} "
        "joint_state_age_s={joint_state_age_s} tactile_ready_seen={tactile_ready_seen}".format(
            **diagnostics
        )
    )
    return False, diagnostics


class TactileTestNode:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float64, String

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.node = _Node("mvp_visual_grasp_tactile_test")
        self.ready = False
        self.contact = False
        self.score = 0.0
        self.status = "unknown"
        self.latest: StampedTactileState | None = None
        self.ready_seen = False
        self.false_seen = False
        self.true_seen = False
        self.release_seen_after_true = False
        self.last_logged_contact: bool | None = None
        qos = self._state_qos_profile()
        self.node.create_subscription(Bool, "/mvp/tactile_ready", self._ready_cb, qos)
        self.node.create_subscription(Bool, "/mvp/tactile_contact", self._contact_cb, qos)
        self.node.create_subscription(Float64, "/mvp/tactile_score", self._score_cb, qos)
        self.node.create_subscription(String, "/mvp/tactile_status", self._status_cb, qos)

    def _state_qos_profile(self) -> Any:
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def _record(self) -> None:
        fields = parse_tactile_status_fields(self.status)
        self.latest = StampedTactileState(
            self.ready,
            self.contact,
            self.score,
            self.status,
            time.monotonic(),
            source=fields.get("source", ""),
            port=fields.get("port", ""),
            state_age_s=optional_status_float(fields.get("age_s")),
            error=fields.get("error") or None,
            frame_count=optional_status_int(fields.get("frame_count")),
        )
        if self.ready:
            self.ready_seen = True
        if not self.contact:
            if self.true_seen:
                self.release_seen_after_true = True
            self.false_seen = True
        if self.contact and self.false_seen:
            self.true_seen = True
        if self.last_logged_contact is None or self.last_logged_contact != self.contact:
            print(f"TACTILE_TEST contact={str(self.contact).lower()} score={self.score:.2f}", flush=True)
            self.last_logged_contact = self.contact

    def _ready_cb(self, msg: Any) -> None:
        self.ready = bool(msg.data)
        self._record()

    def _contact_cb(self, msg: Any) -> None:
        self.contact = bool(msg.data)
        self._record()

    def _score_cb(self, msg: Any) -> None:
        self.score = float(msg.data)
        self._record()

    def _status_cb(self, msg: Any) -> None:
        self.status = str(msg.data)
        self._record()

    def run(self, timeout_s: float) -> dict[str, Any]:
        started = time.monotonic()
        while self.rclpy.ok() and time.monotonic() - started <= timeout_s:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.ready_seen and self.false_seen and self.true_seen and self.release_seen_after_true:
                break
        observed_transition = self.false_seen and self.true_seen and self.release_seen_after_true
        latest = self.latest
        return {
            "success": bool(self.ready_seen and observed_transition),
            "reason": "tactile_static_test_pass" if self.ready_seen and observed_transition else "tactile_static_test_timeout",
            "mode": "tactile_test",
            "timeout_s": float(timeout_s),
            "tactile_source": "" if latest is None else latest.source,
            "tactile_port": "" if latest is None else latest.port,
            "tactile_ready_seen": bool(self.ready_seen),
            "tactile_false_seen": bool(self.false_seen),
            "tactile_true_seen": bool(self.true_seen),
            "tactile_release_seen_after_true": bool(self.release_seen_after_true),
            "tactile_ready": bool(self.ready),
            "tactile_contact_detected": bool(self.contact),
            "tactile_contact_score": float(self.score),
            "tactile_state_age_s": None if latest is None else latest.state_age_s,
            "tactile_error": None if latest is None else latest.error,
            "tactile_frame_count": 0 if latest is None else latest.frame_count,
            "tactile_status": self.status,
            "hardware_command_sent": False,
            "camera_used": False,
            "pregrasp_compute_called": False,
            "ros_publish_count": 0,
        }

    def destroy(self) -> None:
        self.node.destroy_node()


def run_tactile_test(args: argparse.Namespace) -> int:
    del args
    config = load_grasp_config()
    import rclpy

    rclpy.init()
    node = TactileTestNode()
    try:
        result = node.run(config.tactile_static_test_timeout_s)
        json_print(result)
        return 0 if result["success"] else 18
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def run(args: argparse.Namespace) -> int:
    config = load_grasp_config()
    motor_mapping = load_motor_mapping()
    if not motor_mapping["motor_mapping_verified"]:
        json_print({"success": False, "reason": "gripper_motor_mapping_mismatch", **motor_mapping})
        return 2

    import rclpy

    model = create_model()
    rclpy.init()
    node = VisualGraspNode(config)
    try:
        runtime_ready, runtime_diagnostics = wait_for_mvp_runtime_ready(
            node,
            config,
            execute_mode=bool(args.execute and not args.plan_only and config.tactile_stop_enabled),
        )
        if not runtime_ready:
            json_print(
                {
                    "success": False,
                    "reason": runtime_diagnostics["reason"],
                    "mode": "plan_only" if args.plan_only or not args.execute else "execute",
                    "hardware_command_sent": False,
                    **runtime_diagnostics,
                }
            )
            return 5
        if not node.spin_until(
            lambda: node.latest_object_pose is not None
            and time.monotonic() - node.latest_object_pose_time <= config.object_pose_max_age_s,
            config.object_pose_max_age_s,
        ):
            json_print({"success": False, "reason": "object_pose_unavailable_or_stale"})
            return 6

        assert node.latest_joint_state is not None
        assert node.latest_gripper_state is not None
        assert node.latest_object_pose is not None
        current = [float(value) for value in node.latest_joint_state.positions_rad]
        initial_gripper = float(node.latest_gripper_state.position)
        open_valid, open_reason, open_target = validate_runtime_gripper_targets(
            initial_gripper,
            config.gripper_open_delta,
        )
        if not open_valid:
            json_print(
                {
                    "success": False,
                    "reason": open_reason,
                    "mode": "plan_only" if args.plan_only or not args.execute else "execute",
                    "initial_gripper_position": initial_gripper,
                    "gripper_open_delta": config.gripper_open_delta,
                    "gripper_open_target_position": open_target,
                    "hardware_command_sent": False,
                }
            )
            return 5
        compute_started = time.monotonic()
        compute_success, compute_message = node.call_trigger(node.compute_client, 10.0)
        if not compute_success:
            json_print({"success": False, "reason": "compute_pregrasp_failed", "message": compute_message})
            return 7
        if not node.spin_until(
            lambda: node.latest_pregrasp_target is not None
            and node.latest_pregrasp_target_time >= compute_started
            and node.latest_pregrasp_pose is not None
            and time.monotonic() - node.latest_pregrasp_target.received_monotonic_s <= config.pregrasp_target_max_age_s,
            config.pregrasp_target_max_age_s,
        ):
            json_print({"success": False, "reason": "pregrasp_target_unavailable_or_stale"})
            return 8
        assert node.latest_pregrasp_target is not None
        assert node.latest_pregrasp_pose is not None
        valid_target, target_reason = validate_joint_contract(node.latest_pregrasp_target.names, node.latest_pregrasp_target.positions_rad)
        if not valid_target:
            json_print({"success": False, "reason": target_reason})
            return 9
        if not node.pregrasp_valid or not status_is_accepted(node.pregrasp_status, compute_message):
            json_print({"success": False, "reason": "pregrasp_not_ready", "status": node.pregrasp_status})
            return 10
        frozen_target = [float(value) for value in node.latest_pregrasp_target.positions_rad]
        if not target_within_urdf_limits(model, frozen_target):
            json_print({"success": False, "reason": "joint_limit_failed"})
            return 11
        pregrasp_delta = joint_delta(current, frozen_target)
        if float(pregrasp_delta["maximum_abs_joint_delta_rad"]) > config.pregrasp_max_abs_joint_delta_rad:
            json_print({"success": False, "reason": "pregrasp_joint_delta_exceeded"})
            return 12

        object_pose_base = pose_to_list(node.latest_object_pose)
        pregrasp_pose_base = pose_to_list(node.latest_pregrasp_pose)

        # ---- MVP-4E-X-AXIS-GRASP-OFFSET: apply grasp_x_offset_m ----
        object_x_raw = float(object_pose_base[0])
        hardware_raw = json.loads(HARDWARE_CONFIG_PATH.read_text(encoding="utf-8"))
        grasp_x_offset_m = float(hardware_raw.get("grasp_x_offset_m", 0.0))
        object_pose_base[0] = object_x_raw + grasp_x_offset_m
        pregrasp_pose_base[0] = float(pregrasp_pose_base[0]) + grasp_x_offset_m

        created_at_unix_s = time.time()
        planned_snapshot = make_pregrasp_snapshot(
            snapshot_state="planned",
            created_at_unix_s=created_at_unix_s,
            object_pose_base=object_pose_base,
            pregrasp_pose_base=pregrasp_pose_base,
            frozen_target_rad=frozen_target,
            compute_message=compute_message,
            compute_pregrasp_success=True,
            pregrasp_valid=node.pregrasp_valid,
            pregrasp_status=node.pregrasp_status,
            hardware_command_sent=False,
            execute_response_message=None,
            motion_completed=None,
            final_joint_positions_rad=None,
            final_errors=None,
            config=MoveConfig(),
            tcp_connected_after_motion=None,
            tcp_status_after_motion=None,
        )
        atomic_write_json(SNAPSHOT_PATH, planned_snapshot)

        summary = build_integrated_plan_summary(
            mode="plan_only" if args.plan_only or not args.execute else "execute",
            config=config,
            motor_mapping=motor_mapping,
            object_pose_base=object_pose_base,
            pregrasp_pose_base=pregrasp_pose_base,
            current_joint_positions_rad=current,
            pregrasp_joint_target_rad=frozen_target,
            initial_gripper_position=initial_gripper,
            compute_message=compute_message,
            tactile_state=node.latest_tactile_state,
            object_x_raw=object_x_raw,
            grasp_x_offset_m=grasp_x_offset_m,
        )
        atomic_write_json(
            INTEGRATED_SNAPSHOT_PATH,
            {
                "schema_version": 1,
                "stage": "MVP-4D-INTEGRATED-VISUAL-GRASP",
                "created_at_unix_s": created_at_unix_s,
                "updated_at_unix_s": time.time(),
                "snapshot_state": "planned",
                **summary,
            },
        )
        if args.plan_only or not args.execute:
            summary.update(
                {
                    "tcp_connected": bool(node.tcp_connected),
                    "tcp_status": node.tcp_status,
                    "tcp_status_age_s": runtime_diagnostics.get("tcp_status_age_s"),
                }
            )
            json_print(summary)
            return 0 if summary["success"] else 13

        if args.confirm != CONFIRM_PHRASE:
            summary.update({"success": False, "reason": "wrong_confirmation", "required_confirm": CONFIRM_PHRASE})
            json_print(summary)
            return 2
        if not summary["success"]:
            json_print(summary)
            return 13

        execute_count = 0
        arm_publish_count = 0
        gripper_publish_count = 0
        tactile_stop_publish_count = 0
        lift_execute_count = 0
        node.publish_stop_gripper_on_tactile_once(False)
        tactile_stop_publish_count += 1
        node.publish_arm_target_once(frozen_target)
        arm_publish_count += 1
        pregrasp_success, pregrasp_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
        execute_count += 1
        if not pregrasp_success:
            summary.update({"success": False, "reason": pregrasp_message, "hardware_command_sent": True})
            json_print(summary)
            return 14
        for waypoint in summary["descent_waypoints"]:
            node.publish_arm_target_once(waypoint["arm_joint_target_rad"])
            arm_publish_count += 1
            node.publish_gripper_target_once(float(waypoint["gripper_target_position"]))
            gripper_publish_count += 1
            waypoint_success, waypoint_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
            execute_count += 1
            if not waypoint_success:
                summary.update({"success": False, "reason": waypoint_message, "hardware_command_sent": True})
                json_print(summary)
                return 15
            time.sleep(config.inter_waypoint_hold_s)
        if not node.spin_until(
            lambda: validate_fresh_joint_state(
                node.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=config.joint_state_max_age_s,
            )[0],
            3.0,
        ):
            summary.update({"success": False, "reason": "final_arm_state_unavailable", "hardware_command_sent": True})
            json_print(summary)
            return 16
        assert node.latest_joint_state is not None
        hold_arm = [float(value) for value in node.latest_joint_state.positions_rad]
        gripper_before_close = None if node.latest_gripper_state is None else float(node.latest_gripper_state.position)
        gripper_close_start_position = gripper_before_close
        gripper_close_reference_g0 = float(initial_gripper)
        safe_close_limit = float(config.gripper_safe_close_limit)
        gripper_close_step = float(config.gripper_close_step)
        node.publish_arm_target_once(hold_arm)
        arm_publish_count += 1
        node.publish_stop_gripper_on_tactile_once(True)
        tactile_stop_publish_count += 1
        # ------------------------------------------------------------------
        # Incremental close-until-tactile-contact loop
        # ------------------------------------------------------------------
        close_success = False
        close_message = "close_not_started"
        tactile_contact_confirmed = False
        safe_close_limit_reached = False
        gripper_motion_stalled = False
        close_steps_commanded = 0
        close_steps_completed = 0
        close_termination_reason = "close_not_started"
        close_start_time = time.monotonic()
        gripper_close_contact_position: float | None = None
        gripper_hold_position: float | None = None
        stall_count = 0
        previous_close_gripper = gripper_before_close
        current_close_target = gripper_before_close
        while True:
            if time.monotonic() - close_start_time > config.gripper_close_timeout_s:
                close_success = False
                close_message = "gripper_close_timeout"
                close_termination_reason = "close_timeout"
                break
            next_target = current_close_target - gripper_close_step
            if next_target < safe_close_limit:
                next_target = safe_close_limit
            current_close_target = float(next_target)
            close_steps_commanded += 1
            node.publish_gripper_target_once(current_close_target)
            gripper_publish_count += 1
            step_success, step_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
            execute_count += 1
            node.spin_until(
                lambda: node.latest_gripper_state is not None
                and time.monotonic() - node.latest_gripper_state.received_monotonic_s <= config.gripper_state_max_age_s,
                2.0,
            )
            node.spin_until(
                lambda: node.latest_tactile_state is not None
                and time.monotonic() - node.latest_tactile_state.received_monotonic_s <= config.tactile_state_max_age_s,
                2.0,
            )
            # --- primary stop: tactile contact confirmed ---
            if node.latest_tactile_state is not None and node.latest_tactile_state.contact_detected:
                tactile_contact_confirmed = True
                close_success = True
                close_message = "tactile_contact_stop"
                close_termination_reason = "tactile_contact"
                close_steps_completed = close_steps_commanded
                gripper_hold_position = (
                    None if node.latest_gripper_state is None
                    else float(node.latest_gripper_state.position)
                )
                gripper_close_contact_position = gripper_hold_position
                break
            if not step_success:
                close_success = False
                close_message = step_message
                close_termination_reason = "motion_error"
                close_steps_completed = close_steps_commanded - 1
                break
            close_steps_completed = close_steps_commanded
            # --- stall detection (diagnostic only, does not allow lift) ---
            current_gripper = None if node.latest_gripper_state is None else float(node.latest_gripper_state.position)
            if current_gripper is not None and previous_close_gripper is not None:
                if abs(float(current_gripper) - float(previous_close_gripper)) < config.gripper_close_stall_threshold:
                    stall_count += 1
                else:
                    stall_count = 0
            previous_close_gripper = current_gripper
            if stall_count >= config.gripper_close_stall_steps:
                gripper_motion_stalled = True
            # --- secondary stop: safe close limit reached ---
            if abs(current_close_target - safe_close_limit) < 1e-6:
                safe_close_limit_reached = True
                close_success = False
                close_message = "gripper_closed_without_tactile_contact"
                close_termination_reason = "safe_close_limit_without_contact"
                break
            time.sleep(config.gripper_close_hold_s * 0.2)
        # ------------------------------------------------------------------
        # Post-close state
        # ------------------------------------------------------------------
        gripper_final = None if node.latest_gripper_state is None else float(node.latest_gripper_state.position)
        if gripper_hold_position is None:
            gripper_hold_position = gripper_final
        lift_completed = False
        lift_failure_reason: str | None = None
        if tactile_contact_confirmed:
            node.publish_stop_gripper_on_tactile_once(False)
            tactile_stop_publish_count += 1
            held_gripper = gripper_hold_position if gripper_hold_position is not None else initial_gripper
            for lift_waypoint in summary["lift_waypoints"]:
                if not node.spin_until(
                    lambda: node.latest_tactile_state is not None
                    and time.monotonic() - node.latest_tactile_state.received_monotonic_s <= config.tactile_state_max_age_s
                    and node.latest_tactile_state.ready
                    and node.latest_tactile_state.contact_detected,
                    2.0,
                ):
                    lift_failure_reason = "tactile_contact_lost_during_lift"
                    break
                node.publish_arm_target_once(lift_waypoint["arm_joint_target_rad"])
                arm_publish_count += 1
                node.publish_gripper_target_once(float(held_gripper))
                gripper_publish_count += 1
                lift_success, lift_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
                execute_count += 1
                lift_execute_count += 1
                if not lift_success:
                    lift_failure_reason = lift_message
                    break
                time.sleep(config.inter_lift_waypoint_hold_s)
            lift_completed = lift_failure_reason is None and lift_execute_count == len(summary["lift_waypoints"])
        summary.update(
            {
                "success": bool(tactile_contact_confirmed and lift_completed),
                "reason": "tactile_grasp_lift_completed"
                if tactile_contact_confirmed and lift_completed
                else close_termination_reason
                if not tactile_contact_confirmed
                else lift_failure_reason
                if lift_failure_reason is not None
                else "grasp_close_attempt_failed",
                "close_execute_response_message": close_message,
                "hardware_command_sent": True,
                "arm_target_publish_count": arm_publish_count,
                "gripper_target_publish_count": gripper_publish_count,
                "tactile_stop_publish_count": tactile_stop_publish_count,
                "execute_call_count": execute_count,
                "lift_execute_count": lift_execute_count,
                "gripper_initial_position": initial_gripper,
                "gripper_open_target_position": initial_gripper + config.gripper_open_delta,
                "gripper_close_reference_g0": gripper_close_reference_g0,
                "gripper_close_start_position": gripper_close_start_position,
                "gripper_safe_close_limit": safe_close_limit,
                "gripper_close_step": gripper_close_step,
                "gripper_close_steps_commanded": close_steps_commanded,
                "gripper_close_steps_completed": close_steps_completed,
                "gripper_position_before_close": gripper_before_close,
                "gripper_close_target_position": current_close_target,
                "gripper_final_position": gripper_final,
                "gripper_final_error": None if gripper_final is None else abs(float(gripper_final) - float(gripper_close_reference_g0)),
                "gripper_close_command_completed": bool(close_success),
                "gripper_stop_triggered": bool(tactile_contact_confirmed),
                "gripper_stopped_on_tactile_contact": bool(tactile_contact_confirmed),
                "gripper_hold_position": gripper_hold_position,
                "gripper_contact_preload_offset": 0.0,
                "gripper_closed_without_tactile_contact": bool(safe_close_limit_reached and not tactile_contact_confirmed),
                "gripper_close_target_reached": False if safe_close_limit_reached else bool(close_success),
                "possible_object_blocking_gripper": bool(gripper_motion_stalled),
                "object_may_be_grasped": bool(tactile_contact_confirmed),
                "safe_close_limit_reached": bool(safe_close_limit_reached),
                "gripper_motion_stalled": bool(gripper_motion_stalled),
                "close_termination_reason": close_termination_reason,
                "lift_completed": bool(lift_completed),
                "lift_failure_reason": lift_failure_reason,
                "lift_waypoints_executed": lift_execute_count,
                "tactile_contact_confirmed": bool(tactile_contact_confirmed),
            }
        )
        json_print(summary)
        return 0 if summary["success"] else 17
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Integrated SO-101 visual pregrasp, 7 cm descent, and gripper close.")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--tactile-test", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.tactile_test and (args.execute or args.plan_only):
        print("--tactile-test cannot be combined with --plan-only or --execute", file=sys.stderr)
        return 2
    if args.execute and args.plan_only:
        print("--plan-only and --execute are mutually exclusive", file=sys.stderr)
        return 2
    if args.tactile_test:
        return run_tactile_test(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
