from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from so101_kinematics.urdf_fk import ARM_JOINT_NAMES, UrdfForwardKinematics

from .command_gate_validator import (
    extract_status_plan_id,
    joint_limit_margin,
    load_command_gate_config,
    make_invalid,
    parse_plan_id_from_frame_id,
    reorder_joint_state_positions,
    seconds_from_duration,
    validate_command_candidate,
)
from .connection_parameterizer import (
    CONNECTION_STATUS,
    ConnectionConfig,
    ConnectionPlan,
)


@dataclass(frozen=True)
class ConnectionValidationResult:
    valid: bool
    reason: str
    upstream_plan_id: str | None
    minimum_joint_limit_margin_rad: float | None = None
    minimum_tcp_z_m_observed: float | None = None
    tcp_x_range_m_observed: list[float] | None = None
    tcp_y_range_m_observed: list[float] | None = None
    time_strictly_increasing: bool = False


def invalid_result(reason: str, upstream_plan_id: str | None = None) -> ConnectionValidationResult:
    return ConnectionValidationResult(False, reason, upstream_plan_id)


def parse_status_json(raw_status: str | None) -> dict[str, Any] | None:
    if raw_status is None:
        return None
    try:
        value = json.loads(raw_status)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def validate_source_inputs(
    *,
    real_joint_state_valid: bool,
    safe_timed_valid: bool,
    source_status: dict[str, Any] | None,
    source_trajectory: JointTrajectory | None,
    current_joint_state: JointState | None,
    current_joint_state_age_s: float | None,
    source_valid_heartbeat_age_s: float | None,
    config: ConnectionConfig,
    lower: np.ndarray,
    upper: np.ndarray,
    project_root: Path | str,
) -> tuple[ConnectionValidationResult, np.ndarray | None, str | None]:
    upstream_plan_id = extract_status_plan_id(source_status)
    if not real_joint_state_valid:
        return invalid_result("real_joint_state_invalid", upstream_plan_id), None, upstream_plan_id
    if not safe_timed_valid:
        return invalid_result("safe_trajectory_invalid", upstream_plan_id), None, upstream_plan_id
    if source_valid_heartbeat_age_s is None:
        return invalid_result("waiting_for_safe_timed_status", upstream_plan_id), None, upstream_plan_id
    if source_valid_heartbeat_age_s > config.source_valid_heartbeat_timeout_s:
        return invalid_result("source_valid_heartbeat_stale", upstream_plan_id), None, upstream_plan_id
    if source_status is None or source_status.get("status") != "VALID":
        return invalid_result("safe_timed_status_not_valid", upstream_plan_id), None, upstream_plan_id
    if upstream_plan_id is None:
        return invalid_result("safe_timed_status_plan_id_missing"), None, None
    if source_trajectory is None:
        return invalid_result("waiting_for_safe_timed_trajectory", upstream_plan_id), None, upstream_plan_id
    trajectory_plan_id = parse_plan_id_from_frame_id(source_trajectory.header.frame_id)
    if trajectory_plan_id is None:
        return invalid_result("trajectory_plan_id_missing", upstream_plan_id), None, upstream_plan_id
    if trajectory_plan_id != upstream_plan_id:
        return invalid_result("plan_id_mismatch", upstream_plan_id), None, upstream_plan_id
    if current_joint_state is None or current_joint_state_age_s is None:
        return invalid_result("waiting_for_current_joint_state", upstream_plan_id), None, upstream_plan_id
    if current_joint_state_age_s > config.current_joint_state_timeout_s:
        return invalid_result("current_joint_state_stale", upstream_plan_id), None, upstream_plan_id

    current_positions, reason = reorder_joint_state_positions(current_joint_state, config.joint_names)
    if current_positions is None or reason != "ok":
        if reason == "current_joint_state_non_finite":
            reason = "non_finite_input"
        return invalid_result(reason, upstream_plan_id), current_positions, upstream_plan_id
    if not np.all(np.isfinite(current_positions)):
        return invalid_result("non_finite_input", upstream_plan_id), current_positions, upstream_plan_id
    if np.any(current_positions < lower - 1.0e-10) or np.any(current_positions > upper + 1.0e-10):
        return invalid_result("current_joint_out_of_bounds", upstream_plan_id), current_positions, upstream_plan_id
    current_margin = float(np.min(np.minimum(current_positions - lower, upper - current_positions)))
    if current_margin < config.minimum_joint_limit_margin_rad:
        return invalid_result("joint_margin_insufficient", upstream_plan_id), current_positions, upstream_plan_id
    if list(source_trajectory.joint_names) != config.joint_names:
        if sorted(source_trajectory.joint_names) == sorted(config.joint_names):
            reason = "trajectory_wrong_joint_order"
        else:
            reason = "trajectory_wrong_joint_names"
        return invalid_result(reason, upstream_plan_id), current_positions, upstream_plan_id
    if len(source_trajectory.points) < 2:
        return invalid_result("trajectory_too_few_points", upstream_plan_id), current_positions, upstream_plan_id
    try:
        q_start = np.asarray(source_trajectory.points[0].positions, dtype=np.float64)
    except (TypeError, ValueError):
        return invalid_result("non_finite_input", upstream_plan_id), current_positions, upstream_plan_id
    if q_start.shape != (len(config.joint_names),) or not np.all(np.isfinite(q_start)):
        return invalid_result("non_finite_input", upstream_plan_id), current_positions, upstream_plan_id
    if np.any(q_start < lower - 1.0e-10) or np.any(q_start > upper + 1.0e-10):
        return invalid_result("q_start_out_of_bounds", upstream_plan_id), current_positions, upstream_plan_id
    q_start_margin = float(np.min(np.minimum(q_start - lower, upper - q_start)))
    if q_start_margin < config.minimum_joint_limit_margin_rad:
        return invalid_result("joint_margin_insufficient", upstream_plan_id), current_positions, upstream_plan_id

    gate_config = load_command_gate_config(project_root)
    start_state = JointState()
    start_state.name = list(config.joint_names)
    start_state.position = [float(value) for value in q_start]
    gate_result = validate_command_candidate(
        safe_timed_valid=True,
        safe_timed_status=source_status,
        trajectory=source_trajectory,
        joint_state=start_state,
        current_joint_state_age_s=0.0,
        source_valid_heartbeat_age_s=0.0,
        config=gate_config,
        lower=lower,
        upper=upper,
    )
    if not gate_result.ready:
        return invalid_result("original_trajectory_invalid:" + gate_result.reason, upstream_plan_id), current_positions, upstream_plan_id
    return ConnectionValidationResult(True, "source_inputs_valid", upstream_plan_id), current_positions, upstream_plan_id


def validate_connection_plan(
    *,
    plan: ConnectionPlan,
    config: ConnectionConfig,
    lower: np.ndarray,
    upper: np.ndarray,
    project_root: Path | str,
) -> ConnectionValidationResult:
    trajectory = plan.combined_trajectory
    if plan.connection_duration_s < config.minimum_connection_duration_s - 1.0e-9:
        return invalid_result("connection_duration_too_short", plan.upstream_plan_id)
    if plan.connection_duration_s > config.maximum_connection_duration_s + 1.0e-9:
        return invalid_result("connection_duration_too_long", plan.upstream_plan_id)
    if len(trajectory.points) < plan.connection_point_count:
        return invalid_result("combined_trajectory_too_short", plan.upstream_plan_id)

    positions = []
    times = []
    for point in trajectory.points:
        if (
            len(point.positions) != len(config.joint_names)
            or len(point.velocities) != len(config.joint_names)
            or len(point.accelerations) != len(config.joint_names)
        ):
            return invalid_result("trajectory_point_wrong_length", plan.upstream_plan_id)
        positions.append([float(value) for value in point.positions])
        times.append(seconds_from_duration(point.time_from_start))
    positions_array = np.asarray(positions, dtype=np.float64)
    times_array = np.asarray(times, dtype=np.float64)
    time_strict = bool(np.all(np.diff(times_array) > 0.0))
    if abs(float(times_array[0])) > 1.0e-12 or not time_strict:
        return ConnectionValidationResult(False, "trajectory_time_not_strictly_increasing", plan.upstream_plan_id, time_strictly_increasing=time_strict)
    if not np.all(np.isfinite(positions_array)) or not np.all(np.isfinite(times_array)):
        return invalid_result("trajectory_non_finite_values", plan.upstream_plan_id)
    if np.max(np.abs(positions_array[0] - plan.q_current)) > 1.0e-10:
        return invalid_result("connection_first_point_not_current_state", plan.upstream_plan_id)
    if np.max(np.abs(positions_array[plan.connection_point_count - 1] - plan.q_start)) > 1.0e-10:
        return invalid_result("connection_last_point_not_q_start", plan.upstream_plan_id)
    if np.any(positions_array < lower.reshape(1, -1) - 1.0e-10) or np.any(
        positions_array > upper.reshape(1, -1) + 1.0e-10
    ):
        return invalid_result("trajectory_joint_position_out_of_bounds", plan.upstream_plan_id)
    margin = joint_limit_margin(positions_array, lower, upper)
    if margin < config.minimum_joint_limit_margin_rad:
        return ConnectionValidationResult(False, "joint_margin_insufficient", plan.upstream_plan_id, margin, time_strictly_increasing=time_strict)
    if np.any(plan.maximum_connection_velocity_rad_s_observed > config.maximum_connection_velocity_rad_s + 1.0e-12):
        return invalid_result("connection_velocity_limit_exceeded", plan.upstream_plan_id)
    if np.any(plan.maximum_connection_acceleration_rad_s2_observed > config.maximum_connection_acceleration_rad_s2 + 1.0e-12):
        return invalid_result("connection_acceleration_limit_exceeded", plan.upstream_plan_id)

    urdf_path = Path(project_root).resolve() / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
    fk = UrdfForwardKinematics(urdf_path, "base_link", "gripper_frame_link")
    fk.validate_expected_chain()
    tcp_positions = []
    for q in positions_array[: plan.connection_point_count]:
        tcp = compute_tcp_position_with_fk_chain(
            fk,
            dict(zip(ARM_JOINT_NAMES, q.tolist())),
        )
        if tcp.shape != (3,) or not np.all(np.isfinite(tcp)):
            return invalid_result("tcp_fk_non_finite", plan.upstream_plan_id)
        tcp_positions.append(tcp)
    tcp_array = np.asarray(tcp_positions, dtype=np.float64)
    if len(tcp_array) > 1 and float(np.max(np.linalg.norm(np.diff(tcp_array, axis=0), axis=1))) > 0.05:
        return invalid_result("tcp_adjacent_jump_detected", plan.upstream_plan_id)
    min_z = float(np.min(tcp_array[:, 2]))
    x_range = [float(np.min(tcp_array[:, 0])), float(np.max(tcp_array[:, 0]))]
    y_range = [float(np.min(tcp_array[:, 1])), float(np.max(tcp_array[:, 1]))]
    if min_z < config.minimum_tcp_z_m:
        return ConnectionValidationResult(False, "tcp_below_minimum_z", plan.upstream_plan_id, margin, min_z, x_range, y_range, time_strict)
    if x_range[0] < config.tcp_workspace_x_min_m or x_range[1] > config.tcp_workspace_x_max_m:
        return ConnectionValidationResult(False, "tcp_outside_workspace", plan.upstream_plan_id, margin, min_z, x_range, y_range, time_strict)
    if y_range[0] < config.tcp_workspace_y_min_m or y_range[1] > config.tcp_workspace_y_max_m:
        return ConnectionValidationResult(False, "tcp_outside_workspace", plan.upstream_plan_id, margin, min_z, x_range, y_range, time_strict)
    return ConnectionValidationResult(True, "connection_and_grasp_shadow_candidate_valid", plan.upstream_plan_id, margin, min_z, x_range, y_range, time_strict)


def compute_tcp_position_with_fk_chain(
    fk: UrdfForwardKinematics,
    joint_positions: dict[str, float],
) -> np.ndarray:
    fk.check_joint_limits(joint_positions)
    transform = identity4()
    for joint in fk.chain:
        transform = matmul4(
            transform,
            make_transform4(rpy_rotation_matrix(joint.rpy), joint.xyz),
        )
        if joint.joint_type == "revolute":
            angle = float(joint_positions.get(joint.name, 0.0))
            transform = matmul4(
                transform,
                make_transform4(axis_angle_rotation_matrix(joint.axis, angle), (0.0, 0.0, 0.0)),
            )
        elif joint.joint_type == "fixed":
            continue
        else:
            raise ValueError(f"Unsupported joint type {joint.joint_type} for {joint.name}")
    return np.asarray([transform[0][3], transform[1][3], transform[2][3]], dtype=np.float64)


def identity4() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul4(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
        for row in range(4)
    ]


def make_transform4(rotation: list[list[float]], translation: Any) -> list[list[float]]:
    xyz = [float(value) for value in translation]
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], xyz[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], xyz[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def rpy_rotation_matrix(rpy: Any) -> list[list[float]]:
    roll, pitch, yaw = [float(value) for value in rpy]
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    rotation_x = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]
    rotation_y = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]
    rotation_z = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return matmul3(matmul3(rotation_z, rotation_y), rotation_x)


def axis_angle_rotation_matrix(axis: Any, angle: float) -> list[list[float]]:
    values = [float(value) for value in axis]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("Revolute joint axis has zero length")
    x, y, z = [value / norm for value in values]
    cosine = math.cos(float(angle))
    sine = math.sin(float(angle))
    one_minus_cosine = 1.0 - cosine
    return [
        [
            cosine + x * x * one_minus_cosine,
            x * y * one_minus_cosine - z * sine,
            x * z * one_minus_cosine + y * sine,
        ],
        [
            y * x * one_minus_cosine + z * sine,
            cosine + y * y * one_minus_cosine,
            y * z * one_minus_cosine - x * sine,
        ],
        [
            z * x * one_minus_cosine - y * sine,
            z * y * one_minus_cosine + x * sine,
            cosine + z * z * one_minus_cosine,
        ],
    ]
