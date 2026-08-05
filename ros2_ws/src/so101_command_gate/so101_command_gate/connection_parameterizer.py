from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_kinematics.urdf_fk import ARM_JOINT_NAMES

from .command_gate_validator import DEFAULT_PROJECT_ROOT, seconds_from_duration


CONNECTION_STATUS = "shadow_connection_preview_only"
MIN_JERK_MAX_DS = 1.875
MIN_JERK_MAX_D2S = 10.0 * math.sqrt(3.0) / 3.0


@dataclass(frozen=True)
class ConnectionConfig:
    version: str
    status: str
    joint_names: list[str]
    sample_rate_hz: float
    maximum_connection_velocity_rad_s: np.ndarray
    maximum_connection_acceleration_rad_s2: np.ndarray
    minimum_connection_duration_s: float
    maximum_connection_duration_s: float
    minimum_joint_limit_margin_rad: float
    current_joint_state_timeout_s: float
    source_valid_heartbeat_timeout_s: float
    minimum_tcp_z_m: float
    tcp_workspace_x_min_m: float
    tcp_workspace_x_max_m: float
    tcp_workspace_y_min_m: float
    tcp_workspace_y_max_m: float
    allow_direct_joint_connection_only: bool
    hardware_control_enabled: bool


@dataclass(frozen=True)
class ConnectionPlan:
    upstream_plan_id: str
    connection_plan_id: str
    q_current: np.ndarray
    q_start: np.ndarray
    per_joint_start_error_rad: np.ndarray
    connection_duration_s: float
    connection_point_count: int
    original_trajectory_point_count: int
    combined_trajectory: JointTrajectory
    maximum_connection_velocity_rad_s_observed: np.ndarray
    maximum_connection_acceleration_rad_s2_observed: np.ndarray


def load_connection_config(project_root: Path | str = DEFAULT_PROJECT_ROOT) -> ConnectionConfig:
    root = Path(project_root).resolve()
    config_path = root / "config" / "connection_trajectory.json"
    if not config_path.is_file():
        raise FileNotFoundError(f"Connection trajectory config not found: {config_path}")
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"Expected JSON object in {config_path}")
    config = ConnectionConfig(
        version=str(raw["version"]),
        status=str(raw["status"]),
        joint_names=[str(value) for value in raw["joint_names"]],
        sample_rate_hz=float(raw["sample_rate_hz"]),
        maximum_connection_velocity_rad_s=np.asarray(
            raw["maximum_connection_velocity_rad_s"],
            dtype=np.float64,
        ),
        maximum_connection_acceleration_rad_s2=np.asarray(
            raw["maximum_connection_acceleration_rad_s2"],
            dtype=np.float64,
        ),
        minimum_connection_duration_s=float(raw["minimum_connection_duration_s"]),
        maximum_connection_duration_s=float(raw["maximum_connection_duration_s"]),
        minimum_joint_limit_margin_rad=float(raw["minimum_joint_limit_margin_rad"]),
        current_joint_state_timeout_s=float(raw["current_joint_state_timeout_s"]),
        source_valid_heartbeat_timeout_s=float(raw["source_valid_heartbeat_timeout_s"]),
        minimum_tcp_z_m=float(raw["minimum_tcp_z_m"]),
        tcp_workspace_x_min_m=float(raw["tcp_workspace_x_min_m"]),
        tcp_workspace_x_max_m=float(raw["tcp_workspace_x_max_m"]),
        tcp_workspace_y_min_m=float(raw["tcp_workspace_y_min_m"]),
        tcp_workspace_y_max_m=float(raw["tcp_workspace_y_max_m"]),
        allow_direct_joint_connection_only=bool(raw["allow_direct_joint_connection_only"]),
        hardware_control_enabled=bool(raw["hardware_control_enabled"]),
    )
    validate_connection_config(config)
    return config


def validate_connection_config(config: ConnectionConfig) -> None:
    expected = list(ARM_JOINT_NAMES)
    if config.status != CONNECTION_STATUS:
        raise ValueError(f"connection status must be {CONNECTION_STATUS}")
    if config.joint_names != expected:
        raise ValueError(f"Expected joint_names {expected}, got {config.joint_names}")
    if config.hardware_control_enabled:
        raise ValueError("hardware_control_enabled must remain false")
    if not config.allow_direct_joint_connection_only:
        raise ValueError("allow_direct_joint_connection_only must remain true")
    for name, values in (
        ("maximum_connection_velocity_rad_s", config.maximum_connection_velocity_rad_s),
        ("maximum_connection_acceleration_rad_s2", config.maximum_connection_acceleration_rad_s2),
    ):
        if values.shape != (len(expected),):
            raise ValueError(f"{name} must have shape (5,), got {values.shape}")
        if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError(f"{name} values must be finite and positive")
    scalar_values = {
        "sample_rate_hz": config.sample_rate_hz,
        "minimum_connection_duration_s": config.minimum_connection_duration_s,
        "maximum_connection_duration_s": config.maximum_connection_duration_s,
        "minimum_joint_limit_margin_rad": config.minimum_joint_limit_margin_rad,
        "current_joint_state_timeout_s": config.current_joint_state_timeout_s,
        "source_valid_heartbeat_timeout_s": config.source_valid_heartbeat_timeout_s,
        "minimum_tcp_z_m": config.minimum_tcp_z_m,
    }
    for name, value in scalar_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if config.minimum_connection_duration_s > config.maximum_connection_duration_s:
        raise ValueError("minimum_connection_duration_s exceeds maximum_connection_duration_s")
    if config.tcp_workspace_x_min_m >= config.tcp_workspace_x_max_m:
        raise ValueError("TCP x workspace minimum must be below maximum")
    if config.tcp_workspace_y_min_m >= config.tcp_workspace_y_max_m:
        raise ValueError("TCP y workspace minimum must be below maximum")


def duration_from_seconds(seconds: float) -> Duration:
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError(f"Invalid duration: {seconds}")
    total_nanoseconds = int(round(seconds * 1.0e9))
    duration = Duration()
    duration.sec = total_nanoseconds // 1_000_000_000
    duration.nanosec = total_nanoseconds % 1_000_000_000
    return duration


def minimum_jerk_position(tau: np.ndarray) -> np.ndarray:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def minimum_jerk_velocity_scale(tau: np.ndarray) -> np.ndarray:
    return 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4


def minimum_jerk_acceleration_scale(tau: np.ndarray) -> np.ndarray:
    return 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3


def required_connection_duration_s(
    q_current: np.ndarray,
    q_start: np.ndarray,
    config: ConnectionConfig,
) -> float:
    dq_abs = np.abs(q_start - q_current)
    velocity_duration = float(
        np.max(dq_abs * MIN_JERK_MAX_DS / config.maximum_connection_velocity_rad_s)
    )
    acceleration_duration = float(
        math.sqrt(
            np.max(
                dq_abs
                * MIN_JERK_MAX_D2S
                / config.maximum_connection_acceleration_rad_s2
            )
        )
    )
    return max(
        config.minimum_connection_duration_s,
        velocity_duration,
        acceleration_duration,
    )


def connection_plan_id(
    *,
    upstream_plan_id: str,
    q_current: np.ndarray,
    q_start: np.ndarray,
    source_trajectory_signature: str,
    config: ConnectionConfig,
) -> str:
    payload = {
        "upstream_plan_id": upstream_plan_id,
        "q_current": [round(float(value), 12) for value in q_current.tolist()],
        "q_start": [round(float(value), 12) for value in q_start.tolist()],
        "source_trajectory_signature": source_trajectory_signature,
        "connection_config_version": config.version,
        "connection_status": config.status,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return "conn_" + hashlib.sha256(text.encode("utf-8")).hexdigest()[:20]


def trajectory_signature(trajectory: JointTrajectory) -> str:
    payload: dict[str, Any] = {
        "frame_id": str(trajectory.header.frame_id),
        "joint_names": [str(value) for value in trajectory.joint_names],
        "points": [],
    }
    for point in trajectory.points:
        payload["points"].append(
            {
                "positions": [round(float(value), 12) for value in point.positions],
                "velocities": [round(float(value), 12) for value in point.velocities],
                "accelerations": [round(float(value), 12) for value in point.accelerations],
                "time_from_start_s": round(seconds_from_duration(point.time_from_start), 12),
            }
        )
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def parameterize_connection(
    *,
    q_current: np.ndarray,
    source_trajectory: JointTrajectory,
    upstream_plan_id: str,
    config: ConnectionConfig,
) -> ConnectionPlan:
    q_current = np.asarray(q_current, dtype=np.float64)
    q_start = np.asarray(source_trajectory.points[0].positions, dtype=np.float64)
    duration_s = required_connection_duration_s(q_current, q_start, config)
    intervals = max(1, int(math.ceil(duration_s * config.sample_rate_hz)))
    times = np.linspace(0.0, duration_s, intervals + 1, dtype=np.float64)
    tau = times / duration_s
    dq = q_start - q_current
    s = minimum_jerk_position(tau).reshape(-1, 1)
    ds = minimum_jerk_velocity_scale(tau).reshape(-1, 1)
    d2s = minimum_jerk_acceleration_scale(tau).reshape(-1, 1)
    positions = q_current.reshape(1, -1) + s * dq.reshape(1, -1)
    velocities = ds * dq.reshape(1, -1) / duration_s
    accelerations = d2s * dq.reshape(1, -1) / (duration_s * duration_s)

    plan_id = connection_plan_id(
        upstream_plan_id=upstream_plan_id,
        q_current=q_current,
        q_start=q_start,
        source_trajectory_signature=trajectory_signature(source_trajectory),
        config=config,
    )
    combined = JointTrajectory()
    combined.header.stamp = source_trajectory.header.stamp
    combined.header.frame_id = (
        f"base_link;plan_id={plan_id};upstream_plan_id={upstream_plan_id}"
    )
    combined.joint_names = list(config.joint_names)

    for index, seconds in enumerate(times):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions[index]]
        point.velocities = [float(value) for value in velocities[index]]
        point.accelerations = [float(value) for value in accelerations[index]]
        point.time_from_start = duration_from_seconds(float(seconds))
        combined.points.append(point)

    for source_point in source_trajectory.points[1:]:
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in source_point.positions]
        point.velocities = [float(value) for value in source_point.velocities]
        point.accelerations = [float(value) for value in source_point.accelerations]
        point.time_from_start = duration_from_seconds(
            duration_s + seconds_from_duration(source_point.time_from_start)
        )
        combined.points.append(point)

    return ConnectionPlan(
        upstream_plan_id=upstream_plan_id,
        connection_plan_id=plan_id,
        q_current=q_current,
        q_start=q_start,
        per_joint_start_error_rad=np.abs(q_start - q_current),
        connection_duration_s=float(duration_s),
        connection_point_count=len(times),
        original_trajectory_point_count=len(source_trajectory.points),
        combined_trajectory=combined,
        maximum_connection_velocity_rad_s_observed=np.max(np.abs(velocities), axis=0),
        maximum_connection_acceleration_rad_s2_observed=np.max(np.abs(accelerations), axis=0),
    )
