from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from so101_kinematics.urdf_fk import ARM_JOINT_NAMES
from so101_trajectory_safety.trajectory_validator import load_joint_limits


DEFAULT_PROJECT_ROOT = (
    "E:/PycharmProjects/Embodied_AI/"
    "LeRobot_Project/so101_visual_tactile_grasp"
)
COMMAND_GATE_STATUS = "shadow_execution_only"
FORBIDDEN_CONTROLLER_TOPICS = [
    "/joint_trajectory_controller/joint_trajectory",
    "/joint_trajectory_controller/follow_joint_trajectory",
    "/arm_controller/command",
    "/robot_command",
    "/hardware_command",
]


@dataclass(frozen=True)
class CommandGateConfig:
    version: str
    status: str
    joint_names: list[str]
    current_joint_state_timeout_s: float
    source_valid_heartbeat_timeout_s: float
    maximum_start_state_error_rad: float
    minimum_joint_limit_margin_rad: float
    maximum_velocity_rad_s: np.ndarray
    maximum_acceleration_rad_s2: np.ndarray
    maximum_total_duration_s: float
    shadow_publish_rate_hz: float
    require_explicit_shadow_start: bool
    hardware_control_enabled: bool


@dataclass(frozen=True)
class GateValidationResult:
    ready: bool
    reason: str
    plan_id: str | None
    current_joint_state_age_s: float | None
    source_valid_heartbeat_age_s: float | None
    maximum_start_state_error_rad_observed: float | None
    minimum_current_joint_limit_margin_rad: float | None
    minimum_trajectory_joint_limit_margin_rad: float | None
    maximum_velocity_rad_s_observed: list[float]
    maximum_acceleration_rad_s2_observed: list[float]
    total_duration_s: float
    trajectory_point_count: int
    time_strictly_increasing: bool

    def status_dict(self, config: CommandGateConfig, timestamp: float) -> dict[str, Any]:
        status = "READY" if self.ready else "INVALID"
        reason = (
            "shadow_execution_candidate_valid"
            if self.ready
            else self.reason
        )
        return {
            "status": status,
            "reason": reason,
            "plan_id": self.plan_id,
            "current_joint_state_age_s": self.current_joint_state_age_s,
            "source_valid_heartbeat_age_s": self.source_valid_heartbeat_age_s,
            "maximum_start_state_error_rad_observed": (
                self.maximum_start_state_error_rad_observed
            ),
            "maximum_start_state_error_rad_limit": (
                config.maximum_start_state_error_rad
            ),
            "minimum_current_joint_limit_margin_rad": (
                self.minimum_current_joint_limit_margin_rad
            ),
            "minimum_trajectory_joint_limit_margin_rad": (
                self.minimum_trajectory_joint_limit_margin_rad
            ),
            "maximum_velocity_rad_s_observed": (
                self.maximum_velocity_rad_s_observed
            ),
            "maximum_acceleration_rad_s2_observed": (
                self.maximum_acceleration_rad_s2_observed
            ),
            "total_duration_s": self.total_duration_s,
            "trajectory_point_count": self.trajectory_point_count,
            "time_strictly_increasing": self.time_strictly_increasing,
            "hardware_control_enabled": False,
            "command_topics_published": [],
            "shadow_execution_only": True,
            "timestamp": timestamp,
            "status_profile": COMMAND_GATE_STATUS,
            "disabled_controller_command_topics": list(FORBIDDEN_CONTROLLER_TOPICS),
        }


def load_command_gate_config(project_root: Path | str) -> CommandGateConfig:
    root = Path(project_root).resolve()
    config_path = root / "config" / "command_gate.json"
    if not config_path.is_file():
        raise FileNotFoundError(f"Command gate config not found: {config_path}")
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"Expected JSON object in {config_path}")
    config = CommandGateConfig(
        version=str(raw["version"]),
        status=str(raw["status"]),
        joint_names=[str(value) for value in raw["joint_names"]],
        current_joint_state_timeout_s=float(raw["current_joint_state_timeout_s"]),
        source_valid_heartbeat_timeout_s=float(
            raw["source_valid_heartbeat_timeout_s"]
        ),
        maximum_start_state_error_rad=float(
            raw["maximum_start_state_error_rad"]
        ),
        minimum_joint_limit_margin_rad=float(raw["minimum_joint_limit_margin_rad"]),
        maximum_velocity_rad_s=np.asarray(
            raw["maximum_velocity_rad_s"],
            dtype=np.float64,
        ),
        maximum_acceleration_rad_s2=np.asarray(
            raw["maximum_acceleration_rad_s2"],
            dtype=np.float64,
        ),
        maximum_total_duration_s=float(raw["maximum_total_duration_s"]),
        shadow_publish_rate_hz=float(raw["shadow_publish_rate_hz"]),
        require_explicit_shadow_start=bool(raw["require_explicit_shadow_start"]),
        hardware_control_enabled=bool(raw["hardware_control_enabled"]),
    )
    validate_command_gate_config(config)
    return config


def validate_command_gate_config(config: CommandGateConfig) -> None:
    expected = list(ARM_JOINT_NAMES)
    if config.status != COMMAND_GATE_STATUS:
        raise ValueError(
            f"command_gate.json status must be {COMMAND_GATE_STATUS}, "
            f"got {config.status}"
        )
    if config.joint_names != expected:
        raise ValueError(f"Expected joint_names {expected}, got {config.joint_names}")
    if config.hardware_control_enabled:
        raise ValueError("hardware_control_enabled must remain false")
    if not config.require_explicit_shadow_start:
        raise ValueError("require_explicit_shadow_start must remain true")
    for name, values in (
        ("maximum_velocity_rad_s", config.maximum_velocity_rad_s),
        ("maximum_acceleration_rad_s2", config.maximum_acceleration_rad_s2),
    ):
        if values.shape != (len(expected),):
            raise ValueError(f"{name} must have shape (5,), got {values.shape}")
        if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError(f"{name} values must be finite and positive")
    scalar_values = {
        "current_joint_state_timeout_s": config.current_joint_state_timeout_s,
        "source_valid_heartbeat_timeout_s": config.source_valid_heartbeat_timeout_s,
        "maximum_start_state_error_rad": config.maximum_start_state_error_rad,
        "minimum_joint_limit_margin_rad": config.minimum_joint_limit_margin_rad,
        "maximum_total_duration_s": config.maximum_total_duration_s,
        "shadow_publish_rate_hz": config.shadow_publish_rate_hz,
    }
    for name, value in scalar_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")


def load_command_gate_joint_limits(
    project_root: Path | str,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    return load_joint_limits(project_root)


def seconds_from_duration(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def joint_limit_margin(
    positions: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> float:
    lower_margin = positions - lower.reshape(1, -1)
    upper_margin = upper.reshape(1, -1) - positions
    return float(np.min(np.minimum(lower_margin, upper_margin)))


def parse_plan_id_from_frame_id(frame_id: str) -> str | None:
    for token in str(frame_id).replace("|", ";").split(";"):
        token = token.strip()
        if token.startswith("plan_id="):
            value = token.split("=", 1)[1].strip()
            return value or None
    return None


def extract_status_plan_id(status: dict[str, Any] | None) -> str | None:
    if not isinstance(status, dict):
        return None
    for key in ("plan_id", "active_plan_id"):
        value = status.get(key)
        if value is not None:
            text = str(value)
            if text:
                return text
    return None


def reorder_joint_state_positions(
    message: JointState,
    joint_names: list[str],
) -> tuple[np.ndarray | None, str]:
    names = [str(value) for value in message.name]
    if len(set(names)) != len(names):
        return None, "current_joint_state_duplicate_names"
    missing = [name for name in joint_names if name not in names]
    if missing:
        return None, "current_joint_state_missing_joint"
    if len(message.position) < len(names):
        return None, "current_joint_state_wrong_position_length"
    positions_by_name = {
        name: float(message.position[index])
        for index, name in enumerate(names)
    }
    ordered = np.asarray([positions_by_name[name] for name in joint_names], dtype=np.float64)
    if not np.all(np.isfinite(ordered)):
        return ordered, "current_joint_state_non_finite"
    return ordered, "ok"


def make_invalid(
    reason: str,
    plan_id: str | None,
    current_joint_state_age_s: float | None,
    source_valid_heartbeat_age_s: float | None,
    maximum_start_state_error_rad_observed: float | None = None,
    minimum_current_joint_limit_margin_rad: float | None = None,
    minimum_trajectory_joint_limit_margin_rad: float | None = None,
    maximum_velocity_rad_s_observed: list[float] | None = None,
    maximum_acceleration_rad_s2_observed: list[float] | None = None,
    total_duration_s: float = 0.0,
    trajectory_point_count: int = 0,
    time_strictly_increasing: bool = False,
) -> GateValidationResult:
    return GateValidationResult(
        ready=False,
        reason=reason,
        plan_id=plan_id,
        current_joint_state_age_s=current_joint_state_age_s,
        source_valid_heartbeat_age_s=source_valid_heartbeat_age_s,
        maximum_start_state_error_rad_observed=maximum_start_state_error_rad_observed,
        minimum_current_joint_limit_margin_rad=minimum_current_joint_limit_margin_rad,
        minimum_trajectory_joint_limit_margin_rad=minimum_trajectory_joint_limit_margin_rad,
        maximum_velocity_rad_s_observed=maximum_velocity_rad_s_observed or [0.0] * 5,
        maximum_acceleration_rad_s2_observed=(
            maximum_acceleration_rad_s2_observed or [0.0] * 5
        ),
        total_duration_s=total_duration_s,
        trajectory_point_count=trajectory_point_count,
        time_strictly_increasing=time_strictly_increasing,
    )


def validate_command_candidate(
    *,
    safe_timed_valid: bool,
    safe_timed_status: dict[str, Any] | None,
    trajectory: JointTrajectory | None,
    joint_state: JointState | None,
    current_joint_state_age_s: float | None,
    source_valid_heartbeat_age_s: float | None,
    config: CommandGateConfig,
    lower: np.ndarray,
    upper: np.ndarray,
) -> GateValidationResult:
    status_plan_id = extract_status_plan_id(safe_timed_status)

    if not safe_timed_valid:
        return make_invalid(
            "safe_timed_grasp_invalid",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
        )
    if source_valid_heartbeat_age_s is None:
        return make_invalid(
            "waiting_for_safe_timed_status",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
        )
    if source_valid_heartbeat_age_s > config.source_valid_heartbeat_timeout_s:
        return make_invalid(
            "source_valid_heartbeat_stale",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
        )
    if not isinstance(safe_timed_status, dict) or safe_timed_status.get("status") != "VALID":
        return make_invalid(
            "safe_timed_status_not_valid",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
        )
    if status_plan_id is None:
        return make_invalid(
            "safe_timed_status_plan_id_missing",
            None,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
        )
    if trajectory is None:
        return make_invalid(
            "waiting_for_safe_timed_trajectory",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
        )
    trajectory_plan_id = parse_plan_id_from_frame_id(trajectory.header.frame_id)
    if trajectory_plan_id is None:
        return make_invalid(
            "trajectory_plan_id_missing",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )
    if trajectory_plan_id != status_plan_id:
        return make_invalid(
            "plan_id_mismatch",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )
    if joint_state is None:
        return make_invalid(
            "waiting_for_current_joint_state",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )
    if current_joint_state_age_s is None:
        return make_invalid(
            "waiting_for_current_joint_state",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )
    if current_joint_state_age_s > config.current_joint_state_timeout_s:
        return make_invalid(
            "current_joint_state_stale",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )

    current_positions, current_reason = reorder_joint_state_positions(
        joint_state,
        config.joint_names,
    )
    if current_positions is None or current_reason != "ok":
        return make_invalid(
            current_reason,
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )
    if np.any(current_positions < lower - 1.0e-10) or np.any(current_positions > upper + 1.0e-10):
        return make_invalid(
            "current_joint_state_out_of_bounds",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            trajectory_point_count=len(trajectory.points),
        )
    current_margin = float(
        np.min(np.minimum(current_positions - lower, upper - current_positions))
    )
    if current_margin < config.minimum_joint_limit_margin_rad:
        return make_invalid(
            "current_joint_limit_margin_insufficient",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            minimum_current_joint_limit_margin_rad=current_margin,
            trajectory_point_count=len(trajectory.points),
        )

    if list(trajectory.joint_names) != config.joint_names:
        reason = (
            "trajectory_wrong_joint_order"
            if sorted(trajectory.joint_names) == sorted(config.joint_names)
            else "trajectory_wrong_joint_names"
        )
        return make_invalid(
            reason,
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            minimum_current_joint_limit_margin_rad=current_margin,
            trajectory_point_count=len(trajectory.points),
        )
    if len(trajectory.points) < 2:
        return make_invalid(
            "trajectory_too_few_points",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            minimum_current_joint_limit_margin_rad=current_margin,
            trajectory_point_count=len(trajectory.points),
        )

    positions: list[list[float]] = []
    velocities: list[list[float]] = []
    accelerations: list[list[float]] = []
    times: list[float] = []
    for point in trajectory.points:
        if (
            len(point.positions) != len(config.joint_names)
            or len(point.velocities) != len(config.joint_names)
            or len(point.accelerations) != len(config.joint_names)
        ):
            return make_invalid(
                "trajectory_point_wrong_length",
                status_plan_id,
                current_joint_state_age_s,
                source_valid_heartbeat_age_s,
                minimum_current_joint_limit_margin_rad=current_margin,
                trajectory_point_count=len(trajectory.points),
            )
        positions.append([float(value) for value in point.positions])
        velocities.append([float(value) for value in point.velocities])
        accelerations.append([float(value) for value in point.accelerations])
        times.append(seconds_from_duration(point.time_from_start))

    positions_array = np.asarray(positions, dtype=np.float64)
    velocities_array = np.asarray(velocities, dtype=np.float64)
    accelerations_array = np.asarray(accelerations, dtype=np.float64)
    times_array = np.asarray(times, dtype=np.float64)
    max_velocity = np.max(np.abs(velocities_array), axis=0).tolist()
    max_acceleration = np.max(np.abs(accelerations_array), axis=0).tolist()

    if (
        not np.all(np.isfinite(positions_array))
        or not np.all(np.isfinite(velocities_array))
        or not np.all(np.isfinite(accelerations_array))
        or not np.all(np.isfinite(times_array))
    ):
        return make_invalid(
            "trajectory_non_finite_values",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            minimum_current_joint_limit_margin_rad=current_margin,
            maximum_velocity_rad_s_observed=max_velocity,
            maximum_acceleration_rad_s2_observed=max_acceleration,
            trajectory_point_count=len(trajectory.points),
        )

    time_strict = bool(np.all(np.diff(times_array) > 0.0))
    total_duration = float(times_array[-1])
    trajectory_margin = joint_limit_margin(positions_array, lower, upper)
    start_error = float(np.max(np.abs(current_positions - positions_array[0])))

    if abs(float(times_array[0])) > 1.0e-12:
        return make_invalid(
            "trajectory_first_time_not_zero",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )
    if not time_strict:
        return make_invalid(
            "trajectory_time_not_strictly_increasing",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            False,
        )
    if total_duration <= 0.0 or total_duration > config.maximum_total_duration_s:
        return make_invalid(
            "trajectory_total_duration_invalid",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )
    if np.any(positions_array < lower.reshape(1, -1) - 1.0e-10) or np.any(
        positions_array > upper.reshape(1, -1) + 1.0e-10
    ):
        return make_invalid(
            "trajectory_joint_position_out_of_bounds",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )
    if trajectory_margin < config.minimum_joint_limit_margin_rad:
        return make_invalid(
            "trajectory_joint_limit_margin_insufficient",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )
    if np.any(np.asarray(max_velocity) > config.maximum_velocity_rad_s + 1.0e-12):
        return make_invalid(
            "trajectory_velocity_limit_exceeded",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )
    if np.any(np.asarray(max_acceleration) > config.maximum_acceleration_rad_s2 + 1.0e-12):
        return make_invalid(
            "trajectory_acceleration_limit_exceeded",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )
    if start_error > config.maximum_start_state_error_rad:
        return make_invalid(
            "start_state_mismatch",
            status_plan_id,
            current_joint_state_age_s,
            source_valid_heartbeat_age_s,
            start_error,
            current_margin,
            trajectory_margin,
            max_velocity,
            max_acceleration,
            total_duration,
            len(trajectory.points),
            time_strict,
        )

    return GateValidationResult(
        ready=True,
        reason="shadow_execution_candidate_valid",
        plan_id=status_plan_id,
        current_joint_state_age_s=current_joint_state_age_s,
        source_valid_heartbeat_age_s=source_valid_heartbeat_age_s,
        maximum_start_state_error_rad_observed=start_error,
        minimum_current_joint_limit_margin_rad=current_margin,
        minimum_trajectory_joint_limit_margin_rad=trajectory_margin,
        maximum_velocity_rad_s_observed=max_velocity,
        maximum_acceleration_rad_s2_observed=max_acceleration,
        total_duration_s=total_duration,
        trajectory_point_count=len(trajectory.points),
        time_strictly_increasing=True,
    )
