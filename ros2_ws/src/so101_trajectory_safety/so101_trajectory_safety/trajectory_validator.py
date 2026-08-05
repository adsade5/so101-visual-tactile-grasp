from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from so101_kinematics.top_down_ik import create_default_solver
from so101_kinematics.urdf_fk import ARM_JOINT_NAMES


DEFAULT_PROJECT_ROOT = (
    "E:/PycharmProjects/Embodied_AI/"
    "LeRobot_Project/so101_visual_tactile_grasp"
)
LIMIT_PROFILE_STATUS = "provisional_software_preview_limits"


@dataclass(frozen=True)
class TrajectorySafetyConfig:
    version: str
    status: str
    joint_names: list[str]
    maximum_velocity_rad_s: np.ndarray
    maximum_acceleration_rad_s2: np.ndarray
    sample_rate_hz: float
    minimum_segment_duration_s: float
    maximum_segment_duration_s: float
    maximum_total_duration_s: float
    maximum_input_adjacent_delta_rad: float
    minimum_joint_limit_margin_rad: float
    input_stale_timeout_s: float
    source_valid_heartbeat_timeout_s: float
    trajectory_payload_timeout_before_first_plan_s: float
    require_periodic_trajectory_republish: bool
    stop_at_each_source_waypoint: bool


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reason: str
    positions: np.ndarray | None
    minimum_joint_limit_margin_rad: float | None
    maximum_input_adjacent_delta_rad: float | None
    all_positions_finite: bool


def load_safety_config(project_root: Path | str) -> TrajectorySafetyConfig:
    root = Path(project_root).resolve()
    config_path = root / "config" / "trajectory_safety.json"
    if not config_path.is_file():
        raise FileNotFoundError(f"Trajectory safety config not found: {config_path}")
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"Expected JSON object in {config_path}")
    config = TrajectorySafetyConfig(
        version=str(raw["version"]),
        status=str(raw["status"]),
        joint_names=[str(value) for value in raw["joint_names"]],
        maximum_velocity_rad_s=np.asarray(
            raw["maximum_velocity_rad_s"],
            dtype=np.float64,
        ),
        maximum_acceleration_rad_s2=np.asarray(
            raw["maximum_acceleration_rad_s2"],
            dtype=np.float64,
        ),
        sample_rate_hz=float(raw["sample_rate_hz"]),
        minimum_segment_duration_s=float(raw["minimum_segment_duration_s"]),
        maximum_segment_duration_s=float(raw["maximum_segment_duration_s"]),
        maximum_total_duration_s=float(raw["maximum_total_duration_s"]),
        maximum_input_adjacent_delta_rad=float(raw["maximum_input_adjacent_delta_rad"]),
        minimum_joint_limit_margin_rad=float(raw["minimum_joint_limit_margin_rad"]),
        input_stale_timeout_s=float(raw["input_stale_timeout_s"]),
        source_valid_heartbeat_timeout_s=float(
            raw.get(
                "source_valid_heartbeat_timeout_s",
                raw["input_stale_timeout_s"],
            )
        ),
        trajectory_payload_timeout_before_first_plan_s=float(
            raw.get(
                "trajectory_payload_timeout_before_first_plan_s",
                raw["input_stale_timeout_s"],
            )
        ),
        require_periodic_trajectory_republish=bool(
            raw.get("require_periodic_trajectory_republish", False)
        ),
        stop_at_each_source_waypoint=bool(raw["stop_at_each_source_waypoint"]),
    )
    validate_safety_config(config)
    return config


def validate_safety_config(config: TrajectorySafetyConfig) -> None:
    expected_joint_names = list(ARM_JOINT_NAMES)
    if config.status != LIMIT_PROFILE_STATUS:
        raise ValueError(
            "trajectory_safety.json status must be "
            f"{LIMIT_PROFILE_STATUS}, got {config.status}"
        )
    if config.joint_names != expected_joint_names:
        raise ValueError(
            f"Unexpected joint_names. Expected {expected_joint_names}, "
            f"got {config.joint_names}"
        )
    for name, values in (
        ("maximum_velocity_rad_s", config.maximum_velocity_rad_s),
        ("maximum_acceleration_rad_s2", config.maximum_acceleration_rad_s2),
    ):
        if values.shape != (len(expected_joint_names),):
            raise ValueError(f"{name} shape must be (5,), got {values.shape}")
        if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
            raise ValueError(f"{name} values must be finite positive values")
    scalar_checks = {
        "sample_rate_hz": config.sample_rate_hz,
        "minimum_segment_duration_s": config.minimum_segment_duration_s,
        "maximum_segment_duration_s": config.maximum_segment_duration_s,
        "maximum_total_duration_s": config.maximum_total_duration_s,
        "maximum_input_adjacent_delta_rad": config.maximum_input_adjacent_delta_rad,
        "minimum_joint_limit_margin_rad": config.minimum_joint_limit_margin_rad,
        "input_stale_timeout_s": config.input_stale_timeout_s,
        "source_valid_heartbeat_timeout_s": config.source_valid_heartbeat_timeout_s,
        "trajectory_payload_timeout_before_first_plan_s": (
            config.trajectory_payload_timeout_before_first_plan_s
        ),
    }
    for name, value in scalar_checks.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if config.minimum_segment_duration_s > config.maximum_segment_duration_s:
        raise ValueError("minimum_segment_duration_s exceeds maximum_segment_duration_s")
    if not config.stop_at_each_source_waypoint:
        raise ValueError("stop_at_each_source_waypoint must remain true for Stage 2D-2")


def load_joint_limits(project_root: Path | str) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    solver, metadata = create_default_solver(Path(project_root).resolve())
    return solver.lower.copy(), solver.upper.copy(), metadata


def compute_margin(
    positions: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> float:
    lower_margin = positions - lower.reshape(1, -1)
    upper_margin = upper.reshape(1, -1) - positions
    return float(np.min(np.minimum(lower_margin, upper_margin)))


def compute_max_adjacent_delta(positions: np.ndarray) -> float:
    if len(positions) < 2:
        return 0.0
    return float(np.max(np.abs(np.diff(positions, axis=0))))


def validate_source_path(
    joint_names: list[str],
    raw_positions: list[list[float]],
    config: TrajectorySafetyConfig,
    lower: np.ndarray,
    upper: np.ndarray,
) -> ValidationResult:
    if joint_names != config.joint_names:
        if sorted(joint_names) == sorted(config.joint_names):
            reason = "wrong_joint_order"
        else:
            reason = "wrong_joint_names"
        return ValidationResult(False, reason, None, None, None, False)

    if len(raw_positions) < 2:
        return ValidationResult(
            False,
            "fewer_than_two_waypoints",
            None,
            None,
            None,
            False,
        )

    if any(len(point) != len(config.joint_names) for point in raw_positions):
        return ValidationResult(
            False,
            "wrong_position_length",
            None,
            None,
            None,
            False,
        )

    try:
        positions = np.asarray(raw_positions, dtype=np.float64)
    except (TypeError, ValueError):
        return ValidationResult(
            False,
            "positions_not_numeric",
            None,
            None,
            None,
            False,
        )

    if positions.shape != (len(raw_positions), len(config.joint_names)):
        return ValidationResult(
            False,
            "wrong_position_shape",
            None,
            None,
            None,
            False,
        )

    all_positions_finite = bool(np.all(np.isfinite(positions)))
    if not all_positions_finite:
        return ValidationResult(
            False,
            "non_finite_position",
            positions,
            None,
            None,
            False,
        )

    tolerance = 1.0e-10
    if np.any(positions < lower.reshape(1, -1) - tolerance) or np.any(
        positions > upper.reshape(1, -1) + tolerance
    ):
        return ValidationResult(
            False,
            "joint_position_out_of_bounds",
            positions,
            compute_margin(positions, lower, upper),
            compute_max_adjacent_delta(positions),
            True,
        )

    margin = compute_margin(positions, lower, upper)
    if margin < config.minimum_joint_limit_margin_rad:
        return ValidationResult(
            False,
            "joint_limit_margin_insufficient",
            positions,
            margin,
            compute_max_adjacent_delta(positions),
            True,
        )

    adjacent_delta = compute_max_adjacent_delta(positions)
    if adjacent_delta > config.maximum_input_adjacent_delta_rad:
        return ValidationResult(
            False,
            "input_adjacent_delta_exceeds_limit",
            positions,
            margin,
            adjacent_delta,
            True,
        )

    return ValidationResult(
        True,
        "source_path_valid",
        positions,
        margin,
        adjacent_delta,
        True,
    )
