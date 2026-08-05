from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .trajectory_validator import TrajectorySafetyConfig


MIN_JERK_MAX_S_DOT = 1.875
MIN_JERK_MAX_ABS_S_DDOT = 10.0 / math.sqrt(3.0)


@dataclass(frozen=True)
class TimedTrajectoryPoint:
    time_from_start_s: float
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray


@dataclass(frozen=True)
class TimedTrajectoryResult:
    success: bool
    reason: str
    points: list[TimedTrajectoryPoint]
    source_waypoint_times_s: list[float]
    segment_durations_s: list[float]
    total_duration_s: float
    maximum_velocity_rad_s_observed: list[float]
    maximum_acceleration_rad_s2_observed: list[float]
    time_strictly_increasing: bool
    all_positions_finite: bool
    all_velocities_finite: bool
    all_accelerations_finite: bool


def minimum_jerk_s(tau: float) -> float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def minimum_jerk_s_dot(tau: float) -> float:
    return 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4


def minimum_jerk_s_ddot(tau: float) -> float:
    return 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3


def finite_flags(points: list[TimedTrajectoryPoint]) -> tuple[bool, bool, bool]:
    return (
        bool(all(np.all(np.isfinite(point.positions)) for point in points)),
        bool(all(np.all(np.isfinite(point.velocities)) for point in points)),
        bool(all(np.all(np.isfinite(point.accelerations)) for point in points)),
    )


def strictly_increasing(points: list[TimedTrajectoryPoint]) -> bool:
    if not points:
        return False
    previous = -math.inf
    for point in points:
        current = float(point.time_from_start_s)
        if current <= previous:
            return False
        previous = current
    return True


def observed_limits(points: list[TimedTrajectoryPoint], joint_count: int) -> tuple[list[float], list[float]]:
    if not points:
        zeros = [0.0] * joint_count
        return zeros, zeros
    velocities = np.asarray([np.abs(point.velocities) for point in points], dtype=np.float64)
    accelerations = np.asarray(
        [np.abs(point.accelerations) for point in points],
        dtype=np.float64,
    )
    return (
        [float(value) for value in np.max(velocities, axis=0).tolist()],
        [float(value) for value in np.max(accelerations, axis=0).tolist()],
    )


def estimate_segment_duration(
    q0: np.ndarray,
    q1: np.ndarray,
    config: TrajectorySafetyConfig,
) -> float:
    dq_abs = np.abs(q1 - q0)
    velocity_time = np.max(
        dq_abs * MIN_JERK_MAX_S_DOT / config.maximum_velocity_rad_s
    )
    acceleration_time = np.max(
        np.sqrt(dq_abs * MIN_JERK_MAX_ABS_S_DDOT / config.maximum_acceleration_rad_s2)
    )
    return float(
        max(
            config.minimum_segment_duration_s,
            float(velocity_time),
            float(acceleration_time),
        )
    )


def sample_segment(
    q0: np.ndarray,
    q1: np.ndarray,
    duration_s: float,
    start_time_s: float,
    sample_rate_hz: float,
    include_start: bool,
) -> list[TimedTrajectoryPoint]:
    dq = q1 - q0
    step_count = max(1, int(math.ceil(duration_s * sample_rate_hz)))
    first_index = 0 if include_start else 1
    points: list[TimedTrajectoryPoint] = []
    for index in range(first_index, step_count + 1):
        tau = float(index) / float(step_count)
        local_time = tau * duration_s
        s = minimum_jerk_s(tau)
        s_dot = minimum_jerk_s_dot(tau)
        s_ddot = minimum_jerk_s_ddot(tau)
        positions = q0 + s * dq
        velocities = (dq / duration_s) * s_dot
        accelerations = (dq / (duration_s * duration_s)) * s_ddot
        if index == 0 or index == step_count:
            velocities = np.zeros_like(q0, dtype=np.float64)
            accelerations = np.zeros_like(q0, dtype=np.float64)
        points.append(
            TimedTrajectoryPoint(
                time_from_start_s=float(start_time_s + local_time),
                positions=positions.astype(np.float64),
                velocities=velocities.astype(np.float64),
                accelerations=accelerations.astype(np.float64),
            )
        )
    return points


def grow_duration_until_limits_pass(
    q0: np.ndarray,
    q1: np.ndarray,
    config: TrajectorySafetyConfig,
) -> tuple[bool, str, float, list[TimedTrajectoryPoint]]:
    duration = estimate_segment_duration(q0, q1, config)
    if duration > config.maximum_segment_duration_s:
        return False, "segment_duration_exceeds_limit", duration, []

    for _ in range(24):
        points = sample_segment(
            q0=q0,
            q1=q1,
            duration_s=duration,
            start_time_s=0.0,
            sample_rate_hz=config.sample_rate_hz,
            include_start=True,
        )
        velocity_observed, acceleration_observed = observed_limits(points, len(q0))
        velocity_ratio = float(
            np.max(np.asarray(velocity_observed) / config.maximum_velocity_rad_s)
        )
        acceleration_ratio = float(
            np.max(
                np.asarray(acceleration_observed)
                / config.maximum_acceleration_rad_s2
            )
        )
        if velocity_ratio <= 1.0 + 1.0e-12 and acceleration_ratio <= 1.0 + 1.0e-12:
            return True, "segment_valid", duration, points
        growth = max(velocity_ratio, math.sqrt(max(0.0, acceleration_ratio))) * 1.02
        duration *= max(1.02, growth)
        if duration > config.maximum_segment_duration_s:
            return False, "segment_duration_exceeds_limit", duration, []

    return False, "parameterization_iteration_limit", duration, []


def parameterize_path(
    source_positions: np.ndarray,
    config: TrajectorySafetyConfig,
) -> TimedTrajectoryResult:
    positions = np.asarray(source_positions, dtype=np.float64)
    joint_count = positions.shape[1] if len(positions.shape) == 2 else 0
    all_points: list[TimedTrajectoryPoint] = []
    source_waypoint_times = [0.0]
    segment_durations: list[float] = []
    current_time = 0.0

    if positions.shape[0] < 2 or joint_count <= 0:
        return TimedTrajectoryResult(
            success=False,
            reason="invalid_source_path_shape",
            points=[],
            source_waypoint_times_s=[],
            segment_durations_s=[],
            total_duration_s=0.0,
            maximum_velocity_rad_s_observed=[],
            maximum_acceleration_rad_s2_observed=[],
            time_strictly_increasing=False,
            all_positions_finite=False,
            all_velocities_finite=False,
            all_accelerations_finite=False,
        )

    for segment_index in range(positions.shape[0] - 1):
        q0 = positions[segment_index]
        q1 = positions[segment_index + 1]
        success, reason, duration, _ = grow_duration_until_limits_pass(q0, q1, config)
        if not success:
            return finish_failure(
                reason,
                all_points,
                source_waypoint_times,
                segment_durations,
                current_time,
                joint_count,
            )
        if current_time + duration > config.maximum_total_duration_s:
            return finish_failure(
                "total_duration_exceeds_limit",
                all_points,
                source_waypoint_times,
                segment_durations + [duration],
                current_time + duration,
                joint_count,
            )
        sampled = sample_segment(
            q0=q0,
            q1=q1,
            duration_s=duration,
            start_time_s=current_time,
            sample_rate_hz=config.sample_rate_hz,
            include_start=(segment_index == 0),
        )
        all_points.extend(sampled)
        current_time += duration
        segment_durations.append(float(duration))
        source_waypoint_times.append(float(current_time))

    velocity_observed, acceleration_observed = observed_limits(all_points, joint_count)
    all_positions, all_velocities, all_accelerations = finite_flags(all_points)
    return TimedTrajectoryResult(
        success=bool(
            strictly_increasing(all_points)
            and all_positions
            and all_velocities
            and all_accelerations
        ),
        reason="time_parameterized_preview_valid",
        points=all_points,
        source_waypoint_times_s=source_waypoint_times,
        segment_durations_s=segment_durations,
        total_duration_s=float(current_time),
        maximum_velocity_rad_s_observed=velocity_observed,
        maximum_acceleration_rad_s2_observed=acceleration_observed,
        time_strictly_increasing=strictly_increasing(all_points),
        all_positions_finite=all_positions,
        all_velocities_finite=all_velocities,
        all_accelerations_finite=all_accelerations,
    )


def finish_failure(
    reason: str,
    points: list[TimedTrajectoryPoint],
    source_waypoint_times: list[float],
    segment_durations: list[float],
    current_time: float,
    joint_count: int,
) -> TimedTrajectoryResult:
    velocity_observed, acceleration_observed = observed_limits(points, joint_count)
    all_positions, all_velocities, all_accelerations = finite_flags(points)
    return TimedTrajectoryResult(
        success=False,
        reason=reason,
        points=points,
        source_waypoint_times_s=source_waypoint_times,
        segment_durations_s=segment_durations,
        total_duration_s=float(current_time),
        maximum_velocity_rad_s_observed=velocity_observed,
        maximum_acceleration_rad_s2_observed=acceleration_observed,
        time_strictly_increasing=strictly_increasing(points),
        all_positions_finite=all_positions,
        all_velocities_finite=all_velocities,
        all_accelerations_finite=all_accelerations,
    )
