from __future__ import annotations

import numpy as np

from so101_mvp_kinematics.model import So101KinematicModel


def _points(trajectory: dict[str, object]) -> list[object]:
    return list(trajectory.get("points", []))


def _reason(success: bool, reason: str, **extra: object) -> dict[str, object]:
    result = {"success": success, "reason": reason}
    result.update(extra)
    return result


def validate_trajectory(
    trajectory: dict[str, object],
    model: So101KinematicModel,
    maximum_speed_rad_s: float,
    maximum_acceleration_rad_s2: float,
    maximum_total_duration_s: float,
) -> dict[str, object]:
    joint_names = list(trajectory.get("joint_names", []))
    points = _points(trajectory)

    if joint_names != model.joint_names:
        return _reason(False, "invalid_input")
    if len(points) < 2:
        return _reason(False, "invalid_input")

    start = np.asarray(trajectory.get("start_positions_rad"), dtype=np.float64)
    target = np.asarray(trajectory.get("target_positions_rad"), dtype=np.float64)
    if start.shape != (len(joint_names),) or target.shape != (len(joint_names),):
        return _reason(False, "invalid_input")
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(target)):
        return _reason(False, "invalid_input")
    if np.any(start < model.lower_limits) or np.any(start > model.upper_limits):
        return _reason(False, "invalid_start_joint_limits")
    if np.any(target < model.lower_limits) or np.any(target > model.upper_limits):
        return _reason(False, "invalid_target_joint_limits")

    times: list[float] = []
    max_speed = 0.0
    max_acceleration = 0.0
    max_moving = 0
    all_within_limits = True
    finite = True

    for point in points:
        time_s = float(point.time_s)
        positions = np.asarray(point.positions_rad, dtype=np.float64)
        velocities = np.asarray(point.velocities_rad_s, dtype=np.float64)
        accelerations = np.asarray(point.accelerations_rad_s2, dtype=np.float64)
        if (
            positions.shape != (len(joint_names),)
            or velocities.shape != (len(joint_names),)
            or accelerations.shape != (len(joint_names),)
        ):
            return _reason(False, "invalid_input")
        if not (
            np.isfinite(time_s)
            and np.all(np.isfinite(positions))
            and np.all(np.isfinite(velocities))
            and np.all(np.isfinite(accelerations))
        ):
            finite = False
        if np.any(positions < model.lower_limits - 1.0e-12) or np.any(
            positions > model.upper_limits + 1.0e-12
        ):
            all_within_limits = False
        max_speed = max(max_speed, float(np.max(np.abs(velocities))))
        max_acceleration = max(max_acceleration, float(np.max(np.abs(accelerations))))
        max_moving = max(max_moving, int(np.count_nonzero(np.abs(velocities) > 1.0e-10)))
        times.append(time_s)

    if not finite:
        return _reason(False, "invalid_input")
    time_strict = all(after > before for before, after in zip(times, times[1:]))
    if not time_strict:
        return _reason(False, "invalid_input")

    first = np.asarray(points[0].positions_rad, dtype=np.float64)
    last = np.asarray(points[-1].positions_rad, dtype=np.float64)
    exact_start = bool(np.allclose(first, start, rtol=0.0, atol=0.0))
    exact_target = bool(np.allclose(last, target, rtol=0.0, atol=0.0))
    if not exact_start or not exact_target:
        return _reason(False, "invalid_input", exact_start=exact_start, exact_target=exact_target)

    if not all_within_limits:
        return _reason(False, "trajectory_joint_limit_violation")
    if max_speed > maximum_speed_rad_s + 1.0e-12:
        return _reason(False, "speed_limit_exceeded", maximum_speed_rad_s_observed=max_speed)
    if max_acceleration > maximum_acceleration_rad_s2 + 1.0e-12:
        return _reason(
            False,
            "acceleration_limit_exceeded",
            maximum_acceleration_rad_s2_observed=max_acceleration,
        )
    if max_moving > 1:
        return _reason(False, "invalid_input", maximum_simultaneously_moving_joints=max_moving)

    total_duration = times[-1] - times[0]
    if total_duration > maximum_total_duration_s + 1.0e-12:
        return _reason(False, "duration_limit_exceeded", total_duration_s=total_duration)

    return _reason(
        True,
        "valid",
        point_count=len(points),
        total_duration_s=float(total_duration),
        maximum_velocity_rad_s_observed=float(max_speed),
        maximum_acceleration_rad_s2_observed=float(max_acceleration),
        maximum_simultaneously_moving_joints=int(max_moving),
        time_strictly_increasing=bool(time_strict),
        all_points_within_urdf_limits=bool(all_within_limits),
        exact_final_target=bool(exact_target),
    )
