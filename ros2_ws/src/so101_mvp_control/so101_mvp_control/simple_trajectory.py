from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    positions_rad: list[float]
    velocities_rad_s: list[float]
    accelerations_rad_s2: list[float]
    active_joint_index: int | None
    active_joint_name: str | None
    phase: str
    motion_stage: str = ""


def _finite_scalar(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _sample_times(duration_s: float, control_rate_hz: float) -> list[float]:
    dt = 1.0 / control_rate_hz
    if duration_s <= 0.0:
        return [0.0]

    count = int(math.floor(duration_s / dt))
    times = [round(index * dt, 12) for index in range(count + 1)]
    if not math.isclose(times[-1], duration_s, rel_tol=0.0, abs_tol=1.0e-12):
        times.append(float(duration_s))
    return times


def generate_single_joint_profile(
    start_position_rad: float,
    target_position_rad: float,
    speed_rad_s: float,
    acceleration_rad_s2: float,
    control_rate_hz: float,
    minimum_motion_rad: float = 0.001,
) -> dict[str, object]:
    start = _finite_scalar(start_position_rad, "start_position_rad")
    target = _finite_scalar(target_position_rad, "target_position_rad")
    speed = _finite_scalar(speed_rad_s, "speed_rad_s")
    acceleration = _finite_scalar(acceleration_rad_s2, "acceleration_rad_s2")
    rate = _finite_scalar(control_rate_hz, "control_rate_hz")
    minimum_motion = abs(_finite_scalar(minimum_motion_rad, "minimum_motion_rad"))

    if speed <= 0.0 or acceleration <= 0.0 or rate <= 0.0:
        raise ValueError("speed, acceleration, and control rate must be positive")

    delta = target - start
    distance = abs(delta)
    direction = 1.0 if delta >= 0.0 else -1.0
    dt = 1.0 / rate

    if distance < minimum_motion:
        return {
            "profile_type": "static",
            "duration_s": dt,
            "distance_rad": distance,
            "max_speed_rad_s": 0.0,
            "max_acceleration_rad_s2": 0.0,
            "samples": [
                {
                    "time_s": 0.0,
                    "position_rad": start,
                    "velocity_rad_s": 0.0,
                    "acceleration_rad_s2": 0.0,
                    "phase": "static",
                },
                {
                    "time_s": dt,
                    "position_rad": target,
                    "velocity_rad_s": 0.0,
                    "acceleration_rad_s2": 0.0,
                    "phase": "static",
                },
            ],
        }

    accel_distance = speed * speed / (2.0 * acceleration)
    if 2.0 * accel_distance <= distance:
        profile_type = "trapezoidal"
        t_accel = speed / acceleration
        cruise_distance = distance - 2.0 * accel_distance
        t_cruise = cruise_distance / speed
        peak_speed = speed
    else:
        profile_type = "triangular"
        t_accel = math.sqrt(distance / acceleration)
        t_cruise = 0.0
        peak_speed = acceleration * t_accel
        accel_distance = distance / 2.0

    duration = 2.0 * t_accel + t_cruise
    times = _sample_times(duration, rate)
    samples: list[dict[str, float | str]] = []

    for time_s in times:
        if time_s <= t_accel:
            phase = "accelerate"
            traveled = 0.5 * acceleration * time_s * time_s
            velocity = acceleration * time_s
            sample_acceleration = acceleration
        elif time_s <= t_accel + t_cruise:
            phase = "cruise" if t_cruise > 0.0 else "decelerate"
            cruise_time = time_s - t_accel
            traveled = accel_distance + peak_speed * cruise_time
            velocity = peak_speed
            sample_acceleration = 0.0
        else:
            phase = "decelerate"
            decel_time = time_s - t_accel - t_cruise
            traveled = (
                accel_distance
                + peak_speed * t_cruise
                + peak_speed * decel_time
                - 0.5 * acceleration * decel_time * decel_time
            )
            velocity = max(0.0, peak_speed - acceleration * decel_time)
            sample_acceleration = -acceleration

        samples.append(
            {
                "time_s": float(time_s),
                "position_rad": float(start + direction * traveled),
                "velocity_rad_s": float(direction * velocity),
                "acceleration_rad_s2": float(direction * sample_acceleration),
                "phase": phase,
            }
        )

    samples[0] = {
        "time_s": 0.0,
        "position_rad": start,
        "velocity_rad_s": 0.0,
        "acceleration_rad_s2": 0.0,
        "phase": "start",
    }
    samples[-1] = {
        "time_s": float(duration),
        "position_rad": target,
        "velocity_rad_s": 0.0,
        "acceleration_rad_s2": 0.0,
        "phase": "end",
    }

    return {
        "profile_type": profile_type,
        "duration_s": float(duration),
        "distance_rad": float(distance),
        "max_speed_rad_s": float(peak_speed),
        "max_acceleration_rad_s2": float(acceleration),
        "samples": samples,
    }


def _as_joint_vector(values: np.ndarray | list[float], joint_names: list[str], name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (len(joint_names),):
        raise ValueError(f"{name} must have {len(joint_names)} values")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} contains NaN or Inf")
    return vector


def _settle_times(duration_s: float, control_rate_hz: float) -> list[float]:
    if duration_s <= 0.0:
        return []
    return [time for time in _sample_times(duration_s, control_rate_hz) if time > 0.0]


def generate_sequential_joint_trajectory(
    start_joint_positions_rad: np.ndarray | list[float],
    target_joint_positions_rad: np.ndarray | list[float],
    joint_names: list[str],
    joint_order: list[str],
    speed_rad_s: float,
    acceleration_rad_s2: float,
    control_rate_hz: float,
    minimum_motion_rad: float = 0.001,
    settle_time_between_joints_s: float = 0.20,
    motion_stage: str = "",
    start_time_s: float = 0.0,
) -> dict[str, object]:
    start = _as_joint_vector(start_joint_positions_rad, joint_names, "start_joint_positions_rad")
    target = _as_joint_vector(target_joint_positions_rad, joint_names, "target_joint_positions_rad")
    if sorted(joint_order) != sorted(joint_names):
        raise ValueError("joint_order must contain the same names as joint_names")

    current = start.copy()
    time_s = float(start_time_s)
    zero = [0.0 for _ in joint_names]
    points = [
        TrajectoryPoint(
            time_s=time_s,
            positions_rad=[float(value) for value in current.tolist()],
            velocities_rad_s=zero.copy(),
            accelerations_rad_s2=zero.copy(),
            active_joint_index=None,
            active_joint_name=None,
            phase="start",
            motion_stage=motion_stage,
        )
    ]
    segments: list[dict[str, object]] = []

    for joint_name in joint_order:
        joint_index = joint_names.index(joint_name)
        segment_start_time = time_s
        profile = generate_single_joint_profile(
            current[joint_index],
            target[joint_index],
            speed_rad_s,
            acceleration_rad_s2,
            control_rate_hz,
            minimum_motion_rad,
        )
        samples = profile["samples"]
        moving_samples = 0
        for sample in samples[1:]:
            local_time = float(sample["time_s"])
            positions = current.copy()
            positions[joint_index] = float(sample["position_rad"])
            velocities = np.zeros(len(joint_names), dtype=np.float64)
            accelerations = np.zeros(len(joint_names), dtype=np.float64)
            velocities[joint_index] = float(sample["velocity_rad_s"])
            accelerations[joint_index] = float(sample["acceleration_rad_s2"])
            active_name = joint_name if abs(velocities[joint_index]) > 1.0e-12 else None
            if active_name:
                moving_samples += 1
            points.append(
                TrajectoryPoint(
                    time_s=float(segment_start_time + local_time),
                    positions_rad=[float(value) for value in positions.tolist()],
                    velocities_rad_s=[float(value) for value in velocities.tolist()],
                    accelerations_rad_s2=[float(value) for value in accelerations.tolist()],
                    active_joint_index=joint_index if active_name else None,
                    active_joint_name=active_name,
                    phase=str(sample["phase"]),
                    motion_stage=motion_stage,
                )
            )

        time_s = points[-1].time_s
        current[joint_index] = target[joint_index]

        if profile["distance_rad"] >= minimum_motion_rad and settle_time_between_joints_s > 0.0:
            for settle_time in _settle_times(settle_time_between_joints_s, control_rate_hz):
                points.append(
                    TrajectoryPoint(
                        time_s=float(time_s + settle_time),
                        positions_rad=[float(value) for value in current.tolist()],
                        velocities_rad_s=zero.copy(),
                        accelerations_rad_s2=zero.copy(),
                        active_joint_index=None,
                        active_joint_name=None,
                        phase="settle",
                        motion_stage=motion_stage,
                    )
                )
            time_s = points[-1].time_s

        segments.append(
            {
                "joint_name": joint_name,
                "joint_index": joint_index,
                "start_time_s": float(segment_start_time),
                "end_time_s": float(time_s),
                "motion_duration_s": float(profile["duration_s"]),
                "settle_time_s": float(settle_time_between_joints_s)
                if profile["distance_rad"] >= minimum_motion_rad
                else 0.0,
                "delta_rad": float(target[joint_index] - start[joint_index]),
                "profile_type": profile["profile_type"],
                "point_count": int(len(samples)),
                "moving_sample_count": int(moving_samples),
                "max_speed_rad_s": float(profile["max_speed_rad_s"]),
                "max_acceleration_rad_s2": float(profile["max_acceleration_rad_s2"]),
            }
        )

    final_positions = np.asarray(points[-1].positions_rad, dtype=np.float64)
    if not np.allclose(final_positions, target, atol=0.0, rtol=0.0):
        time_s = points[-1].time_s + 1.0 / control_rate_hz
        points.append(
            TrajectoryPoint(
                time_s=float(time_s),
                positions_rad=[float(value) for value in target.tolist()],
                velocities_rad_s=zero.copy(),
                accelerations_rad_s2=zero.copy(),
                active_joint_index=None,
                active_joint_name=None,
                phase="final",
                motion_stage=motion_stage,
            )
        )

    return {
        "joint_names": joint_names.copy(),
        "joint_order": joint_order.copy(),
        "start_positions_rad": [float(value) for value in start.tolist()],
        "target_positions_rad": [float(value) for value in target.tolist()],
        "points": points,
        "segments": segments,
        "motion_stage": motion_stage,
    }
