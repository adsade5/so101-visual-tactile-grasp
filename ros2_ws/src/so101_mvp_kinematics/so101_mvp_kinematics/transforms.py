from __future__ import annotations

import math

import numpy as np


def normalize_vector(vector: np.ndarray, name: str = "vector") -> np.ndarray:
    values = np.asarray(vector, dtype=np.float64)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be a finite 3-vector")
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} has zero length")
    return values / norm


def rotation_x(angle_rad: float) -> np.ndarray:
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    return np.asarray(
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
        dtype=np.float64,
    )


def rotation_y(angle_rad: float) -> np.ndarray:
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    return np.asarray(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        dtype=np.float64,
    )


def rotation_z(angle_rad: float) -> np.ndarray:
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    return np.asarray(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def rpy_to_rotation(rpy_rad: np.ndarray) -> np.ndarray:
    rpy = np.asarray(rpy_rad, dtype=np.float64)
    if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
        raise ValueError("RPY must be a finite 3-vector")
    roll, pitch, yaw = rpy.tolist()
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def axis_angle_to_rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    x, y, z = normalize_vector(axis, "axis")
    angle = float(angle_rad)
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def homogeneous_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    rot = np.asarray(rotation, dtype=np.float64)
    xyz = np.asarray(translation, dtype=np.float64)
    if rot.shape != (3, 3) or xyz.shape != (3,):
        raise ValueError("Expected a 3x3 rotation and 3-vector translation")
    if not np.all(np.isfinite(rot)) or not np.all(np.isfinite(xyz)):
        raise ValueError("Transform contains NaN or Inf")

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    transform[:3, 3] = xyz
    return transform


def rotation_angle_error(first: np.ndarray, second: np.ndarray) -> float:
    first_axis = normalize_vector(first, "first")
    second_axis = normalize_vector(second, "second")
    dot = float(np.clip(np.dot(first_axis, second_axis), -1.0, 1.0))
    return math.acos(dot)


def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
    rot = np.asarray(rotation, dtype=np.float64)
    if rot.shape != (3, 3) or not np.all(np.isfinite(rot)):
        raise ValueError("Rotation must be a finite 3x3 matrix")
    trace = float(np.trace(rot))
    cos_angle = float(np.clip((trace - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cos_angle)
    skew = np.asarray(
        [
            rot[2, 1] - rot[1, 2],
            rot[0, 2] - rot[2, 0],
            rot[1, 0] - rot[0, 1],
        ],
        dtype=np.float64,
    )
    if angle < 1.0e-9:
        return 0.5 * skew
    return skew * (angle / (2.0 * math.sin(angle)))
