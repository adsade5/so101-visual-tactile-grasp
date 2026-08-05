from __future__ import annotations

import numpy as np

from .fk import forward_kinematics
from .model import So101KinematicModel
from .transforms import rotation_vector_from_matrix


def geometric_jacobian(
    model: So101KinematicModel,
    joint_positions_rad: np.ndarray,
) -> np.ndarray:
    fk = forward_kinematics(model, joint_positions_rad)
    tip = np.asarray(fk["position_m"], dtype=np.float64)
    origins = np.asarray(fk["joint_origins_base_m"], dtype=np.float64)
    axes = np.asarray(fk["joint_axes_base"], dtype=np.float64)

    jacobian = np.zeros((6, len(model.joint_names)), dtype=np.float64)
    for index, (origin, axis) in enumerate(zip(origins, axes, strict=True)):
        jacobian[:3, index] = np.cross(axis, tip - origin)
        jacobian[3:, index] = axis

    if not np.all(np.isfinite(jacobian)):
        raise ValueError("Jacobian contains NaN or Inf")
    return jacobian


def finite_difference_jacobian(
    model: So101KinematicModel,
    joint_positions_rad: np.ndarray,
    step_rad: float = 1.0e-6,
) -> np.ndarray:
    q = np.asarray(joint_positions_rad, dtype=np.float64)
    if q.shape != (len(model.joint_names),):
        raise ValueError("Joint vector has wrong shape")
    if not np.all(np.isfinite(q)):
        raise ValueError("Joint vector contains NaN or Inf")

    jacobian = np.zeros((6, len(model.joint_names)), dtype=np.float64)
    for index in range(len(model.joint_names)):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[index] += step_rad
        q_minus[index] -= step_rad

        fk_plus = forward_kinematics(model, q_plus)
        fk_minus = forward_kinematics(model, q_minus)
        p_plus = np.asarray(fk_plus["position_m"], dtype=np.float64)
        p_minus = np.asarray(fk_minus["position_m"], dtype=np.float64)
        r_plus = np.asarray(fk_plus["rotation_matrix"], dtype=np.float64)
        r_minus = np.asarray(fk_minus["rotation_matrix"], dtype=np.float64)

        jacobian[:3, index] = (p_plus - p_minus) / (2.0 * step_rad)
        delta_rotation = r_plus @ r_minus.T
        jacobian[3:, index] = rotation_vector_from_matrix(delta_rotation) / (2.0 * step_rad)

    return jacobian


def jacobian_max_error(
    model: So101KinematicModel,
    joint_positions_rad: np.ndarray,
    step_rad: float = 1.0e-6,
) -> float:
    analytical = geometric_jacobian(model, joint_positions_rad)
    numerical = finite_difference_jacobian(model, joint_positions_rad, step_rad)
    return float(np.max(np.abs(analytical - numerical)))
