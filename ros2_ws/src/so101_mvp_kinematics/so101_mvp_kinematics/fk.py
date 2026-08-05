from __future__ import annotations

import numpy as np

from .model import So101KinematicModel
from .transforms import (
    axis_angle_to_rotation,
    homogeneous_transform,
    normalize_vector,
    rpy_to_rotation,
)


def _validate_joint_positions(model: So101KinematicModel, joint_positions_rad: np.ndarray) -> np.ndarray:
    values = np.asarray(joint_positions_rad, dtype=np.float64)
    expected = len(model.joint_names)
    if values.shape != (expected,):
        raise ValueError(f"Expected {expected} joint positions, got shape {values.shape}")
    if not np.all(np.isfinite(values)):
        raise ValueError("Joint positions contain NaN or Inf")
    return values


def forward_kinematics(
    model: So101KinematicModel,
    joint_positions_rad: np.ndarray,
) -> dict[str, object]:
    q = _validate_joint_positions(model, joint_positions_rad)
    q_by_name = model.joint_dict(q)

    transform = np.eye(4, dtype=np.float64)
    joint_origins: list[np.ndarray] = []
    joint_axes: list[np.ndarray] = []
    link_transforms: dict[str, np.ndarray] = {model.base_link: transform.copy()}

    for joint in model.chain:
        origin_transform = homogeneous_transform(
            rpy_to_rotation(joint.origin_rpy),
            joint.origin_xyz,
        )
        joint_frame_transform = transform @ origin_transform

        if joint.joint_type == "fixed":
            transform = joint_frame_transform
        elif joint.joint_type == "revolute":
            axis_base = normalize_vector(
                joint_frame_transform[:3, :3] @ normalize_vector(joint.axis, joint.name),
                f"{joint.name} axis in base",
            )
            joint_origins.append(joint_frame_transform[:3, 3].copy())
            joint_axes.append(axis_base)

            angle = q_by_name[joint.name]
            joint_rotation = homogeneous_transform(
                axis_angle_to_rotation(joint.axis, angle),
                np.zeros(3, dtype=np.float64),
            )
            transform = joint_frame_transform @ joint_rotation
        else:
            raise ValueError(f"Unsupported joint type {joint.joint_type!r} for {joint.name}")

        link_transforms[joint.child] = transform.copy()

    if not np.all(np.isfinite(transform)):
        raise ValueError("FK produced NaN or Inf")

    return {
        "transform_base_to_tip": transform.copy(),
        "position_m": transform[:3, 3].copy(),
        "rotation_matrix": transform[:3, :3].copy(),
        "joint_origins_base_m": np.asarray(joint_origins, dtype=np.float64),
        "joint_axes_base": np.asarray(joint_axes, dtype=np.float64),
        "link_transforms": link_transforms,
    }
