from __future__ import annotations

import numpy as np

from .model import So101KinematicModel


def validate_joint_positions(
    model: So101KinematicModel,
    joint_positions_rad: np.ndarray,
    *,
    require_within_limits: bool = True,
) -> np.ndarray:
    values = np.asarray(joint_positions_rad, dtype=np.float64)
    expected = len(model.joint_names)
    if values.shape != (expected,):
        raise ValueError(f"Expected {expected} joint positions, got shape {values.shape}")
    if not np.all(np.isfinite(values)):
        raise ValueError("Joint positions contain NaN or Inf")
    if require_within_limits:
        below = values < model.lower_limits
        above = values > model.upper_limits
        if np.any(below) or np.any(above):
            names = [
                name
                for name, is_bad in zip(
                    model.joint_names,
                    np.logical_or(below, above).tolist(),
                    strict=True,
                )
                if is_bad
            ]
            raise ValueError(f"Joint positions outside URDF limits: {names}")
    return values


def clamp_to_limits(model: So101KinematicModel, joint_positions_rad: np.ndarray) -> np.ndarray:
    values = np.asarray(joint_positions_rad, dtype=np.float64)
    return np.minimum(np.maximum(values, model.lower_limits), model.upper_limits)


def joints_within_limits(model: So101KinematicModel, joint_positions_rad: np.ndarray) -> bool:
    values = np.asarray(joint_positions_rad, dtype=np.float64)
    return bool(
        values.shape == (len(model.joint_names),)
        and np.all(np.isfinite(values))
        and np.all(values >= model.lower_limits)
        and np.all(values <= model.upper_limits)
    )


def limit_hit_joints(
    model: So101KinematicModel,
    joint_positions_rad: np.ndarray,
    tolerance_rad: float = 1.0e-8,
) -> list[str]:
    values = np.asarray(joint_positions_rad, dtype=np.float64)
    hits: list[str] = []
    if values.shape != (len(model.joint_names),):
        return model.joint_names.copy()
    for name, value, lower, upper in zip(
        model.joint_names,
        values.tolist(),
        model.lower_limits.tolist(),
        model.upper_limits.tolist(),
        strict=True,
    ):
        if value <= lower + tolerance_rad or value >= upper - tolerance_rad:
            hits.append(name)
    return hits
