from __future__ import annotations

import math

import numpy as np

from .fk import forward_kinematics
from .jacobian import geometric_jacobian
from .joint_limits import clamp_to_limits, joints_within_limits, limit_hit_joints
from .model import So101KinematicModel
from .transforms import normalize_vector, rotation_angle_error


def _failure(
    reason: str,
    model: So101KinematicModel,
    q: np.ndarray | None,
    iterations: int = 0,
    position_error_m: float = math.inf,
    approach_error_deg: float = math.inf,
    final_position_m: np.ndarray | None = None,
) -> dict[str, object]:
    joint_values = None if q is None else [float(value) for value in q.tolist()]
    return {
        "success": False,
        "joint_positions_rad": joint_values,
        "iterations": int(iterations),
        "position_error_m": float(position_error_m),
        "approach_error_deg": float(approach_error_deg),
        "reason": reason,
        "limit_hit_joints": [] if q is None else limit_hit_joints(model, q),
        "final_position_m": None
        if final_position_m is None
        else [float(value) for value in final_position_m.tolist()],
    }


def _pose_errors(
    model: So101KinematicModel,
    q: np.ndarray,
    target_position: np.ndarray,
    desired_approach: np.ndarray,
    tool_axis_local: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float, float, np.ndarray]:
    fk = forward_kinematics(model, q)
    position = np.asarray(fk["position_m"], dtype=np.float64)
    rotation = np.asarray(fk["rotation_matrix"], dtype=np.float64)
    current_approach = normalize_vector(rotation @ tool_axis_local, "current approach")
    position_error = target_position - position
    approach_error = np.cross(current_approach, desired_approach)
    position_error_m = float(np.linalg.norm(position_error))
    approach_error_deg = math.degrees(
        rotation_angle_error(current_approach, desired_approach)
    )
    return position_error, approach_error, position_error_m, approach_error_deg, position


def solve_ik(
    model: So101KinematicModel,
    target_position_m: np.ndarray,
    seed_joint_positions_rad: np.ndarray,
    desired_approach_base: np.ndarray | None = None,
    tool_approach_axis_local: np.ndarray | None = None,
    max_iterations: int = 200,
    damping: float = 0.05,
    maximum_step_rad: float = 0.10,
    position_tolerance_m: float = 0.002,
    approach_tolerance_deg: float = 5.0,
    orientation_weight: float = 0.25,
) -> dict[str, object]:
    desired_raw = np.asarray(
        [0.0, 0.0, -1.0] if desired_approach_base is None else desired_approach_base,
        dtype=np.float64,
    )
    tool_raw = np.asarray(
        [0.0, 0.0, 1.0]
        if tool_approach_axis_local is None
        else tool_approach_axis_local,
        dtype=np.float64,
    )

    try:
        target = np.asarray(target_position_m, dtype=np.float64)
        seed = np.asarray(seed_joint_positions_rad, dtype=np.float64)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            return _failure("invalid_input", model, None)
        if seed.shape != (len(model.joint_names),) or not np.all(np.isfinite(seed)):
            return _failure("invalid_input", model, None)
        if not joints_within_limits(model, seed):
            return _failure("invalid_input", model, seed)
        if float(np.linalg.norm(target)) > 0.55:
            return _failure("target_unreachable", model, seed)

        desired_approach = normalize_vector(desired_raw, "desired approach")
        tool_axis = normalize_vector(tool_raw, "tool approach axis")
    except ValueError:
        return _failure("invalid_input", model, None)

    q = seed.copy()
    best_q = q.copy()
    best_position = None
    best_position_error = math.inf
    best_approach_error = math.inf

    task_damping = float(damping)
    if task_damping <= 0.0 or maximum_step_rad <= 0.0:
        return _failure("invalid_input", model, q)

    for iteration in range(0, int(max_iterations) + 1):
        try:
            (
                position_error,
                approach_error,
                position_error_m,
                approach_error_deg,
                final_position,
            ) = _pose_errors(model, q, target, desired_approach, tool_axis)
        except (ValueError, np.linalg.LinAlgError):
            return _failure("numerical_failure", model, q, iteration)

        if position_error_m < best_position_error:
            best_q = q.copy()
            best_position = final_position.copy()
            best_position_error = position_error_m
            best_approach_error = approach_error_deg

        if (
            position_error_m <= position_tolerance_m
            and approach_error_deg <= approach_tolerance_deg
            and joints_within_limits(model, q)
        ):
            return {
                "success": True,
                "joint_positions_rad": [float(value) for value in q.tolist()],
                "iterations": int(iteration),
                "position_error_m": float(position_error_m),
                "approach_error_deg": float(approach_error_deg),
                "reason": "converged",
                "limit_hit_joints": limit_hit_joints(model, q),
                "final_position_m": [float(value) for value in final_position.tolist()],
            }

        if iteration >= int(max_iterations):
            break

        error = np.concatenate(
            [
                position_error,
                float(orientation_weight) * approach_error,
            ]
        )
        jacobian = geometric_jacobian(model, q)
        task_jacobian = jacobian.copy()
        task_jacobian[3:, :] *= float(orientation_weight)

        lhs = task_jacobian @ task_jacobian.T
        lhs += (task_damping * task_damping) * np.eye(6, dtype=np.float64)
        try:
            step = task_jacobian.T @ np.linalg.solve(lhs, error)
        except np.linalg.LinAlgError:
            return _failure(
                "numerical_failure",
                model,
                q,
                iteration,
                position_error_m,
                approach_error_deg,
                final_position,
            )

        if not np.all(np.isfinite(step)):
            return _failure(
                "numerical_failure",
                model,
                q,
                iteration,
                position_error_m,
                approach_error_deg,
                final_position,
            )

        step_norm = float(np.linalg.norm(step, ord=np.inf))
        if step_norm > maximum_step_rad:
            step *= maximum_step_rad / step_norm

        q = clamp_to_limits(model, q + step)

    reason = "target_unreachable"
    if np.all(np.isfinite(target)) and best_position_error < math.inf:
        reason = "max_iterations"

    return _failure(
        reason,
        model,
        best_q,
        int(max_iterations),
        best_position_error,
        best_approach_error,
        best_position,
    )
