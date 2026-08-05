from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_SRC = PROJECT_ROOT / "ros2_ws" / "src"
sys.path.insert(0, str(ROS2_SRC / "so101_mvp_kinematics"))
sys.path.insert(0, str(ROS2_SRC / "so101_kinematics"))

from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.ik import solve_ik
from so101_mvp_kinematics.jacobian import (
    finite_difference_jacobian,
    geometric_jacobian,
)
from so101_mvp_kinematics.joint_limits import joints_within_limits
from so101_mvp_kinematics.model import BASE_LINK, JOINT_NAMES, TIP_LINK, So101KinematicModel
from so101_mvp_kinematics.transforms import normalize_vector, rotation_angle_error


URDF_PATH = PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_kinematics.yaml"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp1_kinematics_report.json"
LOG_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp1_kinematics.log"
EXPECTED_URDF_SHA256 = "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_output(*args: str) -> str:
    import subprocess

    return subprocess.check_output(
        ["git", *args],
        cwd=PROJECT_ROOT,
        text=True,
        stderr=subprocess.STDOUT,
    ).strip()


def vec(values: list[float]) -> np.ndarray:
    return np.asarray(values, dtype=np.float64)


def listf(values: np.ndarray | list[float]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def approach_from_fk(fk: dict[str, object], tool_axis: np.ndarray) -> np.ndarray:
    rotation = np.asarray(fk["rotation_matrix"], dtype=np.float64)
    return normalize_vector(rotation @ tool_axis, "approach")


def approach_error_deg(fk: dict[str, object], desired: np.ndarray, tool_axis: np.ndarray) -> float:
    return math.degrees(rotation_angle_error(approach_from_fk(fk, tool_axis), desired))


def result_ok(result: dict[str, object], max_pos: float, max_approach: float) -> bool:
    return bool(
        result.get("success")
        and float(result["position_error_m"]) <= max_pos
        and float(result["approach_error_deg"]) <= max_approach
    )


def reference_poses() -> list[np.ndarray]:
    return [
        vec([0.0, 0.0, 0.0, 0.0, 0.0]),
        vec([0.25, -0.30, 0.35, 0.80, 0.10]),
        vec([-0.35, -0.20, 0.55, 0.65, -0.25]),
        vec([0.55, -0.55, 0.70, 0.55, 0.35]),
        vec([-0.50, -0.40, 0.45, 1.00, -0.45]),
        vec([0.20, -0.75, 0.90, 0.70, 0.55]),
        vec([-0.25, -0.65, 1.00, 0.45, -0.60]),
        vec([0.80, -0.30, 0.40, 0.90, 0.20]),
        vec([-0.75, -0.25, 0.75, 0.50, -0.20]),
        vec([0.10, -1.00, 1.10, 0.35, 0.00]),
    ]


def round_trip_poses() -> list[np.ndarray]:
    return [
        vec([0.0, -0.35, 0.35, 1.22, 0.0]),
        vec([0.20, -0.45, 0.50, 1.05, 0.10]),
        vec([-0.20, -0.55, 0.70, 0.85, -0.15]),
        vec([0.35, -0.70, 0.95, 0.65, 0.20]),
        vec([-0.35, -0.80, 1.05, 0.55, -0.20]),
        vec([0.10, -0.60, 0.80, 0.75, 0.30]),
    ]


def compare_legacy_fk(model: So101KinematicModel, poses: list[np.ndarray]) -> dict[str, object]:
    from so101_kinematics.urdf_fk import UrdfForwardKinematics

    legacy = UrdfForwardKinematics(URDF_PATH, BASE_LINK, TIP_LINK)
    legacy.validate_expected_chain()
    cases = []
    max_position = 0.0
    max_rotation = 0.0

    for index, q in enumerate(poses, start=1):
        mvp_fk = forward_kinematics(model, q)
        legacy_fk = legacy.compute(
            {name: float(value) for name, value in zip(JOINT_NAMES, q.tolist(), strict=True)}
        )
        position_diff = float(
            np.linalg.norm(np.asarray(mvp_fk["position_m"]) - legacy_fk[:3, 3])
        )
        rotation_diff = float(
            np.max(np.abs(np.asarray(mvp_fk["rotation_matrix"]) - legacy_fk[:3, :3]))
        )
        max_position = max(max_position, position_diff)
        max_rotation = max(max_rotation, rotation_diff)
        cases.append(
            {
                "name": f"legacy_fk_{index}",
                "joint_positions_rad": listf(q),
                "position_diff_m": position_diff,
                "rotation_matrix_max_abs_diff": rotation_diff,
                "success": position_diff <= 1.0e-8 and rotation_diff <= 1.0e-8,
            }
        )

    return {
        "cases": cases,
        "maximum_position_diff_m": max_position,
        "maximum_rotation_matrix_abs_diff": max_rotation,
        "success": max_position <= 1.0e-8 and max_rotation <= 1.0e-8,
    }


def verify_jacobian(model: So101KinematicModel, poses: list[np.ndarray]) -> dict[str, object]:
    cases = []
    maximum = 0.0
    for index, q in enumerate(poses, start=1):
        analytical = geometric_jacobian(model, q)
        numerical = finite_difference_jacobian(model, q)
        error = float(np.max(np.abs(analytical - numerical)))
        maximum = max(maximum, error)
        cases.append(
            {
                "name": f"jacobian_{index}",
                "joint_positions_rad": listf(q),
                "max_abs_error": error,
                "success": error <= 1.0e-4,
            }
        )
    return {"cases": cases, "maximum_error": maximum, "success": maximum <= 1.0e-4}


def verify_round_trip(
    model: So101KinematicModel,
    tool_axis: np.ndarray,
    ik_params: dict[str, float],
) -> list[dict[str, object]]:
    cases = []
    seed_offset = vec([0.035, -0.025, 0.030, -0.020, 0.015])

    for index, q in enumerate(round_trip_poses(), start=1):
        fk = forward_kinematics(model, q)
        target = np.asarray(fk["position_m"], dtype=np.float64)
        desired = approach_from_fk(fk, tool_axis)
        seed = np.minimum(np.maximum(q + seed_offset, model.lower_limits), model.upper_limits)
        result = solve_ik(
            model,
            target,
            seed,
            desired_approach_base=desired,
            tool_approach_axis_local=tool_axis,
            **ik_params,
        )
        success = result_ok(result, 0.002, 5.0)
        cases.append(
            {
                "name": f"round_trip_{index}",
                "target_position_m": listf(target),
                "seed_joint_positions_rad": listf(seed),
                "success": success,
                **result,
            }
        )

    return cases


def verify_workspace_targets(
    model: So101KinematicModel,
    reference_seed: np.ndarray,
    desired: np.ndarray,
    tool_axis: np.ndarray,
    ik_params: dict[str, float],
) -> list[dict[str, object]]:
    xy_targets = [
        ("F1", 0.15, 0.00),
        ("F2", 0.15, -0.08),
        ("F3", 0.15, 0.08),
        ("F4", 0.23, 0.00),
        ("F5", 0.23, -0.08),
        ("F6", 0.23, 0.08),
        ("H1", 0.20, 0.00),
        ("H2", 0.18, -0.04),
    ]
    cases = []
    seed = reference_seed.copy()

    for name, x, y in xy_targets:
        target = vec([x, y, 0.080])
        result = solve_ik(
            model,
            target,
            seed,
            desired_approach_base=desired,
            tool_approach_axis_local=tool_axis,
            **ik_params,
        )
        within = False
        if result.get("joint_positions_rad") is not None:
            within = joints_within_limits(model, np.asarray(result["joint_positions_rad"]))
        success = result_ok(result, 0.003, 5.0) and within
        if success:
            seed = np.asarray(result["joint_positions_rad"], dtype=np.float64)
        cases.append(
            {
                "name": name,
                "target_position_m": listf(target),
                "success": success,
                "within_limits": within,
                **result,
            }
        )

    return cases


def verify_failure_cases(
    model: So101KinematicModel,
    reference_seed: np.ndarray,
    desired: np.ndarray,
    tool_axis: np.ndarray,
    ik_params: dict[str, float],
) -> list[dict[str, object]]:
    cases = []

    checks = [
        (
            "wrong_joint_count_fk",
            lambda: forward_kinematics(model, vec([0.0, 0.0])),
        ),
        (
            "nan_joint_fk",
            lambda: forward_kinematics(model, vec([0.0, math.nan, 0.0, 0.0, 0.0])),
        ),
        (
            "nan_target_ik",
            lambda: solve_ik(model, vec([math.nan, 0.0, 0.1]), reference_seed, desired, tool_axis, **ik_params),
        ),
        (
            "unreachable_target_ik",
            lambda: solve_ik(model, vec([0.60, 0.0, 0.10]), reference_seed, desired, tool_axis, **ik_params),
        ),
        (
            "seed_out_of_range_ik",
            lambda: solve_ik(model, vec([0.15, 0.0, 0.08]), model.upper_limits + 0.5, desired, tool_axis, **ik_params),
        ),
        (
            "zero_approach_vector_ik",
            lambda: solve_ik(model, vec([0.15, 0.0, 0.08]), reference_seed, vec([0.0, 0.0, 0.0]), tool_axis, **ik_params),
        ),
    ]

    for name, callback in checks:
        try:
            value = callback()
            if isinstance(value, dict):
                success = not bool(value.get("success"))
                reason = str(value.get("reason"))
            else:
                success = False
                reason = "unexpected_success"
        except Exception as exc:  # Expected for FK invalid input cases.
            success = True
            reason = type(exc).__name__ + ": " + str(exc)

        cases.append({"name": name, "success": success, "reason": reason})

    return cases


def main() -> int:
    logs: list[str] = []
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)

    urdf_hash = sha256_file(URDF_PATH)
    model = So101KinematicModel(URDF_PATH, BASE_LINK, TIP_LINK, JOINT_NAMES)

    tool_axis = vec([0.0, 0.0, 1.0])
    desired = vec([0.0, 0.0, -1.0])
    reference_seed = vec([0.0, -0.35, 0.35, 1.22, 0.0])
    ik_params = {
        "max_iterations": 200,
        "damping": 0.05,
        "maximum_step_rad": 0.10,
        "position_tolerance_m": 0.002,
        "approach_tolerance_deg": 5.0,
        "orientation_weight": 0.25,
    }

    if urdf_hash.lower() != EXPECTED_URDF_SHA256:
        raise RuntimeError(f"Frozen URDF SHA-256 mismatch: {urdf_hash}")
    if not joints_within_limits(model, reference_seed):
        raise RuntimeError("Reference joint posture is outside URDF limits")

    reference_fk = forward_kinematics(model, reference_seed)
    jacobian_report = verify_jacobian(model, reference_poses())
    legacy_report = compare_legacy_fk(model, reference_poses())
    round_trip_cases = verify_round_trip(model, tool_axis, ik_params)
    workspace_cases = verify_workspace_targets(model, reference_seed, desired, tool_axis, ik_params)
    failure_cases = verify_failure_cases(model, reference_seed, desired, tool_axis, ik_params)

    all_solution_vectors = [
        np.asarray(case["joint_positions_rad"], dtype=np.float64)
        for case in round_trip_cases + workspace_cases
        if case.get("success") and case.get("joint_positions_rad") is not None
    ]
    all_solutions_within_limits = all(
        joints_within_limits(model, q) for q in all_solution_vectors
    )
    maximum_position_error = max(
        [float(case["position_error_m"]) for case in round_trip_cases + workspace_cases]
        or [0.0]
    )
    maximum_approach_error = max(
        [float(case["approach_error_deg"]) for case in round_trip_cases + workspace_cases]
        or [0.0]
    )

    final_status = "PASS"
    pass_checks = [
        jacobian_report["success"],
        legacy_report["success"],
        all(bool(case["success"]) for case in round_trip_cases),
        all(bool(case["success"]) for case in workspace_cases),
        all(bool(case["success"]) for case in failure_cases),
        maximum_position_error <= 0.003,
        maximum_approach_error <= 5.0,
        all_solutions_within_limits,
    ]
    if not all(pass_checks):
        final_status = "FAIL"

    report = {
        "stage": "MVP-1",
        "git_branch": git_output("branch", "--show-current"),
        "urdf_path": str(URDF_PATH),
        "urdf_sha256": urdf_hash,
        "joint_names": JOINT_NAMES,
        "base_link": BASE_LINK,
        "tip_link": TIP_LINK,
        "tool_approach_axis_local": listf(tool_axis),
        "desired_approach_base": listf(desired),
        "reference_joint_positions_rad": listf(reference_seed),
        "reference_fk_position_m": listf(np.asarray(reference_fk["position_m"])),
        "jacobian_max_error": jacobian_report["maximum_error"],
        "jacobian_cases": jacobian_report["cases"],
        "legacy_fk_comparison": legacy_report,
        "round_trip_cases": round_trip_cases,
        "workspace_target_cases": workspace_cases,
        "failure_cases": failure_cases,
        "maximum_position_error_m": maximum_position_error,
        "maximum_approach_error_deg": maximum_approach_error,
        "all_solutions_within_limits": all_solutions_within_limits,
        "opened_com_ports": False,
        "torque_enable_written": False,
        "torque_disable_written": False,
        "goal_position_written": False,
        "motion_command_sent": False,
        "final_status": final_status,
    }

    REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")

    logs.extend(
        [
            "Stage MVP-1 SO-101 kinematics verification",
            f"urdf_sha256={urdf_hash}",
            f"reference_fk_position_m={report['reference_fk_position_m']}",
            f"jacobian_max_error={jacobian_report['maximum_error']:.12e}",
            "legacy_fk_max_position_diff_m="
            f"{legacy_report['maximum_position_diff_m']:.12e}",
            "legacy_fk_max_rotation_diff="
            f"{legacy_report['maximum_rotation_matrix_abs_diff']:.12e}",
            f"maximum_position_error_m={maximum_position_error:.12e}",
            f"maximum_approach_error_deg={maximum_approach_error:.12e}",
            f"final_status={final_status}",
            f"report={REPORT_PATH}",
        ]
    )
    LOG_PATH.write_text("\n".join(logs) + "\n", encoding="utf-8")

    print("\n".join(logs))
    return 0 if final_status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
