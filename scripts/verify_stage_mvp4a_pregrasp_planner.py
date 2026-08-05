from __future__ import annotations

import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4a_pregrasp_planner_report.json"
for package_path in (
    ROS_SRC / "so101_mvp_control",
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_control.pregrasp_planner import (
    ARM_JOINT_NAMES,
    DESIRED_APPROACH_BASE,
    REFERENCE_SEED_RAD,
    TOOL_APPROACH_AXIS_LOCAL,
    JointStateSnapshot,
    PoseSnapshot,
    compute_pregrasp_plan,
    create_model,
)
from so101_mvp_kinematics.fk import forward_kinematics


def listf(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {
        "name": name,
        "passed": bool(passed),
        "details": {} if details is None else details,
    }


def fake_ik_success_with(q_rad: np.ndarray):
    def _solver(*args, **kwargs) -> dict[str, object]:
        del args, kwargs
        return {
            "success": True,
            "joint_positions_rad": listf(q_rad),
            "position_error_m": 0.0,
            "approach_error_deg": 0.0,
            "reason": "fake_success",
            "limit_hit_joints": [],
            "final_position_m": None,
        }

    return _solver


def fake_ik_failure(*args, **kwargs) -> dict[str, object]:
    del args, kwargs
    return {
        "success": False,
        "joint_positions_rad": None,
        "position_error_m": 1.0,
        "approach_error_deg": 90.0,
        "reason": "fake_failure",
    }


def main() -> int:
    model = create_model(PROJECT_ROOT)
    now = time.monotonic()
    height = 0.08
    reference_fk = forward_kinematics(model, REFERENCE_SEED_RAD)
    reference_tcp = np.asarray(reference_fk["position_m"], dtype=np.float64)
    object_position = reference_tcp - np.asarray([0.0, 0.0, height], dtype=np.float64)
    valid_pose = PoseSnapshot(
        frame_id="base_link",
        position_m=object_position,
        received_monotonic_s=now,
    )
    current_seed = np.asarray([0.05, -0.32, 0.34, 1.18, 0.03], dtype=np.float64)
    fresh_joint_state = JointStateSnapshot(
        names=ARM_JOINT_NAMES,
        positions_rad=current_seed,
        received_monotonic_s=now,
    )
    stale_joint_state = JointStateSnapshot(
        names=ARM_JOINT_NAMES,
        positions_rad=current_seed,
        received_monotonic_s=now - 2.0,
    )

    cases: list[dict[str, object]] = []

    actual_plan = compute_pregrasp_plan(
        model=model,
        object_pose=valid_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("valid_object_pose", actual_plan.success, {"reason": actual_plan.reason}))

    no_pose = compute_pregrasp_plan(
        model=model,
        object_pose=None,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("no_object_pose", no_pose.reason == "no_object_pose"))

    stale_pose = PoseSnapshot("base_link", object_position, now - 2.0)
    stale_plan = compute_pregrasp_plan(
        model=model,
        object_pose=stale_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("stale_object_pose", stale_plan.reason == "object_pose_stale"))

    wrong_frame = compute_pregrasp_plan(
        model=model,
        object_pose=PoseSnapshot("workspace_plane", object_position, now),
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("wrong_frame", wrong_frame.reason == "invalid_frame"))

    for axis, index in (("x", 0), ("y", 1), ("z", 2)):
        bad = object_position.copy()
        bad[index] = math.nan
        plan = compute_pregrasp_plan(
            model=model,
            object_pose=PoseSnapshot("base_link", bad, now),
            joint_state=None,
            base_frame="base_link",
            now_monotonic_s=now,
            max_object_pose_age_s=1.0,
            pregrasp_height_m=height,
            use_joint_state_seed=True,
            max_joint_state_age_s=1.0,
        )
        cases.append(case(f"non_finite_{axis}", plan.reason == "non_finite_object_pose"))

    cases.append(
        case(
            "pregrasp_height_added",
            actual_plan.pregrasp_position_m is not None
            and abs(float(actual_plan.pregrasp_position_m[2] - object_position[2]) - height) < 1.0e-12,
            {
                "object_z_m": float(object_position[2]),
                "pregrasp_z_m": None
                if actual_plan.pregrasp_position_m is None
                else float(actual_plan.pregrasp_position_m[2]),
            },
        )
    )
    cases.append(
        case(
            "fixed_downward_approach",
            np.allclose(TOOL_APPROACH_AXIS_LOCAL, [0.0, 0.0, 1.0])
            and np.allclose(DESIRED_APPROACH_BASE, [0.0, 0.0, -1.0]),
            {
                "tool_axis_local": listf(TOOL_APPROACH_AXIS_LOCAL),
                "desired_approach_base": listf(DESIRED_APPROACH_BASE),
            },
        )
    )
    cases.append(case("reference_seed_fallback", actual_plan.seed_source == "reference"))

    used_seeds: list[np.ndarray] = []

    def capture_seed_solver(model_arg, target, seed, *args, **kwargs):
        del model_arg, target, args, kwargs
        used_seeds.append(np.asarray(seed, dtype=np.float64).copy())
        return {
            "success": False,
            "joint_positions_rad": None,
            "position_error_m": 1.0,
            "approach_error_deg": 90.0,
            "reason": "forced_failure",
        }

    compute_pregrasp_plan(
        model=model,
        object_pose=valid_pose,
        joint_state=fresh_joint_state,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=capture_seed_solver,
    )
    cases.append(
        case(
            "joint_state_seed_used",
            len(used_seeds) >= 1 and np.allclose(used_seeds[-1], current_seed),
        )
    )

    compute_pregrasp_plan(
        model=model,
        object_pose=valid_pose,
        joint_state=stale_joint_state,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=capture_seed_solver,
    )
    cases.append(
        case(
            "joint_state_seed_stale_fallback",
            len(used_seeds) >= 2 and np.allclose(used_seeds[-1], REFERENCE_SEED_RAD),
        )
    )
    cases.append(case("ik_success", actual_plan.success and actual_plan.reason == "pregrasp_ready"))

    failure = compute_pregrasp_plan(
        model=model,
        object_pose=valid_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=fake_ik_failure,
    )
    cases.append(case("ik_failure", failure.reason == "ik_failed"))

    limit_q = model.upper_limits + 0.1
    limit_plan = compute_pregrasp_plan(
        model=model,
        object_pose=valid_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=fake_ik_success_with(limit_q),
    )
    cases.append(case("joint_limits_valid", limit_plan.reason == "joint_limit_failed"))

    cases.append(
        case(
            "fk_position_validation",
            actual_plan.position_error_m is not None and actual_plan.position_error_m <= 0.002,
            {"position_error_m": actual_plan.position_error_m},
        )
    )
    cases.append(
        case(
            "fk_approach_validation",
            actual_plan.approach_error_deg is not None and actual_plan.approach_error_deg <= 5.0,
            {"approach_error_deg": actual_plan.approach_error_deg},
        )
    )
    cases.append(case("joint_name_order", ARM_JOINT_NAMES == tuple(model.joint_names)))
    cases.append(
        case(
            "exactly_five_joint_positions",
            actual_plan.joint_positions_rad is not None
            and actual_plan.joint_positions_rad.shape == (5,),
        )
    )

    node_path = PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "mvp_pregrasp_planner_node.py"
    helper_path = PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "pregrasp_planner.py"
    preview_path = PROJECT_ROOT / "scripts" / "mvp_pregrasp_preview.py"
    source_text = "\n".join(path.read_text(encoding="utf-8") for path in (node_path, helper_path, preview_path))
    node_text = node_path.read_text(encoding="utf-8")
    cases.append(
        case(
            "pregrasp_not_published_to_joint_target",
            '"/mvp/joint_target"' not in node_text and "'/mvp/joint_target'" not in node_text,
        )
    )
    cases.append(case("no_execute_service_call", "/mvp/execute_target" not in source_text))
    cases.append(
        case(
            "no_tcp_connection",
            all(token not in source_text for token in ("socket.", "create_connection", "MvpTcpClient")),
        )
    )
    cases.append(case("no_com_port_open", all(token not in source_text for token in ("COM4", "serial.Serial", "Serial("))))
    cases.append(
        case(
            "no_goal_position_write",
            all(token not in source_text for token in ("Goal_Position", "goal_position", "send_action(")),
        )
    )
    cases.append(
        case(
            "no_physical_motion",
            all(token not in source_text for token in ("move_joints", "execute_target", "hardware_bridge")),
        )
    )

    passed = sum(1 for item in cases if item["passed"])
    report = {
        "stage": "MVP-4A",
        "git_branch": subprocess.check_output(
            ["git", "branch", "--show-current"],
            cwd=PROJECT_ROOT,
            text=True,
        ).strip(),
        "existing_object_pose_node": "so101_object_perception/object_pose_node",
        "existing_object_pose_topic": "/object_pose",
        "existing_workspace_frame": "workspace_plane",
        "existing_workspace_to_base_node": "so101_frame_transform/workspace_to_base_node",
        "existing_base_pose_topic": "/object_pose_base",
        "existing_base_frame": "base_link",
        "pregrasp_node": "so101_mvp_control/mvp_pregrasp_planner_node",
        "compute_service": "/mvp/compute_pregrasp",
        "clear_service": "/mvp/clear_pregrasp",
        "pregrasp_height_m": height,
        "approach_axis_local": listf(TOOL_APPROACH_AXIS_LOCAL),
        "desired_approach_base": listf(DESIRED_APPROACH_BASE),
        "ik_module_reused": "so101_mvp_kinematics.ik.solve_ik",
        "reference_seed": listf(REFERENCE_SEED_RAD),
        "output_topics": [
            "/mvp/pregrasp_pose",
            "/mvp/pregrasp_joint_target",
            "/mvp/pregrasp_valid",
            "/mvp/pregrasp_status",
        ],
        "offline_test_cases": cases,
        "offline_tests_passed": passed == len(cases),
        "offline_test_count": len(cases),
        "ros2_build_result": "not_run",
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "motion_command_sent": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4A_PREGRASP_MANUAL_ACCEPTANCE.md",
        "final_status": "OFFLINE_TESTS_PASS" if passed == len(cases) else "OFFLINE_TESTS_FAIL",
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    for item in cases:
        status = "PASS" if item["passed"] else "FAIL"
        print(f"{status} {item['name']}")
    print(f"offline_tests_passed={passed}/{len(cases)}")
    print(f"report={REPORT_PATH}")
    return 0 if passed == len(cases) else 1


if __name__ == "__main__":
    raise SystemExit(main())
