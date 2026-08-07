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
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4a_ik_multiseed_fix_report.json"
for package_path in (
    ROS_SRC / "so101_mvp_control",
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_control.pregrasp_planner import (
    ARM_JOINT_NAMES,
    REFERENCE_SEED_RAD,
    JointStateSnapshot,
    PoseSnapshot,
    build_seed_candidates,
    compute_pregrasp_plan,
    create_model,
    make_failure_message,
    make_success_message,
)
from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.joint_limits import joints_within_limits


def listf(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


def fk_object_pose(model, q_rad: np.ndarray, height: float, now: float) -> PoseSnapshot:
    tcp = np.asarray(forward_kinematics(model, q_rad)["position_m"], dtype=np.float64)
    return PoseSnapshot("base_link", tcp - np.asarray([0.0, 0.0, height]), now)


def always_fail_solver(*args, **kwargs) -> dict[str, object]:
    del args, kwargs
    return {
        "success": False,
        "joint_positions_rad": None,
        "iterations": 200,
        "position_error_m": 0.123,
        "approach_error_deg": 45.0,
        "reason": "forced_failure",
    }


def main() -> int:
    model = create_model(PROJECT_ROOT)
    now = time.monotonic()
    height = 0.08
    reference_pose = fk_object_pose(model, REFERENCE_SEED_RAD, height, now)
    pregrasp = reference_pose.position_m + np.asarray([0.0, 0.0, height])
    joint_state = JointStateSnapshot(
        names=ARM_JOINT_NAMES,
        positions_rad=np.asarray([0.1, -0.3, 0.3, 1.2, 0.0], dtype=np.float64),
        received_monotonic_s=now,
    )
    stale_joint_state = JointStateSnapshot(
        names=ARM_JOINT_NAMES,
        positions_rad=joint_state.positions_rad,
        received_monotonic_s=now - 2.0,
    )

    seeds, _ = build_seed_candidates(
        model,
        pregrasp,
        None,
        now_monotonic_s=now,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    seeds_with_joint, joint_available = build_seed_candidates(
        model,
        pregrasp,
        joint_state,
        now_monotonic_s=now,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    seeds_with_stale, stale_available = build_seed_candidates(
        model,
        pregrasp,
        stale_joint_state,
        now_monotonic_s=now,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )

    cases: list[dict[str, object]] = []
    cases.append(case("reference_seed_still_supported", any(seed.source == "reference" for seed in seeds)))
    cases.append(case("target_yaw_seed_created", any(seed.source == "target_yaw" for seed in seeds)))
    target_yaw_seed = next(seed for seed in seeds if seed.source == "target_yaw")
    cases.append(
        case(
            "target_yaw_uses_atan2",
            abs(float(target_yaw_seed.q_rad[0]) - math.atan2(float(pregrasp[1]), float(pregrasp[0]))) < 1.0e-12,
        )
    )
    cases.append(case("all_seeds_within_limits", all(joints_within_limits(model, seed.q_rad) for seed in seeds)))
    rounded = [tuple(round(float(value), 12) for value in seed.q_rad.tolist()) for seed in seeds]
    cases.append(case("seed_list_deduplicated", len(rounded) == len(set(rounded))))
    cases.append(case("seed_count_at_most_7", len(seeds_with_joint) <= 7))
    cases.append(
        case(
            "fresh_joint_state_seed_first",
            joint_available
            and seeds_with_joint[0].source == "joint_state"
            and np.allclose(seeds_with_joint[0].q_rad, joint_state.positions_rad),
        )
    )
    cases.append(
        case(
            "stale_joint_state_not_used",
            not stale_available and all(seed.source != "joint_state" for seed in seeds_with_stale),
        )
    )

    side_q = REFERENCE_SEED_RAD.copy()
    side_q[0] = 0.45
    side_pose = fk_object_pose(model, side_q, height, now)
    side_plan = compute_pregrasp_plan(
        model=model,
        object_pose=side_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("target_yaw_seed_solves_side_target", side_plan.success, {"seed_source": side_plan.seed_source}))

    reference_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("reference_seed_solves_reference_target", reference_plan.success))

    call_count = {"value": 0}
    fallback_solution = np.asarray(reference_plan.joint_positions_rad, dtype=np.float64)

    def second_success_solver(model_arg, target, seed, *args, **kwargs):
        del model_arg, target, seed, args, kwargs
        call_count["value"] += 1
        if call_count["value"] == 1:
            return {
                "success": False,
                "joint_positions_rad": None,
                "iterations": 200,
                "position_error_m": 0.5,
                "approach_error_deg": 80.0,
                "reason": "first_forced_failure",
            }
        return {
            "success": True,
            "joint_positions_rad": listf(fallback_solution),
            "iterations": 0,
            "position_error_m": 0.0,
            "approach_error_deg": 0.0,
            "reason": "forced_second_success",
            "limit_hit_joints": [],
        }

    fallback_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=second_success_solver,
    )
    cases.append(
        case(
            "fallback_to_later_seed",
            fallback_plan.success and fallback_plan.selected_attempt_index == 2,
            {"selected_attempt": fallback_plan.selected_attempt_index},
        )
    )

    fail_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=always_fail_solver,
    )
    failure_message = make_failure_message(fail_plan)
    cases.append(case("all_seed_failure_reason", fail_plan.reason == "ik_failed_all_candidates"))
    cases.append(case("object_xyz_in_failure_message", "object_xyz_m=[" in failure_message))
    cases.append(case("pregrasp_xyz_in_failure_message", "requested_xyz=[" in failure_message))
    cases.append(case("attempt_count_in_failure_message", "attempt_count=" in failure_message))
    cases.append(case("success_reports_seed_source", "seed_source=" in make_success_message(reference_plan)))
    cases.append(
        case(
            "fk_position_validation",
            reference_plan.position_error_m is not None and reference_plan.position_error_m <= 0.002,
            {"position_error_m": reference_plan.position_error_m},
        )
    )
    cases.append(
        case(
            "fk_approach_validation",
            reference_plan.approach_error_deg is not None and reference_plan.approach_error_deg <= 5.0,
            {"approach_error_deg": reference_plan.approach_error_deg},
        )
    )
    original_target = reference_pose.position_m.copy()
    _ = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=height,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("target_not_clipped", np.allclose(reference_pose.position_m, original_target)))

    workspace_diff = subprocess.run(
        ["git", "diff", "--quiet", "--", "config/workspace_to_base.json"],
        cwd=PROJECT_ROOT,
        text=True,
    )
    cases.append(case("workspace_transform_not_modified", workspace_diff.returncode == 0))

    checked_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "pregrasp_planner.py",
            PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "mvp_pregrasp_planner_node.py",
            PROJECT_ROOT / "scripts" / "mvp_pregrasp_replay.py",
        )
    )
    node_source = (PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "mvp_pregrasp_planner_node.py").read_text(encoding="utf-8")
    cases.append(case("no_joint_target_publish", "/mvp/joint_target" not in node_source))
    cases.append(case("no_execute_service_call", "/mvp/execute_target" not in checked_sources))
    cases.append(
        case(
            "no_tcp_connection",
            all(token not in checked_sources for token in ("socket.", "create_connection", "MvpTcpClient")),
        )
    )
    cases.append(case("no_com_port_open", all(token not in checked_sources for token in ("COM4", "serial.Serial", "Serial("))))
    cases.append(
        case(
            "no_physical_motion",
            all(token not in checked_sources for token in ("send_action(", "move_joints", "execute_target")),
        )
    )

    passed = sum(1 for item in cases if item["passed"])
    report = {
        "stage": "MVP-4A-IK-MULTISEED-FIX",
        "original_live_error": "ik_failed",
        "exact_failure_path": "mvp_pregrasp_planner_node.handle_compute_pregrasp -> pregrasp_planner.compute_pregrasp_plan -> so101_mvp_kinematics.ik.solve_ik",
        "previous_seed_count": 1,
        "previous_seed_source": "joint_state if fresh by name mapping else reference",
        "multiseed_enabled": True,
        "maximum_seed_count": 7,
        "seed_sources": [
            "joint_state",
            "target_yaw",
            "reference",
            "target_yaw_plus_15deg",
            "target_yaw_minus_15deg",
            "elbow_high",
            "elbow_low",
        ],
        "target_yaw_seed_formula": "shoulder_pan=clamp_to_urdf_limits(atan2(pregrasp_y, pregrasp_x)); remaining joints=[-0.35,0.35,1.22,0.0]",
        "ik_algorithm_modified": False,
        "workspace_transform_modified": False,
        "target_clipping_added": False,
        "detailed_diagnostics_added": True,
        "replay_script": "scripts/mvp_pregrasp_replay.py",
        "offline_test_cases": cases,
        "offline_tests_passed": passed == len(cases),
        "offline_test_count": len(cases),
        "ros2_build_result": "not_run",
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "final_status": "OFFLINE_TESTS_PASS" if passed == len(cases) else "OFFLINE_TESTS_FAIL",
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    for item in cases:
        print(("PASS" if item["passed"] else "FAIL") + f" {item['name']}")
    print(f"offline_tests_passed={passed}/{len(cases)}")
    print(f"report={REPORT_PATH}")
    return 0 if passed == len(cases) else 1


if __name__ == "__main__":
    raise SystemExit(main())
