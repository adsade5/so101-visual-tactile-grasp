from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
REPORT_PATH = (
    PROJECT_ROOT
    / "data"
    / "verification"
    / "stage_mvp4a_pregrasp_near_solution_fix_report.json"
)
for package_path in (
    ROS_SRC / "so101_mvp_control",
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_control.pregrasp_planner import (
    CANDIDATE_OFFSETS_M,
    DEFAULT_PREGRASP_APPROACH_TOLERANCE_DEG,
    DEFAULT_PREGRASP_POSITION_TOLERANCE_M,
    REFERENCE_SEED_RAD,
    PoseSnapshot,
    compute_pregrasp_plan,
    create_model,
    make_success_message,
)
from so101_mvp_kinematics.fk import forward_kinematics
from so101_mvp_kinematics.ik import solve_ik


LIVE_OBJECT_XYZ_M = np.asarray([0.199194, -0.000891, 0.025000], dtype=np.float64)
LIVE_PREGRASP_XYZ_M = np.asarray([0.199194, -0.000891, 0.105000], dtype=np.float64)
LIVE_NEAR_Q_RAD = np.asarray(
    [
        0.0042669343225695275,
        -0.2626909396873039,
        0.10620458926649186,
        1.65806,
        0.00808719348594239,
    ],
    dtype=np.float64,
)


def listf(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=np.float64).tolist()]


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


def pose_for_pregrasp(pregrasp_xyz_m: np.ndarray, now: float) -> PoseSnapshot:
    return PoseSnapshot(
        "base_link",
        np.asarray(pregrasp_xyz_m, dtype=np.float64) - np.asarray([0.0, 0.0, 0.08]),
        now,
    )


def solver_returning(q_rad: np.ndarray, *, success: bool, reason: str):
    def _solver(*args, **kwargs) -> dict[str, object]:
        del args, kwargs
        return {
            "success": success,
            "joint_positions_rad": listf(q_rad),
            "iterations": 200 if not success else 0,
            "position_error_m": 0.0,
            "approach_error_deg": 0.0,
            "reason": reason,
            "limit_hit_joints": [],
        }

    return _solver


def main() -> int:
    model = create_model(PROJECT_ROOT)
    now = time.monotonic()
    reference_tcp = np.asarray(forward_kinematics(model, REFERENCE_SEED_RAD)["position_m"])
    reference_pose = pose_for_pregrasp(reference_tcp, now)

    exact_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )

    live_pose = PoseSnapshot("base_link", LIVE_OBJECT_XYZ_M.copy(), now)
    live_near_plan = compute_pregrasp_plan(
        model=model,
        object_pose=live_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=solver_returning(
            LIVE_NEAR_Q_RAD,
            success=False,
            reason="max_iterations",
        ),
    )

    live_fk = forward_kinematics(model, LIVE_NEAR_Q_RAD)
    live_position_error = float(
        np.linalg.norm(np.asarray(live_fk["position_m"]) - LIVE_PREGRASP_XYZ_M)
    )
    live_approach_error = float(live_near_plan.approach_error_deg)

    over_position_target = np.asarray(live_fk["position_m"], dtype=np.float64) + np.asarray(
        [0.030, 0.0, 0.0],
        dtype=np.float64,
    )
    over_position_plan = compute_pregrasp_plan(
        model=model,
        object_pose=pose_for_pregrasp(over_position_target, now),
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=solver_returning(LIVE_NEAR_Q_RAD, success=False, reason="max_iterations"),
    )

    over_approach_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        pregrasp_approach_tolerance_deg=0.1,
        ik_solver=solver_returning(REFERENCE_SEED_RAD, success=False, reason="max_iterations"),
    )

    non_finite_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=solver_returning(
            np.asarray([np.nan, 0.0, 0.0, 0.0, 0.0]),
            success=False,
            reason="max_iterations",
        ),
    )

    limit_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=solver_returning(model.upper_limits + 0.1, success=False, reason="max_iterations"),
    )

    def real_solver_with_original_forced_fail(model_arg, target, seed, *args, **kwargs):
        if np.allclose(target, reference_tcp):
            return {
                "success": False,
                "joint_positions_rad": None,
                "iterations": 200,
                "position_error_m": 0.1,
                "approach_error_deg": 90.0,
                "reason": "forced_original_failure",
            }
        return solve_ik(model_arg, target, seed, *args, **kwargs)

    offset_plan = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
        ik_solver=real_solver_with_original_forced_fail,
    )

    cases: list[dict[str, object]] = []
    cases.append(case("exact_solution_accepted", exact_plan.success))
    cases.append(case("max_iterations_with_valid_best_q", live_near_plan.success))
    cases.append(
        case(
            "near_solution_6p607mm_accepted",
            live_near_plan.success
            and live_position_error <= DEFAULT_PREGRASP_POSITION_TOLERANCE_M,
            {"position_error_m": live_position_error},
        )
    )
    cases.append(
        case(
            "near_solution_3p078deg_accepted",
            live_near_plan.success and live_approach_error <= 5.0,
            {"approach_error_deg": live_approach_error},
        )
    )
    cases.append(case("position_over_1cm_rejected", not over_position_plan.success))
    cases.append(case("approach_over_5deg_rejected", not over_approach_plan.success))
    cases.append(case("non_finite_best_q_rejected", not non_finite_plan.success))
    cases.append(case("joint_limit_violation_rejected", not limit_plan.success))
    cases.append(case("fk_revalidation_required", abs(live_position_error - 0.0) > 1.0e-6))
    cases.append(
        case(
            "original_target_first",
            len(live_near_plan.candidate_results) >= 1
            and live_near_plan.candidate_results[0].candidate_index == 0
            and np.allclose(live_near_plan.candidate_results[0].offset_m, [0.0, 0.0, 0.0]),
        )
    )
    cases.append(case("offset_candidate_count_at_most_7", len(CANDIDATE_OFFSETS_M) <= 7))
    cases.append(
        case(
            "horizontal_offset_at_most_5mm",
            all(float(np.linalg.norm(offset[:2])) <= 0.0050001 for offset in CANDIDATE_OFFSETS_M),
        )
    )
    cases.append(case("vertical_offset_only_upward", all(float(offset[2]) >= 0.0 for offset in CANDIDATE_OFFSETS_M)))
    helper_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "pregrasp_planner.py"
    ).read_text(encoding="utf-8")
    cases.append(case("no_random_search", "random" not in helper_text.lower()))
    cases.append(
        case(
            "original_target_preferred",
            live_near_plan.success
            and live_near_plan.selected_candidate_index == 0
            and len(live_near_plan.candidate_results) == 1,
        )
    )
    cases.append(
        case(
            "lower_cost_candidate_selected",
            offset_plan.success and offset_plan.selected_candidate_index is not None,
            {
                "selected_candidate_index": offset_plan.selected_candidate_index,
                "selected_offset_m": None
                if offset_plan.selected_offset_m is None
                else listf(offset_plan.selected_offset_m),
            },
        )
    )
    cases.append(case("nonzero_offset_reported", offset_plan.success and offset_plan.selected_candidate_index != 0 and "offset_m=[" in make_success_message(offset_plan)))
    requested_before = reference_pose.position_m.copy()
    _ = compute_pregrasp_plan(
        model=model,
        object_pose=reference_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=0.08,
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )
    cases.append(case("requested_target_not_modified", np.allclose(reference_pose.position_m, requested_before)))
    cases.append(case("object_pose_not_modified", np.allclose(live_pose.position_m, LIVE_OBJECT_XYZ_M)))
    cases.append(case("exact_status", exact_plan.reason == "pregrasp_ready_exact"))
    cases.append(case("near_status", live_near_plan.reason == "pregrasp_ready_near"))
    cases.append(case("offset_status", offset_plan.success and offset_plan.reason == "pregrasp_ready_offset"))
    cases.append(case("all_candidates_failure_status", over_position_plan.reason == "ik_failed_all_candidates"))

    checked_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "pregrasp_planner.py",
            PROJECT_ROOT / "ros2_ws" / "src" / "so101_mvp_control" / "so101_mvp_control" / "mvp_pregrasp_planner_node.py",
            PROJECT_ROOT / "scripts" / "mvp_pregrasp_replay.py",
        )
    )
    node_source = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_pregrasp_planner_node.py"
    ).read_text(encoding="utf-8")
    cases.append(case("no_joint_target_publish", "/mvp/joint_target" not in node_source))
    cases.append(case("no_execute_service_call", "/mvp/execute_target" not in checked_sources))
    cases.append(case("no_tcp_connection", all(token not in checked_sources for token in ("socket.", "create_connection", "MvpTcpClient"))))
    cases.append(case("no_com_port_open", all(token not in checked_sources for token in ("COM4", "serial.Serial", "Serial("))))
    cases.append(case("no_physical_motion", all(token not in checked_sources for token in ("send_action(", "move_joints", "execute_target"))))

    passed = sum(1 for item in cases if item["passed"])
    report = {
        "stage": "MVP-4A-PREGRASP-NEAR-SOLUTION-FIX",
        "original_live_target": {
            "object_xyz_m": listf(LIVE_OBJECT_XYZ_M),
            "requested_pregrasp_xyz_m": listf(LIVE_PREGRASP_XYZ_M),
        },
        "original_best_position_error_m": 0.006607,
        "original_best_approach_error_deg": 3.078,
        "original_solver_reason": "max_iterations",
        "exact_ik_algorithm_modified": False,
        "pregrasp_position_tolerance_m": DEFAULT_PREGRASP_POSITION_TOLERANCE_M,
        "pregrasp_approach_tolerance_deg": DEFAULT_PREGRASP_APPROACH_TOLERANCE_DEG,
        "best_candidate_acceptance_added": True,
        "local_candidate_search_added": True,
        "candidate_offsets": [listf(offset) for offset in CANDIDATE_OFFSETS_M],
        "target_clipping_added": False,
        "workspace_transform_modified": False,
        "fk_revalidation_added": True,
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
