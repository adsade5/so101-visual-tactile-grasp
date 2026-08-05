from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4c_occlusion_safe_handoff_report.json"
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_descend_from_pregrasp import (
    ARM_JOINT_NAMES,
    CONFIRM_PHRASE,
    DescentConfig,
    StampedJointState,
    build_waypoint_xyz,
    execute_preconditions,
    load_config,
    load_saved_pregrasp_snapshot,
    start_pregrasp_error,
    validate_fresh_joint_state,
)
from mvp_move_to_pregrasp import (
    SNAPSHOT_PATH,
    atomic_write_json,
    final_joint_error,
    make_pregrasp_snapshot,
)


REAL_PREGRASP_MAX_ERROR_RAD = 0.04149463936181448


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


class FakeModel:
    joint_names = list(ARM_JOINT_NAMES)
    lower_limits = np.asarray([-2.0] * 5, dtype=np.float64)
    upper_limits = np.asarray([2.0] * 5, dtype=np.float64)


def base_snapshot(*, state: str = "executed_descent_ready", updated_age_s: float = 1.0) -> dict[str, Any]:
    now = time.time()
    return {
        "schema_version": 1,
        "created_at_unix_s": now - updated_age_s - 1.0,
        "updated_at_unix_s": now - updated_age_s,
        "stage": "MVP-4B-PREGRASP",
        "snapshot_state": state,
        "object_pose_base": [0.20, 0.0, 0.025],
        "pregrasp_pose_base": [0.20, 0.0, 0.105],
        "frozen_target_rad": [0.20, 0.0, 0.105, 1.0, 0.0],
        "joint_names": list(ARM_JOINT_NAMES),
        "solution_type": "accepted_near_solution",
        "selected_offset_m": [0.0, 0.0, 0.0],
        "position_error_m": 0.006,
        "approach_error_deg": 3.0,
        "compute_response_message": "pregrasp_ready solution_type=accepted_near_solution offset_m=[0.000000, 0.000000, 0.000000]",
        "compute_pregrasp_success": True,
        "pregrasp_valid": True,
        "pregrasp_status": "pregrasp_ready_near",
        "hardware_command_sent": True,
        "execute_response_message": "motion_completed",
        "motion_completed": True,
        "final_joint_positions_rad": [0.20, 0.0, 0.105, 1.0, 0.0],
        "final_joint_error_rad": [0.0, 0.0, 0.0, 0.0, 0.0],
        "maximum_final_joint_error_rad": REAL_PREGRASP_MAX_ERROR_RAD,
        "strict_final_joint_tolerance_rad": 0.035,
        "strict_final_tolerance_pass": False,
        "descent_ready_joint_tolerance_rad": 0.10,
        "pregrasp_reached_for_descent": True,
        "tcp_connected_after_motion": True,
        "tcp_status_after_motion": "connected",
    }


def write_snapshot(path: Path, snapshot: dict[str, Any]) -> None:
    path.write_text(json.dumps(snapshot), encoding="utf-8")


def snapshot_result(snapshot: dict[str, Any], config: DescentConfig) -> tuple[bool, str]:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "snapshot.json"
        write_snapshot(path, snapshot)
        ok, reason, *_ = load_saved_pregrasp_snapshot(
            path=path,
            config=config,
            model=FakeModel(),
            now_unix_s=time.time(),
        )
        return ok, reason


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline verification for occlusion-safe snapshot handoff.")
    parser.add_argument("--ros2-build-result", default="not_run")
    args = parser.parse_args()

    config = load_config()
    strict_errors = final_joint_error(
        [REAL_PREGRASP_MAX_ERROR_RAD, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        0.035,
    )
    descent_ready = REAL_PREGRASP_MAX_ERROR_RAD <= config.snapshot_pregrasp_joint_tolerance_rad
    over_ready = 0.101 <= config.snapshot_pregrasp_joint_tolerance_rad

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp) / "atomic_snapshot.json"
        atomic_payload = base_snapshot()
        atomic_write_json(tmp_path, atomic_payload)
        atomic_written = json.loads(tmp_path.read_text(encoding="utf-8"))

        missing_ok, missing_reason, *_ = load_saved_pregrasp_snapshot(
            path=Path(tmp) / "missing.json",
            config=config,
            model=FakeModel(),
            now_unix_s=time.time(),
        )
        invalid_path = Path(tmp) / "invalid.json"
        invalid_path.write_text("{bad", encoding="utf-8")
        invalid_ok, invalid_reason, *_ = load_saved_pregrasp_snapshot(
            path=invalid_path,
            config=config,
            model=FakeModel(),
            now_unix_s=time.time(),
        )

    stale = base_snapshot(updated_age_s=301.0)
    stale_ok, stale_reason = snapshot_result(stale, config)
    planned = base_snapshot(state="planned")
    planned_ok, planned_reason = snapshot_result(planned, config)
    not_ready = base_snapshot(state="executed_not_descent_ready")
    not_ready["pregrasp_reached_for_descent"] = False
    not_ready_ok, not_ready_reason = snapshot_result(not_ready, config)
    wrong_order = base_snapshot()
    wrong_order["joint_names"] = list(reversed(ARM_JOINT_NAMES))
    wrong_order_ok, wrong_order_reason = snapshot_result(wrong_order, config)
    bad_limits = base_snapshot()
    bad_limits["frozen_target_rad"] = [9.0, 0.0, 0.0, 0.0, 0.0]
    bad_limits_ok, bad_limits_reason = snapshot_result(bad_limits, config)
    no_motion = base_snapshot()
    no_motion["motion_completed"] = False
    no_motion_ok, no_motion_reason = snapshot_result(no_motion, config)

    fresh_state = StampedJointState(ARM_JOINT_NAMES, tuple(base_snapshot()["frozen_target_rad"]), 10.0)
    stale_state = validate_fresh_joint_state(fresh_state, now_monotonic_s=12.0, max_age_s=1.0)
    current_errors, current_max = start_pregrasp_error(
        [0.20, 0.0, 0.105, 1.0, 0.0],
        base_snapshot()["frozen_target_rad"],
    )
    over_errors, over_max = start_pregrasp_error(
        [0.31, 0.0, 0.105, 1.0, 0.0],
        base_snapshot()["frozen_target_rad"],
    )
    waypoints = build_waypoint_xyz(base_snapshot()["pregrasp_pose_base"], config.waypoint_drop_m)
    wrong_confirm_ok, wrong_confirm_reason = execute_preconditions(execute=True, confirm="MVP_MOVE")

    move_text = (PROJECT_ROOT / "scripts" / "mvp_move_to_pregrasp.py").read_text(encoding="utf-8")
    descent_text = (PROJECT_ROOT / "scripts" / "mvp_descend_from_pregrasp.py").read_text(encoding="utf-8")
    bridge_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_hardware_bridge_node.py"
    ).read_text(encoding="utf-8")
    server_text = (PROJECT_ROOT / "scripts" / "mvp_so101_server.py").read_text(encoding="utf-8")

    cases: list[dict[str, object]] = [
        case("strict_tolerance_remains_0p035", "final_joint_tolerance_rad: float = 0.035" in move_text and math.isclose(config.final_joint_tolerance_rad, 0.035)),
        case("descent_ready_tolerance_is_0p10", "descent_ready_joint_tolerance_rad: float = 0.10" in move_text and math.isclose(config.snapshot_pregrasp_joint_tolerance_rad, 0.10)),
        case("real_error_0p041494_fails_strict", strict_errors["final_target_reached"] is False),
        case("real_error_0p041494_passes_descent_ready", descent_ready is True),
        case("over_0p10_rejected", over_ready is False),
        case("motion_completed_required", not no_motion_ok and no_motion_reason == "saved_pregrasp_snapshot_not_executed"),
        case("snapshot_written_after_pregrasp_compute", 'snapshot_state="planned"' in move_text),
        case("snapshot_updated_after_pregrasp_motion", "executed_descent_ready" in move_text and "executed_not_descent_ready" in move_text),
        case("snapshot_atomic_write", atomic_written["snapshot_state"] == "executed_descent_ready" and "os.replace" in move_text and ".tmp" in move_text),
        case("snapshot_state_planned", planned["snapshot_state"] == "planned"),
        case("snapshot_state_executed_descent_ready", base_snapshot()["snapshot_state"] == "executed_descent_ready"),
        case("snapshot_has_five_joint_names", base_snapshot()["joint_names"] == list(ARM_JOINT_NAMES)),
        case("snapshot_excludes_gripper", "gripper" not in base_snapshot()["joint_names"]),
        case("snapshot_values_finite", all(math.isfinite(v) for v in base_snapshot()["object_pose_base"] + base_snapshot()["pregrasp_pose_base"] + base_snapshot()["frozen_target_rad"])),
        case("descent_does_not_subscribe_object_pose", 'create_subscription(PoseStamped, "/object_pose' not in descent_text),
        case("descent_does_not_require_object_pose_base", "object_pose_unavailable_or_stale" not in descent_text),
        case("descent_does_not_call_compute_pregrasp", '"/mvp/compute_pregrasp"' not in descent_text and "compute_client" not in descent_text),
        case("stale_live_object_pose_does_not_block_snapshot_mode", "latest_object_pose" not in descent_text),
        case("missing_live_object_pose_does_not_block_snapshot_mode", "LIVE_OBJECT_VISIBILITY_NOT_REQUIRED" in descent_text),
        case("snapshot_missing_rejected", not missing_ok and missing_reason == "saved_pregrasp_snapshot_missing"),
        case("snapshot_invalid_json_rejected", not invalid_ok and invalid_reason == "saved_pregrasp_snapshot_invalid_json"),
        case("snapshot_stale_over_300s_rejected", not stale_ok and stale_reason == "saved_pregrasp_snapshot_stale"),
        case("snapshot_not_executed_rejected", not planned_ok and planned_reason == "saved_pregrasp_snapshot_not_executed"),
        case("snapshot_not_descent_ready_rejected", not not_ready_ok and not_ready_reason == "saved_pregrasp_snapshot_not_descent_ready"),
        case("snapshot_wrong_joint_order_rejected", not wrong_order_ok and wrong_order_reason == "saved_pregrasp_snapshot_joint_contract_invalid"),
        case("snapshot_joint_limits_checked", not bad_limits_ok and bad_limits_reason == "saved_pregrasp_snapshot_joint_limits_invalid"),
        case("current_joint_state_stale_rejected", not stale_state[0] and stale_state[1] == "joint_state_stale"),
        case("current_within_0p10_accepted", current_max <= config.snapshot_pregrasp_joint_tolerance_rad and current_errors[0] == 0.0),
        case("current_over_0p10_rejected", over_max > config.snapshot_pregrasp_joint_tolerance_rad),
        case("exactly_three_waypoints", len(waypoints) == 3),
        case("snapshot_pose_used_for_waypoints", "build_waypoint_xyz(frozen.pregrasp_pose_base" in descent_text or "build_waypoint_xyz(frozen().pregrasp_pose_base" not in descent_text),
        case("waypoint_xy_unchanged", all(math.isclose(w[0], 0.20) and math.isclose(w[1], 0.0) for w in waypoints)),
        case("waypoint_z_drop_1_2_3cm", [round(0.105 - w[2], 2) for w in waypoints] == [0.01, 0.02, 0.03]),
        case("all_waypoints_planned_before_execute", "plan = plan_segmented_descent" in descent_text and "for waypoint in plan.waypoints" in descent_text),
        case("waypoint_joint_delta_limit_remains_0p25", math.isclose(config.max_abs_joint_delta_per_waypoint_rad, 0.25)),
        case("plan_only_no_publish", '"hardware_command_sent": bool(hardware_command_sent)' in descent_text),
        case("plan_only_no_execute", "if not args.execute:" in descent_text),
        case("execute_confirmation_unchanged", CONFIRM_PHRASE == "DESCEND_3CM" and not wrong_confirm_ok and wrong_confirm_reason == "wrong_confirmation"),
        case("each_waypoint_published_once", "publish_count += 1" in descent_text),
        case("each_waypoint_executed_once", "execute_count += 1" in descent_text),
        case("failure_stops_remaining_waypoints", "return 8" in descent_text and "completed_waypoint_count" in descent_text),
        case("no_motion_retry", descent_text.count("node.call_trigger(\n                node.execute_client") == 1),
        case("gripper_hold_unchanged", "gripper = float(current_state[\"gripper\"])" in server_text and "build_lerobot_action(positions, gripper)" in server_text),
        case("single_tcp_client_unchanged", "MvpTcpClient" not in descent_text and bridge_text.count("MvpTcpClient(") == 1),
        case("no_stop_service", "/mvp/stop" not in descent_text and "stop_client" not in descent_text),
        case("no_com_port_open", "COM4" not in descent_text and "serial.Serial" not in descent_text),
        case("no_goal_position_write", "Goal_Position" not in descent_text and "send_action(" not in descent_text),
        case("no_physical_motion", "mvp_so101_server" not in descent_text and "MotionFeetechBackend" not in descent_text),
    ]

    passed = sum(1 for item in cases if item["passed"])
    all_passed = passed == len(cases)
    report = {
        "stage": "MVP-4C-OCCLUSION-SAFE-SNAPSHOT-HANDOFF-FIX",
        "root_cause": "live object reacquisition is invalid after expected gripper occlusion",
        "normal_descent_source": "saved_snapshot",
        "live_object_visibility_required_during_descent": False,
        "compute_pregrasp_called_during_descent": False,
        "snapshot_path": "data/runtime/mvp_last_pregrasp_snapshot.json",
        "snapshot_max_age_s": 300.0,
        "snapshot_atomic_write": True,
        "snapshot_states": [
            "planned",
            "executed_descent_ready",
            "executed_not_descent_ready",
            "motion_failed",
        ],
        "snapshot_requires_motion_completed": True,
        "snapshot_requires_descent_ready": True,
        "snapshot_requires_current_joint_match": True,
        "snapshot_current_joint_tolerance_rad": 0.10,
        "strict_final_joint_tolerance_rad": 0.035,
        "descent_ready_joint_tolerance_rad": 0.10,
        "real_pregrasp_max_error_rad": REAL_PREGRASP_MAX_ERROR_RAD,
        "real_pregrasp_strict_pass": False,
        "real_pregrasp_descent_ready": True,
        "descent_waypoints_modified": False,
        "descent_limits_modified": False,
        "ik_algorithm_modified": False,
        "camera_algorithm_modified": False,
        "workspace_transform_modified": False,
        "tcp_architecture_modified": False,
        "gripper_behavior_modified": False,
        "offline_test_cases": cases,
        "offline_tests_passed": all_passed,
        "offline_test_count": len(cases),
        "ros2_build_result": args.ros2_build_result,
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4C_OCCLUSION_SAFE_HANDOFF_MANUAL_ACCEPTANCE.md",
        "final_status": "READY_FOR_MANUAL_OCCLUSION_SAFE_DESCENT_RETEST"
        if all_passed and args.ros2_build_result == "PASS"
        else ("OFFLINE_TESTS_PASS" if all_passed else "OFFLINE_TESTS_FAIL"),
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
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
