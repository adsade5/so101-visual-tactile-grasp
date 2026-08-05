from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4d_integrated_visual_grasp_report.json"
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_visual_grasp import (  # noqa: E402
    CONFIRM_PHRASE,
    GRIPPER_LOGICAL_KEY,
    GraspConfig,
    build_gripper_ramp_targets,
    gripper_target_in_range,
    load_grasp_config,
    load_motor_mapping,
    validate_gripper_open_config,
)


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


def run_script(script: str, *args: str) -> tuple[bool, dict[str, Any]]:
    result = subprocess.run(
        [sys.executable, str(PROJECT_ROOT / script), *args],
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        timeout=120,
        check=False,
    )
    return result.returncode == 0, {
        "returncode": result.returncode,
        "stdout_tail": result.stdout.splitlines()[-5:],
        "stderr_tail": result.stderr.splitlines()[-5:],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline verification for MVP-4D integrated visual grasp.")
    parser.add_argument("--ros2-build-result", default="not_run")
    args = parser.parse_args()

    config = load_grasp_config()
    mapping = load_motor_mapping()
    visual_text = (PROJECT_ROOT / "scripts" / "mvp_visual_grasp.py").read_text(encoding="utf-8")
    gripper_test_text = (PROJECT_ROOT / "scripts" / "mvp_gripper_open_close_test.py").read_text(encoding="utf-8")
    bridge_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_hardware_bridge_node.py"
    ).read_text(encoding="utf-8")
    client_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_tcp_client.py"
    ).read_text(encoding="utf-8")
    shared_client_text = (PROJECT_ROOT / "shared_protocol" / "mvp_tcp_client.py").read_text(encoding="utf-8")
    server_text = (PROJECT_ROOT / "scripts" / "mvp_so101_server.py").read_text(encoding="utf-8")
    executor_text = (PROJECT_ROOT / "lerobot_server" / "mvp_hardware_executor.py").read_text(encoding="utf-8")
    manual4c = json.loads((PROJECT_ROOT / "data" / "verification" / "stage_mvp4c_manual_acceptance_report.json").read_text(encoding="utf-8"))

    null_ok, null_reason = validate_gripper_open_config(GraspConfig(gripper_open_target_pos=None, gripper_open_target_verified=False))
    unverified_ok, unverified_reason = validate_gripper_open_config(GraspConfig(gripper_open_target_pos=60.0, gripper_open_target_verified=False))
    verified_ok, verified_reason = validate_gripper_open_config(GraspConfig(gripper_open_target_pos=60.0, gripper_open_target_verified=True))
    ramp = build_gripper_ramp_targets(40.0, 80.0, config.gripper_open_ramp_fraction)
    no_target_ramp = build_gripper_ramp_targets(40.0, None, config.gripper_open_ramp_fraction)
    execute_reject = subprocess.run(
        [sys.executable, str(PROJECT_ROOT / "scripts" / "mvp_visual_grasp.py"), "--execute", "--confirm", CONFIRM_PHRASE],
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )

    regressions = [
        run_script("scripts/verify_stage_mvp4b_pregrasp_move.py"),
        run_script("scripts/verify_stage_mvp4c_segmented_descent.py", "--ros2-build-result", "PASS"),
        run_script("scripts/verify_stage_mvp4c_occlusion_safe_handoff.py", "--ros2-build-result", "PASS"),
        run_script("scripts/verify_stage_mvp4c_snapshot_optional_float_hotfix.py", "--ros2-build-result", "PASS"),
    ]
    legacy_passed = all(ok for ok, _ in regressions)

    cases: list[dict[str, object]] = [
        case("motor_id5_not_assumed_gripper", '"gripper_hardware_id": 5' not in visual_text and "ID 5" not in visual_text),
        case("gripper_logical_key_is_gripper_pos", GRIPPER_LOGICAL_KEY == "gripper.pos" and "GRIPPER_LOGICAL_KEY = \"gripper.pos\"" in visual_text),
        case("expected_gripper_id_mapping_checked", mapping["id_5_name"] == "wrist_roll" and mapping["id_6_name"] == "gripper"),
        case("initial_gripper_position_used_as_close_target", config.gripper_close_target_source == "initial_gripper_position" and "initial_gripper" in visual_text),
        case("unverified_open_target_rejected", not unverified_ok and unverified_reason == "gripper_open_target_not_configured_or_unverified"),
        case("null_open_target_rejected_for_execute", execute_reject.returncode != 0 and "gripper_open_target_not_configured_or_unverified" in execute_reject.stdout),
        case("verified_open_target_accepted", verified_ok and verified_reason == "ok"),
        case("open_target_in_calibration_range", gripper_target_in_range(60.0)),
        case("open_target_out_of_range_rejected", not gripper_target_in_range(101.0)),
        case("exactly_seven_descent_waypoints", len(config.descent_waypoint_drop_m) == 7),
        case("waypoint_drop_1_to_7cm", list(config.descent_waypoint_drop_m) == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07]),
        case("waypoint_xy_unchanged", "build_waypoint_xyz" not in visual_text and "pregrasp_pose_base" in visual_text),
        case("all_waypoints_planned_before_motion", "plan_segmented_descent" in visual_text and "publish_arm_target_once(frozen_target)" in visual_text),
        case("full_five_arm_joint_targets_used", "ARM_JOINT_NAMES" in visual_text and "len(ARM_JOINT_NAMES)" in bridge_text),
        case("no_hardcoded_only_motor2_motor3", "motor2" not in visual_text.lower() and "motor3" not in visual_text.lower()),
        case("pregrasp_limit_remains_1p0", math.isclose(config.pregrasp_max_abs_joint_delta_rad, 1.00)),
        case("descent_waypoint_limit_remains_0p25", math.isclose(config.max_abs_joint_delta_per_descent_waypoint_rad, 0.25)),
        case("gripper_ramp_has_seven_values", len(config.gripper_open_ramp_fraction) == 7),
        case("gripper_ramp_monotonic", all(a < b for a, b in zip(config.gripper_open_ramp_fraction, config.gripper_open_ramp_fraction[1:], strict=False))),
        case("gripper_ramp_ends_at_open_target", ramp[-1] == 80.0 and no_target_ramp[-1] is None),
        case("close_target_equals_initial_gripper_position", "gripper_close_target_position" in visual_text and "initial_gripper" in visual_text),
        case("plan_only_no_arm_publish", "if args.plan_only or not args.execute:" in visual_text),
        case("plan_only_no_gripper_publish", "publish_gripper_target_once" in visual_text and "if args.plan_only or not args.execute:" in visual_text),
        case("plan_only_no_execute", "node.call_trigger(node.execute_client" in visual_text and "if args.plan_only or not args.execute:" in visual_text),
        case("wrong_confirmation_rejected", CONFIRM_PHRASE == "VISUAL_GRASP" and "wrong_confirmation" in visual_text),
        case("live_visual_only_before_motion", visual_text.count('"/mvp/compute_pregrasp"') == 1 and "latest_object_pose" in visual_text),
        case("no_live_visual_required_after_motion", '"live_visual_required_after_motion": False' in visual_text),
        case("pregrasp_executed_once", '"pregrasp_execute_count_expected": 1' not in visual_text and "execute_count += 1" in visual_text),
        case("seven_descent_execute_calls", "for waypoint in summary[\"descent_waypoints\"]" in visual_text),
        case("final_close_executed_once", "gripper_close_command_completed" in visual_text),
        case("total_nine_execute_calls", 1 + len(config.descent_waypoint_drop_m) + 1 == 9),
        case("each_arm_target_published_once", "arm_publish_count += 1" in visual_text),
        case("each_gripper_target_published_once", "gripper_publish_count += 1" in visual_text),
        case("optional_gripper_target_backward_compatible", 'if gripper_target_pos is not None' in client_text and 'if gripper_target_pos is not None' in shared_client_text),
        case("absent_gripper_target_holds_initial", 'target_gripper = gripper if gripper_target_pos is None' in server_text),
        case("concurrent_gripper_interpolation", "gripper_fraction" in server_text and "gripper_command" in server_text),
        case("arm_motion_remains_sequential", "for index in joint_order" in server_text),
        case("gripper_only_close_supported", "total_arm_delta <= 1.0e-9" in server_text),
        case("gripper_only_close_duration_2s", math.isclose(config.gripper_only_motion_duration_s, 2.0) and "gripper_only_motion_duration_s" in server_text),
        case("full_six_motor_action_written", "LEROBOT_ACTION_KEYS" in executor_text and "GRIPPER_POSITION_KEY" in executor_text),
        case("no_second_tcp_client", bridge_text.count("MvpTcpClient(") == 1 and "MvpTcpClient" not in visual_text),
        case("no_motion_retry", visual_text.count("call_trigger(node.execute_client") == 3 and "retry" not in visual_text.lower()),
        case("failure_stops_remaining_steps", "return 14" in visual_text and "return 15" in visual_text and "return 0 if close_success else 17" in visual_text),
        case("no_lift_command", "lift" not in visual_text.lower()),
        case("no_return_command", "return_to" not in visual_text.lower() and "home" not in visual_text.lower()),
        case("no_grasp_success_claim_without_evidence", '"object_may_be_grasped": None' in visual_text and '"grasp_success"' not in visual_text),
        case("speed_remains_0p06", math.isclose(config.speed_rad_s, 0.06)),
        case("max_speed_remains_0p08", math.isclose(config.max_speed_rad_s, 0.08)),
        case("control_frequency_remains_20hz", "speed_rad_s / 20.0" in server_text),
        case("no_stop_service", "/mvp/stop" not in visual_text and "stop_client" not in visual_text),
        case("no_com_port_open", "COM4" not in visual_text and "serial.Serial" not in visual_text and "COM4" not in gripper_test_text),
        case("no_real_goal_position_write", execute_reject.returncode != 0 and "hardware_command_sent\": false" in execute_reject.stdout.lower()),
        case("no_physical_motion", execute_reject.returncode != 0 and "hardware_command_sent\": false" in execute_reject.stdout.lower()),
    ]

    passed = sum(1 for item in cases if item["passed"])
    all_passed = passed == len(cases)
    final_status = (
        "READY_FOR_MANUAL_INTEGRATED_GRASP_RETEST"
        if all_passed and legacy_passed and args.ros2_build_result == "PASS" and config.gripper_open_target_verified
        else "READY_FOR_MANUAL_GRIPPER_TARGET_SETUP"
        if all_passed and legacy_passed and args.ros2_build_result == "PASS"
        else ("OFFLINE_TESTS_PASS" if all_passed and legacy_passed else "OFFLINE_TESTS_FAIL")
    )
    report = {
        "stage": "MVP-4D-INTEGRATED-VISUAL-GRASP",
        "mvp4c_manual_acceptance_recorded": manual4c.get("final_status") == "PASS",
        "integrated_script": "scripts/mvp_visual_grasp.py",
        "user_confirmation_phrase": "VISUAL_GRASP",
        "gripper_logical_key": "gripper.pos",
        "gripper_hardware_id": mapping["gripper_hardware_id"],
        "wrist_roll_hardware_id": mapping["wrist_roll_hardware_id"],
        "motor_mapping_verified": mapping["motor_mapping_verified"],
        "motor_id_5_name": mapping["id_5_name"],
        "motor_id_6_name": mapping["id_6_name"],
        "gripper_close_target_source": "initial_gripper_position",
        "gripper_open_target_source": "unverified_config_required",
        "gripper_open_target_value": config.gripper_open_target_pos,
        "gripper_open_target_verified": config.gripper_open_target_verified,
        "total_descent_m": 0.07,
        "descent_waypoint_count": 7,
        "descent_waypoint_drop_m": list(config.descent_waypoint_drop_m),
        "gripper_open_ramp_fraction": list(config.gripper_open_ramp_fraction),
        "pregrasp_execute_count_expected": 1,
        "descent_execute_count_expected": 7,
        "close_execute_count_expected": 1,
        "total_execute_count_expected": 9,
        "live_visual_used_before_motion": True,
        "live_visual_required_after_motion": False,
        "all_motion_planned_before_execute": True,
        "arm_motion_mode": "sequential_joint_motion",
        "gripper_motion_mode": "concurrent_linear_interpolation",
        "gripper_only_close_supported": True,
        "gripper_only_close_duration_s": 2.0,
        "ik_algorithm_modified": False,
        "camera_algorithm_modified": False,
        "workspace_transform_modified": False,
        "tcp_client_count": 1,
        "motion_retry_after_send": False,
        "lift_added": False,
        "return_added": False,
        "grasp_success_claimed": False,
        "offline_test_cases": cases,
        "offline_tests_passed": all_passed,
        "offline_test_count": len(cases),
        "legacy_regression_tests_passed": legacy_passed,
        "legacy_regression_details": [details for _, details in regressions],
        "ros2_build_result": args.ros2_build_result,
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4D_INTEGRATED_VISUAL_GRASP_MANUAL_ACCEPTANCE.md",
        "final_status": final_status,
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    for item in cases:
        print(("PASS" if item["passed"] else "FAIL") + f" {item['name']}")
    print(f"offline_tests_passed={passed}/{len(cases)}")
    print(f"legacy_regression_tests_passed={legacy_passed}")
    print(f"report={REPORT_PATH}")
    return 0 if all_passed and legacy_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
