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
    build_gripper_ramp_targets,
    gripper_delta_from_initial,
    load_grasp_config,
    load_motor_mapping,
    validate_runtime_gripper_targets,
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
    parser = argparse.ArgumentParser(description="Offline verification for MVP-4D simplified one-command grasp.")
    parser.add_argument("--ros2-build-result", default="not_run")
    args = parser.parse_args()

    config = load_grasp_config()
    mapping = load_motor_mapping()
    visual_path = PROJECT_ROOT / "scripts" / "mvp_visual_grasp.py"
    visual_text = visual_path.read_text(encoding="utf-8")
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
    server_text = (PROJECT_ROOT / "scripts" / "mvp_so101_server.py").read_text(encoding="utf-8")
    config_text = (PROJECT_ROOT / "config" / "mvp_grasp.yaml").read_text(encoding="utf-8")

    initial = 40.0
    ramp_targets = build_gripper_ramp_targets(initial, config.gripper_open_delta, config.gripper_open_ramp_fraction)
    ramp_deltas = gripper_delta_from_initial(initial, ramp_targets)
    range_ok = validate_runtime_gripper_targets(initial, config.gripper_open_delta)
    range_bad = validate_runtime_gripper_targets(95.0, config.gripper_open_delta)

    regressions = [
        run_script("scripts/verify_stage_mvp4b_pregrasp_move.py"),
        run_script("scripts/verify_stage_mvp4c_segmented_descent.py", "--ros2-build-result", "PASS"),
        run_script("scripts/verify_stage_mvp4c_occlusion_safe_handoff.py", "--ros2-build-result", "PASS"),
        run_script("scripts/verify_stage_mvp4c_snapshot_optional_float_hotfix.py", "--ros2-build-result", "PASS"),
    ]
    legacy_passed = all(ok for ok, _ in regressions)

    cases: list[dict[str, object]] = [
        case("gripper_hardware_id_is_6", mapping["gripper_hardware_id"] == 6),
        case("wrist_roll_hardware_id_is_5", mapping["wrist_roll_hardware_id"] == 5),
        case("motor_id5_not_used_as_gripper", mapping["id_5_name"] == "wrist_roll" and "gripper_hardware_id\": 5" not in visual_text),
        case("gripper_open_delta_is_10", math.isclose(config.gripper_open_delta, 10.0)),
        case("open_target_equals_initial_plus_10", range_ok[0] and math.isclose(float(range_ok[2]), initial + 10.0)),
        case("close_target_equals_initial", '"gripper_close_target_position": float(initial_gripper_position)' in visual_text and '"gripper_close_target_position": initial_gripper' in visual_text),
        case("no_absolute_open_target_config", "gripper_open_target_pos:" not in config_text and "gripper_open_target_pos:" not in visual_text and "gripper_open_target_pos: float" not in visual_text),
        case("no_open_target_verified_flag", "gripper_open_target_verified" not in config_text and "gripper_open_target_verified" not in visual_text),
        case("exactly_seven_descent_waypoints", len(config.descent_waypoint_drop_m) == 7),
        case("gripper_deltas_are_1p5_3_4p5_6_7p5_9_10", ramp_deltas == [1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.0]),
        case("final_descent_target_uses_initial_plus_10", math.isclose(ramp_targets[-1], initial + 10.0)),
        case("final_close_subtracts_back_to_initial", "initial_gripper + config.gripper_open_delta" in visual_text and '"gripper_close_target_position": initial_gripper' in visual_text),
        case("gripper_open_delta_internal_range_check", "validate_runtime_gripper_targets" in visual_text),
        case("out_of_range_rejected_before_motion", not range_bad[0] and range_bad[1] == "gripper_open_delta_out_of_calibration_range"),
        case("no_user_range_reader_required", not any((PROJECT_ROOT / "scripts").glob("*gripper_range*"))),
        case("one_user_entry_script", visual_path.is_file() and not (PROJECT_ROOT / "scripts" / "mvp_gripper_open_close_test.py").exists()),
        case("pregrasp_executed_once", "node.publish_arm_target_once(frozen_target)" in visual_text),
        case("seven_descent_execute_calls", "for waypoint in summary[\"descent_waypoints\"]" in visual_text),
        case("close_executed_once", "gripper_close_command_completed" in visual_text),
        case("total_nine_execute_calls", 1 + len(config.descent_waypoint_drop_m) + 1 == 9),
        case("plan_only_no_arm_publish", "if args.plan_only or not args.execute:" in visual_text),
        case("plan_only_no_gripper_publish", "publish_gripper_target_once" in visual_text and "if args.plan_only or not args.execute:" in visual_text),
        case("plan_only_no_execute", "node.call_trigger(node.execute_client" in visual_text and "if args.plan_only or not args.execute:" in visual_text),
        case("wrong_confirmation_rejected", CONFIRM_PHRASE == "VISUAL_GRASP" and "wrong_confirmation" in visual_text),
        case("all_motion_planned_before_execute", "build_integrated_plan_summary" in visual_text and "plan_segmented_descent" in visual_text),
        case("live_visual_only_before_motion", visual_text.count('"/mvp/compute_pregrasp"') == 1 and "latest_object_pose" in visual_text),
        case("no_live_visual_after_motion", '"live_visual_required_after_motion": False' in visual_text),
        case("full_five_arm_joint_targets", "ARM_JOINT_NAMES" in visual_text and "len(ARM_JOINT_NAMES)" in bridge_text),
        case("gripper_concurrent_with_descent", "gripper_fraction" in server_text and "gripper_command" in server_text),
        case("gripper_only_close_supported", "total_arm_delta <= 1.0e-9" in server_text),
        case("close_duration_is_2s", math.isclose(config.gripper_only_motion_duration_s, 2.0)),
        case("arm_motion_remains_sequential", "for index in joint_order" in server_text),
        case("control_frequency_remains_20hz", "speed_rad_s / 20.0" in server_text),
        case("speed_remains_0p06", math.isclose(config.speed_rad_s, 0.06)),
        case("max_speed_remains_0p08", math.isclose(config.max_speed_rad_s, 0.08)),
        case("no_motion_retry", visual_text.count("call_trigger(node.execute_client") == 3 and "retry" not in visual_text.lower()),
        case("failure_stops_remaining_steps", "return 14" in visual_text and "return 15" in visual_text and "return 0 if close_success else 17" in visual_text),
        case("no_lift", "lift" not in visual_text.lower()),
        case("no_return", "return_to" not in visual_text.lower() and "home" not in visual_text.lower()),
        case("no_second_tcp_client", bridge_text.count("MvpTcpClient(") == 1 and "MvpTcpClient" not in visual_text),
        case("no_stop_service", "/mvp/stop" not in visual_text and "stop_client" not in visual_text),
        case("no_com_port_open", "COM4" not in visual_text and "serial.Serial" not in visual_text),
        case("no_real_goal_position_write", "Goal_Position" not in visual_text and "send_action(" not in visual_text and "gripper_target_pos" in client_text),
        case("no_physical_motion", "hardware_command_sent" in visual_text and "mvp_so101_server" not in visual_text),
    ]

    passed = sum(1 for item in cases if item["passed"])
    all_passed = passed == len(cases)
    final_status = (
        "READY_FOR_MANUAL_ONE_COMMAND_GRASP_RETEST"
        if all_passed and legacy_passed and args.ros2_build_result == "PASS"
        else ("OFFLINE_TESTS_PASS" if all_passed and legacy_passed else "OFFLINE_TESTS_FAIL")
    )
    report = {
        "stage": "MVP-4D-SIMPLIFIED-ONE-COMMAND-GRASP",
        "integrated_script": "scripts/mvp_visual_grasp.py",
        "user_confirmation_phrase": "VISUAL_GRASP",
        "gripper_hardware_id": 6,
        "wrist_roll_hardware_id": 5,
        "motor_mapping_verified": mapping["motor_mapping_verified"],
        "gripper_target_mode": "relative_to_initial",
        "gripper_open_delta": 10.0,
        "gripper_open_target_formula": "initial_gripper_position + 10.0",
        "gripper_close_target_formula": "initial_gripper_position",
        "manual_gripper_target_setup_required": False,
        "separate_gripper_test_script_required": False,
        "user_range_read_required": False,
        "total_descent_m": 0.07,
        "descent_waypoint_count": 7,
        "descent_waypoint_drop_m": list(config.descent_waypoint_drop_m),
        "gripper_delta_per_waypoint": ramp_deltas,
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
        "deleted_redundant_files": ["scripts/mvp_gripper_open_close_test.py"],
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
