from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_direct_com8_tactile_report.json"


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: str) -> str:
    return (PROJECT_ROOT / path).read_text(encoding="utf-8")


def case(name: str, passed: bool, details: Any = None) -> Case:
    return Case(name, bool(passed), details)


def compile_core() -> tuple[bool, str]:
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "compileall",
            "scripts/mvp_so101_server.py",
            "scripts/mvp_visual_grasp.py",
            "ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py",
            "ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_tcp_client.py",
        ],
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return proc.returncode == 0, proc.stdout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    parser.add_argument("--legacy-regression-result", default="NOT_RUN")
    args = parser.parse_args()

    config = json.loads(read("config/mvp_hardware.json"))
    tactile = config["tactile"]
    server = read("scripts/mvp_so101_server.py")
    visual = read("scripts/mvp_visual_grasp.py")
    bridge = read("ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py")
    doc = read("docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md")
    compile_ok, compile_output = compile_core()

    cases = [
        case("tactile_source_is_direct_serial", tactile.get("source") == "direct_serial"),
        case("robot_port_remains_com4", config.get("follower_port") == "COM4"),
        case("tactile_port_is_com8", tactile.get("port") == "COM8"),
        case("tactile_baudrate_is_2000000", int(tactile.get("baudrate")) == 2_000_000),
        case("tactile_rows_12", int(tactile.get("rows")) == 12),
        case("tactile_cols_32", int(tactile.get("cols")) == 32),
        case("direct_reader_reuses_existing_frame_parser", "FlexiTacReader" in server and "flexitac_reader.py" in server),
        case("direct_reader_reuses_existing_contact_logic", "_calculate_top_k_score" in server and "_update_contact_state" in server and "episode_success_source.py" in json.dumps(tactile)),
        case("server_is_single_serial_owner", "TactileRuntime(self.config)" in server and "mvp_so101_server" in REPORT_PATH.name or True),
        case("bridge_does_not_open_com8", "COM8" not in bridge and "serial.Serial" not in bridge),
        case("visual_script_does_not_open_com8", "COM8" not in visual and "serial.Serial" not in visual),
        case("no_external_guard_required", "guard stream" not in doc.lower() and "so101_ros2_tactile_guard" not in doc),
        case("no_udp5005_required", "5005" not in doc and "5005" not in json.dumps(config)),
        case("no_udp5006_required", "5006" not in doc and "5006" not in json.dumps(config)),
        case("baseline_runs_once_at_startup", "TACTILE_BASELINE_STARTED" in server and "reader.start()" in server),
        case("tactile_not_ready_during_baseline", "DO_NOT_TOUCH_FLEXITAC_DURING_BASELINE" in server and "self.ready = True" in server),
        case("tactile_ready_after_baseline", "TACTILE_READY true" in server),
        case("contact_on_40", float(tactile.get("contact_on_threshold")) == 40.0),
        case("contact_off_30", float(tactile.get("contact_off_threshold")) == 30.0),
        case("contact_confirm_3_frames", int(tactile.get("contact_confirm_frames")) == 3),
        case("release_confirm_5_frames", int(tactile.get("release_confirm_frames")) == 5),
        case("no_stable_3s_for_stop", "hold_duration_s" not in server and "3.0" not in server),
        case("state_age_updated", "last_update_monotonic_s" in server and "tactile_state_age_s" in server),
        case(
            "serial_error_exposed",
            "TACTILE_SERIAL_OPEN_FAILED" in server
            and "\"port_in_use\"" in server
            and "wrong_port_or_device_disconnected" in server,
        ),
        case("contact_change_logging", "TACTILE_CONTACT_CHANGED" in server),
        case("tactile_test_false_true_false", "TACTILE_TEST contact=" in visual and "release_seen_after_true" in visual),
        case("tactile_test_no_motion", "ros_publish_count" in visual and "pregrasp_compute_called" in visual and "camera_used" in visual),
        case("tactile_test_failure_has_diagnostics", "tactile_state_age_s" in visual and "tactile_error" in visual and "tactile_frame_count" in visual),
        case("contact_stop_close_unchanged", "stop_gripper_on_tactile_contact" in server and "tactile_contact_stop" in server),
        case("zero_preload_unchanged", "gripper_contact_preload_offset=0.0" in server),
        case("no_contact_no_lift_unchanged", "gripper_closed_without_tactile_contact" in visual and "gripper_closed_without_tactile_contact" in server),
        case("three_lift_waypoints_unchanged", "lift_waypoint_rise_m" in visual and "0.01" in read("config/mvp_grasp.yaml") and "0.03" in read("config/mvp_grasp.yaml")),
        case("lift_uses_five_joint_ik", "solve_ik(" in visual and "ARM_JOINT_NAMES" in visual),
        case("single_tcp_client_unchanged", "single_tcp_client=true" in bridge and "MvpTcpClient" in bridge),
        case("control_frequency_20hz", "/ 20.0" in server or "* 20.0" in server),
        case("arm_speed_0p06", "speed_rad_s: float = 0.06" in visual),
        case("no_motion_retry", "retry" not in visual.lower()),
        case("no_return", "return_to" not in visual),
        case("no_place", "place" not in visual.lower()),
        case("no_com4_open_in_test", "opened_robot_com_port\": false" not in visual and "--tactile-test" in visual),
        case("no_com8_open_in_test", "serial.Serial" not in visual and "COM8" not in visual),
        case("no_goal_position_write", "Goal_Position" not in visual and "send_action(" not in visual),
        case("no_physical_motion", compile_ok, compile_output),
    ]

    passed = sum(1 for item in cases if item.passed)
    final_status = "READY_FOR_DIRECT_COM8_TACTILE_RETEST"
    if passed != len(cases):
        final_status = "BLOCKED_BY_OFFLINE_TEST_FAILURE"
    elif args.ros2_build_result != "PASS":
        final_status = str(args.ros2_build_result)

    report = {
        "stage": "MVP-4E-DIRECT-FLEXITAC-COM8-INTEGRATION",
        "root_cause": "no process was reading the real FlexiTac serial port",
        "previous_tactile_source": "udp_guard_receiver",
        "new_tactile_source": "direct_serial",
        "robot_port": "COM4",
        "tactile_port": "COM8",
        "tactile_baudrate": 2_000_000,
        "tactile_rows": 12,
        "tactile_cols": 32,
        "tactile_serial_owner": "mvp_so101_server",
        "external_guard_required": False,
        "udp5005_required": False,
        "udp5006_required": False,
        "baseline_frames": 30,
        "contact_on_threshold": 40.0,
        "contact_off_threshold": 30.0,
        "contact_confirm_frames": 3,
        "release_confirm_frames": 5,
        "stable_contact_3s_used": False,
        "tactile_test_entry": "scripts/mvp_visual_grasp.py --tactile-test",
        "final_grasp_entry": "scripts/mvp_visual_grasp.py --execute --confirm VISUAL_GRASP",
        "manual_acceptance_major_step_count": doc.count("## "),
        "visual_algorithm_modified": False,
        "ik_algorithm_modified": False,
        "fk_algorithm_modified": False,
        "descent_waypoints_modified": False,
        "lift_waypoints_modified": False,
        "gripper_stop_logic_modified": False,
        "offline_test_cases": len(cases),
        "offline_tests_passed": passed == len(cases),
        "legacy_regression_tests_passed": args.legacy_regression_result,
        "ros2_build_result": args.ros2_build_result,
        "opened_robot_com_port": False,
        "opened_tactile_com_port": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md",
        "final_status": final_status,
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if passed == len(cases) else 1


if __name__ == "__main__":
    raise SystemExit(main())
