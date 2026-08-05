from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4e_tactile_grasp_lift_report.json"


@dataclass(frozen=True)
class Case:
    name: str
    passed: bool
    details: Any = None


def read(path: str) -> str:
    return (PROJECT_ROOT / path).read_text(encoding="utf-8")


def case(name: str, predicate: bool | Callable[[], Any], details: Any = None) -> Case:
    try:
        if callable(predicate):
            value = predicate()
            return Case(name, bool(value), value)
        return Case(name, bool(predicate), details)
    except Exception as exc:
        return Case(name, False, f"{type(exc).__name__}: {exc}")


def run_compile(paths: list[str]) -> tuple[bool, str]:
    command = [sys.executable, "-m", "compileall", *paths]
    proc = subprocess.run(command, cwd=PROJECT_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return proc.returncode == 0, proc.stdout


def load_grasp_config() -> dict[str, Any]:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
    import mvp_visual_grasp

    cfg = mvp_visual_grasp.load_grasp_config()
    return cfg.__dict__.copy()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros2-build-result", default="NOT_RUN")
    args = parser.parse_args()

    visual = read("scripts/mvp_visual_grasp.py")
    server = read("scripts/mvp_so101_server.py")
    bridge = read("ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py")
    client = read("ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_tcp_client.py")
    shared_client = read("shared_protocol/mvp_tcp_client.py")
    hardware_config = json.loads(read("config/mvp_hardware.json"))
    grasp_config_text = read("config/mvp_grasp.yaml")
    doc = read("docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md")
    cfg = load_grasp_config()
    compile_ok, compile_output = run_compile(
        [
            "scripts/mvp_visual_grasp.py",
            "scripts/mvp_so101_server.py",
            "ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_hardware_bridge_node.py",
            "ros2_ws/src/so101_mvp_control/so101_mvp_control/mvp_tcp_client.py",
            "shared_protocol/mvp_tcp_client.py",
        ]
    )

    cases: list[Case] = []
    cases.append(case("compileall_core_files", compile_ok, compile_output))
    cases.append(case("tactile_config_enabled", hardware_config.get("tactile_enabled") is True))
    cases.append(case("tactile_source_udp_guard_receiver", hardware_config.get("tactile_source") == "udp_guard_receiver"))
    cases.append(case("tactile_guard_host_loopback", hardware_config.get("tactile_guard_host") == "127.0.0.1"))
    cases.append(case("tactile_guard_port_5006", hardware_config.get("tactile_guard_port") == 5006))
    cases.append(case("tactile_timeout_positive", float(hardware_config.get("tactile_guard_timeout_s", 0.0)) > 0.0))
    cases.append(case("old_on_score_recorded", abs(float(hardware_config.get("tactile_contact_on_score")) - 0.80) < 1e-9))
    cases.append(case("old_off_score_recorded", abs(float(hardware_config.get("tactile_contact_off_score")) - 0.20) < 1e-9))
    cases.append(case("old_confirm_frames_recorded", int(hardware_config.get("tactile_confirm_frames")) == 3))
    cases.append(case("old_release_frames_recorded", int(hardware_config.get("tactile_release_frames")) == 3))
    cases.append(case("no_fake_flexitac_com_port", "tactile_port" not in hardware_config and "tactile_baudrate" not in hardware_config))
    cases.append(case("gripper_open_delta_10", abs(float(cfg["gripper_open_delta"]) - 10.0) < 1e-9))
    cases.append(case("tactile_stop_enabled_config", cfg["tactile_stop_enabled"] is True))
    cases.append(case("tactile_require_clear_config", cfg["tactile_require_clear_before_grasp"] is True))
    cases.append(case("tactile_static_test_timeout_30s", abs(float(cfg["tactile_static_test_timeout_s"]) - 30.0) < 1e-9))
    cases.append(case("lift_enabled_config", cfg["lift_enabled"] is True))
    cases.append(case("lift_total_3cm", abs(float(cfg["lift_total_m"]) - 0.03) < 1e-9))
    cases.append(case("lift_waypoints_1_2_3cm", tuple(cfg["lift_waypoint_rise_m"]) == (0.01, 0.02, 0.03)))
    cases.append(case("lift_speed_006", abs(float(cfg["lift_speed_rad_s"]) - 0.06) < 1e-9))
    cases.append(case("lift_xy_tolerance_1cm", abs(float(cfg["lift_max_xy_error_m"]) - 0.010) < 1e-9))
    cases.append(case("server_guard_packet_struct_reused", 'struct.Struct("!4sBBI")' in server and "GUARD_MAGIC = b\"GRIP\"" in server))
    cases.append(case("server_tactile_snapshot_fields", all(token in server for token in ("tactile_ready", "tactile_contact_detected", "tactile_contact_score", "tactile_state_age_s", "tactile_error"))))
    cases.append(case("server_udp_guard_receiver_class", "class TactileUdpGuardReceiver" in server))
    cases.append(case("server_nonblocking_udp_poll", "setblocking(False)" in server and "recvfrom(1024)" in server))
    cases.append(case("server_get_state_extends_tactile", "**self.tactile.snapshot().to_tcp_fields()" in server))
    cases.append(case("server_move_optional_stop_flag", "stop_gripper_on_tactile_contact" in server))
    cases.append(case("server_tactile_unavailable_abort", "tactile_unavailable_during_gripper_close" in server))
    cases.append(case("server_contact_stop_reason", "tactile_contact_stop" in server))
    cases.append(case("server_no_contact_no_lift_reason", "gripper_closed_without_tactile_contact" in server))
    cases.append(case("server_zero_preload", '"gripper_contact_preload_offset=0.0' not in server and "gripper_contact_preload_offset=0.0" in server))
    cases.append(case("client_accepts_stop_flag", "stop_gripper_on_tactile_contact" in client))
    cases.append(case("shared_client_accepts_stop_flag", "stop_gripper_on_tactile_contact" in shared_client))
    cases.append(case("bridge_tactile_ready_pub", '"/mvp/tactile_ready"' in bridge))
    cases.append(case("bridge_tactile_contact_pub", '"/mvp/tactile_contact"' in bridge))
    cases.append(case("bridge_tactile_score_pub", '"/mvp/tactile_score"' in bridge))
    cases.append(case("bridge_tactile_status_pub", '"/mvp/tactile_status"' in bridge))
    cases.append(case("bridge_stop_flag_sub", '"/mvp/stop_gripper_on_tactile_contact"' in bridge))
    cases.append(case("bridge_execute_passes_stop_flag", "fresh_stop_gripper_on_tactile_contact" in bridge))
    cases.append(case("bridge_old_behavior_default_false", "return False" in bridge and "last_stop_gripper_on_tactile_contact = False" in bridge))
    cases.append(case("visual_tactile_test_arg", "--tactile-test" in visual))
    cases.append(case("visual_tactile_test_no_publish_contract", "ros_publish_count" in visual and "pregrasp_compute_called" in visual and "camera_used" in visual))
    cases.append(case("visual_tactile_precheck_ready_clear", "validate_fresh_tactile_state" in visual and "tactile_contact_already_active" in visual))
    cases.append(case("visual_lift_plan_function", "def plan_lift_waypoints" in visual))
    cases.append(case("visual_lift_xy_unchanged", "build_lift_waypoint_xyz" in visual and "[[x, y, z + float(rise)]" in visual))
    cases.append(case("visual_lift_uses_fk_ik", "solve_ik(" in visual and "forward_kinematics(" in visual and "joints_within_limits" in visual))
    cases.append(case("visual_close_sets_stop_true", "publish_stop_gripper_on_tactile_once(True)" in visual))
    cases.append(case("visual_lift_only_after_contact", 'close_message == "tactile_contact_stop"' in visual))
    cases.append(case("visual_no_contact_no_lift", '"gripper_closed_without_tactile_contact"' in visual))
    cases.append(case("visual_contact_lost_abort", "tactile_contact_lost_during_lift" in visual))
    cases.append(case("visual_all_motion_count_12", "all_motion_waypoint_count" in visual))
    cases.append(case("manual_doc_two_major_steps", doc.count("## ") == 2))
    cases.append(case("manual_doc_uses_single_script", doc.count("mvp_visual_grasp.py") >= 3 and "mvp_gripper_open_close_test.py" not in doc))
    cases.append(case("grasp_yaml_contains_lift_fields", "lift_waypoint_rise_m" in grasp_config_text and "tactile_stop_enabled" in grasp_config_text))

    if len(cases) != 53:
        cases.append(Case("verification_case_count_is_53", False, len(cases)))

    passed = sum(1 for item in cases if item.passed)
    report = {
        "stage": "MVP-4E-TACTILE-STOP-GRASP-AND-LIFT",
        "success": passed == len(cases) == 53,
        "passed": passed,
        "total": len(cases),
        "ros2_build_result": args.ros2_build_result,
        "opened_tactile_com_port": False,
        "hardware_command_sent": False,
        "tactile_port_source": hardware_config.get("tactile_port_source"),
        "cases": [item.__dict__ for item in cases],
    }
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
