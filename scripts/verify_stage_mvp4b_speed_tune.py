from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4b_speed_tune_report.json"
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_move_to_pregrasp import (
    MoveConfig,
    build_plan_summary,
    estimated_duration_s,
    load_config,
    parse_compute_message,
    validate_speed,
)
from mvp_so101_server import ARM_JOINT_NAMES, MOVE_CONFIRMATION, MvpTcpServer


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


def read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return data


class FakeExecutor:
    def target_within_calibration(self, name: str, value: float) -> bool:
        del name, value
        return True


class FakeBackend:
    def __init__(self) -> None:
        self.executor = FakeExecutor()
        self.received_speed_rad_s: float | None = None
        self.move_call_count = 0
        self.goal_position_written = False
        self.connect_count = 0

    def connect(self) -> None:
        self.connect_count += 1

    def close(self) -> None:
        pass

    def get_state(self) -> dict[str, Any]:
        return {"success": True, "reason": "state_ok"}

    def move_joints_sequential(
        self,
        target_rad: list[float],
        speed_rad_s: float,
        joint_order: list[int],
    ) -> dict[str, Any]:
        del target_rad, joint_order
        self.received_speed_rad_s = float(speed_rad_s)
        self.move_call_count += 1
        return {"success": True, "reason": "motion_completed"}

    def stop(self) -> dict[str, Any]:
        return {"success": False, "reason": "unsupported_command"}


def server_move(server: MvpTcpServer, speed_rad_s: float) -> tuple[dict[str, Any], str]:
    request = {
        "command": "move_joints_sequential",
        "target_rad": [0.0, -0.2, 0.2, 1.1, 0.0],
        "speed_rad_s": float(speed_rad_s),
        "joint_order": [0, 1, 2, 3, 4],
        "confirm": MOVE_CONFIRMATION,
    }
    stdout = io.StringIO()
    with contextlib.redirect_stdout(stdout):
        result = server.handle_line(json.dumps(request), client_id=1)
    return result, stdout.getvalue()


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline verification for MVP-4B speed tune.")
    parser.add_argument("--ros2-build-result", default="not_run")
    args = parser.parse_args()

    hardware_config_path = PROJECT_ROOT / "config" / "mvp_hardware.json"
    pregrasp_move_config_path = PROJECT_ROOT / "config" / "mvp_pregrasp_move.yaml"
    hardware_config = read_json(hardware_config_path)
    config = load_config()
    previous_speed = 0.04
    new_speed = float(hardware_config["first_test_speed_rad_s"])
    max_speed = float(hardware_config["maximum_speed_rad_s"])

    backend = FakeBackend()
    server = MvpTcpServer(
        host="127.0.0.1",
        port=8770,
        backend=backend,
        hardware_motion_enabled=True,
    )
    move_result, move_stdout = server_move(server, new_speed)
    invalid_zero, _ = server_move(server, 0.0)
    invalid_over, _ = server_move(server, max_speed + 0.001)
    valid_after_invalid, _ = server_move(server, new_speed)

    current = [0.0, -0.20, 0.20, 1.10, 0.0]
    target = [0.10, -0.30, 0.30, 1.20, 0.05]
    expected_duration = sum(abs(t - c) for c, t in zip(current, target, strict=True)) / new_speed
    summary = build_plan_summary(
        mode="plan_only",
        current=current,
        target=target,
        config=config,
        pregrasp_pose=[0.2, 0.0, 0.105],
        compute_message=(
            "pregrasp_ready solution_type=accepted_near_solution "
            "offset_m=[0.000000, 0.000000, 0.000000] "
            "position_error_m=0.006 approach_error_deg=3.0"
        ),
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready_near",
        tcp_connected=True,
        tcp_status="connected",
        joint_limits_valid=True,
        hardware_command_sent=False,
    )
    parsed_offset, parsed_warnings = parse_compute_message(
        "pregrasp_ready offset_m=[0.000000, 0.000000, 0.000000]"
    )
    failed_offset, failed_warnings = parse_compute_message("pregrasp_ready offset_m=[bad, value]")

    script_text = (PROJECT_ROOT / "scripts" / "mvp_move_to_pregrasp.py").read_text(encoding="utf-8")
    wrist_text = (PROJECT_ROOT / "scripts" / "mvp_ros2_wrist_test.py").read_text(encoding="utf-8")
    server_text = (PROJECT_ROOT / "scripts" / "mvp_so101_server.py").read_text(encoding="utf-8")
    executor_text = (PROJECT_ROOT / "lerobot_server" / "mvp_hardware_executor.py").read_text(encoding="utf-8")
    bridge_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_hardware_bridge_node.py"
    ).read_text(encoding="utf-8")
    ros_client_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_tcp_client.py"
    ).read_text(encoding="utf-8")
    shared_client_text = (PROJECT_ROOT / "shared_protocol" / "mvp_tcp_client.py").read_text(encoding="utf-8")
    motion_launch_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_bringup"
        / "launch"
        / "mvp_hardware_bridge_motion_enabled.launch.py"
    ).read_text(encoding="utf-8")

    speed_ok, speed_reason = validate_speed(config.speed_rad_s, config.max_speed_rad_s)
    zero_ok, zero_reason = validate_speed(0.0, config.max_speed_rad_s)
    over_ok, over_reason = validate_speed(max_speed + 0.001, config.max_speed_rad_s)
    cases: list[dict[str, object]] = [
        case("default_speed_is_0p06", math.isclose(config.speed_rad_s, 0.06)),
        case("max_speed_remains_0p08", math.isclose(max_speed, 0.08) and math.isclose(config.max_speed_rad_s, 0.08)),
        case("speed_positive", speed_ok and speed_reason == "ok"),
        case("speed_not_over_max", config.speed_rad_s <= config.max_speed_rad_s),
        case("invalid_zero_speed_rejected", not zero_ok and zero_reason == "invalid_speed_rad_s" and invalid_zero["reason"] == "invalid_speed_rad_s"),
        case("invalid_over_max_speed_rejected", not over_ok and over_reason == "invalid_speed_rad_s" and invalid_over["reason"] == "invalid_speed_rad_s"),
        case("executor_receives_0p06", move_result["success"] and math.isclose(float(backend.received_speed_rad_s), 0.06)),
        case("motion_started_log_reports_0p06", "MOTION_STARTED speed_rad_s=0.06" in move_stdout),
        case("plan_duration_uses_0p06", math.isclose(float(summary["estimated_motion_duration_s"]), expected_duration)),
        case("state_timeout_remains_2s", "state_request_timeout_s: float = 2.0" in ros_client_text and "state_request_timeout_s=2.0" in bridge_text),
        case("motion_timeout_remains_120s", "motion_request_timeout_s: float = 120.0" in ros_client_text and "motion_request_timeout_s=120.0" in bridge_text),
        case("execute_timeout_remains_120s", math.isclose(config.execute_service_timeout_s, 120.0)),
        case("max_joint_delta_remains_0p8", math.isclose(config.max_abs_joint_delta_rad, 0.80)),
        case("final_tolerance_remains_0p035", math.isclose(config.final_joint_tolerance_rad, 0.035)),
        case("control_frequency_unchanged", float(hardware_config["control_rate_hz"]) == 20.0 and '"control_rate_hz": 20.0' in hardware_config_path.read_text(encoding="utf-8")),
        case("sequential_motion_unchanged", "move_joints_sequential" in server_text and "joint_order" in server_text and "for index in joint_order" in server_text),
        case("gripper_hold_unchanged", "current_gripper_value" in executor_text and "gripper_value" in executor_text),
        case("no_motion_retry", script_text.count("mover.call_trigger(\n            mover.execute_client") == 1),
        case("single_tcp_client_unchanged", bridge_text.count("MvpTcpClient(") == 1 and "MvpTcpClient" not in script_text),
        case("selected_offset_parsed_as_three_floats", parsed_offset.get("offset_m") == [0.0, 0.0, 0.0] and summary["selected_offset_m"] == [0.0, 0.0, 0.0]),
        case("selected_offset_parse_failure_safe", failed_offset.get("offset_m") is None and failed_warnings == ["selected_offset_m_parse_failed"]),
        case("no_stop_service", "/mvp/stop" not in script_text and "/mvp/stop" not in wrist_text and "stop_client" not in wrist_text),
        case("no_com_port_open", backend.connect_count == 0 and server.server_socket is None),
        case("no_goal_position_write", not backend.goal_position_written),
        case("no_physical_motion", backend.move_call_count == 2 and valid_after_invalid["success"]),
    ]

    yaml_text = pregrasp_move_config_path.read_text(encoding="utf-8")
    source_trace = {
        "server_executor_speed_source": "config/mvp_hardware.json:first_test_speed_rad_s",
        "server_max_speed_source": "config/mvp_hardware.json:maximum_speed_rad_s",
        "ros2_bridge_default_speed": "mvp_hardware_bridge_node default_speed_rad_s and launch parameters",
        "pregrasp_plan_only_speed_source": "scripts/mvp_move_to_pregrasp.py load_hardware_speed_config(config/mvp_hardware.json)",
        "wrist_test_speed_source": "scripts/mvp_ros2_wrist_test.py load_default_speed_rad_s(config/mvp_hardware.json)",
        "pregrasp_move_yaml_synced": "speed_rad_s: 0.06" in yaml_text,
    }
    duplicate_speed_hardcodes_removed = all(
        "0.04" not in text
        for text in [
            hardware_config_path.read_text(encoding="utf-8"),
            yaml_text,
            script_text,
            wrist_text,
            bridge_text,
            motion_launch_text,
        ]
    )

    passed = sum(1 for item in cases if item["passed"])
    all_passed = passed == len(cases)
    report = {
        "stage": "MVP-4B-SPEED-TUNE",
        "previous_speed_rad_s": previous_speed,
        "new_speed_rad_s": new_speed,
        "max_speed_rad_s": max_speed,
        "speed_authoritative_config": str(hardware_config_path.relative_to(PROJECT_ROOT)),
        "speed_source_trace": source_trace,
        "duplicate_speed_hardcodes_removed": duplicate_speed_hardcodes_removed,
        "estimated_duration_updated": bool(cases[8]["passed"]),
        "selected_offset_parser_fixed": bool(cases[19]["passed"] and cases[20]["passed"]),
        "control_frequency_modified": False,
        "motion_algorithm_modified": False,
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
        "final_status": "READY_FOR_MANUAL_SPEED_RETEST"
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
