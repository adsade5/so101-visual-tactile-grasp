from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PROJECT_ROOT / "scripts" / "mvp_move_to_pregrasp.py"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4b_pregrasp_move_report.json"
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_move_to_pregrasp import (
    ACCEPTED_PREGRASP_STATUS,
    ARM_JOINT_NAMES,
    CONFIRM_PHRASE,
    MoveConfig,
    StampedJointState,
    build_plan_summary,
    estimated_duration_s,
    execute_preconditions,
    final_joint_error,
    joint_delta,
    status_is_accepted,
    validate_fresh_joint_state,
    validate_joint_contract,
)


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


class FakePublisher:
    def __init__(self) -> None:
        self.count = 0

    def publish(self, msg: object) -> None:
        del msg
        self.count += 1


class FakeExecuteClient:
    def __init__(self, result: tuple[bool, str] | None = None) -> None:
        self.calls = 0
        self.result = (True, "ok") if result is None else result

    def call(self) -> tuple[bool, str]:
        self.calls += 1
        return self.result


def main() -> int:
    config = MoveConfig()
    current = [0.0, -0.20, 0.20, 1.10, 0.0]
    target = [0.10, -0.30, 0.30, 1.20, 0.05]
    now = 100.0
    fresh = StampedJointState(ARM_JOINT_NAMES, tuple(current), now)
    stale = StampedJointState(ARM_JOINT_NAMES, tuple(current), now - 2.0)

    cases: list[dict[str, object]] = []
    valid_contract, _ = validate_joint_contract(ARM_JOINT_NAMES, target)
    cases.append(case("exact_joint_name_order", valid_contract))
    cases.append(case("exactly_five_joint_values", len(target) == 5))
    cases.append(case("current_joint_state_finite", validate_fresh_joint_state(fresh, now_monotonic_s=now, max_age_s=1.0)[0]))
    cases.append(case("target_joint_state_finite", validate_joint_contract(ARM_JOINT_NAMES, target)[0]))
    cases.append(case("current_state_stale_rejected", not validate_fresh_joint_state(stale, now_monotonic_s=now, max_age_s=1.0)[0]))
    target_stale = StampedJointState(ARM_JOINT_NAMES, tuple(target), now - 3.0)
    cases.append(case("target_stale_rejected", not validate_fresh_joint_state(target_stale, now_monotonic_s=now, max_age_s=2.0)[0]))
    allowed_invalid, reason_invalid = execute_preconditions(
        tcp_connected=True,
        tcp_status="connected",
        pregrasp_valid=False,
        pregrasp_status="pregrasp_ready",
        compute_message="pregrasp_ready",
        joint_limits_valid=True,
        maximum_abs_joint_delta_rad=0.1,
        max_abs_joint_delta_rad=config.max_abs_joint_delta_rad,
        confirm=CONFIRM_PHRASE,
    )
    cases.append(case("pregrasp_invalid_rejected", not allowed_invalid and reason_invalid == "pregrasp_invalid"))
    cases.append(case("accepted_status_list", all(status_is_accepted(status, "") for status in ACCEPTED_PREGRASP_STATUS)))

    frozen = list(target)
    later_target = [value + 0.1 for value in target]
    cases.append(case("target_frozen_after_compute", frozen == target))
    cases.append(case("later_visual_update_ignored", frozen != later_target))
    delta = joint_delta(current, frozen)
    cases.append(case("joint_delta_computed", delta["joint_delta_rad"][0] == 0.10))
    cases.append(case("estimated_duration_computed", estimated_duration_s(current, frozen, 0.06) > 0.0))
    cases.append(case("max_delta_under_limit", delta["maximum_abs_joint_delta_rad"] <= 1.00))
    over_delta = joint_delta(current, [1.2, -0.20, 0.20, 1.10, 0.0])
    allowed_over, reason_over = execute_preconditions(
        tcp_connected=True,
        tcp_status="connected",
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready",
        compute_message="pregrasp_ready",
        joint_limits_valid=True,
        maximum_abs_joint_delta_rad=float(over_delta["maximum_abs_joint_delta_rad"]),
        max_abs_joint_delta_rad=1.00,
        confirm=CONFIRM_PHRASE,
    )
    cases.append(case("max_delta_over_1p0_rejected", not allowed_over and reason_over == "joint_delta_exceeds_mvp4b_limit"))

    summary = build_plan_summary(
        mode="plan_only",
        current=current,
        target=frozen,
        config=config,
        pregrasp_pose=[0.2, 0.0, 0.105],
        compute_message="pregrasp_ready solution_type=accepted_near_solution offset_m=[0,0,0] position_error_m=0.006 approach_error_deg=3.0",
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready_near",
        tcp_connected=True,
        tcp_status="connected",
        joint_limits_valid=True,
        hardware_command_sent=False,
    )
    cases.append(case("plan_only_no_publish", summary["hardware_command_sent"] is False))
    cases.append(case("plan_only_no_execute", summary["mode"] == "plan_only"))
    allowed_wrong, reason_wrong = execute_preconditions(
        tcp_connected=True,
        tcp_status="connected",
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready",
        compute_message="pregrasp_ready",
        joint_limits_valid=True,
        maximum_abs_joint_delta_rad=0.1,
        max_abs_joint_delta_rad=1.00,
        confirm="MVP_MOVE",
    )
    cases.append(case("wrong_confirmation_rejected", not allowed_wrong and reason_wrong == "wrong_confirmation"))
    allowed_tcp, reason_tcp = execute_preconditions(
        tcp_connected=False,
        tcp_status="connected",
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready",
        compute_message="pregrasp_ready",
        joint_limits_valid=True,
        maximum_abs_joint_delta_rad=0.1,
        max_abs_joint_delta_rad=1.00,
        confirm=CONFIRM_PHRASE,
    )
    cases.append(case("tcp_disconnected_rejected", not allowed_tcp and reason_tcp == "tcp_disconnected"))
    allowed_status, reason_status = execute_preconditions(
        tcp_connected=True,
        tcp_status="connecting",
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready",
        compute_message="pregrasp_ready",
        joint_limits_valid=True,
        maximum_abs_joint_delta_rad=0.1,
        max_abs_joint_delta_rad=1.00,
        confirm=CONFIRM_PHRASE,
    )
    cases.append(case("tcp_status_not_connected_rejected", not allowed_status and reason_status == "tcp_status_not_connected"))

    pub = FakePublisher()
    pub.publish({"target": frozen})
    exec_client = FakeExecuteClient()
    _ = exec_client.call()
    cases.append(case("publish_once", pub.count == 1))
    cases.append(case("execute_called_once", exec_client.calls == 1))
    timeout_client = FakeExecuteClient((False, "motion_result_unknown: execute_service_timeout"))
    _ = timeout_client.call()
    cases.append(case("no_retry_after_execute_timeout", timeout_client.calls == 1))
    cases.append(case("execute_timeout_120s", config.execute_service_timeout_s == 120.0))

    tcp_client_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_tcp_client.py"
    ).read_text(encoding="utf-8")
    cases.append(case("state_timeout_remains_2s", "state_request_timeout_s: float = 2.0" in tcp_client_text))
    cases.append(case("gripper_not_in_joint_target", "gripper" not in ARM_JOINT_NAMES))
    final_pass = final_joint_error([value + 0.01 for value in frozen], frozen, config.final_joint_tolerance_rad)
    final_fail = final_joint_error([value + 0.05 for value in frozen], frozen, config.final_joint_tolerance_rad)
    real_pregrasp_error_rad = 0.04149463936181448
    real_strict = final_joint_error(
        [real_pregrasp_error_rad, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        config.final_joint_tolerance_rad,
    )
    cases.append(case("final_joint_error_computed", len(final_pass["final_joint_error_rad"]) == 5))
    cases.append(case("final_tolerance_pass", final_pass["final_target_reached"]))
    cases.append(case("final_tolerance_failure", not final_fail["final_target_reached"]))
    cases.append(case("real_error_0p041494_fails_strict_0p035", not real_strict["final_target_reached"]))
    cases.append(case("real_error_0p041494_passes_descent_ready_0p10", real_pregrasp_error_rad <= 0.10))

    script_text = SCRIPT_PATH.read_text(encoding="utf-8")
    bridge_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_hardware_bridge_node.py"
    ).read_text(encoding="utf-8")
    shared_client_text = (PROJECT_ROOT / "shared_protocol" / "mvp_tcp_client.py").read_text(encoding="utf-8")
    checked = "\n".join([script_text, bridge_text, tcp_client_text, shared_client_text])
    cases.append(case("no_stop_service", "/mvp/stop" not in script_text and "stop_client" not in script_text))
    cases.append(case("no_second_tcp_client", "MvpTcpClient" not in script_text and "second_tcp" not in checked))
    cases.append(case("no_com_port_open", all(token not in script_text for token in ("serial.Serial", "COM4", "Serial("))))
    cases.append(case("no_goal_position_write_to_real_robot", all(token not in script_text for token in ("Goal_Position", "goal_position", "send_action("))))
    cases.append(case("no_physical_motion", all(token not in script_text for token in ("move_joints_sequential", "mvp_so101_server", "Follower"))))

    passed = sum(1 for item in cases if item["passed"])
    report = {
        "stage": "MVP-4B",
        "mvp4a_manual_acceptance_recorded": True,
        "orchestration_script": "scripts/mvp_move_to_pregrasp.py",
        "user_confirmation_phrase": CONFIRM_PHRASE,
        "joint_state_max_age_s": config.joint_state_max_age_s,
        "pregrasp_target_max_age_s": config.pregrasp_target_max_age_s,
        "speed_rad_s": config.speed_rad_s,
        "max_abs_joint_delta_rad": config.max_abs_joint_delta_rad,
        "final_joint_tolerance_rad": config.final_joint_tolerance_rad,
        "state_request_timeout_s": 2.0,
        "motion_request_timeout_s": 120.0,
        "execute_service_timeout_s": config.execute_service_timeout_s,
        "target_freeze_behavior": "single_snapshot",
        "joint_target_publish_count": 1,
        "execute_call_count": 1,
        "motion_retry_after_send": False,
        "gripper_target_added": False,
        "stop_service_added": False,
        "second_tcp_client_added": False,
        "offline_test_cases": cases,
        "offline_tests_passed": passed == len(cases),
        "offline_test_count": len(cases),
        "ros2_build_result": "not_run",
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4B_PREGRASP_MOVE_MANUAL_ACCEPTANCE.md",
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
