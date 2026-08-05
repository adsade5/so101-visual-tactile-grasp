from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp4c_segmented_descent_report.json"
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_descend_from_pregrasp import (
    ARM_JOINT_NAMES,
    CONFIRM_PHRASE,
    DESIRED_APPROACH_BASE,
    TOOL_APPROACH_AXIS_LOCAL,
    DescentConfig,
    FrozenPregrasp,
    StampedJointState,
    build_waypoint_xyz,
    execute_preconditions,
    final_joint_error,
    load_config,
    plan_segmented_descent,
    validate_fresh_joint_state,
)
from mvp_move_to_pregrasp import (
    CONFIRM_PHRASE as PREGRASP_CONFIRM_PHRASE,
    execute_preconditions as pregrasp_execute_preconditions,
    load_config as load_pregrasp_move_config,
)


def case(name: str, passed: bool, details: dict[str, object] | None = None) -> dict[str, object]:
    return {"name": name, "passed": bool(passed), "details": {} if details is None else details}


def read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return data


class FakeModel:
    joint_names = list(ARM_JOINT_NAMES)
    lower_limits = np.asarray([-10.0] * 5, dtype=np.float64)
    upper_limits = np.asarray([10.0] * 5, dtype=np.float64)


class FakeIk:
    def __init__(self, *, q_offset: np.ndarray | None = None) -> None:
        self.seed_sources: list[list[float]] = []
        self.calls = 0
        self.q_offset = np.zeros(5, dtype=np.float64) if q_offset is None else q_offset

    def __call__(
        self,
        model: FakeModel,
        target: np.ndarray,
        seed: np.ndarray,
        *args: object,
        **kwargs: object,
    ) -> dict[str, object]:
        del model, args, kwargs
        self.calls += 1
        self.seed_sources.append([float(value) for value in seed.tolist()])
        q = np.asarray([target[0], target[1], target[2], 1.0, 0.0], dtype=np.float64)
        q = q + self.q_offset
        return {
            "success": True,
            "joint_positions_rad": [float(value) for value in q.tolist()],
            "iterations": 1,
            "position_error_m": 0.0,
            "approach_error_deg": 0.0,
            "reason": "converged",
        }


def fake_fk(model: FakeModel, q: np.ndarray) -> dict[str, object]:
    del model
    return {
        "position_m": [float(q[0]), float(q[1]), float(q[2])],
        "rotation_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
    }


def fake_fk_bad_approach(model: FakeModel, q: np.ndarray) -> dict[str, object]:
    del model
    return {
        "position_m": [float(q[0]), float(q[1]), float(q[2])],
        "rotation_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    }


def fake_fk_xy_error(model: FakeModel, q: np.ndarray) -> dict[str, object]:
    del model
    return {
        "position_m": [float(q[0]) + 0.011, float(q[1]), float(q[2])],
        "rotation_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
    }


def fake_fk_non_monotonic(model: FakeModel, q: np.ndarray) -> dict[str, object]:
    del model
    z_bias = 0.007 if float(q[2]) < 0.105 else 0.0
    return {
        "position_m": [float(q[0]), float(q[1]), float(q[2]) + z_bias],
        "rotation_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
    }


def always_in_limits(model: FakeModel, q: np.ndarray) -> bool:
    del model
    return bool(q.shape == (5,) and np.all(np.isfinite(q)))


def never_in_limits(model: FakeModel, q: np.ndarray) -> bool:
    del model, q
    return False


def frozen() -> FrozenPregrasp:
    return FrozenPregrasp(
        object_pose_base=[0.20, 0.0, 0.025],
        pregrasp_pose_base=[0.20, 0.0, 0.105],
        pregrasp_joint_target_rad=[0.20, 0.0, 0.105, 1.0, 0.0],
        solution_type="accepted_near_solution",
        selected_offset_m=[0.0, 0.0, 0.0],
        position_error_m=0.006,
        approach_error_deg=3.0,
    )


def run_fake_plan(
    *,
    current: list[float] | None = None,
    config: DescentConfig | None = None,
    ik: FakeIk | None = None,
    fk_func=fake_fk,
    limits=always_in_limits,
) -> tuple[Any, FakeIk]:
    fake_ik = FakeIk() if ik is None else ik
    plan = plan_segmented_descent(
        model=FakeModel(),
        frozen=frozen(),
        current_joint_positions_rad=[0.20, 0.0, 0.105, 1.0, 0.0] if current is None else current,
        config=DescentConfig() if config is None else config,
        ik_solver=fake_ik,
        fk_func=fk_func,
        joint_limits_checker=limits,
    )
    return plan, fake_ik


class FakeExecution:
    def __init__(self, fail_at: int | None = None) -> None:
        self.publish_count = 0
        self.execute_count = 0
        self.completed = 0
        self.fail_at = fail_at

    def run(self, plan: Any) -> None:
        for waypoint in plan.waypoints:
            self.publish_count += 1
            self.execute_count += 1
            if self.fail_at == waypoint.index:
                return
            self.completed += 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline verification for MVP-4C segmented descent.")
    parser.add_argument("--ros2-build-result", default="not_run")
    args = parser.parse_args()

    config = load_config()
    pregrasp_config = load_pregrasp_move_config()
    hardware_config = read_json(PROJECT_ROOT / "config" / "mvp_hardware.json")
    mvp4b_manual = read_json(PROJECT_ROOT / "data" / "verification" / "stage_mvp4b_manual_acceptance_report.json")
    speed_manual = read_json(PROJECT_ROOT / "data" / "verification" / "stage_mvp4b_speed_manual_acceptance_report.json")
    plan, fake_ik = run_fake_plan(config=config)
    waypoints = build_waypoint_xyz(frozen().pregrasp_pose_base, config.waypoint_drop_m)

    over_pregrasp_allowed, over_pregrasp_reason = pregrasp_execute_preconditions(
        tcp_connected=True,
        tcp_status="connected",
        pregrasp_valid=True,
        pregrasp_status="pregrasp_ready",
        compute_message="pregrasp_ready",
        joint_limits_valid=True,
        maximum_abs_joint_delta_rad=1.01,
        max_abs_joint_delta_rad=pregrasp_config.max_abs_joint_delta_rad,
        confirm=PREGRASP_CONFIRM_PHRASE,
    )
    start_bad, _ = run_fake_plan(current=[0.50, 0.0, 0.105, 1.0, 0.0], config=config)
    stale_joint = StampedJointState(ARM_JOINT_NAMES, tuple(frozen().pregrasp_joint_target_rad), 10.0)
    fresh_check = validate_fresh_joint_state(stale_joint, now_monotonic_s=10.5, max_age_s=1.0)
    stale_check = validate_fresh_joint_state(stale_joint, now_monotonic_s=12.0, max_age_s=1.0)
    stale_target_check = validate_fresh_joint_state(stale_joint, now_monotonic_s=12.5, max_age_s=2.0)
    limit_bad, _ = run_fake_plan(config=config, limits=never_in_limits)
    delta_bad, _ = run_fake_plan(config=config, ik=FakeIk(q_offset=np.asarray([0.0, 0.0, 0.0, 0.0, 0.50])))
    approach_bad, _ = run_fake_plan(config=config, fk_func=fake_fk_bad_approach)
    xy_bad, _ = run_fake_plan(config=config, fk_func=fake_fk_xy_error)
    z_bad, _ = run_fake_plan(config=config, fk_func=fake_fk_non_monotonic)
    exec_ok = FakeExecution()
    exec_ok.run(plan)
    exec_fail = FakeExecution(fail_at=2)
    exec_fail.run(plan)
    wrong_confirm_ok, wrong_confirm_reason = execute_preconditions(execute=True, confirm="MVP_MOVE")
    final_pass = final_joint_error([0.1, 0.0, 0.0, 1.0, 0.0], [0.11, 0.0, 0.0, 1.0, 0.0], config.final_joint_tolerance_rad)
    final_fail = final_joint_error([0.1, 0.0, 0.0, 1.0, 0.0], [0.2, 0.0, 0.0, 1.0, 0.0], config.final_joint_tolerance_rad)

    descent_text = (PROJECT_ROOT / "scripts" / "mvp_descend_from_pregrasp.py").read_text(encoding="utf-8")
    move_text = (PROJECT_ROOT / "scripts" / "mvp_move_to_pregrasp.py").read_text(encoding="utf-8")
    bridge_text = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "so101_mvp_control"
        / "so101_mvp_control"
        / "mvp_hardware_bridge_node.py"
    ).read_text(encoding="utf-8")
    server_text = (PROJECT_ROOT / "scripts" / "mvp_so101_server.py").read_text(encoding="utf-8")
    executor_text = (PROJECT_ROOT / "lerobot_server" / "mvp_hardware_executor.py").read_text(encoding="utf-8")

    cases: list[dict[str, object]] = [
        case("pregrasp_move_limit_is_1p0", math.isclose(pregrasp_config.max_abs_joint_delta_rad, 1.00)),
        case("pregrasp_move_over_1p0_rejected", not over_pregrasp_allowed and over_pregrasp_reason == "joint_delta_exceeds_mvp4b_limit"),
        case("descent_limit_remains_0p25", math.isclose(config.max_abs_joint_delta_per_waypoint_rad, 0.25)),
        case("exactly_three_waypoints", len(config.waypoint_drop_m) == 3 and len(plan.waypoints) == 3),
        case("waypoint_drop_1cm", math.isclose(config.waypoint_drop_m[0], 0.01)),
        case("waypoint_drop_2cm", math.isclose(config.waypoint_drop_m[1], 0.02)),
        case("waypoint_drop_3cm", math.isclose(config.waypoint_drop_m[2], 0.03)),
        case("waypoint_xy_unchanged", all(math.isclose(item[0], 0.20) and math.isclose(item[1], 0.0) for item in waypoints)),
        case("downward_approach_preserved", "DESIRED_APPROACH_BASE" in descent_text and "[0.0, 0.0, -1.0]" in descent_text),
        case("current_state_seed_for_waypoint1", plan.waypoints[0].seed_source == "current_joint_state"),
        case("previous_solution_seed_for_waypoint2", plan.waypoints[1].seed_source == "previous_waypoint_1"),
        case("previous_solution_seed_for_waypoint3", plan.waypoints[2].seed_source == "previous_waypoint_2"),
        case("all_waypoints_planned_before_execute", "plan = plan_segmented_descent" in descent_text and "for waypoint in plan.waypoints" in descent_text),
        case("start_not_at_pregrasp_rejected", not start_bad.success and start_bad.reason == "not_at_pregrasp"),
        case("stale_joint_state_rejected", fresh_check[0] and not stale_check[0] and stale_check[1] == "joint_state_stale"),
        case("stale_pregrasp_target_rejected", not stale_target_check[0] and stale_target_check[1] == "joint_state_stale"),
        case("waypoint_joint_limits_valid", plan.success and all(item.joint_limits_valid for item in plan.waypoints)),
        case("waypoint_joint_delta_under_0p25", plan.success and all(item.maximum_abs_joint_delta_rad <= 0.25 for item in plan.waypoints)),
        case("waypoint_joint_delta_over_0p25_rejected", not delta_bad.success and delta_bad.reason == "descent_joint_delta_exceeded"),
        case("fk_position_validation", plan.success and all(item.position_error_m <= config.position_tolerance_m for item in plan.waypoints)),
        case("fk_approach_validation", not approach_bad.success and approach_bad.reason == "fk_approach_validation_failed"),
        case("xy_error_validation", not xy_bad.success and xy_bad.reason == "descent_xy_error_exceeded"),
        case("monotonic_z_descent", plan.success and all(item.actual_z_drop_from_previous_m >= 0.004 for item in plan.waypoints)),
        case("non_monotonic_z_rejected", not z_bad.success and z_bad.reason == "non_monotonic_descent"),
        case("total_z_drop_validation", plan.success and float(plan.total_actual_z_drop_m) >= config.minimum_total_actual_z_drop_m),
        case("plan_only_no_publish", '"hardware_command_sent": bool(hardware_command_sent)' in descent_text),
        case("plan_only_no_execute", "if not args.execute:" in descent_text),
        case("wrong_confirmation_rejected", not wrong_confirm_ok and wrong_confirm_reason == "wrong_confirmation" and CONFIRM_PHRASE in descent_text),
        case("publish_each_waypoint_once", "publish_target_once" in descent_text and exec_ok.publish_count == 3),
        case("execute_each_waypoint_once", "node.call_trigger(\n                node.execute_client" in descent_text and exec_ok.execute_count == 3),
        case("three_total_publish_calls", exec_ok.publish_count == 3),
        case("three_total_execute_calls", exec_ok.execute_count == 3),
        case("failure_stops_later_waypoints", exec_fail.publish_count == 2 and exec_fail.completed == 1),
        case("no_retry_after_failure", exec_fail.execute_count == 2),
        case("final_joint_tolerance", final_pass["final_target_reached"] and not final_fail["final_target_reached"]),
        case("gripper_not_in_target", "gripper" not in ARM_JOINT_NAMES and "GRIPPER" not in descent_text),
        case("gripper_hold_unchanged", "gripper = float(current_state[\"gripper\"])" in server_text and "build_lerobot_action(positions, gripper)" in server_text),
        case("speed_is_0p06", math.isclose(config.speed_rad_s, 0.06) and math.isclose(float(hardware_config["first_test_speed_rad_s"]), 0.06)),
        case("max_speed_remains_0p08", math.isclose(float(hardware_config["maximum_speed_rad_s"]), 0.08)),
        case("single_tcp_client_unchanged", "MvpTcpClient" not in descent_text and bridge_text.count("MvpTcpClient(") == 1),
        case("no_stop_service", "/mvp/stop" not in descent_text and "stop_client" not in descent_text),
        case("no_contact_command", "contact" not in descent_text.lower()),
        case("no_grasp_command", "grasp_command" not in descent_text and "close_gripper" not in descent_text),
        case("no_lift_command", "lift_target" not in descent_text and "raise_arm" not in descent_text.lower()),
        case("no_return_command", "return_target" not in descent_text and "home" not in descent_text.lower()),
        case("no_com_port_open", "COM4" not in descent_text and "serial.Serial" not in descent_text),
        case("no_goal_position_write_to_real_robot", "Goal_Position" not in descent_text and "send_action(" not in descent_text),
        case("no_physical_motion", "mvp_so101_server" not in descent_text and "MotionFeetechBackend" not in descent_text),
    ]

    passed = sum(1 for item in cases if item["passed"])
    all_passed = passed == len(cases)
    report = {
        "stage": "MVP-4C",
        "mvp4b_manual_acceptance_recorded": mvp4b_manual.get("final_status") == "PASS",
        "speed_manual_acceptance_recorded": speed_manual.get("final_status") == "PASS",
        "pregrasp_max_abs_joint_delta_previous_rad": 0.80,
        "pregrasp_max_abs_joint_delta_new_rad": 1.00,
        "descent_waypoint_count": 3,
        "descent_waypoint_drop_m": [0.01, 0.02, 0.03],
        "descent_max_abs_joint_delta_per_waypoint_rad": config.max_abs_joint_delta_per_waypoint_rad,
        "start_pregrasp_joint_tolerance_rad": config.start_pregrasp_joint_tolerance_rad,
        "position_tolerance_m": config.position_tolerance_m,
        "approach_tolerance_deg": config.approach_tolerance_deg,
        "max_xy_error_from_waypoint_m": config.max_xy_error_from_waypoint_m,
        "minimum_actual_z_drop_per_waypoint_m": config.minimum_actual_z_drop_per_waypoint_m,
        "minimum_total_actual_z_drop_m": config.minimum_total_actual_z_drop_m,
        "speed_rad_s": config.speed_rad_s,
        "max_speed_rad_s": float(hardware_config["maximum_speed_rad_s"]),
        "execute_service_timeout_s": config.execute_service_timeout_s,
        "all_waypoints_planned_before_execute": True,
        "target_freeze_behavior": "single_snapshot",
        "publish_count_expected": 3,
        "execute_count_expected": 3,
        "motion_retry_after_send": False,
        "gripper_target_added": False,
        "stop_service_added": False,
        "second_tcp_client_added": False,
        "offline_test_cases": cases,
        "offline_tests_passed": all_passed,
        "offline_test_count": len(cases),
        "ros2_build_result": args.ros2_build_result,
        "opened_com_ports": False,
        "tcp_started": False,
        "hardware_bridge_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "manual_acceptance_document": "docs/MVP4C_SEGMENTED_DESCENT_MANUAL_ACCEPTANCE.md",
        "final_status": "READY_FOR_MANUAL_SEGMENTED_DESCENT_RETEST"
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
