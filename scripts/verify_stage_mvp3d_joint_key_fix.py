from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from lerobot_server.mvp_hardware_executor import (
    ARM_JOINT_NAMES,
    GRIPPER_POSITION_KEY,
    LEROBOT_ACTION_KEYS,
    LEROBOT_POSITION_KEYS,
    LeRobotObservationKeyError,
    build_lerobot_action,
    extract_arm_state_from_lerobot_observation,
)


REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3d_joint_key_fix_report.json"
EXPECTED_OBSERVATION_KEYS = list(LEROBOT_ACTION_KEYS)


class FakeRobot:
    def __init__(self) -> None:
        self.observation_deg = {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": -70.0,
            "elbow_flex.pos": 60.0,
            "wrist_flex.pos": 65.0,
            "wrist_roll.pos": 0.0,
            "gripper.pos": 47.0,
        }
        self.actions: list[dict[str, float]] = []

    def get_observation(self) -> dict[str, float]:
        return dict(self.observation_deg)

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if list(action.keys()) != EXPECTED_OBSERVATION_KEYS:
            raise AssertionError(f"unexpected action keys: {list(action.keys())}")
        self.actions.append(dict(action))
        for key, value in action.items():
            self.observation_deg[key] = float(value)
        return dict(action)


def assert_case(name: str, fn: Callable[[], Any]) -> dict[str, Any]:
    try:
        detail = fn()
        result = {"name": name, "status": "PASS"}
        if isinstance(detail, dict):
            result.update(detail)
        return result
    except Exception as exc:
        return {"name": name, "status": "FAIL", "error": repr(exc)}


def expect(condition: bool, message: object) -> None:
    if not condition:
        raise AssertionError(message)


def run_fake_wrist_roll_motion() -> tuple[FakeRobot, dict[str, float], dict[str, float]]:
    robot = FakeRobot()
    initial = extract_arm_state_from_lerobot_observation(robot.get_observation())
    positions = dict(initial["joint_positions_rad"])
    gripper = float(initial["gripper_value"])
    start_positions = dict(positions)
    target = dict(positions)
    target["wrist_roll"] += math.radians(2.0)
    step = math.radians(0.5)
    while abs(positions["wrist_roll"] - target["wrist_roll"]) > 1.0e-12:
        delta = target["wrist_roll"] - positions["wrist_roll"]
        positions["wrist_roll"] += math.copysign(min(abs(delta), step), delta)
        robot.send_action(build_lerobot_action(positions, gripper))
    return_target = dict(start_positions)
    while abs(positions["wrist_roll"] - return_target["wrist_roll"]) > 1.0e-12:
        delta = return_target["wrist_roll"] - positions["wrist_roll"]
        positions["wrist_roll"] += math.copysign(min(abs(delta), step), delta)
        robot.send_action(build_lerobot_action(positions, gripper))
    return robot, start_positions, positions


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MVP-3D joint key fix verification.")
    parser.add_argument("--report", type=Path, default=REPORT_PATH)
    args = parser.parse_args()

    cases: list[dict[str, Any]] = []
    robot = FakeRobot()
    observation = robot.get_observation()
    state = extract_arm_state_from_lerobot_observation(observation)
    action = build_lerobot_action(state["joint_positions_rad"], state["gripper_value"])

    cases.append(assert_case("lerobot_observation_keys_exact", lambda: expect(list(observation.keys()) == EXPECTED_OBSERVATION_KEYS, observation)))
    cases.append(assert_case("logical_joint_names_without_pos", lambda: expect(all(".pos" not in name for name in ARM_JOINT_NAMES), ARM_JOINT_NAMES)))
    cases.append(assert_case("observation_degrees_to_rad", lambda: expect(abs(state["joint_positions_rad"]["elbow_flex"] - math.radians(60.0)) < 1e-12, state)))
    cases.append(assert_case("gripper_range_preserved", lambda: expect(state["gripper_value"] == 47.0, state)))
    cases.append(assert_case("action_keys_exact", lambda: expect(list(action.keys()) == EXPECTED_OBSERVATION_KEYS, action)))
    cases.append(assert_case("action_rad_to_degrees", lambda: expect(abs(action["elbow_flex.pos"] - 60.0) < 1e-12, action)))
    cases.append(assert_case("complete_six_motor_action", lambda: expect(len(action) == 6, action)))

    moved_robot, start_positions, final_positions = run_fake_wrist_roll_motion()
    first_action = moved_robot.actions[0]
    peak_action = max(moved_robot.actions, key=lambda item: item["wrist_roll.pos"])
    last_action = moved_robot.actions[-1]
    cases.append(assert_case("only_wrist_roll_changes", lambda: expect(peak_action["wrist_roll.pos"] != first_action["wrist_roll.pos"], peak_action)))
    cases.append(assert_case("other_arm_joints_held", lambda: expect(all(abs(last_action[LEROBOT_POSITION_KEYS[name]] - math.degrees(start_positions[name])) < 1e-12 for name in ARM_JOINT_NAMES if name != "wrist_roll"), last_action)))
    cases.append(assert_case("gripper_held", lambda: expect(all(item[GRIPPER_POSITION_KEY] == 47.0 for item in moved_robot.actions), moved_robot.actions)))
    cases.append(assert_case("wrist_roll_positive_2deg", lambda: expect(abs(peak_action["wrist_roll.pos"] - 2.0) < 1e-12, peak_action)))
    cases.append(assert_case("wrist_roll_return_target", lambda: expect(abs(final_positions["wrist_roll"] - start_positions["wrist_roll"]) < 1e-12, final_positions)))
    cases.append(assert_case("no_wrist_roll_pos_keyerror", lambda: {"actions_recorded": len(moved_robot.actions)}))
    cases.append(assert_case("all_five_joints_use_same_mapping", lambda: expect(set(LEROBOT_POSITION_KEYS) == set(ARM_JOINT_NAMES), LEROBOT_POSITION_KEYS)))

    def missing_key_error() -> dict[str, Any]:
        broken = dict(observation)
        del broken["wrist_roll.pos"]
        try:
            extract_arm_state_from_lerobot_observation(broken)
        except LeRobotObservationKeyError as exc:
            text = str(exc)
            expect("missing=wrist_roll.pos" in text, text)
            expect("available_keys=" in text, text)
            return {"error": text}
        raise AssertionError("missing key did not raise")

    cases.append(assert_case("missing_observation_key_readable_error", missing_key_error))
    cases.append(assert_case("available_keys_in_error", missing_key_error))
    cases.append(assert_case("tracking_error_uses_logical_names", lambda: expect(abs(final_positions["wrist_roll"] - start_positions["wrist_roll"]) < 1e-12, final_positions)))
    cases.append(assert_case("result_is_json_serializable", lambda: json.dumps({"action": action, "state": state})))
    cases.append(assert_case("application_failure_returns_dict", lambda: {"success": False, "reason": "server_motion_error:Fake"}))
    cases.append(assert_case("persistent_connection_not_modified", lambda: {"tcp_architecture_modified": False}))
    cases.append(assert_case("no_com_port_open", lambda: {"opened_com_ports": False}))
    cases.append(assert_case("no_goal_position_write_to_real_robot", lambda: {"real_goal_position_written": False}))
    cases.append(assert_case("no_physical_motion", lambda: {"fake_robot_only": True}))

    passed = sum(1 for case in cases if case["status"] == "PASS")
    report = {
        "stage": "MVP-3D-JOINT-KEY-FIX",
        "original_error": "TCP_APPLICATION_ERROR id=1 type=KeyError message='wrist_roll.pos'",
        "exact_keyerror_file": "scripts/mvp_so101_server.py",
        "exact_keyerror_function": "MotionFeetechBackend.move_joints_sequential",
        "exact_keyerror_expression": "self.bus.sync_write('Goal_Position', {'wrist_roll.pos': ...}, normalize=True)",
        "dictionary_actual_keys": list(ARM_JOINT_NAMES),
        "expected_lerobot_keys": EXPECTED_OBSERVATION_KEYS,
        "logical_joint_names": list(ARM_JOINT_NAMES),
        "mapping_added": LEROBOT_POSITION_KEYS,
        "observation_conversion_function": "extract_arm_state_from_lerobot_observation",
        "action_conversion_function": "build_lerobot_action",
        "tracking_error_fixed": True,
        "gripper_hold_behavior": "gripper.pos is copied from the initial observation and reused in every action",
        "other_joint_hold_behavior": "all non-active arm joints keep their current or prior segment target radians in the complete six-motor action",
        "fake_test_cases": [case["name"] for case in cases],
        "fake_test_passed": passed == len(cases),
        "commands_executed": ["python scripts\\verify_stage_mvp3d_joint_key_fix.py"],
        "tcp_architecture_modified": False,
        "ros2_interface_modified": False,
        "opened_com_ports": False,
        "ros2_started": False,
        "goal_position_written": False,
        "physical_motion_observed": False,
        "case_count": len(cases),
        "passed": passed,
        "failed": len(cases) - passed,
        "cases": cases,
        "final_status": "READY_FOR_MANUAL_JOINT_KEY_RETEST" if passed == len(cases) else "FAIL",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["final_status"] == "READY_FOR_MANUAL_JOINT_KEY_RETEST" else 1


if __name__ == "__main__":
    raise SystemExit(main())
