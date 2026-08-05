from __future__ import annotations

import json
import math
import shutil
import sys
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from lerobot_server.mvp_hardware_executor import ARM_JOINT_NAMES, MvpSo101HardwareExecutor


CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_hardware.json"
STATE_PATH = PROJECT_ROOT / "data" / "verification" / "mvp3a_current_state.json"
PLAN_PATH = PROJECT_ROOT / "data" / "verification" / "mvp3a_wrist_roll_2deg_plan.json"
REPORT_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3a_hardware_ready_report.json"
LOG_PATH = PROJECT_ROOT / "data" / "verification" / "stage_mvp3a_hardware_ready.log"


class FakeRobotAdapter:
    def __init__(self, initial_degrees: dict[str, float], *, tracking_offset_deg: float = 0.0, interrupt: bool = False):
        self.positions = dict(initial_degrees)
        self.tracking_offset_deg = tracking_offset_deg
        self.interrupt = interrupt
        self.connected = False
        self.sent_actions: list[dict[str, float]] = []
        self.disconnect_called = False

    def connect(self, *args, **kwargs) -> None:
        self.connected = True

    def read_state(self) -> dict[str, float]:
        obs = {f"{name}.pos": float(value) for name, value in self.positions.items()}
        if self.tracking_offset_deg:
            obs["wrist_roll.pos"] = obs["wrist_roll.pos"] + self.tracking_offset_deg
        return obs

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if self.interrupt:
            raise KeyboardInterrupt()
        self.sent_actions.append(dict(action))
        for key, value in action.items():
            if key.endswith(".pos"):
                self.positions[key.removesuffix(".pos")] = float(value)
        return dict(action)

    def disconnect(self) -> None:
        self.disconnect_called = True
        self.connected = False


def git_branch() -> str:
    import subprocess

    return subprocess.check_output(["git", "branch", "--show-current"], cwd=PROJECT_ROOT, text=True).strip()


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def listf(values: dict[str, float]) -> dict[str, float]:
    return {name: float(value) for name, value in values.items()}


def fake_current_rad() -> dict[str, float]:
    return {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.20,
        "elbow_flex": 0.40,
        "wrist_flex": 0.80,
        "wrist_roll": 0.0,
    }


def fake_degrees_from_rad(joints_rad: dict[str, float], gripper: float = 50.0) -> dict[str, float]:
    values = {name: math.degrees(value) for name, value in joints_rad.items()}
    values["gripper"] = gripper
    return values


def run_offline_tests(executor: MvpSo101HardwareExecutor) -> list[dict]:
    tests: list[dict] = []
    current = fake_current_rad()
    gripper = 50.0
    plan = executor.build_wrist_roll_test_plan(current, gripper)

    def add(name: str, success: bool, **extra) -> None:
        item = {"name": name, "success": bool(success)}
        item.update(extra)
        tests.append(item)

    add("config_load", executor.config["robot_id"] == "my_follower" and executor.config["follower_port"] == "COM4")

    original_list_ports = executor.list_serial_ports
    executor.list_serial_ports = lambda: []
    add("missing_com_port", executor.check_port_and_usb()["reason"] == "missing_com_port")
    executor.list_serial_ports = lambda: [{"device": "COM4", "serial_number": "WRONG"}]
    add("wrong_usb_serial", executor.check_port_and_usb()["reason"] == "wrong_usb_serial")
    executor.list_serial_ports = original_list_ports

    with tempfile.TemporaryDirectory() as tmp:
        cfg = dict(executor.config)
        cfg["calibration_path"] = str(Path(tmp) / "missing.json")
        cfg_path = Path(tmp) / "config.json"
        cfg_path.write_text(json.dumps(cfg), encoding="utf-8")
        try:
            MvpSo101HardwareExecutor(cfg_path)
            missing_calibration = False
        except FileNotFoundError:
            missing_calibration = True
        add("missing_calibration", missing_calibration)

    round_trip_ok = True
    action = executor.internal_rad_to_action(current, gripper)
    observed_rad, observed_gripper = executor.observation_to_internal_rad(action)
    for name in ARM_JOINT_NAMES:
        round_trip_ok = round_trip_ok and abs(observed_rad[name] - current[name]) < 1.0e-12
    add("unit_conversion_round_trip", round_trip_ok and abs(observed_gripper - gripper) < 1.0e-12)

    add("calibration_target_in_range", executor.target_within_calibration("wrist_roll", plan["positive_target_rad"]))
    add("calibration_target_out_of_range", not executor.target_within_calibration("shoulder_pan", math.radians(200.0)))

    first = plan["points"][0]["joint_positions_rad"]
    moving = [p for p in plan["points"] if p["phase"] == "positive"]
    only_wrist = all(
        all(abs(p["joint_positions_rad"][name] - first[name]) < 1.0e-12 for name in ARM_JOINT_NAMES if name != "wrist_roll")
        for p in moving
    )
    add("only_wrist_roll_changes", only_wrist)
    add("exact_return_target", abs(plan["points"][-1]["joint_positions_rad"]["wrist_roll"] - current["wrist_roll"]) < 1.0e-12)
    steps = [
        abs(after["joint_positions_rad"]["wrist_roll"] - before["joint_positions_rad"]["wrist_roll"])
        for before, after in zip(plan["points"], plan["points"][1:])
    ]
    add("fixed_step_limit", max(steps) <= executor.config["first_test_speed_rad_s"] / executor.config["control_rate_hz"] + 1.0e-12, max_step=max(steps))

    missing = executor.execute_plan(plan, enable_hardware_motion=False, confirm="SMALL_WRIST_ROLL_2DEG", adapter=FakeRobotAdapter(fake_degrees_from_rad(current)))
    add("confirmation_missing", missing["reason"] == "confirmation_missing", reason=missing["reason"])
    wrong = executor.execute_plan(plan, enable_hardware_motion=True, confirm="WRONG", adapter=FakeRobotAdapter(fake_degrees_from_rad(current)))
    add("wrong_confirmation_text", wrong["reason"] == "confirmation_missing", reason=wrong["reason"])

    tracking = executor.execute_plan(plan, enable_hardware_motion=True, confirm="SMALL_WRIST_ROLL_2DEG", adapter=FakeRobotAdapter(fake_degrees_from_rad(current), tracking_offset_deg=20.0))
    add("tracking_error_abort", tracking["reason"] == "tracking_error_exceeded", reason=tracking["reason"])

    old_timeout = executor.config["motion_timeout_s"]
    executor.config["motion_timeout_s"] = 0.0
    timeout = executor.execute_plan(plan, enable_hardware_motion=True, confirm="SMALL_WRIST_ROLL_2DEG", adapter=FakeRobotAdapter(fake_degrees_from_rad(current)))
    executor.config["motion_timeout_s"] = old_timeout
    add("timeout_abort", timeout["reason"] == "motion_timeout", reason=timeout["reason"])

    interrupted = executor.execute_plan(plan, enable_hardware_motion=True, confirm="SMALL_WRIST_ROLL_2DEG", adapter=FakeRobotAdapter(fake_degrees_from_rad(current), interrupt=True))
    add("keyboard_interrupt_stop", interrupted["reason"] == "keyboard_interrupt_stop", reason=interrupted["reason"])

    fake = FakeRobotAdapter(fake_degrees_from_rad(current))
    completed = executor.execute_plan(plan, enable_hardware_motion=True, confirm="SMALL_WRIST_ROLL_2DEG", adapter=fake)
    add("no_gripper_motion", completed["success"] and all("gripper.pos" not in action for action in fake.sent_actions))
    other_ok = True
    for action in fake.sent_actions:
        for name in ARM_JOINT_NAMES:
            if name != "wrist_roll" and f"{name}.pos" in action:
                other_ok = False
    add("no_other_joint_motion", completed["success"] and other_ok)

    return tests


def current_state_payload(executor: MvpSo101HardwareExecutor) -> dict:
    if STATE_PATH.is_file():
        return load_json(STATE_PATH)
    current = fake_current_rad()
    plan = executor.build_wrist_roll_test_plan(current, 50.0)
    return {
        "success": True,
        "reason": "offline_fake_state_for_initial_verification",
        "opened_com_ports": False,
        "joint_positions_rad": current,
        "gripper_value": 50.0,
        "positive_target_within_calibration": plan["positive_target_within_calibration"],
        "return_target_within_calibration": plan["return_target_within_calibration"],
        "goal_position_written": False,
        "motion_command_sent": False,
        "torque_enable_written": False,
        "torque_disable_written": False,
    }


def plan_payload(executor: MvpSo101HardwareExecutor, state: dict) -> dict:
    if PLAN_PATH.is_file():
        return load_json(PLAN_PATH)
    return executor.build_wrist_roll_test_plan(
        {name: float(value) for name, value in state["joint_positions_rad"].items()},
        float(state["gripper_value"]),
    )


def main() -> int:
    executor = MvpSo101HardwareExecutor(CONFIG_PATH)
    tests = run_offline_tests(executor)
    state = current_state_payload(executor)
    plan = plan_payload(executor, state)
    all_tests = all(bool(test["success"]) for test in tests)
    state_success = bool(state.get("success"))
    plan_ready = bool(plan.get("positive_target_within_calibration")) and bool(plan.get("return_target_within_calibration"))
    real_read_state = state_success and state.get("opened_com_ports") is True and state.get("reason") == "read_only_state_ok"
    final_status = "READY_FOR_MANUAL_MOTION_TEST" if all_tests and state_success and plan_ready and real_read_state else "OFFLINE_TEST_PASS_AWAITING_READ_STATE"

    report = {
        "stage": "MVP-3A",
        "git_branch": git_branch(),
        **executor.audit_lerobot_api(),
        "robot_id": executor.config["robot_id"],
        "follower_port": executor.config["follower_port"],
        "detected_usb_serial_number": None if not state.get("port_info") else state["port_info"].get("serial_number"),
        "calibration_path": str(executor.calibration_path),
        "calibration_summary": executor.calibration_summary(),
        "unit_conversion": "internal radians <-> LeRobot calibrated degrees for five arm joints; gripper remains range_0_100",
        "current_joint_positions_rad": state.get("joint_positions_rad"),
        "current_gripper_value": state.get("gripper_value"),
        "test_joint": executor.config["first_test_joint"],
        "test_delta_deg": executor.config["first_test_delta_deg"],
        "test_speed_rad_s": executor.config["first_test_speed_rad_s"],
        "plan_point_count": plan.get("point_count"),
        "plan_duration_s": plan.get("duration_s"),
        "per_cycle_max_step_rad": plan.get("per_cycle_max_step_rad"),
        "positive_target_within_calibration": plan.get("positive_target_within_calibration"),
        "return_target_within_calibration": plan.get("return_target_within_calibration"),
        "offline_test_cases": tests,
        "read_only_preflight": state,
        "execute_command_generated": True,
        "execute_command_not_run_by_codex": True,
        "opened_com_ports": bool(state.get("opened_com_ports")),
        "goal_position_written": False,
        "motion_command_sent": False,
        "torque_enable_written": False,
        "torque_disable_written": False,
        "physical_motion_observed": False,
        "final_status": final_status,
    }
    write_json(REPORT_PATH, report)
    lines = [
        "Stage MVP-3A hardware executor readiness",
        f"offline_tests_pass={all_tests}",
        f"read_state_success={state_success}",
        f"real_read_state={real_read_state}",
        f"plan_point_count={plan.get('point_count')}",
        f"plan_duration_s={plan.get('duration_s')}",
        f"final_status={final_status}",
        f"report={REPORT_PATH}",
    ]
    LOG_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0 if all_tests and state_success and plan_ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
