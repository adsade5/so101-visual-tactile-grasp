from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol


ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
GRIPPER_NAME = "gripper"
ALL_MOTOR_NAMES = ARM_JOINT_NAMES + (GRIPPER_NAME,)
LEROBOT_POSITION_KEYS = {
    "shoulder_pan": "shoulder_pan.pos",
    "shoulder_lift": "shoulder_lift.pos",
    "elbow_flex": "elbow_flex.pos",
    "wrist_flex": "wrist_flex.pos",
    "wrist_roll": "wrist_roll.pos",
}
GRIPPER_POSITION_KEY = "gripper.pos"
LEROBOT_ACTION_KEYS = tuple(LEROBOT_POSITION_KEYS[name] for name in ARM_JOINT_NAMES) + (
    GRIPPER_POSITION_KEY,
)
FEETECH_RESOLUTION = 4096
FEETECH_MAX_STEP = FEETECH_RESOLUTION - 1
LEROBOT_SRC = Path(__file__).resolve().parents[1].parents[0] / "repos" / "lerobot" / "src"


class HardwareAdapter(Protocol):
    def connect(self) -> None: ...

    def read_state(self) -> dict[str, float]: ...

    def send_action(self, action: dict[str, float]) -> dict[str, float]: ...

    def disconnect(self) -> None: ...


class LeRobotObservationKeyError(KeyError):
    pass


@dataclass(frozen=True)
class CalibrationEntry:
    name: str
    id: int
    homing_offset: int
    range_min: int
    range_max: int
    drive_mode: int


def load_json(path: str | Path) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return data


def degrees_to_radians(value_deg: float) -> float:
    return math.radians(float(value_deg))


def radians_to_degrees(value_rad: float) -> float:
    return math.degrees(float(value_rad))


def _available_keys_text(value: dict[str, Any]) -> str:
    return "[" + ",".join(sorted(str(key) for key in value.keys())) + "]"


def extract_arm_state_from_lerobot_observation(observation: dict[str, Any]) -> dict[str, Any]:
    joints_rad: dict[str, float] = {}
    for name in ARM_JOINT_NAMES:
        key = LEROBOT_POSITION_KEYS[name]
        if key not in observation:
            raise LeRobotObservationKeyError(
                f"missing={key}; available_keys={_available_keys_text(observation)}"
            )
        value = float(observation[key])
        if not math.isfinite(value):
            raise ValueError(f"Observation {key} is not finite")
        joints_rad[name] = degrees_to_radians(value)
    if GRIPPER_POSITION_KEY not in observation:
        raise LeRobotObservationKeyError(
            f"missing={GRIPPER_POSITION_KEY}; available_keys={_available_keys_text(observation)}"
        )
    gripper_value = float(observation[GRIPPER_POSITION_KEY])
    if not math.isfinite(gripper_value):
        raise ValueError(f"Observation {GRIPPER_POSITION_KEY} is not finite")
    return {
        "joint_positions_rad": joints_rad,
        "gripper_value": gripper_value,
    }


def build_lerobot_action(
    target_joint_positions_rad: dict[str, float],
    gripper_value: float,
) -> dict[str, float]:
    missing = [name for name in ARM_JOINT_NAMES if name not in target_joint_positions_rad]
    if missing:
        raise ValueError(f"Missing target joints: {missing}")
    action: dict[str, float] = {}
    for name in ARM_JOINT_NAMES:
        value_rad = float(target_joint_positions_rad[name])
        if not math.isfinite(value_rad):
            raise ValueError(f"Target {name} is not finite")
        action[LEROBOT_POSITION_KEYS[name]] = float(radians_to_degrees(value_rad))
    gripper = float(gripper_value)
    if not math.isfinite(gripper):
        raise ValueError("Gripper target is not finite")
    action[GRIPPER_POSITION_KEY] = gripper
    if tuple(action.keys()) != LEROBOT_ACTION_KEYS:
        raise ValueError(f"Unexpected LeRobot action keys: {list(action.keys())}")
    return action


def calibrated_degree_range(entry: CalibrationEntry) -> tuple[float, float]:
    mid = (entry.range_min + entry.range_max) / 2.0
    lower = (entry.range_min - mid) * 360.0 / FEETECH_MAX_STEP
    upper = (entry.range_max - mid) * 360.0 / FEETECH_MAX_STEP
    return lower, upper


def calibrated_degree_to_raw(entry: CalibrationEntry, value_deg: float) -> int:
    mid = (entry.range_min + entry.range_max) / 2.0
    return int((float(value_deg) * FEETECH_MAX_STEP / 360.0) + mid)


def raw_to_calibrated_degree(entry: CalibrationEntry, raw_value: float) -> float:
    mid = (entry.range_min + entry.range_max) / 2.0
    return (float(raw_value) - mid) * 360.0 / FEETECH_MAX_STEP


class MvpSo101HardwareExecutor:
    def __init__(
        self,
        config_path: str | Path,
        adapter: HardwareAdapter | None = None,
    ) -> None:
        self.config_path = Path(config_path)
        self.config = load_json(self.config_path)
        self.adapter = adapter
        self.stop_requested = False
        self.goal_position_written = False
        self.motion_command_sent = False

        self.calibration_path = Path(str(self.config["calibration_path"]))
        self.calibration = self.load_calibration(self.calibration_path)

    @staticmethod
    def load_calibration(path: Path) -> dict[str, CalibrationEntry]:
        raw = load_json(path)
        missing = [name for name in ALL_MOTOR_NAMES if name not in raw]
        if missing:
            raise ValueError(f"Calibration missing motors: {missing}")
        calibration: dict[str, CalibrationEntry] = {}
        for name in ALL_MOTOR_NAMES:
            item = raw[name]
            calibration[name] = CalibrationEntry(
                name=name,
                id=int(item["id"]),
                homing_offset=int(item["homing_offset"]),
                range_min=int(item["range_min"]),
                range_max=int(item["range_max"]),
                drive_mode=int(item["drive_mode"]),
            )
        return calibration

    def calibration_summary(self) -> dict[str, Any]:
        return {
            name: {
                "id": entry.id,
                "homing_offset": entry.homing_offset,
                "range_min": entry.range_min,
                "range_max": entry.range_max,
                "drive_mode": entry.drive_mode,
                "calibrated_degrees_min": calibrated_degree_range(entry)[0]
                if name in ARM_JOINT_NAMES
                else 0.0,
                "calibrated_degrees_max": calibrated_degree_range(entry)[1]
                if name in ARM_JOINT_NAMES
                else 100.0,
            }
            for name, entry in self.calibration.items()
        }

    def audit_lerobot_api(self) -> dict[str, Any]:
        return {
            "lerobot_version": "0.5.2",
            "robot_class": "lerobot.robots.so_follower.so_follower.SOFollower",
            "config_class": "lerobot.robots.so_follower.config_so_follower.SOFollowerRobotConfig (registered SOFollowerConfig with id)",
            "observation_position_unit": "arm joints: calibrated degrees via *.pos; gripper: range_0_100",
            "action_position_unit": "arm joints: calibrated degrees via *.pos; gripper: range_0_100",
            "calibration_applied_by": "Robot loads my_follower.json; FeetechMotorsBus normalizes Present_Position/Goal_Position",
            "connect_torque_behavior": "SOFollower.connect configures motors under torque_disabled; read-only preflight avoids SOFollower.connect",
            "disconnect_torque_behavior": "SOFollower.disconnect calls bus.disconnect(disable_torque_on_disconnect=True by default)",
            "action_key_names": [f"{name}.pos" for name in ALL_MOTOR_NAMES],
        }

    def check_static_config(self) -> dict[str, Any]:
        return {
            "config_path": str(self.config_path),
            "robot_type": self.config["robot_type"],
            "robot_id": self.config["robot_id"],
            "follower_port": self.config["follower_port"],
            "expected_usb_serial_number": self.config["expected_usb_serial_number"],
            "calibration_path": str(self.calibration_path),
            "calibration_exists": self.calibration_path.is_file(),
            "hardware_enabled": bool(self.config["hardware_enabled"]),
            "test_joint": self.config["first_test_joint"],
            "test_delta_deg": float(self.config["first_test_delta_deg"]),
            "test_speed_rad_s": float(self.config["first_test_speed_rad_s"]),
            "opens_serial_port": False,
        }

    def list_serial_ports(self) -> list[dict[str, Any]]:
        import serial.tools.list_ports

        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append(
                {
                    "device": port.device,
                    "serial_number": port.serial_number,
                    "vid": port.vid,
                    "pid": port.pid,
                    "description": port.description,
                    "manufacturer": port.manufacturer,
                }
            )
        return ports

    def configured_port_info(self) -> dict[str, Any] | None:
        target = str(self.config["follower_port"]).upper()
        for item in self.list_serial_ports():
            if str(item["device"]).upper() == target:
                return item
        return None

    def check_port_and_usb(self) -> dict[str, Any]:
        port_info = self.configured_port_info()
        if port_info is None:
            return {"success": False, "reason": "missing_com_port", "port_info": None}
        expected = str(self.config["expected_usb_serial_number"])
        actual = str(port_info.get("serial_number") or "")
        if expected and actual and actual != expected:
            return {"success": False, "reason": "wrong_usb_serial", "port_info": port_info}
        return {"success": True, "reason": "valid", "port_info": port_info}

    def observation_to_internal_rad(self, observation: dict[str, float]) -> tuple[dict[str, float], float]:
        state = extract_arm_state_from_lerobot_observation(observation)
        return state["joint_positions_rad"], state["gripper_value"]

    def internal_rad_to_action(self, joints_rad: dict[str, float], gripper_value: float) -> dict[str, float]:
        return build_lerobot_action(joints_rad, gripper_value)

    def target_within_calibration(self, joint_name: str, target_rad: float) -> bool:
        entry = self.calibration[joint_name]
        target_deg = radians_to_degrees(target_rad)
        raw = calibrated_degree_to_raw(entry, target_deg)
        return entry.range_min <= raw <= entry.range_max

    def all_targets_within_calibration(self, joints_rad: dict[str, float], gripper_value: float) -> bool:
        for name in ARM_JOINT_NAMES:
            if not self.target_within_calibration(name, joints_rad[name]):
                return False
        return 0.0 <= float(gripper_value) <= 100.0

    def build_wrist_roll_test_plan(
        self,
        current_joints_rad: dict[str, float],
        current_gripper_value: float,
    ) -> dict[str, Any]:
        joint_name = str(self.config["first_test_joint"])
        if joint_name != "wrist_roll":
            raise ValueError("MVP-3A fixed test joint must be wrist_roll")
        delta_rad = math.radians(float(self.config["first_test_delta_deg"]))
        speed = float(self.config["first_test_speed_rad_s"])
        rate = float(self.config["control_rate_hz"])
        max_step = speed / rate
        current = float(current_joints_rad[joint_name])
        positive = current + delta_rad

        if not self.target_within_calibration(joint_name, positive):
            raise ValueError("Positive wrist_roll target is outside calibration range")
        if not self.target_within_calibration(joint_name, current):
            raise ValueError("Return wrist_roll target is outside calibration range")

        points = []
        time_s = 0.0
        forward_values = self._linear_joint_values(current, positive, max_step)
        return_values = self._linear_joint_values(positive, current, max_step)
        for phase, values in [("positive", forward_values), ("hold", [positive] * int(round(0.5 * rate))), ("return", return_values)]:
            for value in values:
                joints = dict(current_joints_rad)
                joints[joint_name] = value
                points.append(
                    {
                        "time_s": time_s,
                        "joint_positions_rad": {name: joints[name] for name in ARM_JOINT_NAMES},
                        "gripper_value": float(current_gripper_value),
                        "active_joint_name": joint_name if phase != "hold" else None,
                        "phase": phase,
                    }
                )
                time_s += 1.0 / rate

        if points:
            points[-1]["joint_positions_rad"][joint_name] = current
            points[-1]["active_joint_name"] = None
            points[-1]["phase"] = "end"

        return {
            "test_joint": joint_name,
            "delta_deg": float(self.config["first_test_delta_deg"]),
            "speed_rad_s": speed,
            "control_rate_hz": rate,
            "per_cycle_max_step_rad": max_step,
            "positive_target_rad": positive,
            "return_target_rad": current,
            "positive_target_within_calibration": True,
            "return_target_within_calibration": True,
            "point_count": len(points),
            "duration_s": 0.0 if not points else points[-1]["time_s"],
            "points": points,
        }

    @staticmethod
    def _linear_joint_values(start: float, target: float, max_step: float) -> list[float]:
        if max_step <= 0.0:
            raise ValueError("max_step must be positive")
        delta = target - start
        distance = abs(delta)
        if distance <= 1.0e-12:
            return [target]
        steps = max(1, int(math.ceil(distance / max_step)))
        return [start + delta * (index / steps) for index in range(1, steps + 1)]

    def make_official_robot(self) -> Any:
        import sys

        if str(LEROBOT_SRC) not in sys.path:
            sys.path.insert(0, str(LEROBOT_SRC))
        from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
        from lerobot.robots.so_follower.so_follower import SOFollower

        config = SOFollowerRobotConfig(
            id=str(self.config["robot_id"]),
            port=str(self.config["follower_port"]),
            use_degrees=True,
            max_relative_target=None,
        )
        return SOFollower(config)

    def make_read_only_bus(self) -> Any:
        import sys

        if str(LEROBOT_SRC) not in sys.path:
            sys.path.insert(0, str(LEROBOT_SRC))
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        motors = {
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }
        calibration = {
            name: MotorCalibration(
                id=entry.id,
                drive_mode=entry.drive_mode,
                homing_offset=entry.homing_offset,
                range_min=entry.range_min,
                range_max=entry.range_max,
            )
            for name, entry in self.calibration.items()
        }
        return FeetechMotorsBus(
            port=str(self.config["follower_port"]),
            motors=motors,
            calibration=calibration,
        )

    def read_state_read_only(self) -> dict[str, Any]:
        port_check = self.check_port_and_usb()
        if not port_check["success"]:
            return {
                "success": False,
                "reason": port_check["reason"],
                "opened_com_ports": False,
                "goal_position_written": False,
                "motion_command_sent": False,
            }
        bus = self.make_read_only_bus()
        opened = False
        try:
            bus.connect(handshake=False)
            opened = True
            raw = bus.sync_read("Present_Position", ALL_MOTOR_NAMES, normalize=False, num_retry=3)
            calibrated = bus.sync_read("Present_Position", ALL_MOTOR_NAMES, normalize=True, num_retry=3)
        finally:
            if opened:
                bus.disconnect(disable_torque=False)

        observation = {f"{name}.pos": float(calibrated[name]) for name in ALL_MOTOR_NAMES}
        joints_rad, gripper = self.observation_to_internal_rad(observation)
        positive = joints_rad["wrist_roll"] + math.radians(float(self.config["first_test_delta_deg"]))
        current = joints_rad["wrist_roll"]
        return {
            "success": True,
            "reason": "read_only_state_ok",
            "opened_com_ports": True,
            "port_info": port_check["port_info"],
            "raw_lerobot_positions": {name: float(raw[name]) for name in ALL_MOTOR_NAMES},
            "calibrated_lerobot_positions": {name: float(calibrated[name]) for name in ALL_MOTOR_NAMES},
            "joint_positions_rad": joints_rad,
            "gripper_value": gripper,
            "all_current_within_calibration": self.all_targets_within_calibration(joints_rad, gripper),
            "positive_target_within_calibration": self.target_within_calibration("wrist_roll", positive),
            "return_target_within_calibration": self.target_within_calibration("wrist_roll", current),
            "goal_position_written": False,
            "motion_command_sent": False,
            "torque_enable_written": False,
            "torque_disable_written": False,
        }

    def execute_plan(
        self,
        plan: dict[str, Any],
        *,
        enable_hardware_motion: bool,
        confirm: str,
        adapter: HardwareAdapter | None = None,
    ) -> dict[str, Any]:
        if not enable_hardware_motion or confirm != "SMALL_WRIST_ROLL_2DEG":
            return {"success": False, "reason": "confirmation_missing"}

        robot = adapter or self.adapter
        if robot is None:
            robot = self.make_official_robot()

        rate = float(self.config["control_rate_hz"])
        timeout_s = float(self.config["motion_timeout_s"])
        threshold_rad = math.radians(float(self.config["maximum_tracking_error_deg"]))
        bad_tracking_count = 0
        records = []
        self.stop_requested = False

        try:
            try:
                robot.connect(calibrate=False)
            except TypeError:
                robot.connect()
            segment_start = time.monotonic()
            last_phase = None
            for point in plan["points"]:
                if self.stop_requested:
                    return {"success": False, "reason": "stopped", "records": records}
                phase = str(point["phase"])
                if phase != last_phase:
                    segment_start = time.monotonic()
                    last_phase = phase
                if time.monotonic() - segment_start > timeout_s:
                    return {"success": False, "reason": "motion_timeout", "records": records}

                joints_rad = dict(point["joint_positions_rad"])
                active_joint = point.get("active_joint_name")
                if active_joint is None:
                    time.sleep(1.0 / rate)
                    continue
                if active_joint != "wrist_roll":
                    return {"success": False, "reason": "unexpected_active_joint", "records": records}
                if not self.target_within_calibration(active_joint, joints_rad[active_joint]):
                    return {"success": False, "reason": "calibration_target_out_of_range", "records": records}

                action = build_lerobot_action(joints_rad, point["gripper_value"])
                sent = robot.send_action(action)
                self.goal_position_written = True
                self.motion_command_sent = True
                observation = robot.read_state() if hasattr(robot, "read_state") else robot.get_observation()
                measured, _ = self.observation_to_internal_rad(observation)
                error = abs(measured["wrist_roll"] - joints_rad["wrist_roll"])
                bad_tracking_count = bad_tracking_count + 1 if error > threshold_rad else 0
                records.append(
                    {
                        "target_wrist_roll_rad": joints_rad["wrist_roll"],
                        "measured_wrist_roll_rad": measured["wrist_roll"],
                        "tracking_error_rad": error,
                        "sent": sent,
                    }
                )
                if bad_tracking_count >= 3:
                    return {"success": False, "reason": "tracking_error_exceeded", "records": records}
                time.sleep(1.0 / rate)
            return {"success": True, "reason": "completed", "records": records}
        except KeyboardInterrupt:
            self.stop_requested = True
            print("MOTION STOP REQUESTED")
            print("NO FURTHER TARGET PROGRESSION")
            print("POWER OFF SERVO SUPPLY IF NEEDED")
            return {"success": False, "reason": "keyboard_interrupt_stop", "records": records}
        finally:
            robot.disconnect()
