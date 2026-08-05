from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
HARDWARE_CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_hardware.json"
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
for package_path in (
    ROS_SRC / "so101_mvp_control",
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_kinematics.joint_limits import joints_within_limits
from so101_mvp_kinematics.model import So101KinematicModel


ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
ACCEPTED_PREGRASP_STATUS = {
    "pregrasp_ready",
    "pregrasp_ready_exact",
    "pregrasp_ready_near",
    "pregrasp_ready_offset",
}
CONFIRM_PHRASE = "MOVE_TO_PREGRASP"


@dataclass(frozen=True)
class MoveConfig:
    joint_state_max_age_s: float = 1.0
    pregrasp_target_max_age_s: float = 2.0
    speed_rad_s: float = 0.06
    max_abs_joint_delta_rad: float = 1.00
    final_joint_tolerance_rad: float = 0.035
    execute_service_timeout_s: float = 120.0
    max_speed_rad_s: float = 0.08


@dataclass(frozen=True)
class StampedJointState:
    names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    received_monotonic_s: float


def load_config(path: Path | None = None) -> MoveConfig:
    config_path = path or PROJECT_ROOT / "config" / "mvp_pregrasp_move.yaml"
    hardware_speed, hardware_max_speed = load_hardware_speed_config(HARDWARE_CONFIG_PATH)
    if not config_path.is_file():
        return MoveConfig(speed_rad_s=hardware_speed, max_speed_rad_s=hardware_max_speed)
    values: dict[str, float] = {}
    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.endswith(":") or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        try:
            values[key] = float(value)
        except ValueError:
            continue
    return MoveConfig(
        joint_state_max_age_s=values.get("joint_state_max_age_s", 1.0),
        pregrasp_target_max_age_s=values.get("pregrasp_target_max_age_s", 2.0),
        speed_rad_s=hardware_speed,
        max_abs_joint_delta_rad=values.get("max_abs_joint_delta_rad", 1.00),
        final_joint_tolerance_rad=values.get("final_joint_tolerance_rad", 0.035),
        execute_service_timeout_s=values.get("execute_service_timeout_s", 120.0),
        max_speed_rad_s=hardware_max_speed,
    )


def load_hardware_speed_config(path: Path) -> tuple[float, float]:
    if not path.is_file():
        return 0.06, 0.08
    data = json.loads(path.read_text(encoding="utf-8"))
    speed = float(data.get("first_test_speed_rad_s", 0.06))
    max_speed = float(data.get("maximum_speed_rad_s", 0.08))
    return speed, max_speed


def create_model() -> So101KinematicModel:
    return So101KinematicModel(
        PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
    )


def finite_list(values: list[float] | tuple[float, ...]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def validate_joint_contract(names: list[str] | tuple[str, ...], positions: list[float] | tuple[float, ...]) -> tuple[bool, str]:
    if tuple(names) != ARM_JOINT_NAMES:
        return False, "joint_name_order_invalid"
    if len(positions) != len(ARM_JOINT_NAMES):
        return False, "joint_position_count_invalid"
    if not finite_list(tuple(float(value) for value in positions)):
        return False, "joint_position_non_finite"
    return True, "ok"


def validate_fresh_joint_state(
    state: StampedJointState | None,
    *,
    now_monotonic_s: float,
    max_age_s: float,
) -> tuple[bool, str]:
    if state is None:
        return False, "missing_joint_state"
    valid, reason = validate_joint_contract(state.names, state.positions_rad)
    if not valid:
        return False, reason
    if now_monotonic_s - state.received_monotonic_s > max_age_s:
        return False, "joint_state_stale"
    return True, "ok"


def target_within_urdf_limits(model: So101KinematicModel, target_rad: list[float]) -> bool:
    return joints_within_limits(model, np.asarray(target_rad, dtype=np.float64))


def joint_delta(current_rad: list[float], target_rad: list[float]) -> dict[str, Any]:
    delta = [float(t) - float(c) for c, t in zip(current_rad, target_rad, strict=True)]
    delta_deg = [math.degrees(value) for value in delta]
    maximum = max(abs(value) for value in delta) if delta else 0.0
    return {
        "joint_delta_rad": delta,
        "joint_delta_deg": delta_deg,
        "maximum_abs_joint_delta_rad": maximum,
    }


def estimated_duration_s(current_rad: list[float], target_rad: list[float], speed_rad_s: float) -> float:
    if speed_rad_s <= 0.0:
        return math.inf
    return float(sum(abs(float(t) - float(c)) for c, t in zip(current_rad, target_rad, strict=True)) / speed_rad_s)


def parse_float_list(value: str) -> list[float] | None:
    try:
        parsed = [float(part.strip()) for part in value.split(",")]
    except ValueError:
        return None
    if len(parsed) != 3 or not all(math.isfinite(item) for item in parsed):
        return None
    return parsed


def parse_compute_message(message: str) -> tuple[dict[str, Any], list[str]]:
    fields: dict[str, Any] = {}
    warnings: list[str] = []
    text = str(message)
    offset_match = re.search(r"\boffset_m=\[([^\]]*)\]", text)
    if offset_match:
        offset = parse_float_list(offset_match.group(1))
        if offset is None:
            fields["offset_m"] = None
            warnings.append("selected_offset_m_parse_failed")
        else:
            fields["offset_m"] = offset
        text = text.replace(offset_match.group(0), "")
    elif "offset_m=" in text:
        fields["offset_m"] = None
        warnings.append("selected_offset_m_parse_failed")
    for part in text.split():
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields, warnings


def validate_speed(speed_rad_s: float, max_speed_rad_s: float) -> tuple[bool, str]:
    speed = float(speed_rad_s)
    max_speed = float(max_speed_rad_s)
    if not math.isfinite(speed) or not math.isfinite(max_speed):
        return False, "invalid_speed_rad_s"
    if speed <= 0.0 or speed > max_speed:
        return False, "invalid_speed_rad_s"
    return True, "ok"


def status_is_accepted(status: str, compute_message: str) -> bool:
    if str(status) in ACCEPTED_PREGRASP_STATUS:
        return True
    return "solution_type=accepted_near_solution" in str(compute_message)


def execute_preconditions(
    *,
    tcp_connected: bool,
    tcp_status: str,
    pregrasp_valid: bool,
    pregrasp_status: str,
    compute_message: str,
    joint_limits_valid: bool,
    maximum_abs_joint_delta_rad: float,
    max_abs_joint_delta_rad: float,
    confirm: str,
) -> tuple[bool, str]:
    if confirm != CONFIRM_PHRASE:
        return False, "wrong_confirmation"
    if not tcp_connected:
        return False, "tcp_disconnected"
    if tcp_status != "connected":
        return False, "tcp_status_not_connected"
    if not pregrasp_valid:
        return False, "pregrasp_invalid"
    if not status_is_accepted(pregrasp_status, compute_message):
        return False, "pregrasp_status_not_ready"
    if not joint_limits_valid:
        return False, "joint_limit_failed"
    if maximum_abs_joint_delta_rad > max_abs_joint_delta_rad:
        return False, "joint_delta_exceeds_mvp4b_limit"
    return True, "ok"


def final_joint_error(final_rad: list[float], target_rad: list[float], tolerance_rad: float) -> dict[str, Any]:
    errors = [abs(float(f) - float(t)) for f, t in zip(final_rad, target_rad, strict=True)]
    maximum = max(errors) if errors else math.inf
    return {
        "final_joint_error_rad": errors,
        "maximum_final_joint_error_rad": maximum,
        "final_joint_tolerance_rad": float(tolerance_rad),
        "final_target_reached": bool(maximum <= tolerance_rad),
    }


def json_print(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False))


class PregraspMoveNode:
    def __init__(self, config: MoveConfig) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.JointState = JointState
        self.Trigger = Trigger
        self.node = _Node("mvp_move_to_pregrasp")
        self.config = config
        self.latest_joint_state: StampedJointState | None = None
        self.latest_pregrasp_target: StampedJointState | None = None
        self.latest_pregrasp_pose: PoseStamped | None = None
        self.latest_pregrasp_target_time = 0.0
        self.pregrasp_valid = False
        self.pregrasp_status = ""
        self.tcp_connected = False
        self.tcp_status = "unknown"

        self.node.create_subscription(JointState, "/mvp/joint_states", self._joint_state_cb, 10)
        self.node.create_subscription(JointState, "/mvp/pregrasp_joint_target", self._pregrasp_target_cb, 10)
        self.node.create_subscription(PoseStamped, "/mvp/pregrasp_pose", self._pregrasp_pose_cb, 10)
        self.node.create_subscription(Bool, "/mvp/pregrasp_valid", self._pregrasp_valid_cb, 10)
        self.node.create_subscription(String, "/mvp/pregrasp_status", self._pregrasp_status_cb, 10)
        self.node.create_subscription(Bool, "/mvp/tcp_connected", self._tcp_connected_cb, 10)
        self.node.create_subscription(String, "/mvp/tcp_status", self._tcp_status_cb, 10)
        self.target_pub = self.node.create_publisher(JointState, "/mvp/joint_target", 10)
        self.compute_client = self.node.create_client(Trigger, "/mvp/compute_pregrasp")
        self.execute_client = self.node.create_client(Trigger, "/mvp/execute_target")

    def _joint_state_cb(self, msg: Any) -> None:
        self.latest_joint_state = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )

    def _pregrasp_target_cb(self, msg: Any) -> None:
        self.latest_pregrasp_target = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )
        self.latest_pregrasp_target_time = self.latest_pregrasp_target.received_monotonic_s

    def _pregrasp_pose_cb(self, msg: Any) -> None:
        self.latest_pregrasp_pose = msg

    def _pregrasp_valid_cb(self, msg: Any) -> None:
        self.pregrasp_valid = bool(msg.data)

    def _pregrasp_status_cb(self, msg: Any) -> None:
        self.pregrasp_status = str(msg.data)

    def _tcp_connected_cb(self, msg: Any) -> None:
        self.tcp_connected = bool(msg.data)

    def _tcp_status_cb(self, msg: Any) -> None:
        self.tcp_status = str(msg.data)

    def spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call_trigger(self, client: Any, timeout_s: float) -> tuple[bool, str, bool]:
        if not client.wait_for_service(timeout_sec=3.0):
            return False, "service_unavailable", False
        future = client.call_async(self.Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, "execute_service_timeout" if timeout_s >= 100.0 else "service_timeout", True
        response = future.result()
        return bool(response.success), str(response.message), True

    def publish_frozen_target_once(self, target_rad: list[float]) -> None:
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(value) for value in target_rad]
        self.target_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def destroy(self) -> None:
        self.node.destroy_node()


def pose_to_list(msg: Any) -> list[float] | None:
    if msg is None:
        return None
    return [
        float(msg.pose.position.x),
        float(msg.pose.position.y),
        float(msg.pose.position.z),
    ]


def build_plan_summary(
    *,
    mode: str,
    current: list[float],
    target: list[float],
    config: MoveConfig,
    pregrasp_pose: list[float] | None,
    compute_message: str,
    pregrasp_valid: bool,
    pregrasp_status: str,
    tcp_connected: bool,
    tcp_status: str,
    joint_limits_valid: bool,
    hardware_command_sent: bool,
) -> dict[str, Any]:
    delta = joint_delta(current, target)
    fields, parse_warnings = parse_compute_message(compute_message)
    return {
        "success": True,
        "mode": mode,
        "current_joint_positions_rad": current,
        "frozen_target_rad": target,
        **delta,
        "max_abs_joint_delta_limit_rad": float(config.max_abs_joint_delta_rad),
        "estimated_motion_duration_s": estimated_duration_s(current, target, config.speed_rad_s),
        "pregrasp_pose_base": pregrasp_pose,
        "compute_response_message": compute_message,
        "solution_type": fields.get("solution_type"),
        "selected_offset_m": fields.get("offset_m"),
        "parse_warnings": parse_warnings,
        "position_error_m": fields.get("position_error_m"),
        "approach_error_deg": fields.get("approach_error_deg"),
        "pregrasp_valid": bool(pregrasp_valid),
        "pregrasp_status": str(pregrasp_status),
        "joint_limits_valid": bool(joint_limits_valid),
        "tcp_connected": bool(tcp_connected),
        "tcp_status": str(tcp_status),
        "hardware_command_sent": bool(hardware_command_sent),
    }


def run(args: argparse.Namespace) -> int:
    import rclpy

    config = load_config()
    speed_ok, speed_reason = validate_speed(config.speed_rad_s, config.max_speed_rad_s)
    if not speed_ok:
        json_print({"success": False, "reason": speed_reason})
        return 2
    model = create_model()
    rclpy.init()
    mover = PregraspMoveNode(config)
    try:
        if not mover.spin_until(
            lambda: validate_fresh_joint_state(
                mover.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=config.joint_state_max_age_s,
            )[0],
            10.0,
        ):
            json_print({"success": False, "reason": "joint_state_unavailable_or_stale"})
            return 3
        assert mover.latest_joint_state is not None
        current = [float(value) for value in mover.latest_joint_state.positions_rad]

        compute_started = time.monotonic()
        compute_success, compute_message, _ = mover.call_trigger(mover.compute_client, 10.0)
        if not compute_success:
            json_print({"success": False, "reason": "compute_pregrasp_failed", "message": compute_message})
            return 4

        if not mover.spin_until(
            lambda: mover.latest_pregrasp_target is not None
            and mover.latest_pregrasp_target_time >= compute_started
            and time.monotonic() - mover.latest_pregrasp_target.received_monotonic_s <= config.pregrasp_target_max_age_s,
            config.pregrasp_target_max_age_s,
        ):
            json_print({"success": False, "reason": "pregrasp_target_unavailable_or_stale"})
            return 5
        assert mover.latest_pregrasp_target is not None
        target_state = mover.latest_pregrasp_target
        valid_target, target_reason = validate_joint_contract(target_state.names, target_state.positions_rad)
        if not valid_target:
            json_print({"success": False, "reason": target_reason})
            return 6
        if not mover.pregrasp_valid:
            json_print({"success": False, "reason": "pregrasp_invalid"})
            return 7
        if not status_is_accepted(mover.pregrasp_status, compute_message):
            json_print({"success": False, "reason": "pregrasp_status_not_ready", "status": mover.pregrasp_status})
            return 8

        frozen_target = [float(value) for value in target_state.positions_rad]
        joint_limits_valid = target_within_urdf_limits(model, frozen_target)
        summary = build_plan_summary(
            mode="plan_only" if args.plan_only or not args.execute else "execute",
            current=current,
            target=frozen_target,
            config=config,
            pregrasp_pose=pose_to_list(mover.latest_pregrasp_pose),
            compute_message=compute_message,
            pregrasp_valid=mover.pregrasp_valid,
            pregrasp_status=mover.pregrasp_status,
            tcp_connected=mover.tcp_connected,
            tcp_status=mover.tcp_status,
            joint_limits_valid=joint_limits_valid,
            hardware_command_sent=False,
        )
        if args.plan_only or not args.execute:
            json_print(summary)
            return 0

        allowed, reason = execute_preconditions(
            tcp_connected=mover.tcp_connected,
            tcp_status=mover.tcp_status,
            pregrasp_valid=mover.pregrasp_valid,
            pregrasp_status=mover.pregrasp_status,
            compute_message=compute_message,
            joint_limits_valid=joint_limits_valid,
            maximum_abs_joint_delta_rad=float(summary["maximum_abs_joint_delta_rad"]),
            max_abs_joint_delta_rad=config.max_abs_joint_delta_rad,
            confirm=args.confirm,
        )
        if not allowed:
            summary["success"] = False
            summary["reason"] = reason
            json_print(summary)
            return 9

        mover.publish_frozen_target_once(frozen_target)
        execute_success, execute_message, _ = mover.call_trigger(
            mover.execute_client,
            config.execute_service_timeout_s,
        )
        if not execute_success:
            summary["success"] = False
            summary["reason"] = f"motion_result_unknown: {execute_message}" if "timeout" in execute_message else execute_message
            summary["hardware_command_sent"] = True
            json_print(summary)
            return 10

        mover.spin_until(
            lambda: validate_fresh_joint_state(
                mover.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=config.joint_state_max_age_s,
            )[0],
            3.0,
        )
        final = current if mover.latest_joint_state is None else [float(value) for value in mover.latest_joint_state.positions_rad]
        final_errors = final_joint_error(final, frozen_target, config.final_joint_tolerance_rad)
        summary.update(
            {
                "success": bool(final_errors["final_target_reached"]),
                "reason": "ok" if final_errors["final_target_reached"] else "final_joint_tolerance_failed",
                "execute_response_message": execute_message,
                "hardware_command_sent": True,
                "final_joint_positions_rad": final,
                **final_errors,
            }
        )
        json_print(summary)
        return 0 if summary["success"] else 11
    finally:
        mover.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Move SO-101 once to the frozen visual pregrasp joint target.")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.execute and args.plan_only:
        print("--plan-only and --execute are mutually exclusive", file=sys.stderr)
        return 2
    if args.execute and args.confirm != CONFIRM_PHRASE:
        json_print({"success": False, "reason": "wrong_confirmation", "required_confirm": CONFIRM_PHRASE})
        return 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
