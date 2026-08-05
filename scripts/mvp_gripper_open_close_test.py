from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_descend_from_pregrasp import ARM_JOINT_NAMES, validate_fresh_joint_state  # noqa: E402


CONFIRM_PHRASE = "GRIPPER_OPEN_TEST"
GRIPPER_MIN_POS = 0.0
GRIPPER_MAX_POS = 100.0
GRIPPER_TEST_DURATION_S = 2.0


@dataclass(frozen=True)
class StampedGripperState:
    position: float
    received_monotonic_s: float


def valid_gripper_target(value: float) -> bool:
    return math.isfinite(float(value)) and GRIPPER_MIN_POS <= float(value) <= GRIPPER_MAX_POS


def json_print(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False))


class GripperTestNode:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, String
        from std_srvs.srv import Trigger

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.JointState = JointState
        self.Float64 = Float64
        self.Trigger = Trigger
        self.node = _Node("mvp_gripper_open_close_test")
        self.latest_joint_state = None
        self.latest_gripper_state: StampedGripperState | None = None
        self.tcp_connected = False
        self.tcp_status = "unknown"
        self.node.create_subscription(JointState, "/mvp/joint_states", self._joint_state_cb, 10)
        self.node.create_subscription(Float64, "/mvp/gripper_state", self._gripper_state_cb, 10)
        self.node.create_subscription(Bool, "/mvp/tcp_connected", self._tcp_connected_cb, 10)
        self.node.create_subscription(String, "/mvp/tcp_status", self._tcp_status_cb, 10)
        self.arm_target_pub = self.node.create_publisher(JointState, "/mvp/joint_target", 10)
        self.gripper_target_pub = self.node.create_publisher(Float64, "/mvp/gripper_target", 10)
        self.execute_client = self.node.create_client(Trigger, "/mvp/execute_target")

    def _joint_state_cb(self, msg: Any) -> None:
        from mvp_descend_from_pregrasp import StampedJointState

        self.latest_joint_state = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )

    def _gripper_state_cb(self, msg: Any) -> None:
        self.latest_gripper_state = StampedGripperState(float(msg.data), time.monotonic())

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

    def publish_arm_hold_and_gripper(self, arm_rad: list[float], gripper_pos: float) -> None:
        arm_msg = self.JointState()
        arm_msg.header.stamp = self.node.get_clock().now().to_msg()
        arm_msg.name = list(ARM_JOINT_NAMES)
        arm_msg.position = [float(value) for value in arm_rad]
        self.arm_target_pub.publish(arm_msg)
        gripper_msg = self.Float64()
        gripper_msg.data = float(gripper_pos)
        self.gripper_target_pub.publish(gripper_msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def call_execute(self) -> tuple[bool, str]:
        if not self.execute_client.wait_for_service(timeout_sec=3.0):
            return False, "service_unavailable"
        future = self.execute_client.call_async(self.Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=120.0)
        if not future.done() or future.result() is None:
            return False, "execute_service_timeout"
        response = future.result()
        return bool(response.success), str(response.message)

    def destroy(self) -> None:
        self.node.destroy_node()


def run(args: argparse.Namespace) -> int:
    candidate = float(args.candidate_open_target)
    if not valid_gripper_target(candidate):
        json_print({"success": False, "reason": "gripper_target_out_of_calibration_range"})
        return 2
    if args.execute and args.confirm != CONFIRM_PHRASE:
        json_print({"success": False, "reason": "wrong_confirmation", "required_confirm": CONFIRM_PHRASE})
        return 2

    import rclpy

    rclpy.init()
    node = GripperTestNode()
    try:
        if not node.spin_until(
            lambda: validate_fresh_joint_state(
                node.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=1.0,
            )[0],
            10.0,
        ):
            json_print({"success": False, "reason": "joint_state_unavailable_or_stale"})
            return 3
        if not node.spin_until(
            lambda: node.latest_gripper_state is not None
            and time.monotonic() - node.latest_gripper_state.received_monotonic_s <= 1.0,
            10.0,
        ):
            json_print({"success": False, "reason": "gripper_state_unavailable_or_stale"})
            return 4
        if not node.tcp_connected or node.tcp_status != "connected":
            json_print({"success": False, "reason": "tcp_not_connected"})
            return 5
        assert node.latest_joint_state is not None
        assert node.latest_gripper_state is not None
        arm_hold = [float(value) for value in node.latest_joint_state.positions_rad]
        initial = float(node.latest_gripper_state.position)
        summary = {
            "success": True,
            "mode": "plan_only" if args.plan_only or not args.execute else "execute",
            "initial_gripper_position": initial,
            "candidate_open_target": candidate,
            "candidate_in_calibration_range": True,
            "return_target": initial,
            "gripper_motion_duration_s": GRIPPER_TEST_DURATION_S,
            "arm_joint_target_rad": arm_hold,
            "arm_motion_expected": False,
            "hardware_command_sent": False,
        }
        if args.plan_only or not args.execute:
            json_print(summary)
            return 0
        node.publish_arm_hold_and_gripper(arm_hold, candidate)
        open_success, open_message = node.call_execute()
        if not open_success:
            summary.update({"success": False, "reason": open_message, "hardware_command_sent": True})
            json_print(summary)
            return 6
        time.sleep(GRIPPER_TEST_DURATION_S)
        node.publish_arm_hold_and_gripper(arm_hold, initial)
        close_success, close_message = node.call_execute()
        summary.update(
            {
                "success": bool(close_success),
                "reason": "gripper_open_close_test_completed" if close_success else close_message,
                "hardware_command_sent": True,
            }
        )
        json_print(summary)
        return 0 if close_success else 7
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Small manual gripper open/close target verification.")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--candidate-open-target", required=True, type=float)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.execute and args.plan_only:
        print("--plan-only and --execute are mutually exclusive", file=sys.stderr)
        return 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
