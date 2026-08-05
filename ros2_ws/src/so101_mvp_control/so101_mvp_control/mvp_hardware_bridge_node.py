from __future__ import annotations

import math
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from so101_mvp_control.mvp_tcp_client import MvpTcpClient


ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


class MvpHardwareBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mvp_hardware_bridge_node")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8770)
        self.declare_parameter("state_poll_rate_hz", 5.0)
        self.declare_parameter("hardware_motion_enabled", False)
        self.declare_parameter("default_speed_rad_s", 0.04)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.poll_rate_hz = float(self.get_parameter("state_poll_rate_hz").value)
        self.hardware_motion_enabled = bool(self.get_parameter("hardware_motion_enabled").value)
        self.default_speed_rad_s = float(self.get_parameter("default_speed_rad_s").value)

        self.client: MvpTcpClient | None = None
        self.last_valid_target: list[float] | None = None
        self.last_target_stamp = None

        self.joint_pub = self.create_publisher(JointState, "/mvp/joint_states", 10)
        self.gripper_pub = self.create_publisher(Float64, "/mvp/gripper_state", 10)
        self.create_subscription(JointState, "/mvp/joint_target", self.handle_joint_target, 10)
        self.create_service(Trigger, "/mvp/execute_target", self.handle_execute_target)
        self.create_service(Trigger, "/mvp/stop", self.handle_stop)

        timer_period = 1.0 / max(self.poll_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.poll_state_once)
        self.get_logger().info(
            f"MVP hardware bridge started host={self.host} port={self.port} "
            f"motion_enabled={str(self.hardware_motion_enabled).lower()}"
        )

    def get_client(self) -> MvpTcpClient:
        if self.client is None:
            self.client = MvpTcpClient(self.host, self.port, timeout_s=2.0)
        return self.client

    def reset_client(self) -> None:
        if self.client is not None:
            self.client.close()
        self.client = None

    def poll_state_once(self) -> None:
        try:
            state = self.get_client().get_state()
        except Exception as exc:
            self.get_logger().warning(f"TCP get_state failed; will retry: {exc}")
            self.reset_client()
            return
        if not state.get("success"):
            self.get_logger().warning(f"TCP get_state returned failure: {state.get('reason')}")
            return
        try:
            names = list(state["joint_names"])
            positions = [float(value) for value in state["positions_rad"]]
            gripper = float(state["gripper"])
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid TCP state payload: {exc}")
            return
        if names != ARM_JOINT_NAMES or len(positions) != len(ARM_JOINT_NAMES):
            self.get_logger().warning("Invalid joint order or length from TCP state")
            return
        if not all(math.isfinite(value) for value in positions):
            self.get_logger().warning("Non-finite joint position from TCP state")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.joint_pub.publish(msg)

        gripper_msg = Float64()
        gripper_msg.data = gripper
        self.gripper_pub.publish(gripper_msg)

    def handle_joint_target(self, msg: JointState) -> None:
        target = self._validate_joint_target(msg)
        if target is None:
            self.last_valid_target = None
            return
        self.last_valid_target = target
        self.last_target_stamp = msg.header.stamp

    def _validate_joint_target(self, msg: JointState) -> list[float] | None:
        if len(msg.position) < len(ARM_JOINT_NAMES):
            self.get_logger().warning("Rejected /mvp/joint_target: not enough positions")
            return None
        if msg.name:
            by_name: dict[str, float] = {}
            for name, value in zip(msg.name, msg.position, strict=False):
                if name in ARM_JOINT_NAMES:
                    by_name[name] = float(value)
            if sorted(by_name) != sorted(ARM_JOINT_NAMES):
                self.get_logger().warning("Rejected /mvp/joint_target: missing MVP arm joints")
                return None
            values = [by_name[name] for name in ARM_JOINT_NAMES]
        else:
            values = [float(value) for value in msg.position[: len(ARM_JOINT_NAMES)]]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning("Rejected /mvp/joint_target: non-finite target")
            return None
        return values

    def handle_execute_target(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if not self.hardware_motion_enabled:
            response.success = False
            response.message = "hardware_motion_disabled"
            return response
        if self.last_valid_target is None:
            response.success = False
            response.message = "no_valid_target"
            return response
        try:
            result = self.get_client().move_joints_sequential(
                self.last_valid_target,
                self.default_speed_rad_s,
                list(range(len(ARM_JOINT_NAMES))),
                confirm="MVP_MOVE",
            )
        except Exception as exc:
            self.get_logger().warning(f"TCP move_joints_sequential failed: {exc}")
            self.reset_client()
            response.success = False
            response.message = "tcp_error"
            return response
        response.success = bool(result.get("success"))
        response.message = str(result.get("reason", "missing_reason"))
        return response

    def handle_stop(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            result = self.get_client().stop()
        except Exception as exc:
            self.get_logger().warning(f"TCP stop failed: {exc}")
            self.reset_client()
            response.success = False
            response.message = "tcp_error"
            return response
        response.success = bool(result.get("success"))
        response.message = str(result.get("reason", "missing_reason"))
        return response

    def destroy_node(self) -> bool:
        self.reset_client()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MvpHardwareBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
