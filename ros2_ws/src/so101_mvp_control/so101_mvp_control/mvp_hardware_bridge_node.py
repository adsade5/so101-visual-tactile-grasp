from __future__ import annotations

import math
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_srvs.srv import Trigger

from so101_mvp_control.mvp_tcp_client import MvpTcpClient, MvpTcpError


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
        self.declare_parameter("default_speed_rad_s", 0.06)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.poll_rate_hz = float(self.get_parameter("state_poll_rate_hz").value)
        self.hardware_motion_enabled = bool(self.get_parameter("hardware_motion_enabled").value)
        self.default_speed_rad_s = float(self.get_parameter("default_speed_rad_s").value)

        self._client: MvpTcpClient | None = None
        self.last_valid_target: list[float] | None = None
        self.last_target_stamp = None
        self.tcp_connected = False
        self.tcp_status = "disconnected"
        self.last_tcp_warning_time = 0.0
        self._motion_request_active = False

        self.joint_pub = self.create_publisher(JointState, "/mvp/joint_states", 10)
        self.gripper_pub = self.create_publisher(Float64, "/mvp/gripper_state", 10)
        self.tcp_connected_pub = self.create_publisher(Bool, "/mvp/tcp_connected", 10)
        self.tcp_status_pub = self.create_publisher(String, "/mvp/tcp_status", 10)
        self.create_subscription(JointState, "/mvp/joint_target", self.handle_joint_target, 10)
        self.create_service(Trigger, "/mvp/execute_target", self.handle_execute_target)
        timer_period = 1.0 / max(self.poll_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.poll_state_once)
        self.get_logger().info("MVP hardware bridge starting")
        self.get_logger().info(f"tcp_host={self.host}")
        self.get_logger().info(f"tcp_port={self.port}")
        self.get_logger().info(f"hardware_motion_enabled={str(self.hardware_motion_enabled).lower()}")
        self.get_logger().info(f"state_poll_rate_hz={self.poll_rate_hz}")
        self.get_logger().info("single_tcp_client=true")
        self.publish_tcp_status(False, "disconnected")

    def get_client(self) -> MvpTcpClient:
        if self._client is None:
            self._client = MvpTcpClient(
                self.host,
                self.port,
                connect_timeout_s=2.0,
                state_request_timeout_s=2.0,
                motion_request_timeout_s=120.0,
            )
        return self._client

    def reset_client(self) -> None:
        if self._client is not None:
            self._client.close()
        self._client = None

    def publish_tcp_status(self, connected: bool, status: str) -> None:
        self.tcp_connected = bool(connected)
        self.tcp_status = str(status)
        connected_msg = Bool()
        connected_msg.data = self.tcp_connected
        self.tcp_connected_pub.publish(connected_msg)
        status_msg = String()
        status_msg.data = self.tcp_status
        self.tcp_status_pub.publish(status_msg)

    def format_tcp_exception(self, exc: BaseException) -> tuple[str, str]:
        if isinstance(exc, MvpTcpError):
            return exc.kind, str(exc)
        return type(exc).__name__, str(exc)

    def service_error_message(self, exc: BaseException) -> str:
        kind, message = self.format_tcp_exception(exc)
        text = f"{kind}: {message}"
        return text[:300]

    def warn_tcp_failure(self, exc: BaseException) -> None:
        kind, message = self.format_tcp_exception(exc)
        now = self.get_clock().now().nanoseconds / 1.0e9
        should_log = self.tcp_connected or (now - self.last_tcp_warning_time) >= 2.0
        if should_log:
            self.get_logger().warning(f"TCP_DISCONNECTED error_type={kind} error={message}")
            self.last_tcp_warning_time = now
        self.publish_tcp_status(False, kind)
        self.reset_client()

    def poll_state_once(self) -> None:
        if self._motion_request_active:
            return
        try:
            state = self.get_client().get_state()
        except Exception as exc:
            self.warn_tcp_failure(exc)
            return
        if not state.get("success"):
            reason = str(state.get("reason", "missing_reason"))
            self.get_logger().warning(f"TCP get_state returned failure: {reason}")
            self.publish_tcp_status(False, f"server_rejected:{reason}")
            return
        try:
            names = list(state["joint_names"])
            positions = [float(value) for value in state["positions_rad"]]
            gripper = float(state["gripper"])
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid TCP state payload: {exc}")
            self.publish_tcp_status(False, "protocol_error")
            return
        if names != ARM_JOINT_NAMES or len(positions) != len(ARM_JOINT_NAMES):
            self.get_logger().warning("Invalid joint order or length from TCP state")
            self.publish_tcp_status(False, "protocol_error")
            return
        if not all(math.isfinite(value) for value in positions):
            self.get_logger().warning("Non-finite joint position from TCP state")
            self.publish_tcp_status(False, "protocol_error")
            return

        if not self.tcp_connected:
            self.get_logger().info(f"TCP_CONNECTED host={self.host} port={self.port}")
        self.publish_tcp_status(True, "connected")

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
            self._motion_request_active = True
            result = self.get_client().move_joints_sequential(
                self.last_valid_target,
                self.default_speed_rad_s,
                list(range(len(ARM_JOINT_NAMES))),
                confirm="MVP_MOVE",
            )
        except Exception as exc:
            self.get_logger().warning(f"TCP move_joints_sequential failed: {type(exc).__name__}: {exc}")
            self.warn_tcp_failure(exc)
            response.success = False
            response.message = self.service_error_message(exc)
            return response
        finally:
            self._motion_request_active = False
        response.success = bool(result.get("success"))
        reason = str(result.get("reason", "missing_reason"))
        response.message = reason
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
