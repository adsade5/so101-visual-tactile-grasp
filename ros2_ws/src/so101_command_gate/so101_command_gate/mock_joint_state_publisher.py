from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from .command_gate_validator import DEFAULT_PROJECT_ROOT, load_command_gate_config


class MockJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_joint_state_publisher")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.declare_parameter("mode", "match_first_trajectory")
        self.declare_parameter("offset_joint_index", 0)
        self.declare_parameter("offset_rad", 0.10)
        self.declare_parameter("stale_publish_duration_s", 0.45)

        self.project_root = Path(str(self.get_parameter("project_root").value)).resolve()
        self.config = load_command_gate_config(self.project_root)
        self.mode = str(self.get_parameter("mode").value)
        self.offset_joint_index = int(self.get_parameter("offset_joint_index").value)
        self.offset_rad = float(self.get_parameter("offset_rad").value)
        self.stale_publish_duration_s = float(
            self.get_parameter("stale_publish_duration_s").value
        )

        self.first_positions: list[float] | None = None
        self.first_trajectory_time: float | None = None
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointTrajectory,
            "/safe_timed_grasp_trajectory",
            self.handle_trajectory,
            10,
        )
        self.create_timer(0.05, self.publish_joint_state)
        self.get_logger().warning(
            "TEST-ONLY MOCK JOINT STATE ENABLED | "
            f"mode={self.mode} | test_only_mock_joint_state=true | "
            "no COM ports, hardware server, or real SO-101 connection are used."
        )

    def handle_trajectory(self, message: JointTrajectory) -> None:
        if not message.points:
            return
        positions = [float(value) for value in message.points[0].positions]
        if len(positions) != len(self.config.joint_names):
            return
        if self.mode == "fixed_offset":
            index = max(0, min(self.offset_joint_index, len(positions) - 1))
            positions[index] += self.offset_rad
        self.first_positions = positions
        self.first_trajectory_time = time.monotonic()

    def publish_joint_state(self) -> None:
        if self.first_positions is None:
            return
        if (
            self.mode == "stale"
            and self.first_trajectory_time is not None
            and time.monotonic() - self.first_trajectory_time
            > self.stale_publish_duration_s
        ):
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.config.joint_names)
        message.position = [float(value) for value in self.first_positions]
        message.velocity = [0.0] * len(self.config.joint_names)
        message.effort = []
        self.publisher.publish(message)

    def mock_metadata(self) -> str:
        return json.dumps(
            {
                "test_only_mock_joint_state": True,
                "mode": self.mode,
                "offset_joint_index": self.offset_joint_index,
                "offset_rad": self.offset_rad,
                "stale_publish_duration_s": self.stale_publish_duration_s,
            },
            allow_nan=False,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MockJointStatePublisher | None = None
    try:
        node = MockJointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
