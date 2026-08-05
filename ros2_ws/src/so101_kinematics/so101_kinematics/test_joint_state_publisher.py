from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class TestJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__(
            "so101_test_joint_state_publisher"
        )

        self.declare_parameter(
            "mode",
            "zero",
        )

        self.declare_parameter(
            "publish_rate_hz",
            20.0,
        )

        self.mode = str(
            self.get_parameter(
                "mode"
            ).value
        )

        publish_rate_hz = float(
            self.get_parameter(
                "publish_rate_hz"
            ).value
        )

        if self.mode not in (
            "zero",
            "sine",
        ):
            raise ValueError(
                "mode must be zero or sine"
            )

        if publish_rate_hz <= 0:
            raise ValueError(
                "publish_rate_hz must be positive"
            )

        self.publisher = (
            self.create_publisher(
                JointState,
                "/joint_states",
                10,
            )
        )

        self.start_time = time.monotonic()

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_joint_state,
        )

        self.get_logger().info(
            "Test JointState publisher started"
        )

        self.get_logger().info(
            f"mode={self.mode}, "
            f"rate={publish_rate_hz:.1f} Hz"
        )

        self.get_logger().info(
            "This node publishes data only; "
            "it does not control hardware."
        )

    def publish_joint_state(self) -> None:
        elapsed = (
            time.monotonic()
            - self.start_time
        )

        if self.mode == "zero":
            arm_positions = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]

        else:
            arm_positions = [
                0.25
                * math.sin(
                    0.40 * elapsed
                ),
                0.20
                * math.sin(
                    0.32 * elapsed + 0.4
                ),
                0.25
                * math.sin(
                    0.28 * elapsed + 0.8
                ),
                0.18
                * math.sin(
                    0.36 * elapsed + 1.2
                ),
                0.20
                * math.sin(
                    0.44 * elapsed + 1.6
                ),
            ]

        message = JointState()

        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.name = list(
            JOINT_NAMES
        )

        message.position = (
            arm_positions + [0.0]
        )

        message.velocity = []
        message.effort = []

        self.publisher.publish(message)


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node = TestJointStatePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()