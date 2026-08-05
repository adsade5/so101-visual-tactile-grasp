from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class TestWorkspacePosePublisher(Node):
    def __init__(self) -> None:
        super().__init__(
            "test_workspace_pose_publisher"
        )

        self.pose_publisher = (
            self.create_publisher(
                PoseStamped,
                "/object_pose",
                10,
            )
        )

        self.stable_publisher = (
            self.create_publisher(
                Bool,
                "/object_pose_stable",
                10,
            )
        )

        self.create_timer(
            0.05,
            self.publish_test_data,
        )

    def publish_test_data(self) -> None:
        stable = Bool()
        stable.data = True

        self.stable_publisher.publish(stable)

        pose = PoseStamped()
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )
        pose.header.frame_id = (
            "workspace_plane"
        )

        pose.pose.position.x = 0.2
        pose.pose.position.y = 0.1
        pose.pose.position.z = 0.025

        yaw_rad = math.radians(30.0)

        pose.pose.orientation.z = math.sin(
            yaw_rad / 2.0
        )

        pose.pose.orientation.w = math.cos(
            yaw_rad / 2.0
        )

        self.pose_publisher.publish(pose)


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node = TestWorkspacePosePublisher()

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