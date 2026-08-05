from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="so101_mvp_control",
                executable="mvp_grasp_controller_node",
                name="mvp_grasp_controller_node",
                output="screen",
            ),
            Node(
                package="so101_mvp_control",
                executable="mvp_hardware_bridge_node",
                name="mvp_hardware_bridge_node",
                output="screen",
            ),
        ]
    )

