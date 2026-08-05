from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory(
            "so101_description"
        )
    )

    urdf_path = (
        package_share
        / "urdf"
        / "so101_visualization.urdf"
    )

    default_rviz_config = (
        package_share
        / "rviz"
        / "so101.rviz"
    )

    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"URDF not found: {urdf_path}"
        )

    robot_description = urdf_path.read_text(
        encoding="utf-8"
    )

    start_rviz = LaunchConfiguration(
        "start_rviz"
    )

    rviz_config = LaunchConfiguration(
        "rviz_config"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="so101_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": (
                    robot_description
                ),
                "publish_frequency": 30.0,
                "frame_prefix": "",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="so101_rviz",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description=(
                    "Start RViz together with "
                    "robot_state_publisher."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    default_rviz_config
                ),
                description=(
                    "RViz configuration file."
                ),
            ),
            robot_state_publisher_node,
            rviz_node,
        ]
    )
