from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    project_root = LaunchConfiguration("project_root")
    show_debug_window = LaunchConfiguration("show_debug_window")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=(
                    "E:/PycharmProjects/Embodied_AI/"
                    "LeRobot_Project/so101_visual_tactile_grasp"
                ),
            ),
            DeclareLaunchArgument(
                "show_debug_window",
                default_value="false",
            ),
            LogInfo(msg="SO-101 object perception started."),
            LogInfo(msg="SO-101 workspace transform started."),
            LogInfo(msg="SO-101 visual grasp planner started."),
            LogInfo(msg="SO-101 timed trajectory safety started."),
            LogInfo(
                msg=(
                    "Hardware control disabled; controller command topics disabled; "
                    "no COM ports or LeRobot hardware server are used."
                )
            ),
            LogInfo(
                msg=(
                    "Trajectory safety limits: provisional_software_preview_limits; "
                    "maximum velocity 0.20 rad/s; maximum acceleration 0.40 rad/s2; "
                    "sample rate 50 Hz."
                )
            ),
            Node(
                package="so101_object_perception",
                executable="object_pose_node",
                name="object_pose_node",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "show_debug_window": ParameterValue(
                            show_debug_window,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="so101_frame_transform",
                executable="workspace_to_base_node",
                name="workspace_to_base_node",
                output="screen",
                parameters=[{"project_root": project_root}],
            ),
            Node(
                package="so101_grasp_planner",
                executable="visual_grasp_planner_node",
                name="visual_grasp_planner_node",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "pregrasp_clearance_m": 0.055,
                        "grasp_clearance_m": 0.015,
                        "vertical_step_m": 0.005,
                        "minimum_tcp_z_m": 0.035,
                    }
                ],
            ),
            Node(
                package="so101_trajectory_safety",
                executable="timed_trajectory_node",
                name="timed_trajectory_node",
                output="screen",
                parameters=[{"project_root": project_root}],
            ),
        ]
    )
