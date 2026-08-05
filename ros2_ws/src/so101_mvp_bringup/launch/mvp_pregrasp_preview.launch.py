from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    project_root = LaunchConfiguration("project_root")
    show_debug_window = LaunchConfiguration("show_debug_window")
    start_object_pose_node = LaunchConfiguration("start_object_pose_node")

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
            DeclareLaunchArgument(
                "start_object_pose_node",
                default_value="true",
            ),
            LogInfo(
                msg=(
                    "Starting SO-101 MVP pregrasp preview only: "
                    "no hardware bridge, no TCP server, no robot command topics."
                )
            ),
            Node(
                package="so101_object_perception",
                executable="object_pose_node",
                name="object_pose_node",
                condition=IfCondition(start_object_pose_node),
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
                package="so101_mvp_control",
                executable="mvp_pregrasp_planner_node",
                name="mvp_pregrasp_planner_node",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "object_pose_topic": "/object_pose_base",
                        "base_frame": "base_link",
                        "max_object_pose_age_s": 1.0,
                        "pregrasp_height_m": 0.08,
                        "use_joint_state_seed": True,
                        "joint_state_topic": "/mvp/joint_states",
                        "max_joint_state_age_s": 1.0,
                        "position_tolerance_m": 0.002,
                        "approach_tolerance_deg": 5.0,
                    }
                ],
            ),
        ]
    )
