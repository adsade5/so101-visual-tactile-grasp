from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    project_root = LaunchConfiguration("project_root")
    show_debug_window = LaunchConfiguration("show_debug_window")
    real_joint_state_host = LaunchConfiguration("real_joint_state_host")
    real_joint_state_port = LaunchConfiguration("real_joint_state_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=(
                    "E:/PycharmProjects/Embodied_AI/"
                    "LeRobot_Project/so101_visual_tactile_grasp"
                ),
            ),
            DeclareLaunchArgument("show_debug_window", default_value="false"),
            DeclareLaunchArgument("real_joint_state_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("real_joint_state_port", default_value="8766"),
            LogInfo(msg="REAL JOINT STATE READ-ONLY"),
            LogInfo(msg="CONNECTION TRAJECTORY SHADOW ONLY"),
            LogInfo(msg="HARDWARE MOTION DISABLED"),
            LogInfo(msg="NO CONTROLLER COMMAND TOPICS"),
            LogInfo(msg="EXPLICIT SHADOW START REQUIRED"),
            LogInfo(msg="MOCK JOINT STATE DISABLED"),
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
            Node(
                package="so101_command_gate",
                executable="real_joint_state_bridge_node",
                name="real_joint_state_bridge_node",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "host": real_joint_state_host,
                        "port": ParameterValue(real_joint_state_port, value_type=int),
                    }
                ],
            ),
            Node(
                package="so101_command_gate",
                executable="connection_trajectory_node",
                name="connection_trajectory_node",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "current_joint_state_topic": "/real_joint_states",
                        "current_joint_state_valid_topic": "/real_joint_state_valid",
                        "source_trajectory_topic": "/safe_timed_grasp_trajectory",
                        "source_valid_topic": "/safe_timed_grasp_valid",
                        "source_status_topic": "/safe_timed_grasp_status",
                    }
                ],
            ),
            Node(
                package="so101_command_gate",
                executable="command_gate_node",
                name="command_gate_node",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "current_joint_state_topic": "/real_joint_states",
                        "current_joint_state_valid_topic": "/real_joint_state_valid",
                        "source_trajectory_topic": "/connected_safe_timed_grasp_trajectory",
                        "source_valid_topic": "/connection_plan_valid",
                        "source_status_topic": "/connection_plan_status",
                    }
                ],
            ),
            Node(
                package="so101_command_gate",
                executable="shadow_executor_node",
                name="shadow_executor_node",
                output="screen",
                parameters=[{"project_root": project_root}],
            ),
        ]
    )
