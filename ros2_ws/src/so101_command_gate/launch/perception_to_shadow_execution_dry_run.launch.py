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
    use_mock_joint_state = LaunchConfiguration("use_mock_joint_state")
    mock_joint_state_mode = LaunchConfiguration("mock_joint_state_mode")

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
            DeclareLaunchArgument("use_mock_joint_state", default_value="false"),
            DeclareLaunchArgument(
                "mock_joint_state_mode",
                default_value="match_first_trajectory",
            ),
            LogInfo(msg="perception started"),
            LogInfo(msg="workspace transform started"),
            LogInfo(msg="grasp planner started"),
            LogInfo(msg="trajectory safety started"),
            LogInfo(msg="command gate started"),
            LogInfo(msg="shadow executor started"),
            LogInfo(msg="hardware control disabled"),
            LogInfo(msg="controller command topics disabled"),
            LogInfo(msg="shadow execution requires explicit trigger"),
            LogInfo(
                condition=IfCondition(use_mock_joint_state),
                msg="TEST-ONLY MOCK JOINT STATE ENABLED",
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
            Node(
                package="so101_command_gate",
                executable="command_gate_node",
                name="command_gate_node",
                output="screen",
                parameters=[{"project_root": project_root}],
            ),
            Node(
                package="so101_command_gate",
                executable="shadow_executor_node",
                name="shadow_executor_node",
                output="screen",
                parameters=[{"project_root": project_root}],
            ),
            Node(
                condition=IfCondition(use_mock_joint_state),
                package="so101_command_gate",
                executable="mock_joint_state_publisher",
                name="mock_joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "project_root": project_root,
                        "mode": mock_joint_state_mode,
                    }
                ],
            ),
        ]
    )
