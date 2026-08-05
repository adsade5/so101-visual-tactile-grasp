from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enable_hardware_motion = LaunchConfiguration("enable_hardware_motion")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_hardware_motion",
                default_value="false",
                description="Set true only during supervised manual hardware acceptance.",
            ),
            Node(
                package="so101_mvp_control",
                executable="mvp_hardware_bridge_node",
                name="mvp_hardware_bridge_node",
                output="screen",
                parameters=[
                    {
                        "host": "127.0.0.1",
                        "port": 8770,
                        "state_poll_rate_hz": 5.0,
                        "hardware_motion_enabled": enable_hardware_motion,
                        "default_speed_rad_s": 0.06,
                    }
                ],
            ),
        ]
    )
