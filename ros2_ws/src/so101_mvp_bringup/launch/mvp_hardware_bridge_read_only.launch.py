from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
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
                        "hardware_motion_enabled": False,
                        "default_speed_rad_s": 0.04,
                    }
                ],
            )
        ]
    )
