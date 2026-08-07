@echo off
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
set ROS_LOG_DIR=E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\data\verification\mvp3c_live_runtime
powershell.exe -NoProfile -ExecutionPolicy Bypass -File audit\run_in_ros2_lyrical.ps1 -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_read_only.launch.py"
