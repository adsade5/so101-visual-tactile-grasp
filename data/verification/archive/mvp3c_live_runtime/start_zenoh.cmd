@echo off
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
set ROS_LOG_DIR=E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\data\verification\mvp3c_live_runtime
powershell.exe -NoProfile -ExecutionPolicy Bypass -File audit\run_in_ros2_lyrical.ps1 -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"
