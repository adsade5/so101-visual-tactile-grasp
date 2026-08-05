# MVP-4B Speed 0.06 Manual Retest

Retest only the small supervised wrist-roll motion. Do not rerun the full visual pregrasp move for this speed check.

## Terminal 0: Zenoh Router

```powershell
& "E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\run_in_ros2_lyrical.ps1" -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"
```

## Terminal 1: Motion-Enabled LeRobot Server

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion
```

## Terminal 2: Motion-Enabled ROS2 Hardware Bridge

```powershell
& "E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\run_in_ros2_lyrical.ps1" -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
```

## Terminal 3: Wrist-Roll 2 Degree Retest

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_ros2_wrist_test.py --execute --confirm ROS2_WRIST_ROLL_2DEG
```

## Manual Checks

- Only `wrist_roll` moves.
- Motion is slightly faster than the previous `0.04 rad/s` behavior.
- Motion remains smooth.
- The 2 degree forward and return motion completes.
- The gripper does not move.
- TCP remains connected after the action.
- Server log contains `MOTION_STARTED speed_rad_s=0.06`.
- No `tcp_connection_closed`.
- No `motion_result_unknown`.
