# MVP-4B Pregrasp Move Manual Acceptance

This stage moves the real SO-101 once to the frozen visual pregrasp joint target. It does not descend to the object, does not close the gripper, and does not run a grasp.

## Preconditions

- Put the object near the center of the workspace.
- Keep the arm workspace clear.
- Do not hold an object in the gripper.
- Keep the Follower power switch easy to reach.
- Do not run TCP probe tools.
- Make sure old ROS2 hardware bridge processes are closed.
- After the TCP server starts, normal operation should show only one ROS2 bridge client.
- Run plan-only first and confirm the maximum single-joint delta is no more than `1.00` rad.

## Terminal 0: Zenoh Router

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"
```

## Terminal 1: Motion-Enabled LeRobot TCP Server

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& E:\Anaconda\Scripts\conda.exe run `
  --no-capture-output `
  -p E:\Anaconda\envs_dirs\lerobot `
  python scripts\mvp_so101_server.py `
  --config config\mvp_hardware.json `
  --enable-hardware-motion
```

Expected:

```text
TCP_LISTENING host=127.0.0.1 port=8770 single_client=true
TCP_CLIENT_CONNECTED id=1 address=...
```

## Terminal 2: Motion-Enabled ROS2 Hardware Bridge

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
```

Expected:

```text
single_tcp_client=true
TCP_CONNECTED host=127.0.0.1 port=8770
```

## Terminal 3: Visual Pregrasp Planner

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp show_debug_window:=false"
```

## Terminal 4: Plan And Execute One Frozen Target

Plan only:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_move_to_pregrasp.py --plan-only"
```

Only if plan-only passes and `maximum_abs_joint_delta_rad <= 1.00`, execute:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_move_to_pregrasp.py --execute --confirm MOVE_TO_PREGRASP"
```

The user confirmation phrase is `MOVE_TO_PREGRASP`. The internal TCP confirmation remains `MVP_MOVE` inside the ROS2 hardware bridge.

## PASS Criteria

- Visual target is normal.
- `/mvp/compute_pregrasp` succeeds.
- The target is frozen after compute.
- Plan-only sends no motion.
- Plan-only prints five target arm joints.
- Target joints are within URDF limits.
- Maximum single-joint delta is no more than `1.00` rad.
- TCP is connected.
- The exact confirmation phrase is required before execution.
- `/mvp/joint_target` is published once.
- `/mvp/execute_target` is called once.
- The arm moves slowly at `0.06` rad/s.
- The existing sequential joint motion behavior is preserved.
- The arm stops at the pregrasp joint position.
- Final per-joint error is no more than `0.035` rad.
- The gripper does not move.
- TCP remains connected after motion.
- No `tcp_connection_closed`.
- No `motion_result_unknown`.
- No repeated execution.
- No descent to the object.
- No grasp.
- No abnormal fast motion.
- No abnormal jitter.

## Abnormal Handling

1. Press Ctrl+C in Terminal 4 if the script is still running.
2. Press Ctrl+C in Terminal 2 to stop the ROS2 hardware bridge.
3. If the arm is still behaving abnormally, turn off Follower power directly.

No software stop service is added in this stage.

## Shutdown Order

1. Stop Terminal 4 command.
2. Stop Terminal 3 visual preview launch.
3. Stop Terminal 2 hardware bridge.
4. Stop Terminal 1 TCP server.
5. Stop Terminal 0 Zenoh router.
