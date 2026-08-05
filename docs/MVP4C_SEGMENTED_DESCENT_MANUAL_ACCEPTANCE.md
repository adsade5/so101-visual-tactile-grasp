# MVP-4C Segmented 3 cm Descent Manual Acceptance

This stage starts only after MVP-4B has already moved the arm to the visual pregrasp pose. It plans three frozen Cartesian descent waypoints from that pregrasp pose: Z minus 1 cm, 2 cm, and 3 cm. The current hardware executor still moves joints sequentially, so this validates a three-waypoint Cartesian approximation, not a guaranteed straight TCP path during each joint motion.

## Before Starting

- The arm has already reached the MVP-4B pregrasp position.
- The object has not moved.
- The camera can still detect the object.
- The gripper area and the 3 cm descent volume are clear.
- The pregrasp pose still has obvious visible clearance from the object.
- The Follower power switch is easy to reach.
- Do not run TCP Probe.
- The server should have only one client.
- Run plan-only first and confirm all three waypoints are valid.
- Each waypoint maximum joint delta must be no more than `0.25` rad.

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

## Terminal 2: Motion-Enabled ROS2 Hardware Bridge

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
```

## Terminal 3: Visual Pregrasp Planner

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp show_debug_window:=false"
```

## Terminal 4: Plan First, Then Execute

Plan only:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_descend_from_pregrasp.py --plan-only"
```

Only if plan-only passes, execute:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_descend_from_pregrasp.py --execute --confirm DESCEND_3CM"
```

The user confirmation phrase is `DESCEND_3CM`. The internal ROS2 bridge to TCP server confirmation remains `MVP_MOVE`.

## PASS Criteria

- The script confirms the current arm is already at the pregrasp pose.
- All three descent waypoints are planned before any execution.
- Plan-only produces no robot motion.
- The requested X and Y values stay fixed for all waypoints.
- Requested Z descends by 1 cm, 2 cm, and 3 cm from pregrasp.
- Actual FK positions move downward overall.
- Each waypoint maximum joint delta is no more than `0.25` rad.
- Execution requires exactly `DESCEND_3CM`.
- Each waypoint target is published once.
- Each waypoint execute service is called once.
- The arm moves toward the object in three visible segments.
- The gripper remains downward.
- The gripper does not open or close.
- The gripper does not contact the object.
- No grasp occurs.
- Final pose still has visible safe clearance.
- Maximum final joint error is no more than `0.035` rad.
- TCP remains connected after motion.
- No `tcp_connection_closed`.
- No `motion_result_unknown`.
- No repeated execution.
- No abnormal high-speed motion.
- No abnormal vibration.

## Emergency Handling

If anything looks wrong, press `Ctrl+C` first. If the arm still behaves abnormally, directly turn off the Follower servo power.

Power off immediately if the gripper points in the wrong direction, the arm does not approach toward the object, several joints move suddenly at high speed, the gripper contacts the object, the gripper opens or closes, the arm keeps vibrating, cables are pulled, or the arm moves significantly sideways.
