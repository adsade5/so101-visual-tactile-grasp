# MVP-4D One-Command Visual Grasp Manual Acceptance

This stage uses a single user-side grasp entry point:

```powershell
python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP
```

The script reads the current `gripper.pos` at the start of each run as `initial_gripper_position`, opens during descent to `initial_gripper_position + 10.0`, then closes back to `initial_gripper_position`. The user does not enter gripper values and does not run a separate gripper setup command.

## Safety Gate

Before execute:

- Place the object in the verified center workspace area.
- Keep the camera, object, table, robot base, and calibration setup fixed.
- Confirm a 7 cm descent will not hit the table.
- Keep the Follower power switch reachable.
- Do not run TCP Probe.
- Use only one TCP client.

Power off Follower immediately if the gripper opens in the wrong direction, wrist_roll moves instead of gripper, the wrist or gripper may hit the table, the arm moves sideways away from the object, motion becomes unexpectedly fast, the gripper keeps squeezing, the arm vibrates, or cables are pulled.

## Terminal 0: Zenoh Router

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
zenohd
```

## Terminal 1: Motion-Enabled TCP Server

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion
```

## Terminal 2: Motion-Enabled ROS2 Hardware Bridge

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 run so101_mvp_control mvp_hardware_bridge_node --ros-args -p hardware_motion_enabled:=true
```

## Terminal 3: Pregrasp Preview Launch

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py
```

## Terminal 4: One-Command Visual Grasp

Plan only:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_visual_grasp.py --plan-only
```

Execute only after the plan is valid:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP
```

## PASS Criteria

- One command performs visual freeze, pregrasp, 7 cm descent, gripper open, and close.
- Live visual object pose is used only before motion.
- Object occlusion after pregrasp does not stop the flow.
- All seven descent waypoints execute.
- The arm moves toward the object.
- The gripper gradually opens by relative deltas `[1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.0]`.
- `wrist_roll` is not mistaken for gripper.
- The final cumulative descent is about 7 cm.
- The final close target is the automatically captured initial gripper position.
- The arm holds still during final close.
- No lift is performed.
- No automatic return is performed.
- No repeated close command is sent.
- TCP remains connected.
- There is only one TCP client.
- No abnormal fast motion, persistent vibration, or table collision occurs.
