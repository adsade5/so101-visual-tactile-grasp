# MVP-4D Integrated Visual Grasp Manual Acceptance

This stage runs one integrated visual grasp command after the gripper open target has been manually verified.

Object occlusion after pregrasp is expected. The integrated flow freezes visual targets before motion and does not require live object visibility after motion starts.

## Safety Gate

Before any execute command:

- Place the object in the verified center workspace area.
- Keep the camera, object, table, robot base, and calibration setup fixed.
- Confirm the 7 cm descent will not hit the table.
- Keep the Follower power switch reachable.
- Do not run TCP Probe.
- Use only one TCP client.
- Confirm `gripper_open_target_pos` was verified with the small gripper test.

Power off Follower immediately if the gripper opens in the wrong direction, wrist_roll moves instead of gripper, the wrist or gripper may hit the table, the arm moves sideways away from the object, motion becomes unexpectedly fast, the gripper keeps squeezing, the arm vibrates, or cables are pulled.

## A. Gripper-Only Small Motion Acceptance

Use this first when `config/mvp_grasp.yaml` has:

```yaml
gripper_open_target_pos: null
gripper_open_target_verified: false
```

Plan a candidate:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_gripper_open_close_test.py --plan-only --candidate-open-target <value>
```

Execute only after checking the candidate is small and within range:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_gripper_open_close_test.py --execute --candidate-open-target <value> --confirm GRIPPER_OPEN_TEST
```

PASS criteria:

- Only `gripper.pos` moves.
- `wrist_roll` does not move.
- The candidate direction is visually confirmed as opening.
- The script returns to the initial gripper position.
- TCP remains connected.

After PASS, update `config/mvp_grasp.yaml`:

```yaml
gripper_open_target_pos: <verified_value>
gripper_open_target_verified: true
```

## B. Integrated Visual Grasp Acceptance

Terminal 0: Zenoh Router

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
zenohd
```

Terminal 1: motion-enabled TCP server

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion
```

Terminal 2: motion-enabled ROS2 hardware bridge

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 run so101_mvp_control mvp_hardware_bridge_node --ros-args -p hardware_motion_enabled:=true
```

Terminal 3: pregrasp preview launch

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py
```

Terminal 4: integrated command

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_visual_grasp.py --plan-only
```

Execute only after the plan is valid:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP
```

PASS criteria:

- One command performs pregrasp, descent, and close.
- Live visual object pose is used only before motion.
- Pregrasp motion is normal.
- All seven descent waypoints execute.
- The arm moves toward the object.
- The gripper gradually opens during descent.
- `wrist_roll` is not mistaken for gripper.
- Final cumulative descent is about 7 cm.
- The final close target is the initial gripper position.
- The arm holds still during final close.
- No lift is performed.
- No automatic return is performed.
- No repeated close command is sent.
- TCP remains connected.
- There is only one TCP client.
- No abnormal fast motion, persistent vibration, or table collision occurs.
