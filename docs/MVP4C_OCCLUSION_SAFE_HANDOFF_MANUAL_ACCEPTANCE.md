# MVP-4C Occlusion-Safe Snapshot Handoff Manual Acceptance

After the arm reaches the pregrasp pose, the gripper may block the camera view of the object. That occlusion is expected during grasp approach, not an error. The descent stage therefore uses the frozen pregrasp snapshot saved before occlusion instead of reacquiring live object pose.

If the object, camera, work table, robot base, or arm is moved after the snapshot is saved, abandon the snapshot and rerun the pregrasp stage. If the snapshot is older than 300 seconds, rerun the pregrasp stage.

For the first retest after this fix, do not reuse old logs. Close old nodes, power off the Follower, manually place the arm back in a safe start area that does not occlude the object, keep the object/camera/table fixed, then run the full flow below.

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

## Terminal 4: Pregrasp, Snapshot Check, Then Descent

Step 1, pregrasp plan-only:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_move_to_pregrasp.py --plan-only"
```

Step 2, execute pregrasp and save/update the snapshot:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_move_to_pregrasp.py --execute --confirm MOVE_TO_PREGRASP"
```

Step 3, inspect the saved snapshot:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
Get-Content data\runtime\mvp_last_pregrasp_snapshot.json -Raw
```

Confirm:

- `snapshot_state` is `executed_descent_ready`
- `motion_completed` is `true`
- `pregrasp_reached_for_descent` is `true`

Step 4, descent plan-only. This must work even if the object is now hidden by the gripper:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_descend_from_pregrasp.py --plan-only"
```

Confirm:

- `pregrasp_source` is `saved_snapshot`
- `live_object_visibility_required` is `false`
- `snapshot_valid` is `true`
- `all_waypoints_valid` is `true`
- `hardware_command_sent` is `false`

Step 5, execute descent:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_descend_from_pregrasp.py --execute --confirm DESCEND_3CM"
```

## Important Contract

- Object occlusion by the gripper after pregrasp is expected.
- Descent does not require live `/object_pose` or `/object_pose_base`.
- Descent does not call `/mvp/compute_pregrasp`.
- The object, camera, table, robot base, and arm must not be moved between snapshot creation and descent completion.
- If any of those move, rerun the pregrasp stage to create a new snapshot.
- If the snapshot is older than 300 seconds, rerun the pregrasp stage.
