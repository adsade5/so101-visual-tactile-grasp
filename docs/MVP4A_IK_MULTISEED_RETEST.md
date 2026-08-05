# MVP-4A IK Multiseed Retest

This retest validates the no-motion visual pregrasp chain after adding deterministic multiseed IK attempts.

Do not start COM4, `scripts/mvp_so101_server.py`, `mvp_hardware_bridge_node`, or any command executor. Do not call `/mvp/execute_target`.

## Terminal 0: Zenoh

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Terminal 1: Preview Chain

This launch starts `object_pose_node`, `workspace_to_base_node`, and `mvp_pregrasp_planner_node`.

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp show_debug_window:=false
```

## Terminal 2: Check And Compute

Check the transformed target once:

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 topic echo /object_pose_base --once
```

Call the planner:

```bat
ros2 service call /mvp/compute_pregrasp std_srvs/srv/Trigger "{}"
```

Check status and target:

```bat
ros2 topic echo /mvp/pregrasp_status --once
ros2 topic echo /mvp/pregrasp_joint_target --once
```

Inspect the latest diagnostic JSON:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
Get-Content data\verification\mvp4a_last_pregrasp_diagnostic.json
```

Optional offline replay with the exact `/object_pose_base` coordinates:

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_pregrasp_replay.py --x 0.18 --y -0.04 --z 0.0
```

## Expected Result

- `/mvp/compute_pregrasp` returns `success=true` for reachable targets.
- The response message includes `seed_source`, `attempt_index`, `x`, `y`, `z`, `position_error_m`, and `approach_error_deg`.
- If it fails, the response starts with `ik_failed_all_seeds`, `joint_limit_failed`, or `fk_validation_failed` and includes object/pregrasp coordinates and attempt count.
- Node logs contain one `PREGRASP_INPUT` line and one `IK_ATTEMPT` line per seed for each service call.
- `/mvp/pregrasp_status` is `pregrasp_ready` after success.
- `/mvp/pregrasp_joint_target` contains exactly five joints: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`.
- No `/mvp/joint_target` is published.
- `/mvp/execute_target` is not called.
- COM4 remains closed.
- The robot does not move.

## Shutdown Order

1. Stop Terminal 2 checks.
2. Stop Terminal 1 preview launch.
3. Stop Terminal 0 Zenoh.
