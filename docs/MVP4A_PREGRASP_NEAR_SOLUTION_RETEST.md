# MVP-4A Pregrasp Near-Solution Retest

This retest validates the no-motion pregrasp planner after accepting safe near solutions for the 8 cm pregrasp hover target.

Do not start COM4, `scripts/mvp_so101_server.py`, `mvp_hardware_bridge_node`, or any hardware executor. Do not call `/mvp/execute_target`.

## Terminal 0: Zenoh Router

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Terminal 1: Visual Pregrasp Preview

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp show_debug_window:=false
```

## Terminal 2: Inspect And Compute

Check the current transformed target:

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 topic echo /object_pose_base --once
```

Compute the pregrasp plan:

```bat
ros2 service call /mvp/compute_pregrasp std_srvs/srv/Trigger "{}"
```

Expected for the current center target:

- `success=true`
- message contains `pregrasp_ready`
- message contains `solution_type=accepted_near_solution` for the original near solution, or an offset solution if a nearby candidate is better
- `/mvp/pregrasp_status` is `pregrasp_ready_near`, `pregrasp_ready_exact`, or `pregrasp_ready_offset`

Inspect outputs:

```bat
ros2 topic echo /mvp/pregrasp_pose --once
ros2 topic echo /mvp/pregrasp_joint_target --once
ros2 topic echo /mvp/pregrasp_valid --once
ros2 topic echo /mvp/pregrasp_status --once
```

Inspect the diagnostic JSON:

```powershell
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
Get-Content data\verification\mvp4a_last_pregrasp_diagnostic.json
```

Optional offline replay with the measured target:

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
python scripts\mvp_pregrasp_replay.py --x 0.199194 --y -0.000891 --z 0.025
```

## PASS Criteria

- `/mvp/compute_pregrasp` succeeds for a reachable center target.
- The response reports requested and selected pregrasp coordinates.
- If a nonzero offset is selected, message and diagnostic both show `requested_xyz`, `selected_xyz`, and `offset_m`.
- `/mvp/pregrasp_pose` publishes the selected pregrasp pose only.
- `/mvp/pregrasp_joint_target` contains exactly the five arm joints.
- `/mvp/pregrasp_valid` is `true`.
- No `/mvp/joint_target` is published.
- `/mvp/execute_target` is not called.
- COM4 remains closed.
- The robot does not move.

## Shutdown Order

1. Stop Terminal 2 checks.
2. Stop Terminal 1 preview launch.
3. Stop Terminal 0 Zenoh router.
