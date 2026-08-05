# MVP-4E Tactile Stop Grasp And Lift Manual Acceptance

## 1. Tactile Static Test

Prerequisite: keep the normal MVP TCP server and ROS2 MVP bridge running, with the existing FlexiTac guard stream available through the reused UDP guard path from `so101_ros2_tactile_guard` (`127.0.0.1:5006`). This step does not publish motion targets, does not call pregrasp compute, and does not use the camera.

```powershell
python scripts\mvp_visual_grasp.py --tactile-test
```

Pass criteria:

- `success=true`
- `reason=tactile_static_test_pass`
- `tactile_ready_seen=true`
- `tactile_false_seen=true`
- `tactile_true_seen=true`
- `tactile_release_seen_after_true=true`
- `hardware_command_sent=false`
- `camera_used=false`
- `pregrasp_compute_called=false`
- `ros_publish_count=0`

## 2. Final Plan-Only And Execute

Plan all visual, descent, tactile stop, and lift waypoints before motion:

```powershell
python scripts\mvp_visual_grasp.py --plan-only
```

Pass criteria:

- `success=true`
- `tactile_stop_enabled=true`
- `tactile_ready_before_motion=true`
- `tactile_contact_before_motion=false`
- `waypoint_count=7`
- `lift_waypoint_count=3`
- `all_lift_waypoints_valid=true`
- `all_motion_waypoint_count=12`
- `hardware_command_sent=false`

Execute the full one-command grasp:

```powershell
python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP
```

Pass criteria:

- Visual object detection and pregrasp happen before motion.
- No live visual update is required after motion starts.
- The gripper opens from `g0` to `g0 + 10`.
- Final close requests `stop_gripper_on_tactile_contact=true`.
- When tactile contact is confirmed, the gripper holds at the trigger position with zero preload.
- The arm lifts in place through +1 cm, +2 cm, and +3 cm waypoints.
- XY remains unchanged by the lift planner within tolerance.
- The gripper target is held during all lift waypoints.
- If contact is not detected before reaching `g0`, the command reports `gripper_closed_without_tactile_contact` and does not lift.
