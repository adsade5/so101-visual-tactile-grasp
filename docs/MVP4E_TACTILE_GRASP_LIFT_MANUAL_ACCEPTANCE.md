# MVP-4E Direct COM8 Tactile Grasp Lift Manual Acceptance

Long-running terminals:

- Terminal 0: Zenoh.
- Terminal 1: `scripts\mvp_so101_server.py`; this process owns SO-101 Follower `COM4` and FlexiTac `COM8`.
- Terminal 2: ROS2 MVP hardware bridge.

## 1. Tactile Static Test

Keep the long-running terminals above active. Do not touch FlexiTac while Terminal 1 prints `DO_NOT_TOUCH_FLEXITAC_DURING_BASELINE`; wait until it prints `TACTILE_READY true`.

```powershell
python scripts\mvp_visual_grasp.py --tactile-test
```

Pass criteria:

- `success=true`
- `reason=tactile_static_test_pass`
- `tactile_source=direct_serial`
- `tactile_port=COM8`
- `tactile_ready_seen=true`
- `tactile_false_seen=true`
- `tactile_true_seen=true`
- `tactile_release_seen_after_true=true`
- `hardware_command_sent=false`

Expected status-change lines:

```text
TACTILE_TEST contact=false score=<value>
TACTILE_TEST contact=true score=<value>
TACTILE_TEST contact=false score=<value>
```

## 2. Final Plan-Only And Execute

Start the visual launch for object pose and pregrasp planning, then plan the full grasp without motion:

```powershell
python scripts\mvp_visual_grasp.py --plan-only
```

Pass criteria:

- `success=true`
- `tactile_stop_enabled=true`
- `tactile_source=direct_serial`
- `tactile_port=COM8`
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
