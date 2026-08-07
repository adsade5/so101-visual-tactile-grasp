# MVP-4E Final Manual Acceptance (Simplified Multi-Terminal)

## A. Single Helper Command

Open 4 independent terminals at once:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\open_mvp4e_terminals.ps1
```

This is NOT a supervisor. It only opens windows. You inspect each terminal by eye.

## B. Four Terminals

| Terminal | Title | What runs |
|----------|-------|-----------|
| 0 | `MVP4E - 0 Zenoh` | Zenoh router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) |
| 1 | `MVP4E - 1 Server COM4 COM8` | LeRobot server: COM4 follower, COM8 FlexiTac, TCP server |
| 2 | `MVP4E - 2 ROS2 Bridge` | ROS2 hardware bridge (sole TCP client) |
| 3 | `MVP4E - 3 Vision` | Visual perception / pregrasp preview nodes |

## C. Ready Signs (Read Each Terminal by Eye)

**Terminal 0 — Zenoh:**
Zenoh router prints its normal startup banner. If the terminal hangs at startup, Zenoh is not ready.

**Terminal 1 — Server:**
You MUST see ALL of these lines:
- `TACTILE_SERIAL_OPENED port=COM8`
- `TACTILE_BASELINE_COMPLETED`
- `TACTILE_READY true`
- `ROBOT_CONNECTED port=COM4`
- `TCP_SERVER_LISTENING`

**Terminal 2 — Bridge:**
You MUST see:
- Bridge node initializes without crash
- `BRIDGE_TCP_CONNECTED` (or equivalent TCP connected message)
- `BRIDGE_TCP_READY true`

**Terminal 3 — Vision:**
You MUST see:
- Object pose publishing repeatedly (`/object_pose_base` or similar)
- No fatal errors in the node output

**Do NOT proceed to plan-only until ALL four terminals show their ready signs.**

## D. Tactile Test (Optional)

Only after Terminals 0, 1, and 2 are healthy (vision not required):

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_visual_grasp.py --tactile-test"
```

## E. Plan-Only

Only after ALL four ready signs are confirmed:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_visual_grasp.py --plan-only"
```

Expected output:
- `success=true`
- `waypoint_count=7`
- `lift_waypoint_count=3`
- `hardware_command_sent=false`

**If plan-only fails, stop. Do not proceed to execute.**

## F. Execute

**Only after plan-only PASS.** Keep the object and camera stationary.

You MUST type this command manually:

```powershell
cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP"
```

No script will type this for you.

## G. Final PASS Criteria

| # | Criterion |
|---|-----------|
| 1 | COM8 tactile ready |
| 2 | COM4 robot connected |
| 3 | Bridge TCP connected |
| 4 | Visual object pose normal |
| 5 | plan-only `success=true` |
| 6 | `waypoint_count=7` |
| 7 | `lift_waypoint_count=3` |
| 8 | `hardware_command_sent=false` in plan-only |
| 9 | Execute reaches pregrasp |
| 10 | 7-segment descent completes |
| 11 | Tactile contact stops further gripper closing |
| 12 | 3-segment lift after contact (+1 cm, +2 cm, +3 cm) |
| 13 | Object physically leaves the table |
| 14 | No contact → no lift |
| 15 | No automatic retry |
| 16 | No automatic return |
| 17 | No automatic place |

## H. Ctrl+C Shutdown Order

1. Wait for action to finish
2. Terminal 3 (Vision) — Ctrl+C
3. Terminal 2 (Bridge) — Ctrl+C
4. Terminal 1 (Server) — Ctrl+C
5. Terminal 0 (Zenoh) — Ctrl+C
6. Follower power off

Do NOT use an automated process killer.

---

> **Legacy note:** The complex one-launch scripts (`scripts/launch_mvp4e_system.ps1`, `scripts/run_mvp4e_bridge.ps1`) are retained only for historical debugging and are NOT part of final MVP-4E acceptance. They are marked `DEPRECATED_FOR_FINAL_ACCEPTANCE`.
