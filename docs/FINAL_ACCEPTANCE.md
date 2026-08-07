# Final Hardware Acceptance Guide

**Status: PASS**

This document describes the validated procedure for running the SO-101 visual-tactile grasp-and-lift pipeline on real hardware.

> **Original acceptance document:** `docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md` (retained as reference)
>
> **Legacy note:** The complex one-launch scripts (`scripts/launch_mvp4e_system.ps1`, `scripts/run_mvp4e_bridge.ps1`) are deprecated and NOT part of final acceptance.

---

## Prerequisites

### Hardware

- SO-101 robot arm (Follower) connected via COM4
- FlexiTac tactile sensor connected via COM8
- USB camera (top-down view) connected and positioned
- Single regular object block with ArUco marker, placed on table within robot workspace
- Robot servo power supply accessible for emergency cutoff

### Software

- Windows workstation
- LeRobot Conda environment (`lerobot`) with SO-101 hardware interface
- ROS2 Lyrical (Pixi environment) with `rmw_zenoh_cpp`
- All ROS2 packages built:
  ```powershell
  & ".\audit\run_in_ros2_lyrical.ps1" -Command `
    "cd /d <PROJECT_ROOT>\ros2_ws && colcon build --merge-install --packages-select so101_mvp_control so101_mvp_bringup so101_mvp_kinematics so101_object_perception so101_frame_transform"
  ```

### Calibration

Ensure these calibration files exist and are valid:
- `config/workspace_to_base.json` — workspace → base_link transform
- `config/camera.yaml` — camera configuration
- `data/calibration/camera_intrinsics.yaml` — camera intrinsics
- `config/object_marker.json` — ArUco marker configuration
- LeRobot follower calibration — path configured in `config/mvp_hardware.json`

---

## Startup Order

### Quick Start

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\open_mvp4e_terminals.ps1
```

This opens 4 independent PowerShell windows. The script is NOT a supervisor — it only opens windows for you.

### Manual Terminal Startup

If the helper script cannot be used, open each terminal manually:

**Terminal 0 — Zenoh Router:**
```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"
```

**Terminal 1 — LeRobot Server (COM4 + COM8 + TCP):**
```powershell
conda run --no-capture-output -p <CONDA_ENV_PATH> python -u scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion
```

**Terminal 2 — ROS2 Hardware Bridge:**
```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command "cd /d <PROJECT_ROOT>\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
```

**Terminal 3 — Visual Perception:**
```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command "cd /d <PROJECT_ROOT>\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py"
```

---

## Ready Signs

Before proceeding to plan-only, verify ALL terminals show their ready signs.

### Terminal 0 — Zenoh

Zenoh router prints its normal startup banner. If it hangs without output, Zenoh is not ready.

### Terminal 1 — Server

All of these lines MUST appear:

```
TACTILE_SERIAL_OPENED port=COM8
TACTILE_BASELINE_COMPLETED
TACTILE_READY true
ROBOT_CONNECTED port=COM4
TCP_SERVER_LISTENING
```

> **Important:** Do not touch the FlexiTac sensor during baseline capture (the first ~1-2 seconds after `TACTILE_SERIAL_OPENED`). Touching during baseline will produce false contact readings.

### Terminal 2 — Bridge

Bridge node initializes without crash. Look for:
```
BRIDGE_TCP_CONNECTED
BRIDGE_TCP_READY true
```

### Terminal 3 — Vision

Object pose is publishing. The `/object_pose_base` topic should show valid pose data. No fatal errors.

**Do NOT proceed until ALL four terminals show their ready signs.**

---

## Optional: Tactile Test

Test the tactile sensor before the full grasp (requires Terminals 0, 1, 2):

```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command `
  "cd /d <PROJECT_ROOT>\ros2_ws && call install\local_setup.bat && cd /d <PROJECT_ROOT> && python scripts\mvp_visual_grasp.py --tactile-test"
```

Touch the FlexiTac sensor to verify contact detection. The test prints tactile scores and contact status.

---

## Plan-Only

**Mandatory first step.** Validates perception, kinematics, and trajectory without sending any hardware motion.

```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command `
  "cd /d <PROJECT_ROOT>\ros2_ws && call install\local_setup.bat && cd /d <PROJECT_ROOT> && python scripts\mvp_visual_grasp.py --plan-only"
```

### Expected Output

```json
{
  "success": true,
  "waypoint_count": 7,
  "lift_waypoint_count": 3,
  "hardware_command_sent": false
}
```

### If Plan-Only Fails

| Symptom | Likely Cause |
|---------|-------------|
| No object pose | Camera not seeing ArUco marker; check lighting, marker visibility |
| IK failure | Object outside reachable workspace; reposition object |
| Joint limit violation | Pregrasp target exceeds URDF joint bounds; check calibration |
| Pregrasp planner error | Vision nodes not publishing; check Terminal 3 |

**If plan-only fails, STOP. Do not proceed to execute.** Fix the issue and re-run plan-only.

---

## Execute

**Only after plan-only PASS.** Keep the object and camera stationary — do not move anything between plan-only and execute.

You must type this command manually (copy-paste is acceptable, but the `--confirm VISUAL_GRASP` flag must be explicit):

```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command `
  "cd /d <PROJECT_ROOT>\ros2_ws && call install\local_setup.bat && cd /d <PROJECT_ROOT> && python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP"
```

### Expected Physical Behavior

1. Arm moves to pregrasp position above the object
2. Arm descends through 7 waypoints (7 cm total descent)
3. Gripper opens slightly, then begins incremental closing
4. Gripper stops immediately upon tactile contact
5. If contact confirmed, arm lifts in 3 segments (+1 cm, +2 cm, +3 cm)
6. Object physically leaves the table

### Safety During Execution

- Keep one hand near the physical power cutoff
- If any unexpected motion occurs, cut servo power immediately
- Ctrl+C in the main shell will abort the script but may not stop in-flight motion — use physical cutoff for emergency stop

---

## PASS Criteria

| # | Criterion | Description |
|---|-----------|-------------|
| 1 | COM8 tactile ready | FlexiTac initialized, baseline captured, streaming |
| 2 | COM4 robot connected | SO-101 follower arm connected and responding |
| 3 | Bridge TCP connected | ROS2 bridge successfully connected to TCP server |
| 4 | Visual object pose normal | Valid ArUco pose detected and transformed to base_link |
| 5 | Plan-only success=true | Planning validation passed |
| 6 | waypoint_count=7 | Correct number of descent waypoints |
| 7 | lift_waypoint_count=3 | Correct number of lift waypoints |
| 8 | hardware_command_sent=false | Plan-only sent no motion to hardware |
| 9 | Execute reaches pregrasp | Arm moves to approach position above object |
| 10 | 7-segment descent completes | Arm descends through all 7 waypoints |
| 11 | Tactile contact stops closing | Gripper stops at contact, not at position limit |
| 12 | 3-segment lift after contact | Arm lifts +1/+2/+3 cm after confirmed contact |
| 13 | Object physically leaves table | Object is lifted off the table surface |
| 14 | No contact → no lift | If no tactile contact, arm does not lift |
| 15 | No automatic retry | System does not retry after error/disconnect |
| 16 | No automatic return | Arm does not return to start position |
| 17 | No automatic place | Arm does not attempt to place the object |

---

## Shutdown Order

1. Wait for the grasp action to finish (or abort with Ctrl+C)
2. Terminal 3 (Vision) — Ctrl+C
3. Terminal 2 (Bridge) — Ctrl+C
4. Terminal 1 (Server) — Ctrl+C
5. Terminal 0 (Zenoh) — Ctrl+C
6. Robot follower power off

Do NOT use an automated process killer. Each terminal should be stopped individually.

---

## Acceptance Verification Evidence

- `data/verification/final/stage_mvp4e_tactile_grasp_lift_report.json` — 51/53 automated checks passed
- `data/verification/final/stage_mvp4e_final_simplification_report.json` — Final simplification and freeze
- `data/verification/final/stage_mvp4e_x_axis_grasp_offset_report.json` — X-axis +20 mm compensation
- `data/verification/final/stage_mvp4e_close_until_tactile_contact_report.json` — Close-until-contact behavior
- `data/verification/final/stage_mvp4e_direct_com8_tactile_report.json` — Direct COM8 tactile integration

See `docs/VERIFICATION.md` for the complete verification summary.
