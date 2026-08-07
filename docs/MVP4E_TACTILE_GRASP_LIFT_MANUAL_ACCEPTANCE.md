# MVP-4E One-Launch Manual Acceptance

Use one PowerShell entry point for normal acceptance. It starts Zenoh, the LeRobot server, the ROS2 hardware bridge, visual nodes when needed, the action command, and then shuts down only the child processes it created.

## 1. Normal Commands

First retest only the static tactile path:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_mvp4e_system.ps1 -Mode TactileTest
```

Only after `TactileTest` starts all required components, completes FlexiTac baseline, and reports static tactile pass, run final acceptance:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_mvp4e_system.ps1 -Mode FinalAcceptance
```

Optional planning-only diagnosis:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\launch_mvp4e_system.ps1 -Mode PlanOnly
```

`FinalAcceptance` runs:

```text
start Zenoh
start mvp_so101_server.py
wait TACTILE_SERIAL_OPENED port=COM8, TACTILE_BASELINE_COMPLETED, TACTILE_READY true, ROBOT_CONNECTED port=COM4, TCP_SERVER_LISTENING
start ROS2 hardware bridge
wait BRIDGE_TCP_CONNECTED, /mvp/tcp_connected=true, fresh JointState, tactile_ready=true
start visual launch
wait /object_pose_base
run python scripts\mvp_visual_grasp.py --plan-only
prompt Type VISUAL_GRASP to execute
run python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP only on exact confirmation
cleanup action, visual nodes, bridge, server, Zenoh
```

Plan-only failure or any confirmation text other than exactly `VISUAL_GRASP` cancels execution and reports `hardware_command_sent=false`.

Runtime logs are written under:

```text
logs\runtime\<timestamp>\
```

Expected files:

```text
zenoh.log
server.log
bridge.log
vision.log
action.log
launcher.log
```

Failure output prints the failing component, exit code when available, and the last 100 log lines. The most useful files are:

- `logs/runtime/<timestamp>/launcher.log`: launcher state machine, owned PIDs, cleanup order, and failed stage.
- `logs/runtime/<timestamp>/zenoh.log`: ROS2 wrapper and Zenoh startup output.
- `logs/runtime/<timestamp>/server.log`: LeRobot server, COM8 FlexiTac, baseline, COM4 robot, and TCP listener startup output.

The launcher treats these Windows/ROS2 setup messages as non-fatal warnings:

- `RTI Connext DDS environment script not found`. MVP-4E uses `rmw_zenoh_cpp`, so missing RTI Connext setup does not block acceptance.
- `WinError 1314` / `Cannot create a symlink to latest log directory`. This only means a normal user PowerShell cannot create the optional ROS `latest` log symlink; timestamped ROS logs are still written.

You do not need an Administrator PowerShell. Acceptance is blocked only by a real fatal log line such as `[ERROR]`, `[FATAL]`, a Python `Traceback`, a component process exit, or a Bridge TCP connection timeout.

Pass criteria remain unchanged:

- FlexiTac source is `direct_serial` on `COM8`.
- Follower remains on `COM4`.
- Baseline completes before tactile use.
- Plan-only reports `success=true`, `waypoint_count=7`, `lift_waypoint_count=3`, and `hardware_command_sent=false`.
- Execute first moves to pregrasp, descends 7 waypoints, opens the gripper from `g0` to `g0 + 10`, closes with `stop_gripper_on_tactile_contact=true`, and lifts only after tactile contact.
- No contact means no lift.
- No automatic retry, no automatic return, and no automatic placement.

## 2. Advanced Troubleshooting

Manual multi-terminal startup is retained only for debugging logs and should not be used for normal acceptance.

Terminal 0:

```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"
```

Terminal 1:

```powershell
python scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion
```

Terminal 2:

```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
```

Terminal 3:

```powershell
& ".\audit\run_in_ros2_lyrical.ps1" -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py"
```

Terminal 4:

```powershell
python scripts\mvp_visual_grasp.py --plan-only
python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP
```
