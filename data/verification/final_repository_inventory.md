# Final Repository Inventory

Generated: 2026-08-07
Stage: FINAL-REPOSITORY-CLEANUP-AND-DOCUMENTATION

Excluded from listing: `.git/`, `__pycache__/`, `.pytest_cache/`, `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`, `logs/runtime/`

---

## Category A: CORE — Required for running the system

### A1. Main Entry Scripts

| File | Purpose |
|------|---------|
| `scripts/mvp_so101_server.py` | LeRobot hardware server: COM4 robot, COM8 FlexiTac, TCP server on 8770 |
| `scripts/mvp_visual_grasp.py` | Integrated visual-tactile grasp planner/executor (ROS2 node) |
| `scripts/mvp_move_to_pregrasp.py` | Pregrasp motion planner (imported by visual_grasp) |
| `scripts/mvp_descend_from_pregrasp.py` | Descent planner (imported by visual_grasp) |
| `scripts/mvp_pregrasp_preview.py` | Pregrasp preview tool |
| `scripts/mvp_pregrasp_replay.py` | Pregrasp replay tool |
| `scripts/open_mvp4e_terminals.ps1` | Multi-terminal opener (FINAL OFFICIAL helper) |

### A2. LeRobot Hardware Layer

| File | Purpose |
|------|---------|
| `lerobot_server/__init__.py` | Package init |
| `lerobot_server/mvp_hardware_executor.py` | SO-101 hardware executor (Feetech bus, calibration, joint conversion) |
| `lerobot_server/mock_hardware_server.py` | Legacy mock server for testing |

### A3. ROS2 Packages (Active)

| Package | Purpose |
|---------|---------|
| `ros2_ws/src/so101_mvp_kinematics/` | FK, IK (damped least-squares), Jacobian, joint limits, kinematic model, transforms |
| `ros2_ws/src/so101_mvp_control/` | Hardware bridge node, TCP client, grasp controller, pregrasp planner, trajectory helpers |
| `ros2_ws/src/so101_mvp_bringup/` | Launch files: bridge, pregrasp preview, skeleton |
| `ros2_ws/src/so101_object_perception/` | ArUco-based object pose detection node |
| `ros2_ws/src/so101_frame_transform/` | Workspace-to-base coordinate transform node |
| `ros2_ws/src/so101_description/` | URDF model, STL assets, RViz config |

### A4. ROS2 Packages (Legacy — retained, not in MVP chain)

| Package | Purpose |
|---------|---------|
| `ros2_ws/src/so101_command_gate/` | Command gate, shadow executor, connection trajectory (archived) |
| `ros2_ws/src/so101_trajectory_safety/` | Timed trajectory, safety validator (archived) |
| `ros2_ws/src/so101_grasp_planner/` | Original grasp planner node (archived) |
| `ros2_ws/src/so101_kinematics/` | Original FK node, TF checker (archived) |
| `ros2_ws/src/so101_robot_bridge/` | Original TCP bridge (port 8765 era, archived) |

### A5. Configuration

| File | Purpose |
|------|---------|
| `config/mvp_hardware.json` | **Primary config**: COM ports, tactile, speeds, grasp offset |
| `config/mvp_grasp.yaml` | Grasp parameters: descent, lift, gripper close |
| `config/mvp_pregrasp.yaml` | Pregrasp planning parameters |
| `config/mvp_pregrasp_move.yaml` | Pregrasp motion parameters |
| `config/mvp_descent.yaml` | Descent planning parameters |
| `config/mvp_motion.yaml` | Motion execution parameters |
| `config/mvp_kinematics.yaml` | Kinematics solver parameters |
| `config/workspace_to_base.json` | Workspace-to-base transform calibration |
| `config/camera.yaml` | Camera configuration |
| `config/object_marker.json` | ArUco marker configuration |
| `config/kinematics.json` | Legacy kinematics config |
| `config/trajectory_safety.json` | Legacy trajectory safety config |
| `config/command_gate.json` | Legacy command gate config |
| `config/connection_trajectory.json` | Legacy connection trajectory config |
| `config/real_joint_state_mapping.json` | Joint state mapping (legacy read-only bridge) |
| `config/bridge.yaml` | Legacy bridge config (port 8765) |
| `config/workspace_to_base_before_stage_2d1.json` | Calibration backup |

### A6. Shared Protocol

| File | Purpose |
|------|---------|
| `shared_protocol/__init__.py` | Package init |
| `shared_protocol/mvp_tcp_client.py` | TCP client library (used by bridge) |
| `shared_protocol/protocol_v1.md` | Protocol v1 spec (legacy, port 8765) |
| `shared_protocol/message_envelope.schema.json` | Legacy message schema |
| `shared_protocol/examples/protocol_examples.jsonl` | Legacy protocol examples |

### A7. Audit/Runtime Helpers

| File | Purpose |
|------|---------|
| `audit/run_in_ros2_lyrical.ps1` | ROS2 Lyrical command wrapper (required for all ROS2 commands) |
| `audit/run_stage_minus1_audit.ps1` | Environment audit harness |
| `audit/check_environment.ps1` | Environment check script |
| `audit/check_python_runtime.py` | Python runtime probe |
| `audit/check_ros2_import.py` | ROS2 import probe |
| `audit/check_combined_import.py` | Combined import probe |
| `audit/check_lerobot_import.py` | LeRobot import probe |
| `audit/check_opencv_camera.py` | OpenCV/camera probe |
| `audit/open_ros2_lyrical_shell.cmd` | ROS2 shell opener |
| `audit/verify_ros2_lyrical_setup.ps1` | ROS2 setup verification |
| `audit/ros2_stage_minus1b_runtime_checks.py` | ROS2 runtime checks |
| `audit/scan_linux_dependencies.py` | Linux dependency scanner |

### A8. Tests

| File | Purpose |
|------|---------|
| `tests/__init__.py` | Test package init |
| `tests/protocol/test_protocol_contract.py` | Protocol contract test |
| `tests/integration/probe_mock_server.py` | Mock server probe |

### A9. Calibration & Model Data

| Path | Purpose |
|------|---------|
| `data/calibration/` | Camera intrinsics, workspace calibration, Charuco board |
| `data/robot_model/so101/` | Frozen URDF + STL CAD assets |
| `data/runtime/` | Runtime snapshots (pregrasp, integrated grasp) |

### A10. Verification Scripts (kept for regression)

All `scripts/verify_stage_*.py` files (approx. 40 scripts) — end-to-end verification scripts for each development stage. Retained as engineering evidence.

---

## Category B: DOCUMENTATION

| File | Purpose |
|------|---------|
| `README.md` | (TO BE CREATED) Root English README |
| `README_zh-CN.md` | (TO BE CREATED) Root Chinese README |
| `docs/MVP_SCOPE.md` | Project scope and frozen assets |
| `docs/MVP_ARCHITECTURE.md` | MVP architecture overview |
| `docs/MVP_TCP_PROTOCOL.md` | TCP JSON-lines protocol spec |
| `docs/MVP_KINEMATICS.md` | Kinematics documentation |
| `docs/MVP_TRAJECTORY.md` | Trajectory planning documentation |
| `docs/MVP_HARDWARE_TEST.md` | Hardware test documentation |
| `docs/LEGACY_COMPONENTS.md` | Legacy component inventory |
| `docs/MVP4E_TACTILE_GRASP_LIFT_MANUAL_ACCEPTANCE.md` | Final acceptance guide (original) |
| `docs/ARCHITECTURE.md` | (TO BE CREATED) Detailed architecture |
| `docs/FINAL_ACCEPTANCE.md` | (TO BE CREATED) Final acceptance guide |
| `docs/TROUBLESHOOTING.md` | (TO BE CREATED) Troubleshooting guide |
| `docs/VERIFICATION.md` | (TO BE CREATED) Verification summary |
| `shared_protocol/protocol_v1.md` | Protocol specification |
| `ros2_ws/src/so101_mvp_bringup/README.md` | Bringup package README |
| `ros2_ws/src/so101_mvp_control/README.md` | Control package README |
| `ros2_ws/src/so101_mvp_kinematics/README.md` | Kinematics package README |

### Development-Stage Documentation (Archive Reference)

| File | Stage |
|------|-------|
| `docs/stage_minus1_windows_audit.md` | Pre-MVP Windows audit |
| `docs/stage_minus1b_ros2_lyrical_setup.md` | ROS2 Lyrical setup |
| `docs/windows_environment_usage.md` | Windows environment usage |
| `docs/MVP3C_LIVE_MANUAL_ACCEPTANCE.md` | Read-only joint bridge |
| `docs/MVP3D_SINGLE_TCP_RETEST.md` | Single TCP retest |
| `docs/MVP3D_STOP_CONFIRM_RETEST.md` | Stop confirm retest |
| `docs/MVP3D_TCP_RETEST.md` | TCP retest |
| `docs/MVP4A_PREGRASP_MANUAL_ACCEPTANCE.md` | Pregrasp planner |
| `docs/MVP4A_IK_MULTISEED_RETEST.md` | IK multiseed retest |
| `docs/MVP4A_PREGRASP_NEAR_SOLUTION_RETEST.md` | Near-solution retest |
| `docs/MVP4B_SPEED_0P06_MANUAL_RETEST.md` | Speed retest |
| `docs/MVP4B_PREGRASP_MOVE_MANUAL_ACCEPTANCE.md` | Pregrasp move |
| `docs/MVP4C_SEGMENTED_DESCENT_MANUAL_ACCEPTANCE.md` | Segmented descent |
| `docs/MVP4C_OCCLUSION_SAFE_HANDOFF_MANUAL_ACCEPTANCE.md` | Occlusion handoff |
| `docs/MVP4D_INTEGRATED_VISUAL_GRASP_MANUAL_ACCEPTANCE.md` | Integrated grasp |

---

## Category C: VERIFICATION

### C1. Final Acceptance Evidence

| File | Purpose |
|------|---------|
| `data/verification/stage_mvp4e_final_simplification_report.json` | Final simplification freeze report |
| `data/verification/stage_mvp4e_tactile_grasp_lift_report.json` | Tactile grasp-lift verification (51/53 checks) |
| `data/verification/stage_mvp4e_x_axis_grasp_offset_report.json` | X-axis +20 mm offset verification |
| `data/verification/stage_mvp4e_close_until_tactile_contact_report.json` | Close-until-contact verification |
| `data/verification/stage_mvp4e_direct_com8_tactile_report.json` | Direct COM8 tactile verification |
| `data/verification/mvp4a_last_pregrasp_diagnostic.json` | Final pregrasp diagnostic |

### C2. Development-Stage Evidence

All `data/verification/stage_2a3_*` through `data/verification/stage_mvp4d_*` reports — complete development history. Retained as engineering evidence.

### C3. Generated Verification Logs

- `data/verification/stage_2a3_logs/` — FK/visualization smoke test logs
- `data/verification/stage_2d1_logs/` — Visual-to-IK pipeline logs
- `data/verification/ros_logs/` — Various ROS2 session logs
- `data/verification/ros_logs_mvp0/` — MVP-0 session logs
- `data/verification/mvp3c_live_runtime/` — Live read-only bridge runtime
- `data/verification/mvp2_trajectory/` — Trajectory plots and CSV

---

## Category D: LEGACY / DEPRECATED

| File | Status |
|------|--------|
| `scripts/launch_mvp4e_system.ps1` | DEPRECATED — complex one-launch supervisor |
| `scripts/run_mvp4e_bridge.ps1` | DEPRECATED — dedicated bridge runner |
| `scripts/so101_read_only_joint_state_server.py` | Legacy read-only joint state server |
| `scripts/legacy/mvp_tcp_probe.py` | Legacy TCP probe |
| `ros2_ws/src/so101_command_gate/` | Legacy command gate package |
| `ros2_ws/src/so101_trajectory_safety/` | Legacy trajectory safety package |
| `ros2_ws/src/so101_grasp_planner/` | Legacy grasp planner package |
| `ros2_ws/src/so101_kinematics/` | Legacy kinematics package |
| `ros2_ws/src/so101_robot_bridge/` | Legacy robot bridge package |
| `config/bridge.yaml` | Legacy bridge config (port 8765) |
| `config/command_gate.json` | Legacy command gate config |
| `config/connection_trajectory.json` | Legacy connection trajectory config |
| `config/real_joint_state_mapping.json` | Legacy joint state mapping |
| `config/trajectory_safety.json` | Legacy trajectory safety config |
| `shared_protocol/protocol_v1.md` | Legacy protocol v1 spec |
| `shared_protocol/message_envelope.schema.json` | Legacy message schema |
| `shared_protocol/examples/protocol_examples.jsonl` | Legacy protocol examples |
| `lerobot_server/mock_hardware_server.py` | Legacy mock server |

---

## Category E: GENERATED (to be cleaned)

| Path | Description |
|------|-------------|
| `__pycache__/` directories | Python bytecode cache |
| `*.pyc` files | Compiled Python files |
| `.pytest_cache/` | Pytest cache |
| `logs/runtime/` | Runtime launcher/bridge/server logs (~176 files) |
| `logs/ros2/` | ROS2 session logs |
| `audit_results/` | Environment audit outputs (~395 files, already gitignored) |
| `data/verification/__pycache__/` | Verification cache |
| `data/verification/ros_logs/` | ROS2 verification logs |
| `data/verification/ros_logs_mvp0/` | MVP-0 ROS2 logs |
| `data/verification/stage_2a3_logs/` | Stage 2A3 ROS logs |
| `data/verification/stage_2d1_logs/` | Stage 2D1 ROS logs |
| `data/verification/mvp3c_live_runtime/` | Live runtime logs |
| `data/verification/ros_logs_2d3b/` | Empty directory |
| `data/verification/ros_logs_2d3c/` | Empty directory |
| `data/verification/ros_logs_heartbeat_refresh/` | Empty directory |
| `data/verification/ros_logs_probe/` | Empty directory |
| `data/verification/mvp4e_bridge_spawn_fake_ws/` | Empty directory |
| `ros2_ws/build/` | Colcon build artifacts |
| `ros2_ws/install/` | Colcon install artifacts |
| `ros2_ws/log/` | Colcon build logs |

---

## Category F: UNKNOWN / UNCLASSIFIED

| File | Notes |
|------|-------|
| `start_claude_deepseek.bat` | Claude Code launcher via DeepSeek API proxy — developer tool |
| `.claude/settings.local.json` | Claude Code local settings |
| `data/camera_probe/*.png` | Camera probe captures — development evidence |
| `data/calibration/captures/` | Camera calibration captures |
| `data/calibration/results_session_*/` | Calibration results with visualizations |
| `audit_results/minimal_ros2_ws/` | Minimal ROS2 probe workspace (generated) |
| `audit_results/tactile_bridge_ws/` | Tactile bridge probe workspace (generated) |
