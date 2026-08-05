# Legacy Components Outside the MVP Main Chain

The following source files and packages are retained for reference and regression history, but the new MVP launch files must not start them.

| Component | Original Purpose | Why It Is Outside MVP | Source Retained | MVP Replacement |
|---|---|---|---|---|
| `ros2_ws/src/so101_command_gate` | Command validation, connection trajectories, shadow execution, real joint-state bridge experiments | Too much lifecycle and safety machinery for one low-speed MVP grasp | Yes | `so101_mvp_control` skeleton, later `mvp_grasp_controller_node` and `mvp_hardware_bridge_node` |
| `so101_command_gate.command_gate_node` | Validate trajectory/current-state consistency before shadow candidate output | MVP does not use command gate tokens or multi-layer readiness | Yes | Simple explicit-start grasp controller |
| `so101_command_gate.connection_trajectory_node` | Generate connection trajectory from current real state to planned trajectory | MVP will use sequential low-speed joint targets | Yes | Simple joint target generation in MVP controller |
| `so101_command_gate.shadow_executor_node` | Preview/shadow execute gated trajectory without hardware motion | MVP does not use shadow execution | Yes | Dry-run logs and explicit hardware-disabled skeleton nodes |
| `so101_trajectory_safety.timed_trajectory_node` | Time-parameterize planned grasp trajectories and publish validity/status heartbeats | MVP does not require timed trajectory cache or high-reliability heartbeat | Yes | Fixed low-speed sequential motion config |
| `so101_command_gate.mock_joint_state_publisher` | Publish synthetic joint states for earlier validation | MVP launch does not need mock joint state flow | Yes | MVP launch starts only MVP controller/bridge skeleton nodes |
| `ros2_ws/src/so101_command_gate/launch/perception_to_connected_shadow_dry_run.launch.py` | Full perception-to-connection-to-shadow dry-run chain | Starts legacy command gate/shadow chain, outside MVP | Yes | `so101_mvp_bringup/launch/mvp_skeleton.launch.py` |
| Stage 2D-3A validation scripts | Command gate and shadow executor validation | Validates legacy complexity outside MVP scope | Yes | `stage_mvp0_report.json` skeleton validation |
| Stage 2D-3B validation scripts | Real read-only joint bridge and gate validation | MVP server/bridge protocol will be simpler and on port 8770 | Yes | `scripts/mvp_so101_server.py --dry-run` |
| Stage 2D-3C validation scripts | Connected shadow chain and heartbeat/plan refresh validation | MVP removes active/pending plans and multi-layer status heartbeats | Yes | MVP skeleton launch node-list check |
| `plan_id` and `trajectory_hash` pairing | Prevent stale trajectory/status mismatches in complex chain | MVP protocol intentionally excludes plan hashes | Yes | Limited JSON Lines commands |
| Active/pending plan switching | Manage complex plan lifecycle | MVP state machine will execute one explicit grasp at a time | Yes | Single simple grasp state machine |
| Multi-layer valid/status heartbeats | High-reliability ROS2 synchronization | MVP only needs basic command/response and timeout | Yes | Basic TCP request/reply and simple ROS service start |
| Connection trajectory safety envelope | Safety preview around current-to-plan connection | MVP does not implement advanced safety envelope | Yes | Calibration joint limits and fixed low speed |

No legacy source is deleted in Stage MVP-0.

