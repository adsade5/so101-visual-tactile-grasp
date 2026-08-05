# MVP-4A Pregrasp Preview Manual Acceptance

Stage MVP-4A validates this no-motion chain:

`object_pose_node -> workspace_to_base_node -> mvp_pregrasp_planner_node -> /mvp/pregrasp_pose -> /mvp/pregrasp_joint_target`

Safety boundary:

- Do not connect COM4.
- Do not start `scripts/mvp_so101_server.py`.
- Do not start `mvp_hardware_bridge_node`.
- Do not publish robot command targets.
- Keep Follower power off if desired.

## Terminal 0: Zenoh Router

Use the same Zenoh router command already used for the camera ROS2 environment:

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp
call ros2_ws\install\local_setup.bat
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Terminal 1: Camera And Object Pose

This launch starts the existing camera-backed object pose node. It publishes `/object_pose`, `/object_detected`, and `/object_pose_stable`.

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 run so101_object_perception object_pose_node --ros-args -p project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp -p show_debug_window:=false
```

## Terminal 2: Transform And Pregrasp Planner

This starts only the existing workspace-to-base transform and the new MVP pregrasp preview planner because Terminal 1 already owns the camera node.

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp show_debug_window:=false start_object_pose_node:=false
```

The same launch can also start all three preview nodes in one terminal if the camera node is not already running:

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py project_root:=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_visual_tactile_grasp show_debug_window:=false
```

## Terminal 3: Preview Check

Call the snapshot planner once and print the frozen target.

```bat
cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws
call install\local_setup.bat
python ..\scripts\mvp_pregrasp_preview.py --once
```

Optional topic checks:

```bat
ros2 topic echo /object_pose --once
ros2 topic echo /object_pose_base --once
ros2 topic echo /mvp/pregrasp_pose --once
ros2 topic echo /mvp/pregrasp_joint_target --once
ros2 topic echo /mvp/pregrasp_valid --once
ros2 topic echo /mvp/pregrasp_status --once
```

## PASS Criteria

- `/object_pose` is a `geometry_msgs/msg/PoseStamped` in `workspace_plane`.
- `/object_pose_base` is a `geometry_msgs/msg/PoseStamped` in `base_link`.
- `/mvp/compute_pregrasp` returns `success=true`.
- `/mvp/pregrasp_pose` is in `base_link`.
- Pregrasp X/Y match object base X/Y.
- Pregrasp Z is object base Z + `0.08` m.
- `/mvp/pregrasp_joint_target` contains exactly `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`.
- All five joint values are finite and inside URDF limits.
- FK validation reports position error <= `0.002` m and approach error <= `5.0` deg.
- `/mvp/pregrasp_valid` is `true`.
- `/mvp/pregrasp_status` is `pregrasp_ready`.
- No command topic is used.
- `/mvp/execute_target` is not called.
- COM4 is not opened.
- The robot does not move.

## Shutdown Order

1. Stop Terminal 3 preview script.
2. Stop Terminal 2 transform/planner launch or nodes.
3. Stop Terminal 1 camera/object pose node.
4. Stop Terminal 0 Zenoh router.
