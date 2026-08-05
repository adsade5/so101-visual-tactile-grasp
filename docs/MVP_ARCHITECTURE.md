# MVP Architecture

## ROS2 Environment

### 1. `object_pose_node`

- Detects a single regular block.
- Publishes the object pose in workspace coordinates.
- Reuses the existing camera and ArUco/object marker configuration.

### 2. `workspace_to_base_node`

- Reuses the existing `workspace_plane` to `base_link` calibration.
- Converts the target position into `base_link`.
- Remains a simple transform component for MVP use.

### 3. `mvp_grasp_controller_node`

- Will call the reimplemented MVP FK/IK.
- Will generate pregrasp, descend, lift, and fixed-place joint targets.
- Will manage a simple grasp state machine.
- Will start only through an explicit service.
- Will not auto-execute.
- Stage MVP-0 skeleton prints hardware-disabled status only.

### 4. `mvp_hardware_bridge_node`

- Will connect to the LeRobot-side server over localhost TCP.
- Will request current joint state.
- Will send simple joint targets.
- Will send `stop`.
- Will not implement complex heartbeat negotiation.
- Stage MVP-0 skeleton does not connect to TCP or hardware.

## LeRobot Environment

### 5. `scripts/mvp_so101_server.py`

- Will read the real SO-101 follower.
- Will use the frozen follower calibration file.
- Will accept only limited commands.
- Will use fixed low speed.
- Will perform calibration range checks.
- Will support `stop`.
- Will not implement ROS2.
- Will not implement complex plan state.
- Stage MVP-0 `--dry-run` loads config and confirms no serial port or motor command is used.

## FlexiTac

### 6. Simple Contact Input

FlexiTac will later provide a simple non-blocking `contact=true/false` signal through existing reusable reading/contact-detection code. MVP does not require complex tactile visualization or high-rate tactile control loops.

## MVP Skeleton Launch

`ros2_ws/src/so101_mvp_bringup/launch/mvp_skeleton.launch.py` starts only:

- `mvp_grasp_controller_node`
- `mvp_hardware_bridge_node`

It does not start camera nodes, real hardware, legacy command gate, connection trajectory, mock joint state publisher, or shadow executor.

