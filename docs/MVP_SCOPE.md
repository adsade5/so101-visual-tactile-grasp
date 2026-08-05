# SO-101 Visual Tactile Grasp MVP Scope

## Project Goal

基于SO-101、顶视相机和FlexiTac，完成单一规则物块的视觉定位、低速抓取、触觉确认、抬升和固定点放置。

本项目的新定位介于课程设计与硕士毕业设计之间。MVP主链路追求能够实际跑通一次规则物块抓取与放置，而不是研究级或工业级控制系统。

## Assets Kept Frozen

- Frozen SO-101 URDF: `data/robot_model/so101/so101_new_calib.urdf`
- LeRobot follower calibration: `C:/Users/82053/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json`
- Workspace-to-base calibration: `config/workspace_to_base.json`
- Camera calibration: `config/camera.yaml`, `data/calibration/camera_intrinsics.yaml`
- ArUco/object marker configuration: `config/object_marker.json`
- FlexiTac reusable reading/contact-detection code from the audited legacy tactile project
- ROS2 and LeRobot runtime separation
- Localhost TCP communication pattern

## MVP Allows

- Python numeric IK
- Fixed downward end-effector orientation
- Sequential joint movement
- Fixed low speed
- Fixed place position
- Single object category
- Basic timeout handling
- Manual emergency stop
- Stopping by cutting servo power

## MVP Does Not Require

- Smooth optimal trajectory generation
- Multiple object types
- Obstacle avoidance
- Self-collision detection
- MoveIt
- Online Cartesian control
- Complex state recovery
- Multi-layer heartbeat
- `trajectory_hash`
- Command gate
- Shadow executor
- Industrial emergency stop
- Multiple automatic retries

## Explicit Non-Goals

The MVP does not pursue research-grade trajectory safety, industrial reliability, multi-layer fault tolerance, real-time collision checking, a complete controller framework, or high-concurrency ROS2 state synchronization.

