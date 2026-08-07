# MVP-3C-LIVE人工只读验收

本阶段不使用自动runner。

用户手动打开四个独立PowerShell终端。

## 终端0：Zenoh Router

作用：

提供ROS2发现和通信。

命令：

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"

保持运行。

## 终端1：LeRobot真实只读TCP服务器

作用：

打开COM4，只读取真实SO-101关节状态，并通过127.0.0.1:8770响应get_state。

命令：

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& E:\Anaconda\Scripts\conda.exe run `
  --no-capture-output `
  -p E:\Anaconda\envs_dirs\lerobot `
  python scripts\mvp_so101_server.py `
  --config config\mvp_hardware.json `
  --read-only

禁止添加：

--enable-hardware-motion

保持运行。

## 终端2：ROS2只读桥接

作用：

通过TCP读取五个机械臂关节，并发布ROS2话题。

命令：

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_read_only.launch.py"

保持运行。

## 终端3：检查节点

命令：

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 node list"

应看到：

/mvp_hardware_bridge_node

## 检查真实五关节状态

命令：

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 topic echo /mvp/joint_states --once"

应包含：

shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll

position长度应为5，并且均为有限弧度值。

## 检查发布频率

命令：

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 topic hz /mvp/joint_states"

观察约10秒后按Ctrl+C。

期望频率：

4～6 Hz。

## 检查夹爪状态

命令：

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 topic echo /mvp/gripper_state --once"

应返回0～100之间的有限数值。

## 检查只读模式拒绝运动

运行：

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp && python scripts\verify_stage_mvp3c_live_read_only.py"

该脚本应在30秒以内结束。

预期包含：

joint_state_received=true
gripper_state_received=true
execute_service_success=false
execute_service_message=hardware_motion_disabled

## 现场人工确认

用户确认：

1. 测试期间机械臂完全没有运动；
2. 没有夹爪动作；
3. 没有异常声音；
4. 关闭服务器时机械臂没有突然下落。

## 关闭顺序

1. 关闭终端3中的检查命令；
2. 终端2按Ctrl+C关闭ROS2桥接；
3. 终端1按Ctrl+C关闭只读TCP服务器；
4. 终端0按Ctrl+C关闭Zenoh Router；
5. 最后关闭Follower舵机电源。
