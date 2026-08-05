# MVP-3D-SINGLE-TCP-FIX人工重测

本阶段恢复最小单TCP架构：

- 1个TCP服务器；
- 1个ROS2桥接客户端；
- 1条持久TCP连接；
- 请求顺序处理；
- 正常命令只包含 `get_state` 和 `move_joints_sequential`。

不再提供 `/mvp/stop` 软件急停。异常处理顺序：

1. 测试脚本按Ctrl+C；
2. ROS2桥接按Ctrl+C；
3. 若机械臂仍有异常，直接关闭Follower舵机电源。

## 终端0：Zenoh

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "ros2 run rmw_zenoh_cpp rmw_zenohd"

## 终端1：LeRobot服务器

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& E:\Anaconda\Scripts\conda.exe run `
  --no-capture-output `
  -p E:\Anaconda\envs_dirs\lerobot `
  python scripts\mvp_so101_server.py `
  --config config\mvp_hardware.json `
  --enable-hardware-motion

服务器应显示：

TCP_LISTENING host=127.0.0.1 port=8770 single_client=true

正常运行期间应只看到：

TCP_CLIENT_CONNECTED id=1 address=...

不应出现 id=2、id=3、id=4，除非ROS2桥接断开并重新启动。

## 终端2：ROS2桥接

cd E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"

桥接应显示：

single_tcp_client=true
TCP_CONNECTED host=127.0.0.1 port=8770

## 终端3：检查与腕部测试

检查TCP状态：

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 topic echo /mvp/tcp_status --once"

检查关节状态：

& ".\audit\run_in_ros2_lyrical.ps1" `
  -Command "cd /d E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\ros2_ws && call install\local_setup.bat && ros2 topic echo /mvp/joint_states --once"

plan-only：

python scripts\mvp_ros2_wrist_test.py --plan-only

真实2°往返：

python scripts\mvp_ros2_wrist_test.py \
  --execute \
  --confirm ROS2_WRIST_ROLL_2DEG

用户确认口令是 `ROS2_WRIST_ROLL_2DEG`。

TCP内部协议确认口令仍是 `MVP_MOVE`，用户不需要输入它。

## TCP probe说明

`scripts\legacy\mvp_tcp_probe.py` 仅保留为遗留手动诊断工具。

启动ROS2桥接后不得再运行TCP probe，因为服务器只允许一个客户端。

## 关闭顺序

1. 关闭终端3中的检查或测试命令；
2. 终端2按Ctrl+C关闭ROS2桥接；
3. 终端1按Ctrl+C关闭LeRobot服务器；
4. 终端0按Ctrl+C关闭Zenoh；
5. 最后关闭Follower舵机电源。
