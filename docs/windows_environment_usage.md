# Windows ROS2/LeRobot 双环境使用说明

本机阶段 -1B 已验证采用双环境架构：

- ROS2 Lyrical：`C:\pixi_ws`
- LeRobot：`E:\Anaconda\envs_dirs\lerobot`
- 后续通信：`127.0.0.1 TCP`

不要混用两个环境。不要在 LeRobot 环境安装 `rclpy`，也不要在 ROS2 环境安装完整 LeRobot。

## ROS2 终端

打开已激活 ROS2 Lyrical 的 CMD：

```bat
E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\open_ros2_lyrical_shell.cmd
```

执行单条 ROS2 命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\run_in_ros2_lyrical.ps1 `
  -Command "ros2 --help"
```

该脚本默认使用：

- Pixi：`C:\Users\82053\.pixi\bin\pixi.exe`
- Workspace：`C:\pixi_ws`
- ROS setup：`C:\pixi_ws\ros2-windows\local_setup.bat`

脚本会拒绝混用非 `lyrical` 的 `ROS_DISTRO`。

## 构建 ROS2 workspace

在 ROS2 环境中构建：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\run_in_ros2_lyrical.ps1 `
  -Command "cd /d <workspace> && colcon build --merge-install --event-handlers console_direct+"
```

C++ 包在本机建议显式使用 Ninja 和 release 兼容构建类型：

```text
colcon build --merge-install --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

加载 overlay：

```bat
call install\local_setup.bat
```

## LeRobot 环境

只读激活/检查：

```powershell
E:\Anaconda\envs_dirs\lerobot\python.exe -c "import sys, lerobot, cv2, serial; print(sys.executable); print(sys.version); print(lerobot.__file__); print(cv2.__version__); print(serial.VERSION)"
```

阶段 -1B 未在该环境安装任何 ROS2 包，未导入 `rclpy` 作为成功条件。

## 常见错误

- 忘记进入 Pixi 环境：使用 `run_in_ros2_lyrical.ps1` 或 `open_ros2_lyrical_shell.cmd`。
- 忘记调用 `local_setup.bat`：ROS2 CLI 可见但 overlay 包不可见。
- Conda 污染 PATH：ROS2 命令应使用 `C:\pixi_ws\.pixi\envs\default\python.exe`，不是 Anaconda Python。
- `ROS_DISTRO` 混用：脚本检测到非 `lyrical` 会拒绝运行。
- 普通终端找不到 `cl`：使用脚本，它会调用 VS Build Tools 环境。
- CMake 不认识 `Visual Studio 18 2026`：C++ workspace 使用 `-G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo`。
- workspace 未加载：先 `call install\local_setup.bat`，再 `ros2 run` 或 `ros2 topic echo`。

## 安全验证命令

完整验证：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\verify_ros2_lyrical_setup.ps1
```

跳过官方 demo：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\verify_ros2_lyrical_setup.ps1 `
  -SkipOfficialDemo
```

跳过构建：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp\audit\verify_ros2_lyrical_setup.ps1 `
  -SkipBuild
```

验证脚本不访问硬件、不访问摄像头、不打开 COM 端口。
