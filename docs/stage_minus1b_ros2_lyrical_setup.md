# 阶段 -1B ROS 2 Lyrical Windows 环境部署与验证报告

执行时间：2026-08-03  
工作目录：`E:\PycharmProjects\Embodied_AI\LeRobot_Project`  
ROS2 环境：`C:\pixi_ws`  
安全声明：本阶段没有打开任何 COM 端口，没有连接机器人或触觉设备，没有启动相机/视觉窗口，没有发送任何机械臂控制指令，没有修改现有 Conda 环境。

官方来源：

- ROS 2 release：<https://github.com/ros2/ros2/releases/tag/release-lyrical-20260623>
- Windows AMD64 asset：<https://github.com/ros2/ros2/releases/download/release-lyrical-20260623/ros2-lyrical-2026-06-23-windows-AMD64.zip.zip>
- ROS2 Lyrical pixi.toml：<https://raw.githubusercontent.com/ros2/ros2/refs/heads/lyrical/pixi.toml>
- ROS 2 Lyrical docs：<https://docs.ros.org/en/lyrical/>
- Pixi：<https://pixi.sh/>

## 1. 执行摘要

| 项目 | 结果 | 证据 | 是否阻塞 |
| --- | --- | --- | --- |
| Windows 11 amd64 | 通过 | CIM: Windows 11 家庭版 中文版, build 26200, 64-bit | 否 |
| Pixi | 通过 | `C:\Users\82053\.pixi\bin\pixi.exe`, `pixi 0.75.0` | 否 |
| ROS2 Lyrical 安装 | 通过 | `release-lyrical-20260623`, `C:\pixi_ws\ros2-windows\local_setup.bat` | 否 |
| ROS2 CLI | 通过 | `C:\pixi_ws\ros2-windows\Scripts\ros2.exe` | 否 |
| rclpy | 通过 | `C:\pixi_ws\ros2-windows\Lib\site-packages\rclpy\__init__.py` | 否 |
| rclcpp 官方 demo | 通过 | C++ talker 发布 12 条 | 否 |
| Python 官方 demo | 通过 | Python listener 接收 30 条 | 否 |
| colcon | 通过 | `C:\pixi_ws\.pixi\envs\default\Scripts\colcon.exe`, `colcon --help` exit 0 | 否 |
| MSVC 工具链 | 通过 | MSVC 19.50.35728.0, `cl.exe` from VS 2026 Build Tools | 否 |
| Python 探针 | 通过 | `/stage_minus1_python_probe`: `stage_minus1_python_probe_ok` | 否 |
| C++ 探针 | 通过 | `/stage_minus1_cpp_probe`: `stage_minus1_cpp_probe_ok` | 否 |
| 旧触觉包构建 | 通过 | 4 个入口脚本/exe 均由 `ros2 pkg executables` 列出 | 否 |
| LeRobot 环境回归 | 通过 | `lerobot`, `cv2`, `serial` 只读导入成功 | 否 |
| 双环境架构 | 方案B | ROS2 与 LeRobot 环境均独立通过验证 | 否 |

结论：阶段 -1B 通过。最终架构确定为方案 B：ROS2 Lyrical 独立环境 + LeRobot Python 3.12 独立环境 + 后续 localhost TCP Bridge。

## 2. ROS2 release 信息

- release tag：`release-lyrical-20260623`
- release 名称：`ROS Lyrical Luth - Patch Release 1 (2026/06/23)`
- 发布时间：`2026-06-23T20:32:22Z`
- Windows asset：`ros2-lyrical-2026-06-23-windows-AMD64.zip.zip`
- asset 大小：`551722068` bytes
- 本地 SHA-256：`E6171A09BA198D8DA17AD05283ED78259F193382F54D6CA10521EF08DD29419D`
- 官方 checksum：该 GitHub release 资产列表未提供 checksum 文件；本报告仅记录本地 SHA-256，未声称完成官方 checksum 校验。
- 安装路径：`C:\pixi_ws\ros2-windows`
- Pixi 配置来源：`ros2/ros2` 官方 `lyrical` 分支 `pixi.toml`
- Pixi 配置 SHA-256：`A1016C44EB266C7F5EDC32651342F8932B8FEFCD4B4F06650DEE4D8F1F9664A8`

## 3. 实际环境路径

- ROS2 Python：`C:\pixi_ws\.pixi\envs\default\python.exe`, Python 3.12.3
- LeRobot Python：`E:\Anaconda\envs_dirs\lerobot\python.exe`, Python 3.12.13
- Pixi：`C:\Users\82053\.pixi\bin\pixi.exe`, version 0.75.0
- `ros2`：`C:\pixi_ws\ros2-windows\Scripts\ros2.exe`
- `rclpy`：`C:\pixi_ws\ros2-windows\Lib\site-packages\rclpy\__init__.py`
- `colcon`：`C:\pixi_ws\.pixi\envs\default\Scripts\colcon.exe`
- `cmake`：`C:\pixi_ws\.pixi\envs\default\Library\bin\cmake.exe`, version 3.28.3
- `ninja`：`E:\Cubeclt\STM32CubeCLT_1.21.0\Ninja\bin\ninja.exe`, version 1.11.1
- `cl`：`C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Tools\MSVC\14.50.35717\bin\Hostx64\x64\cl.exe`, MSVC 19.50.35728.0
- `nmake`：`C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Tools\MSVC\14.50.35717\bin\Hostx64\x64\nmake.exe`
- ROS setup：`C:\pixi_ws\ros2-windows\local_setup.bat`
- RMW：`rmw_fastrtps_cpp`

Windows 交叉验证：

- `[System.Environment]::OSVersion.Version`：10.0.26200.0
- `Get-CimInstance`：`Microsoft Windows 11 家庭版 中文版`, `Version=10.0.26200`, `BuildNumber=26200`, `OSArchitecture=64-bit`
- `cmd /c ver`：`Microsoft Windows [Version 10.0.26200.8875]`
- registry `ProductName`：`Windows 10 Home China`
- registry `DisplayVersion`：`25H2`
- registry `CurrentBuild`：`26200`
- registry `UBR`：`0x22ab`

说明：CIM 和 build 明确指向 Windows 11 64-bit；registry `ProductName` 仍显示 Windows 10，这是 Windows 系统信息接口不一致，未作为阻塞。

## 4. 官方 demo 结果

验证脚本：`so101_visual_tactile_grasp\audit\verify_ros2_lyrical_setup.ps1`  
ROS_DOMAIN_ID：`88`

日志摘录：

```text
[talker]: Publishing: 'Hello World: 5'
[talker]: Publishing: 'Hello World: 12'
[listener]: I heard: [Hello World: 5]
[listener]: I heard: [Hello World: 12]
```

统计结果：talker 发布 12 条，listener 接收 30 条。判定通过。

## 5. 自建探针结果

workspace：`so101_visual_tactile_grasp\audit_results\minimal_ros2_ws`  
构建命令：

```text
colcon build --merge-install --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Python 探针：

- 包类型：`ament_python`
- topic：`/stage_minus1_python_probe`
- echo 结果：`stage_minus1_python_probe_ok`
- 入口安装：`install\lib\stage_minus1_python_probe\python_probe_node.exe`

C++ 探针：

- 包类型：`ament_cmake`
- 编译器：MSVC 19.50.35728.0
- CMake generator：`Ninja`
- build type：`RelWithDebInfo`
- topic：`/stage_minus1_cpp_probe`
- echo 结果：`stage_minus1_cpp_probe_ok`
- 入口安装：`install\lib\stage_minus1_cpp_probe\cpp_probe_node.exe`

修复记录：阶段 -1 创建的探针需要补 `setup.cfg`、修 Python 缩进、C++ 改用 Lyrical 导出的 CMake imported targets，并使用 `RelWithDebInfo`，否则 Windows 上 C++ 节点会卡在 `rclcpp::Node` 构造阶段。

## 6. 旧触觉包结果

隔离 workspace：`so101_visual_tactile_grasp\audit_results\tactile_bridge_ws`  
复制来源：`so101_ros2_tactile_guard\ros2\so101_flexitac_bridge`  
构建命令：

```text
colcon build --merge-install --event-handlers console_direct+
```

`ros2 pkg executables so101_flexitac_bridge` 结果包含：

- `leflexitac_udp_bridge-script.py` / `leflexitac_udp_bridge.exe`
- `tactile_processor-script.py` / `tactile_processor.exe`
- `tactile_contact_detector-script.py` / `tactile_contact_detector.exe`
- `tactile_visualizer-script.py` / `tactile_visualizer.exe`

本阶段仅启动 `tactile_contact_detector` 约 5 秒，确认节点可启动且无 Linux-only 导入错误。未启动 `leflexitac_udp_bridge`，未打开 UDP 硬件读取链路，未启动 `tactile_visualizer` GUI，未启动 `SOTactileFollower`。

## 7. 环境隔离结果

ROS2 环境：

```text
C:\pixi_ws
```

职责：ROS2 CLI、`rclpy`、`rclcpp`、`colcon`、视觉、运动学、轨迹生成、状态机、C++ 轨迹安全节点、触觉 ROS2 bridge、后续 TCP client。

LeRobot 环境：

```text
E:\Anaconda\envs_dirs\lerobot
```

职责：SO-101 硬件连接、关节状态读取、安全轨迹命令执行、后续 TCP hardware server。

边界：

```text
ROS2 环境不得激活 Conda
LeRobot 环境不得加载 ROS2
LeRobot 环境不得安装 rclpy
ROS2 环境不得安装完整 LeRobot
后续环境间通信使用 127.0.0.1 TCP
```

LeRobot 回归结果：

```text
E:\Anaconda\envs_dirs\lerobot\python.exe
Python 3.12.13
LeRobot: E:\PycharmProjects\Embodied_AI\LeRobot_Project\repos\lerobot\src\lerobot\__init__.py
OpenCV: 4.13.0
pyserial: 3.5
```

## 8. 阻塞项

必须解决：无。

推荐解决：

- VS Build Tools 在 `vswhere` 中显示 `isComplete=false`, `isLaunchable=false`，但本阶段 `cl.exe` 实测可用。后续如需更复杂 C++ 包，建议用 Microsoft 官方 Visual Studio Installer 补齐 Desktop development with C++ workload、Windows SDK、x64/x86 MSVC tools。
- `ros2 doctor --report` 显示若干包 local 版本低于 rosdistro latest；当前 ROS2 release 二进制仍可用，未阻塞本阶段。
- `local_setup.bat` 输出 RTI Connext DDS environment script 缺失警告；默认 RMW 为 `rmw_fastrtps_cpp`，官方 demo 和探针通信均通过。

可延期：

- 旧触觉包 GUI/headless 模式。
- UDP bridge 的 Windows 防火墙/端口占用提示。
- 后续 TCP Bridge 具体协议实现。

## 9. 最终结论

阶段 -1B 通过。

最终架构确定为方案 B：

```text
ROS2 Lyrical 独立环境 + LeRobot Python 3.12 独立环境 + 后续 localhost TCP Bridge
```

本阶段未实现 TCP Bridge、ROS2 robot client、LeRobot hardware server、视觉节点、相机标定、检测、FK/Jacobian/IK、轨迹控制、抓取状态机、真机控制、GUI、MoveIt 或 Sim2Real。
