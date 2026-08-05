# SO-101 视觉-触觉闭环抓取与 ROS2 约束轨迹控制系统

阶段 -1 Windows 原生开发环境、旧触觉项目兼容性和 ROS2/LeRobot 集成架构预检报告

审计时间：2026-08-03  
工作目录：`E:\PycharmProjects\Embodied_AI\LeRobot_Project`  
安全声明：本次审计没有连接真实机器人或触觉设备，没有打开任何 COM 端口，没有发送任何机械臂控制指令。相机仅做短暂 OpenCV 只读探测。

## 1. 执行摘要

| 项目 | 结果 | 证据 | 是否阻塞 |
| --- | --- | --- | --- |
| 旧仓库获取 | 通过 | 克隆 `https://github.com/adsade5/so101-ros2-tactile-guard.git` 到 `so101_ros2_tactile_guard` | 否 |
| ROS2 CLI | 失败 | `where.exe ros2` 未找到，`ros2` 不是当前终端命令 | 是 |
| ROS2 Python | 失败 | 默认 Python 与 LeRobot Conda 环境均 `ModuleNotFoundError: rclpy` | 是 |
| ROS2 C++ | 失败 | `colcon` 与 MSVC `cl` 当前不可用；未能构建 `rclcpp` 探针 | 是 |
| OpenCV | 部分通过 | `lerobot` 环境可导入 OpenCV 4.13.0；默认 Python 不可导入 | 否 |
| ArUco | 通过 | `E:\Anaconda\envs_dirs\lerobot\python.exe` 中 `hasattr(cv2, "aruco") == True` | 否 |
| LeRobot | 通过 | `lerobot` 环境 LeRobot 0.5.2；`leflexitac` 环境 LeRobot 0.5.1 | 否 |
| pyserial | 通过 | LeRobot 环境 pyserial 3.5，可只读枚举 COM5/COM6 | 否 |
| ROS2+LeRobot 共存 | 失败 | 所有候选 LeRobot/SO-101 环境均缺 `rclpy` | 是 |
| FlexiTac Windows 可迁移性 | B | 算法和 UDP 多为 A/B；硬件耦合 follower 类为 C；Linux shell 脚本为 D | 否 |

最终建议：方案 C。当前 Windows 环境不足，不能证明 ROS2 Python、ROS2 C++、ROS2/LeRobot 同环境运行。

## 2. 当前系统信息

- Windows：`Get-ComputerInfo` 报告 `Windows 10 Home China`, `WindowsVersion=2009`；Python `platform` 报告 `Windows-11-10.0.26200-SP0`。两者不完全一致，需以系统设置或 `winver` 手动复核。
- PowerShell：5.1.26100.8875。
- Git：2.54.0.windows.1；`core.autocrlf` 未返回值。旧仓库在 Codex 沙箱用户下触发 dubious ownership，审计时使用单次 `git -c safe.directory=...` 读取，没有修改全局 Git 配置。
- Python 默认解释器：`C:\Python314\python.exe`, Python 3.14.3。该解释器只有 NumPy 2.4.4，缺 `cv2`, `serial`, `rclpy`, `std_msgs`, `lerobot`。
- Conda：24.9.2；相关环境包括 `lerobot`, `LeRobot_HIL`, `leflexitac`, `so101_nexus_pnp_bcppo_repro`, `so101_nexus_official_0412` 等。
- ROS2：当前 PATH 未检测到 `ros2`；常见环境变量 `ROS/AMENT/COLCON/RMW` 无可用证据；`ros2 doctor --report` 未执行成功。
- CMake：`E:\Cubeclt\STM32CubeCLT_1.21.0\CMake\bin\cmake.exe`, 3.28.1。
- colcon：未找到。
- MSVC `cl`：当前终端未找到；`vswhere.exe` 在标准 Visual Studio Installer 路径未找到。
- Ninja：`E:\Cubeclt\STM32CubeCLT_1.21.0\Ninja\bin\ninja.exe`。
- OpenCV：`lerobot` Conda 环境可导入 4.13.0，支持 DirectShow、MSMF、Win32 GUI、`cv2.aruco`。
- LeRobot：`lerobot` 环境为 0.5.2 editable at `E:\PycharmProjects\Embodied_AI\LeRobot_Project\repos\lerobot`；`leflexitac` 环境为 0.5.1 at `E:\PycharmProjects\Embodied_AI\LeRobot_Project\lerobot_tactile`。

## 3. 两个目录的状态

`so101_ros2_tactile_guard`

- 路径：`E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_ros2_tactile_guard`
- 是否为空：否，已克隆旧项目。
- 是否为 Git 仓库：是。
- remote：`origin https://github.com/adsade5/so101-ros2-tactile-guard.git`
- branch：`main`
- commit：`22aa24f5a59b8f6802015cdbdea0dff5094cbd51`，`Initial release: ROS 2 tactile guard for SO-101`
- 未提交文件：无，`## main...origin/main`
- 嵌套仓库：只发现顶层 `.git`，未发现嵌套 `.git`。
- 结构摘要：`ros2/so101_flexitac_bridge/package.xml`, `setup.py`, `launch/tactile_system.launch.py`; `lerobot_extension/src/lerobot/robots/so_tactile_follower/*`; `scripts/run_ros2.sh`; `README.md`, `NOTICE.md`, `LICENSE`。

`so101_visual_tactile_grasp`

- 路径：`E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp`
- 初始状态：空目录，非 Git 仓库。
- 当前状态：仍非 Git 仓库；本阶段只新增审计脚本、报告和审计结果，没有执行 `git init`。
- 新增结构：`audit/`, `docs/`, `audit_results/`。

## 4. 旧项目 ROS2 架构

ROS2 包：

- 包名：`so101_flexitac_bridge`
- 构建类型：`ament_python`
- package dependencies：`rclpy`, `std_msgs`
- exec dependencies：`python3-numpy`, `python3-opencv`, `launch`, `launch_ros`
- 自定义消息：未发现。
- C++ 包：未发现，旧项目没有 `CMakeLists.txt`。

节点和入口：

- `leflexitac_udp_bridge = so101_flexitac_bridge.leflexitac_udp_bridge:main`
- `tactile_processor = so101_flexitac_bridge.tactile_processor:main`
- `tactile_contact_detector = so101_flexitac_bridge.tactile_contact_detector:main`
- `tactile_visualizer = so101_flexitac_bridge.tactile_visualizer:main`

Launch 文件：

- `ros2/so101_flexitac_bridge/launch/tactile_system.launch.py`
- 启动上述 4 个节点。

Topics：

- `/tactile/raw`：`Float32MultiArray`
- `/tactile/features`：`Float32MultiArray`
- `/tactile/contact_state`：`Bool`
- `/tactile/contact_score`：`Float32`
- `/tactile/contact_detected`：`Bool`

Services / actions：

- 未发现 ROS2 service/action。

Parameters：

- UDP bridge：`bind_host=127.0.0.1`, `bind_port=5005`, `sensor_name=primary`, `guard_command_host=127.0.0.1`, `guard_command_port=5006`
- Processor：`contact_threshold=30.0`, `min_contact_taxels=3`
- Contact detector：`enabled`, `rows=12`, `cols=32`, `taxel_floor=0.25`, `top_k=5`, `contact_on_score=0.80`, `contact_off_score=0.20`, `confirm_frames=3`, `release_frames=3`
- Visualizer：`rows=12`, `cols=32`, `display_min`, `display_max`, `gamma`, `window_scale`, `refresh_hz`

## 5. FlexiTac 可复用模块

| 文件 | 类或函数 | 当前职责 | Windows 分类 | 需要修改 | 建议 |
| --- | --- | --- | --- | --- | --- |
| `ros2/.../leflexitac_udp_bridge.py` | `LeFlexiTacUDPBridge`, `process_packet`, `poll_socket`, `guard_callback` | 解析 FTAC UDP 包，发布 `/tactile/raw`，把 `/tactile/contact_detected` 发回 UDP 5006 | B | 构造时绑定 socket，需加强端口占用、生命周期、Windows 防火墙提示 | 可复用，适合作 ROS2 侧 bridge |
| `ros2/.../tactile_contact_detector.py` | `_calculate_contact_score`, `_update_no_contact_state`, `_update_contact_state` | top-k 接触分数、滞回阈值、连续帧状态机 | A | 主要是参数调优和测试 | 建议直接复用 |
| `ros2/.../tactile_processor.py` | `tactile_callback` | 阈值面积、峰值/均值、接触中心特征 | A | 非核心闭环可选 | 可复制为调试/特征节点 |
| `ros2/.../tactile_visualizer.py` | `TactileVisualizer`, `render_latest_frame` | OpenCV GUI 热力图 | B | `cv2.namedWindow/imshow` 需做可选 headless 模式 | 只作为可视化工具复用 |
| `lerobot_extension/.../tactile_udp_sender.py` | `TactileUDPSender`, `from_environment`, `send` | 从 LeRobot observation loop 发 FTAC UDP | A | 保持默认关闭，明确环境变量 | LeRobot 侧可复用 |
| `lerobot_extension/.../tactile_guard.py` | `TactileGuardReceiver`, `poll`, `_parse_packet` | 非阻塞接收 GRIP guard 状态 | B | 构造时绑定 UDP 5006，需处理端口冲突 | LeRobot 侧可复用 |
| `lerobot_extension/.../config_so_tactile_follower.py` | `TactileGuardConfig`, `SOTactileFollowerConfig` | 配置 dataclass | B | `/dev/ttyUSB*` 示例改为 Windows COM 示例；确认 LeRobot 包布局 | 作为参考迁移 |
| `lerobot_extension/.../so_tactile_follower.py` | `SOTactileFollower`, `_apply_tactile_guard`, `send_action` | 扩展 SOFollower，读触觉，UDP 发送，动作限幅 | C | 构造可能初始化 `TactileSensor` 并 `start_continuous_read()`；硬件边界需重构 | 不建议直接复制为 Windows 主进程 |
| `scripts/run_ros2.sh` | shell script | Linux ROS2 launch | D | Bash 与 `/opt/ros/humble/setup.bash` Windows 不适用 | 不迁移 |

## 6. Linux 专用问题

必须修改：

- `scripts/run_ros2.sh` 使用 `#!/usr/bin/env bash`, `source /opt/ros/humble/setup.bash`, `install/setup.bash`，Windows PowerShell 下不可直接运行。
- README 构建/运行说明使用 Linux 路径：`~/so101_flexitac_ros2_ws`, `/opt/ros/humble/setup.bash`。

建议修改：

- `config_so_tactile_follower.py` 注释中示例端口为 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`，Windows 文档应改为 `COMx`。
- `tactile_visualizer.py` GUI 创建应可选，避免无 GUI/远程终端失败。
- UDP bridge 和 guard receiver 应在端口占用时给出明确错误，并支持参数改端口。

不影响 Windows：

- 核心 UDP 使用 Python `socket.AF_INET/SOCK_DGRAM`、`setblocking(False)`，在 Windows 可用。
- 触觉包解析使用 `struct` 和 NumPy，跨平台。
- ROS2 package 使用 `ament_python` 和 Python 源文件，不包含 Linux-only C++ 构建逻辑。

无法判断：

- `lerobot.sensors.tactile_sensor.TactileSensor` 在 Windows 上打开串口后的行为未测试，按安全限制未打开 COM 端口。
- LeRobot `robstride` 与 `phone` 模块在 `leflexitac` 环境的遍历导入有异常，未进一步构造硬件对象。

## 7. 摄像头检测结果

执行环境：`E:\Anaconda\envs_dirs\lerobot\python.exe`

- OpenCV：4.13.0
- ArUco：可用
- DirectShow：构建支持
- MSMF：构建支持
- GUI：Win32 UI 可用
- 检测索引：优先 1，然后 0, 2, 3, 4, 5
- 检测后端：DEFAULT, DSHOW, MSMF
- 成功索引：无
- 成功后端：无
- 保存测试图片：无，因为没有成功读取帧
- 稳定性结论：当前没有可打开摄像头，无法比较 DirectShow 与 MSMF 稳定性
- 日志现象：OpenCV 报 `Camera index out of range`，DSHOW/MSMF 后端可用但不能通过这些索引采集

## 8. ROS2 最小测试结果

已创建临时 workspace：

- `audit_results/minimal_ros2_ws/src/stage_minus1_python_probe`
- `audit_results/minimal_ros2_ws/src/stage_minus1_cpp_probe`

Python 探针设计：

- 节点：`python_probe_node`
- 发布 topic：`/stage_minus1_python_probe`
- 内容：`stage_minus1_python_probe_ok`
- 频率：1 Hz
- 硬件访问：无

C++ 探针设计：

- 节点：`cpp_probe_node`
- 发布 topic：`/stage_minus1_cpp_probe`
- 内容：`stage_minus1_cpp_probe_ok`
- 频率：1 Hz
- 硬件访问：无

实际执行：

- `colcon build`：未执行，阻塞原因为 `ros2` 或 `colcon` 当前不可用。
- Python topic 验证：未执行，构建未完成。
- C++ topic 验证：未执行，构建未完成。
- MSVC/rclcpp 链接：未验证，当前终端未找到 `cl`，且没有 ROS2 C++ 环境。

## 9. ROS2 与 LeRobot 兼容性矩阵

| 环境 | Python | rclpy | LeRobot | OpenCV | pyserial | 综合结论 |
| --- | ---: | --- | --- | --- | --- | --- |
| 默认 Python | 3.14.3 | 失败 | 失败 | 失败 | 失败 | 不可用于本项目 |
| `E:\Anaconda\envs_dirs\lerobot` | 3.12.13 | 失败 | 通过，0.5.2 | 通过，4.13.0 | 通过，3.5 | LeRobot 侧可用，但缺 ROS2 |
| `E:\Anaconda\envs_dirs\LeRobot_HIL` | 3.12.13 | 失败 | 通过，0.5.2 | 通过，4.13.0 | 通过，3.5 | LeRobot 侧可用，但缺 ROS2 |
| `E:\Anaconda\envs_dirs\leflexitac` | 3.12.13 | 失败 | 通过，0.5.1 | 通过，4.13.0 | 通过，3.5 | FlexiTac/LeRobot 侧可用，但缺 ROS2 |
| `so101_nexus_pnp_bcppo_repro` | 3.12.13 | 失败 | 失败 | 通过，4.13.0 | 通过，3.5 | 不是当前 LeRobot 集成环境 |
| `so101_nexus_official_0412` | 3.12.13 | 失败 | 失败 | 通过，4.13.0 | 通过，3.5 | 不是当前 LeRobot 集成环境 |

结论：没有任何已测 Python 环境同时导入 `rclpy`, `lerobot`, `cv2`, `serial`。同环境直接集成不成立。

## 10. 推荐架构

最终建议：方案 C

理由：

- 当前 Windows 终端没有 ROS2 CLI。
- 当前 Windows 终端没有 `colcon`。
- 当前 Windows 终端没有 MSVC `cl`。
- 默认 Python 无 `rclpy`、LeRobot、OpenCV、pyserial。
- 已识别的 LeRobot/SO-101 Conda 环境可运行 LeRobot/OpenCV/pyserial，但都缺 `rclpy`。
- ROS2 Python/C++ 最小包没有实际构建和 topic 验证结果，不能作为通过。

阶段 -1 修复后建议的目标架构倾向：方案 B，双环境 Socket Bridge。

推荐接口设计：

- 传输：localhost TCP 优先于 UDP，用于机器人命令/状态同步；触觉高频帧可保留 UDP 或改为 TCP framed stream。
- 地址：只绑定 `127.0.0.1`。
- 消息格式：JSON Lines 或 MessagePack，字段包含 `type`, `seq`, `timestamp_monotonic`, `payload`。
- 心跳：双向 10 Hz，超时 300-500 ms。
- 序列号：所有命令和状态都带单调递增 `seq`，接收端丢弃过期命令。
- 超时：ROS2 侧命令超时后进入停止发布/急停请求状态；LeRobot 侧超时后保持或释放到预定义安全状态，不继续执行旧目标。
- 急停：单独 `estop` 消息，需最高优先级、幂等、可确认。
- 断线行为：LeRobot server 不得继续消费旧命令；需要显式重新握手。
- 状态同步：需要双向同步，至少包含连接状态、最近命令序号、机器人连接状态、关节观测、错误码、触觉状态。

本阶段未实现 Socket Bridge。

## 11. 阶段 0 前置条件

必须：

- 安装或激活 Windows ROS2 环境，使 `ros2 --help` 可用。
- 安装或激活 `colcon`，使 `colcon build` 可用。
- 进入正确的 Visual Studio Developer PowerShell 或安装 Build Tools，使 `cl` 可用。
- 确认 ROS2 使用的 Python 版本，并运行 `audit/check_ros2_import.py`。
- 重新运行 `audit/run_stage_minus1_audit.ps1`，再验证最小 ROS2 Python/C++ topic。

推荐：

- 不要把 `rclpy` 强行装进当前 LeRobot 环境，先建立独立 ROS2 环境并采用 localhost bridge。
- 为相机确认 Windows 隐私权限、设备管理器状态和占用情况，再运行 `run_stage_minus1_audit.ps1 -TestCamera`。
- 为旧 FlexiTac/LeRobot 扩展补 Windows 文档，把 `/dev/ttyUSB*` 改成 `COMx` 示例。

可延期：

- 可视化节点的 headless 模式。
- UDP 协议升级为 TCP framed bridge。
- LeRobot 旧扩展的包结构清理。

## 12. 风险清单

- Windows ROS2 Python 版本风险：ROS2 Windows 发行版通常绑定特定 Python ABI，当前 LeRobot 侧是 Python 3.12，默认 Python 是 3.14。
- ROS2 与 Conda 环境冲突：当前 LeRobot 环境缺 `rclpy`；直接混装可能引入 DLL/NumPy/OpenCV/Torch 冲突。
- LeRobot 导入副作用：`SOTactileFollower` 构造会进入 `SOFollower.__init__`，并可能初始化 `TactileSensor`、启动连续读取；本阶段没有构造该对象。
- 相机后端稳定性：OpenCV 构建支持 DSHOW/MSMF，但 0-5 索引均未打开，可能是权限、占用或设备缺失。
- Windows COM 端口变化：只读枚举仅发现蓝牙 COM5/COM6，不能视为机器人或触觉设备身份。
- UDP/TCP 端口冲突：旧项目默认 UDP 5005/5006，需在 Windows 上检测端口占用。
- ROS2 C++ 工具链：当前缺 `cl`、`colcon`、ROS2 环境，无法验证 `rclcpp`。
- 旧项目 Linux-only 依赖：主要在 shell 脚本和 README 命令，不在核心 Python 算法。
- 机械臂误动作风险：旧扩展 `send_action` 会调用父类真实动作发送；后续测试必须保持硬件测试默认关闭。

## 新增文件和结果位置

新增审计脚本：

- `so101_visual_tactile_grasp/audit/check_environment.ps1`
- `so101_visual_tactile_grasp/audit/check_python_runtime.py`
- `so101_visual_tactile_grasp/audit/check_opencv_camera.py`
- `so101_visual_tactile_grasp/audit/check_ros2_import.py`
- `so101_visual_tactile_grasp/audit/check_lerobot_import.py`
- `so101_visual_tactile_grasp/audit/check_combined_import.py`
- `so101_visual_tactile_grasp/audit/scan_linux_dependencies.py`
- `so101_visual_tactile_grasp/audit/run_stage_minus1_audit.ps1`

新增报告：

- `so101_visual_tactile_grasp/docs/stage_minus1_windows_audit.md`

审计结果：

- `so101_visual_tactile_grasp/audit_results/*.log`
- `so101_visual_tactile_grasp/audit_results/*.json`
- `so101_visual_tactile_grasp/audit_results/env_lerobot/*`
- `so101_visual_tactile_grasp/audit_results/env_leflexitac/*`
- `so101_visual_tactile_grasp/audit_results/env_LeRobot_HIL/*`
- `so101_visual_tactile_grasp/audit_results/env_so101_nexus_pnp_bcppo_repro/*`
- `so101_visual_tactile_grasp/audit_results/env_so101_nexus_official_0412/*`
- `so101_visual_tactile_grasp/audit_results/minimal_ros2_ws/*`

## 执行过的关键命令

- `Get-Location`
- `Get-ChildItem -Force`
- `git -C .\so101_ros2_tactile_guard status`
- `git -C .\so101_visual_tactile_grasp status`
- `git ls-remote https://github.com/adsade5/so101_ros2_tactile_guard.git`
- `git ls-remote https://github.com/adsade5/so101-ros2-tactile-guard.git`
- `git clone https://github.com/adsade5/so101-ros2-tactile-guard.git .\so101_ros2_tactile_guard`
- `git -c safe.directory=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_ros2_tactile_guard -C .\so101_ros2_tactile_guard status --short --branch`
- `git -c safe.directory=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_ros2_tactile_guard -C .\so101_ros2_tactile_guard remote -v`
- `git -c safe.directory=E:/PycharmProjects/Embodied_AI/LeRobot_Project/so101_ros2_tactile_guard -C .\so101_ros2_tactile_guard rev-parse HEAD`
- `rg --files .\so101_ros2_tactile_guard`
- `rg -n ... .\so101_ros2_tactile_guard`
- `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\so101_visual_tactile_grasp\audit\run_stage_minus1_audit.ps1 -TestCamera`
- `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\so101_visual_tactile_grasp\audit\run_stage_minus1_audit.ps1`
- `E:\Anaconda\envs_dirs\lerobot\python.exe audit\check_combined_import.py`
- `E:\Anaconda\envs_dirs\LeRobot_HIL\python.exe audit\check_combined_import.py`
- `E:\Anaconda\envs_dirs\leflexitac\python.exe audit\check_combined_import.py`
- `E:\Anaconda\envs_dirs\so101_nexus_pnp_bcppo_repro\python.exe audit\check_combined_import.py`
- `E:\Anaconda\envs_dirs\so101_nexus_official_0412\python.exe audit\check_combined_import.py`
- `E:\Anaconda\envs_dirs\lerobot\python.exe audit\check_opencv_camera.py --test-camera`
- `where.exe cmake`, `cmake --version`, `where.exe colcon`, `where.exe cl`, `where.exe ninja`

## 未解决错误

- `ros2` 当前不可用。
- `colcon` 当前不可用。
- MSVC `cl` 当前不可用。
- 默认 Python 缺 OpenCV、pyserial、ROS2、LeRobot。
- LeRobot Conda 环境缺 `rclpy` / `std_msgs`。
- 相机 0-5 在 DEFAULT/DSHOW/MSMF 后端均未打开。
- `leflexitac` 环境模块遍历时 `lerobot.motors.robstride` 和 `lerobot.teleoperators.phone` 有导入错误风险。
- Codex 沙箱用户下旧仓库触发 Git dubious ownership；未修改全局配置。

## 阶段 -1B 后续报告

阶段 -1B ROS 2 Lyrical Windows 安装与验证结果见：`docs/stage_minus1b_ros2_lyrical_setup.md`。
