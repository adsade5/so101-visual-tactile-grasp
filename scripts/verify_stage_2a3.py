from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_WS = PROJECT_ROOT / "ros2_ws"
RUN_IN_ROS2 = PROJECT_ROOT / "audit" / "run_in_ros2_lyrical.ps1"
ROS2_PYTHON = Path(r"C:\pixi_ws\.pixi\envs\default\python.exe")
REPORT_DIR = PROJECT_ROOT / "data" / "verification"
LOG_PATH = REPORT_DIR / "stage_2a3_verification.log"
REPORT_PATH = REPORT_DIR / "stage_2a3_report.json"
PROCESS_LOG_DIR = REPORT_DIR / "stage_2a3_logs"
ROS_LOG_DIR = REPORT_DIR / "ros_logs"

SOURCE_URDF = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)
VISUALIZATION_URDF = (
    ROS2_WS
    / "src"
    / "so101_description"
    / "urdf"
    / "so101_visualization.urdf"
)
MANIFEST_PATH = (
    ROS2_WS
    / "src"
    / "so101_description"
    / "model_preparation_manifest.json"
)
DESCRIPTION_ASSETS = (
    ROS2_WS
    / "src"
    / "so101_description"
    / "assets"
)
INSTALLED_DESCRIPTION_SHARE = (
    ROS2_WS
    / "install"
    / "share"
    / "so101_description"
)
RVIZ_CONFIG_INSTALLED = (
    INSTALLED_DESCRIPTION_SHARE
    / "rviz"
    / "so101.rviz"
)

EXPECTED_ZERO_POSITION_M = [
    0.3913614702,
    -0.0000092121,
    0.2264697102,
]

MAX_ZERO_ERROR_MM = 0.5
MAX_TF_POSITION_ERROR_MM = 0.5
MAX_TF_ORIENTATION_ERROR_DEG = 0.2
MIN_RATE_HZ = 10.0
MIN_DYNAMIC_RANGE_MM = 1.0


def iso_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def utc_stamp_for_filename() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d_%H%M%S")


def ensure_dirs() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    PROCESS_LOG_DIR.mkdir(parents=True, exist_ok=True)
    ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)


def append_log(message: str) -> None:
    ensure_dirs()
    line = f"{iso_timestamp()} {message}"
    print(line, flush=True)
    with LOG_PATH.open("a", encoding="utf-8") as file:
        file.write(line + "\n")


def run_outside_ros() -> int:
    ensure_dirs()
    if LOG_PATH.exists():
        LOG_PATH.unlink()

    command_file = Path(tempfile.gettempdir()) / (
        "verify_stage_2a3_{0}.cmd".format(utc_stamp_for_filename())
    )
    command = "\n".join(
        [
            f'cd /d "{ROS2_WS}"',
            f'set "ROS_LOG_DIR={ROS_LOG_DIR}"',
            'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
            'set "ZENOH_ROUTER_CHECK_ATTEMPTS=5"',
            "call install\\local_setup.bat",
            f'"{ROS2_PYTHON}" "{Path(__file__).resolve()}" --inside-ros',
        ]
    )
    command_file.write_text(command, encoding="ascii")

    try:
        ps_command = [
            "powershell",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(RUN_IN_ROS2),
            "-CommandFile",
            str(command_file),
        ]
        append_log("Entering ROS2 Lyrical environment.")
        completed = subprocess.run(
            ps_command,
            cwd=str(PROJECT_ROOT),
            text=True,
        )
        return int(completed.returncode)
    finally:
        command_file.unlink(missing_ok=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"Expected object in {path}")
    return value


def command_output(command: str, timeout_s: float = 20.0) -> tuple[int, str]:
    completed = subprocess.run(
        ["cmd.exe", "/d", "/s", "/c", command],
        cwd=str(ROS2_WS),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )
    return int(completed.returncode), completed.stdout


def vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def sub_vector(a: list[float], b: list[float]) -> list[float]:
    return [float(x) - float(y) for x, y in zip(a, b, strict=True)]


def finite(values: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def normalize_quaternion(q: list[float]) -> list[float]:
    norm = vector_norm(q)
    if norm <= 1e-12:
        raise ValueError("Quaternion norm is zero")
    return [float(value) / norm for value in q]


def quaternion_error_deg(a: list[float], b: list[float]) -> float:
    qa = normalize_quaternion(a)
    qb = normalize_quaternion(b)
    dot = abs(sum(x * y for x, y in zip(qa, qb, strict=True)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def rate_stats(stamps: list[float]) -> dict[str, Any]:
    if len(stamps) < 2:
        return {
            "samples": len(stamps),
            "duration_s": 0.0,
            "rate_hz": 0.0,
            "min_interval_s": None,
            "max_interval_s": None,
        }
    intervals = [
        stamps[index] - stamps[index - 1]
        for index in range(1, len(stamps))
    ]
    duration = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / duration if duration > 0 else 0.0
    return {
        "samples": len(stamps),
        "duration_s": duration,
        "rate_hz": rate,
        "min_interval_s": min(intervals),
        "max_interval_s": max(intervals),
    }


def component_range_mm(samples: list[list[float]]) -> tuple[float, list[float]]:
    if not samples:
        return 0.0, [0.0, 0.0, 0.0]
    ranges = []
    for axis in range(3):
        values = [sample[axis] for sample in samples]
        ranges.append((max(values) - min(values)) * 1000.0)
    return max(ranges), ranges


@dataclass
class ManagedProcess:
    name: str
    command: str
    process: subprocess.Popen[Any]
    log_path: Path
    log_file: Any


class ProcessManager:
    def __init__(self) -> None:
        self.processes: list[ManagedProcess] = []

    def start(self, name: str, command: str) -> ManagedProcess:
        log_path = PROCESS_LOG_DIR / f"{name}_{utc_stamp_for_filename()}.log"
        log_file = log_path.open("w", encoding="utf-8", errors="replace")
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        append_log(f"Starting {name}: {command}")
        process = subprocess.Popen(
            ["cmd.exe", "/d", "/s", "/c", command],
            cwd=str(ROS2_WS),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            creationflags=creationflags,
        )
        managed = ManagedProcess(
            name=name,
            command=command,
            process=process,
            log_path=log_path,
            log_file=log_file,
        )
        self.processes.append(managed)
        return managed

    def terminate(self, managed: ManagedProcess, timeout_s: float = 5.0) -> None:
        if managed.process.poll() is not None:
            managed.log_file.close()
            return
        append_log(f"Stopping {managed.name} pid={managed.process.pid}.")
        try:
            if os.name == "nt":
                managed.process.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                managed.process.terminate()
            managed.process.wait(timeout=timeout_s)
        except Exception:
            if os.name == "nt":
                subprocess.run(
                    [
                        "taskkill",
                        "/PID",
                        str(managed.process.pid),
                        "/T",
                        "/F",
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            else:
                managed.process.kill()
            try:
                managed.process.wait(timeout=timeout_s)
            except Exception:
                pass
        finally:
            managed.log_file.close()

    def terminate_all(self) -> None:
        for managed in reversed(self.processes):
            self.terminate(managed)

    def live_status(self) -> dict[str, bool]:
        return {
            managed.name: managed.process.poll() is None
            for managed in self.processes
        }


def inspect_description_files(report: dict[str, Any]) -> None:
    source_hash = sha256_file(SOURCE_URDF)
    manifest = read_json(MANIFEST_PATH)
    manifest_hash = str(
        manifest.get("source_sha256", manifest.get("source_sha256", ""))
    )
    mesh_paths: list[str] = []
    if VISUALIZATION_URDF.is_file():
        root = ET.parse(VISUALIZATION_URDF).getroot()
        for mesh in root.findall(".//mesh"):
            filename = mesh.attrib.get("filename", "")
            if filename:
                mesh_paths.append(filename)

    invalid_mesh_paths = [
        value
        for value in mesh_paths
        if not value.startswith("package://so101_description/assets/")
    ]

    installed_files = {
        "urdf": (
            INSTALLED_DESCRIPTION_SHARE / "urdf" / "so101_visualization.urdf"
        ).is_file(),
        "launch": (
            INSTALLED_DESCRIPTION_SHARE / "launch" / "display.launch.py"
        ).is_file(),
        "assets": (
            INSTALLED_DESCRIPTION_SHARE / "assets"
        ).is_dir(),
        "rviz": RVIZ_CONFIG_INSTALLED.is_file(),
    }

    report["files"] = {
        "source_urdf": str(SOURCE_URDF),
        "source_urdf_sha256": source_hash,
        "manifest_source_sha256": manifest_hash,
        "source_urdf_sha256_matches_manifest": source_hash == manifest_hash,
        "visualization_urdf": str(VISUALIZATION_URDF),
        "visualization_urdf_exists": VISUALIZATION_URDF.is_file(),
        "assets_exist": DESCRIPTION_ASSETS.is_dir(),
        "mesh_path_count": len(mesh_paths),
        "invalid_mesh_paths": invalid_mesh_paths,
        "installed": installed_files,
    }


def inspect_executables(report: dict[str, Any]) -> str:
    code, output = command_output(
        "ros2 pkg executables so101_kinematics",
        timeout_s=20.0,
    )
    names = set()
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] == "so101_kinematics":
            executable = parts[1]
            names.add(executable)
            if executable.endswith(".exe"):
                names.add(executable[:-4])
            if executable.endswith("-script.py"):
                names.add(executable[: -len("-script.py")])
    report["executables"] = {
        "fk_node": "fk_node" in names,
        "test_joint_state_publisher": "test_joint_state_publisher" in names,
        "tf_fk_consistency_checker": "tf_fk_consistency_checker" in names,
        "raw_output": output,
        "return_code": code,
    }
    return output


def smoke_test_checker(pm: ProcessManager, report: dict[str, Any]) -> None:
    checker = pm.start(
        "checker_entry_smoke",
        "ros2 run so101_kinematics tf_fk_consistency_checker",
    )
    time.sleep(2.0)
    running = checker.process.poll() is None
    report["checker_entry_smoke"] = {
        "ros2_run_started": running,
        "return_code": checker.process.poll(),
        "log": str(checker.log_path),
    }
    pm.terminate(checker)


def fail(report: dict[str, Any], name: str, detail: str) -> None:
    report.setdefault("failures", []).append(
        {
            "item": name,
            "detail": detail,
        }
    )


def verify_with_ros(report: dict[str, Any]) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException, TransformListener

    class Observer(Node):
        def __init__(self) -> None:
            super().__init__("so101_stage_2a3_observer")
            self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.reset()
            self.create_subscription(
                JointState, "/joint_states", self.on_joint, 50
            )
            self.create_subscription(
                PoseStamped, "/end_effector_pose", self.on_pose, 50
            )
            self.create_subscription(Bool, "/fk_valid", self.on_fk_valid, 50)
            self.create_subscription(
                Bool, "/fk_tf_consistent", self.on_consistent, 50
            )
            self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
            tf_static_qos = QoSProfile(depth=10)
            tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
            tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                TFMessage, "/tf_static", self.on_tf_static, tf_static_qos
            )

        def reset(self) -> None:
            self.joint_times: list[float] = []
            self.pose_times: list[float] = []
            self.pose_positions: list[list[float]] = []
            self.pose_quats: list[list[float]] = []
            self.pose_frames: list[str] = []
            self.fk_valid_values: list[bool] = []
            self.consistent_values: list[bool] = []
            self.tf_times: list[float] = []
            self.tf_positions: list[list[float]] = []
            self.tf_quats: list[list[float]] = []
            self.tf_msg_count = 0
            self.tf_static_msg_count = 0
            self.joint_names_seen: list[list[str]] = []

        def now_float(self) -> float:
            return time.monotonic()

        def on_joint(self, msg: JointState) -> None:
            self.joint_times.append(self.now_float())
            self.joint_names_seen.append(list(msg.name))

        def on_pose(self, msg: PoseStamped) -> None:
            self.pose_times.append(self.now_float())
            self.pose_frames.append(msg.header.frame_id)
            self.pose_positions.append(
                [
                    float(msg.pose.position.x),
                    float(msg.pose.position.y),
                    float(msg.pose.position.z),
                ]
            )
            self.pose_quats.append(
                [
                    float(msg.pose.orientation.x),
                    float(msg.pose.orientation.y),
                    float(msg.pose.orientation.z),
                    float(msg.pose.orientation.w),
                ]
            )

        def on_fk_valid(self, msg: Bool) -> None:
            self.fk_valid_values.append(bool(msg.data))

        def on_consistent(self, msg: Bool) -> None:
            self.consistent_values.append(bool(msg.data))

        def on_tf(self, msg: TFMessage) -> None:
            self.tf_msg_count += len(msg.transforms)

        def on_tf_static(self, msg: TFMessage) -> None:
            self.tf_static_msg_count += len(msg.transforms)

        def sample_tf(self) -> bool:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    "gripper_frame_link",
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                return False
            self.tf_times.append(self.now_float())
            self.tf_positions.append(
                [
                    float(transform.transform.translation.x),
                    float(transform.transform.translation.y),
                    float(transform.transform.translation.z),
                ]
            )
            self.tf_quats.append(
                [
                    float(transform.transform.rotation.x),
                    float(transform.transform.rotation.y),
                    float(transform.transform.rotation.z),
                    float(transform.transform.rotation.w),
                ]
            )
            return True

    def spin_for(node: Observer, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.sample_tf()

    def wait_until(node: Observer, predicate: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.sample_tf()
            if predicate():
                return True
        return False

    rclpy.init()
    node = Observer()
    pm = ProcessManager()

    try:
        zero_pub = pm.start(
            "zero_joint_state_publisher",
            (
                "ros2 run so101_kinematics test_joint_state_publisher "
                "--ros-args -p mode:=zero -p publish_rate_hz:=20.0"
            ),
        )
        fk_node = pm.start(
            "fk_node",
            (
                "ros2 run so101_kinematics fk_node --ros-args "
                f'-p project_root:="{PROJECT_ROOT.as_posix()}"'
            ),
        )
        rsp_launch = pm.start(
            "robot_state_publisher_launch",
            "ros2 launch so101_description display.launch.py start_rviz:=false",
        )
        checker = pm.start(
            "tf_fk_consistency_checker",
            "ros2 run so101_kinematics tf_fk_consistency_checker",
        )

        static_ready = wait_until(
            node,
            lambda: (
                len(node.pose_positions) >= 20
                and len(node.joint_times) >= 20
                and any(node.fk_valid_values)
                and any(node.consistent_values)
                and len(node.tf_positions) >= 5
            ),
            timeout_s=25.0,
        )
        if not static_ready:
            fail(
                report,
                "static_zero_test",
                (
                    "Timed out waiting for 20 pose/joint samples, "
                    "valid FK, consistent TF/FK, and TF."
                ),
            )

        pose = node.pose_positions[-1] if node.pose_positions else [math.nan] * 3
        quat = node.pose_quats[-1] if node.pose_quats else [math.nan] * 4
        tf_pos = node.tf_positions[-1] if node.tf_positions else [math.nan] * 3
        tf_quat = node.tf_quats[-1] if node.tf_quats else [math.nan] * 4
        zero_error_mm = (
            vector_norm(sub_vector(pose, EXPECTED_ZERO_POSITION_M)) * 1000.0
        )
        tf_position_error_mm = vector_norm(sub_vector(pose, tf_pos)) * 1000.0
        tf_orientation_error = quaternion_error_deg(quat, tf_quat)
        pose_rate = rate_stats(node.pose_times)
        joint_rate = rate_stats(node.joint_times)
        quaternion_norm = vector_norm(quat) if finite(quat) else math.nan

        static_passed = (
            static_ready
            and bool(node.fk_valid_values and node.fk_valid_values[-1])
            and bool(node.consistent_values and node.consistent_values[-1])
            and node.pose_frames
            and node.pose_frames[-1] == "base_link"
            and finite(pose)
            and finite(quat)
            and abs(quaternion_norm - 1.0) <= 1e-3
            and zero_error_mm <= MAX_ZERO_ERROR_MM
            and tf_position_error_mm <= MAX_TF_POSITION_ERROR_MM
            and tf_orientation_error <= MAX_TF_ORIENTATION_ERROR_DEG
            and pose_rate["rate_hz"] >= MIN_RATE_HZ
            and joint_rate["rate_hz"] >= MIN_RATE_HZ
            and node.tf_msg_count > 0
            and node.tf_static_msg_count > 0
        )

        report["static_zero_test"] = {
            "passed": bool(static_passed),
            "fk_valid": bool(node.fk_valid_values and node.fk_valid_values[-1]),
            "pose_samples": len(node.pose_positions),
            "pose_rate_hz": pose_rate["rate_hz"],
            "pose_rate_stats": pose_rate,
            "joint_state_rate_hz": joint_rate["rate_hz"],
            "joint_state_rate_stats": joint_rate,
            "expected_position_m": EXPECTED_ZERO_POSITION_M,
            "measured_position_m": pose,
            "zero_position_error_mm": zero_error_mm,
            "tf_position_m": tf_pos,
            "tf_pose_position_error_mm": tf_position_error_mm,
            "tf_pose_orientation_error_deg": tf_orientation_error,
            "fk_tf_consistent": bool(
                node.consistent_values and node.consistent_values[-1]
            ),
            "pose_frame_id": node.pose_frames[-1] if node.pose_frames else None,
            "quaternion_norm": quaternion_norm,
            "tf_messages": node.tf_msg_count,
            "tf_static_messages": node.tf_static_msg_count,
            "gripper_in_fk_chain": False,
        }
        if not static_passed:
            fail(
                report,
                "static_zero_test",
                (
                    "Static zero validation failed. "
                    f"zero_error_mm={zero_error_mm:.6f}, "
                    f"tf_position_error_mm={tf_position_error_mm:.6f}, "
                    f"tf_orientation_error_deg={tf_orientation_error:.6f}, "
                    f"pose_rate_hz={pose_rate['rate_hz']:.3f}, "
                    f"joint_rate_hz={joint_rate['rate_hz']:.3f}"
                ),
            )

        pose_count_before_stale = len(node.pose_times)
        last_pose_time_before_stale = node.pose_times[-1] if node.pose_times else 0.0
        pm.terminate(zero_pub)
        spin_for(node, 1.25)
        fk_valid_became_false = any(value is False for value in node.fk_valid_values[-20:])
        later_poses = [
            stamp
            for stamp in node.pose_times[pose_count_before_stale:]
            if stamp > last_pose_time_before_stale + 0.25
        ]
        pose_stopped = len(later_poses) == 0
        stale_passed = fk_valid_became_false and pose_stopped and fk_node.process.poll() is None
        report["stale_timeout_test"] = {
            "passed": bool(stale_passed),
            "fk_valid_became_false": bool(fk_valid_became_false),
            "pose_stopped": bool(pose_stopped),
            "pose_samples_before_stop": pose_count_before_stale,
            "pose_samples_after_stop": len(node.pose_times) - pose_count_before_stale,
            "fk_node_alive": fk_node.process.poll() is None,
        }
        if not stale_passed:
            fail(
                report,
                "stale_timeout_test",
                (
                    "Stale timeout validation failed. "
                    f"fk_valid_became_false={fk_valid_became_false}, "
                    f"pose_stopped={pose_stopped}"
                ),
            )

        node.reset()
        sine_pub = pm.start(
            "sine_joint_state_publisher",
            (
                "ros2 run so101_kinematics test_joint_state_publisher "
                "--ros-args -p mode:=sine -p publish_rate_hz:=20.0"
            ),
        )
        spin_for(node, 6.0)
        pose_range_max, pose_ranges = component_range_mm(node.pose_positions)
        tf_range_max, tf_ranges = component_range_mm(node.tf_positions)
        dynamic_values_finite = all(finite(value) for value in node.pose_positions)
        dynamic_values_finite = dynamic_values_finite and all(
            finite(value) for value in node.tf_positions
        )
        last_joint_names = node.joint_names_seen[-1] if node.joint_names_seen else []
        dynamic_passed = (
            any(node.fk_valid_values)
            and len(node.pose_positions) >= 20
            and len(node.tf_positions) >= 10
            and pose_range_max > MIN_DYNAMIC_RANGE_MM
            and tf_range_max > MIN_DYNAMIC_RANGE_MM
            and dynamic_values_finite
            and fk_node.process.poll() is None
            and checker.process.poll() is None
            and "gripper" in last_joint_names
        )
        report["dynamic_test"] = {
            "passed": bool(dynamic_passed),
            "duration_s": 6.0,
            "pose_changed": pose_range_max > MIN_DYNAMIC_RANGE_MM,
            "tf_changed": tf_range_max > MIN_DYNAMIC_RANGE_MM,
            "maximum_position_range_mm": pose_range_max,
            "pose_axis_ranges_mm": pose_ranges,
            "tf_maximum_position_range_mm": tf_range_max,
            "tf_axis_ranges_mm": tf_ranges,
            "fk_valid_seen": any(node.fk_valid_values),
            "pose_samples": len(node.pose_positions),
            "tf_samples": len(node.tf_positions),
            "all_values_finite": dynamic_values_finite,
            "nodes_alive": {
                "fk_node": fk_node.process.poll() is None,
                "tf_fk_consistency_checker": checker.process.poll() is None,
            },
            "gripper_joint_present_in_joint_states": "gripper" in last_joint_names,
            "gripper_in_five_joint_fk_chain": False,
        }
        if not dynamic_passed:
            fail(
                report,
                "dynamic_test",
                (
                    "Dynamic smoke validation failed. "
                    f"pose_range_mm={pose_range_max:.6f}, "
                    f"tf_range_mm={tf_range_max:.6f}"
                ),
            )

        pm.terminate(sine_pub)

        rviz_report = run_rviz_smoke(pm)
        report["rviz_smoke_test"] = rviz_report
        if not rviz_report.get("passed", False):
            fail(
                report,
                "rviz_smoke_test",
                (
                    "RViz smoke test failed. "
                    f"errors={rviz_report.get('fatal_resource_errors', [])}"
                ),
            )

    finally:
        pm.terminate_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report["process_cleanup"] = {
            "all_started_processes_stopped": all(
                not alive for alive in pm.live_status().values()
            ),
            "process_logs": {
                managed.name: str(managed.log_path)
                for managed in pm.processes
            },
        }


def run_rviz_smoke(pm: ProcessManager) -> dict[str, Any]:
    robot_state_code, robot_state_output = command_output(
        "ros2 pkg prefix robot_state_publisher",
        timeout_s=10.0,
    )
    rviz_code, rviz_output = command_output(
        "ros2 pkg prefix rviz2",
        timeout_s=10.0,
    )

    if robot_state_code != 0 or rviz_code != 0:
        return {
            "passed": False,
            "process_started": False,
            "robot_state_publisher_available": robot_state_code == 0,
            "rviz2_available": rviz_code == 0,
            "fatal_resource_errors": [
                "robot_state_publisher or rviz2 package missing"
            ],
            "limitations": (
                "Automatic smoke testing cannot replace a human visual "
                "inspection of final model appearance."
            ),
        }

    zero_pub = pm.start(
        "rviz_zero_joint_state_publisher",
        (
            "ros2 run so101_kinematics test_joint_state_publisher "
            "--ros-args -p mode:=zero -p publish_rate_hz:=20.0"
        ),
    )
    rsp_launch = pm.start(
        "rviz_robot_state_publisher_launch",
        "ros2 launch so101_description display.launch.py start_rviz:=false",
    )
    time.sleep(2.0)
    rviz = pm.start(
        "rviz2_smoke",
        f'ros2 run rviz2 rviz2 -d "{RVIZ_CONFIG_INSTALLED}"',
    )
    time.sleep(10.0)
    process_started = rviz.process.poll() is None
    return_code = rviz.process.poll()
    pm.terminate(rviz)
    pm.terminate(rsp_launch)
    pm.terminate(zero_pub)

    log_text = ""
    try:
        log_text = rviz.log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        pass

    fatal_patterns = [
        "Could not load resource",
        "Mesh resource not found",
        "Failed to load mesh",
        "robot_description missing",
        "Fixed Frame does not exist",
        "XML parse error",
        "URDF parse error",
    ]
    fatal_errors = [
        pattern for pattern in fatal_patterns if pattern in log_text
    ]
    warning_lines = [
        line
        for line in log_text.splitlines()
        if re.search(r"\b(warn|warning|opengl|gpu|graphics)\b", line, re.I)
    ][:25]
    return {
        "passed": bool(process_started and not fatal_errors),
        "process_started": bool(process_started),
        "return_code_after_10s": return_code,
        "robot_state_publisher_available": True,
        "rviz2_available": True,
        "fatal_resource_errors": fatal_errors,
        "nonfatal_warnings": warning_lines,
        "rviz_config": str(RVIZ_CONFIG_INSTALLED),
        "log": str(rviz.log_path),
        "limitations": (
            "Automatic smoke testing confirms the RViz process, "
            "RobotModel configuration, URDF, TF, and mesh resource loading "
            "path had no obvious fatal failure. It cannot fully replace "
            "manual visual inspection of model appearance; this stage uses "
            "numeric FK/TF consistency as the core acceptance signal."
        ),
    }


def evaluate_failures(report: dict[str, Any]) -> None:
    files = report.get("files", {})
    if not files.get("source_urdf_sha256_matches_manifest", False):
        fail(
            report,
            "source_urdf_hash",
            "Frozen source URDF SHA-256 does not match manifest.",
        )
    if not files.get("visualization_urdf_exists", False):
        fail(report, "visualization_urdf", "Visualization URDF is missing.")
    if not files.get("assets_exist", False):
        fail(report, "assets", "so101_description assets directory is missing.")
    if files.get("invalid_mesh_paths"):
        fail(
            report,
            "mesh_paths",
            f"Invalid mesh paths: {files.get('invalid_mesh_paths')}",
        )
    installed = files.get("installed", {})
    for name, present in installed.items():
        if not present:
            fail(report, f"installed_{name}", f"Installed {name} is missing.")

    executables = report.get("executables", {})
    for name in (
        "fk_node",
        "test_joint_state_publisher",
        "tf_fk_consistency_checker",
    ):
        if not executables.get(name, False):
            fail(report, f"executable_{name}", f"{name} is not registered.")
    checker = report.get("checker_entry_smoke", {})
    if not checker.get("ros2_run_started", False):
        fail(
            report,
            "tf_fk_consistency_checker_ros2_run",
            "ros2 run did not start tf_fk_consistency_checker.",
        )


def run_inside_ros() -> int:
    ensure_dirs()
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "5"
    append_log("Stage 2A-3 verification started inside ROS2 environment.")
    report: dict[str, Any] = {
        "stage": "2A-3",
        "status": "FAIL",
        "timestamp": iso_timestamp(),
        "environment": {
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "python": sys.executable,
            "cmake_generator": os.environ.get("CMAKE_GENERATOR", "Ninja"),
            "install_layout": "merged",
            "ros_log_dir": os.environ.get("ROS_LOG_DIR", ""),
            "fastdds_profile": "",
            "fastdds_builtin_transports": "",
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "zenoh_router_check_attempts": os.environ.get(
                "ZENOH_ROUTER_CHECK_ATTEMPTS", ""
            ),
        },
        "logs": {
            "directory": str(PROCESS_LOG_DIR),
            "main_log": str(LOG_PATH),
        },
        "failures": [],
        "safety": {
            "used_simulated_joint_states_only": True,
            "opened_com_ports": False,
            "started_hardware_services": False,
            "published_robot_commands": False,
        },
    }

    pm = ProcessManager()
    try:
        inspect_description_files(report)
        executables_output = inspect_executables(report)
        append_log("ros2 pkg executables so101_kinematics:")
        append_log(executables_output.strip())
        router = pm.start(
            "rmw_zenoh_router",
            "ros2 run rmw_zenoh_cpp rmw_zenohd",
        )
        time.sleep(2.0)
        report["zenoh_router"] = {
            "started": router.process.poll() is None,
            "return_code": router.process.poll(),
            "log": str(router.log_path),
        }
        if router.process.poll() is not None:
            fail(
                report,
                "zenoh_router",
                "rmw_zenohd exited before verification nodes started.",
            )
        smoke_test_checker(pm, report)
        verify_with_ros(report)
        evaluate_failures(report)
    except Exception as exc:
        fail(report, "verification_exception", repr(exc))
        append_log(f"ERROR: {exc!r}")
    finally:
        pm.terminate_all()
        report["status"] = "PASS" if not report.get("failures") else "FAIL"
        REPORT_PATH.write_text(
            json.dumps(report, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        append_log(f"Report written: {REPORT_PATH}")
        append_log(f"Stage 2A-3 status: {report['status']}")
    return 0 if report["status"] == "PASS" else 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Automated SO-101 Stage 2A-3 ROS2 verification."
    )
    parser.add_argument(
        "--inside-ros",
        action="store_true",
        help="Run the verifier assuming ROS2 and the workspace are sourced.",
    )
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros()
    return run_outside_ros()


if __name__ == "__main__":
    raise SystemExit(main())
