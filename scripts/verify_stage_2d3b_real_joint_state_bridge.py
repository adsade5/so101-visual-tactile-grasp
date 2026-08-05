from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import queue
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_WS = PROJECT_ROOT / "ros2_ws"
RUN_IN_ROS2 = PROJECT_ROOT / "audit" / "run_in_ros2_lyrical.ps1"
ROS2_PYTHON = Path(r"C:\pixi_ws\.pixi\envs\default\python.exe")
LEROBOT_PYTHON = Path(r"E:\Anaconda\envs_dirs\lerobot\python.exe")
REPORT_DIR = PROJECT_ROOT / "data" / "verification"
REPORT_PATH = REPORT_DIR / "stage_2d3b_report.json"
OFFLINE_REPORT_PATH = REPORT_DIR / "stage_2d3b_offline_report.json"
LOG_PATH = REPORT_DIR / "stage_2d3b_verification.log"
OFFLINE_LOG_PATH = REPORT_DIR / "stage_2d3b_offline.log"
ROS_LOG_DIR = REPORT_DIR / "ros_logs_2d3b"
MAPPING_PATH = PROJECT_ROOT / "config" / "real_joint_state_mapping.json"
READ_ONLY_SERVER = PROJECT_ROOT / "scripts" / "so101_read_only_joint_state_server.py"
URDF_PATH = PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
EXPECTED_URDF_SHA256 = "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
EXPECTED_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def iso_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def filename_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d_%H%M%S")


def ensure_dirs() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    ROS_LOG_DIR.mkdir(parents=True, exist_ok=True)


def append_log(message: str) -> None:
    ensure_dirs()
    line = f"{iso_timestamp()} {message}"
    print(line, flush=True)
    with LOG_PATH.open("a", encoding="utf-8") as file:
        file.write(line + "\n")
    with OFFLINE_LOG_PATH.open("a", encoding="utf-8") as file:
        file.write(line + "\n")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def fail(report: dict[str, Any], item: str, detail: str) -> None:
    report.setdefault("failures", []).append({"item": item, "detail": detail})


def run_outside_ros() -> int:
    ensure_dirs()
    for path in (LOG_PATH, OFFLINE_LOG_PATH):
        if path.exists():
            path.unlink()
    command_file = Path(tempfile.gettempdir()) / (
        f"verify_stage_2d3b_{filename_timestamp()}.cmd"
    )
    command = "\n".join(
        [
            f'cd /d "{ROS2_WS}"',
            f'set "ROS_LOG_DIR={ROS_LOG_DIR}"',
            'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
            'set "ZENOH_ROUTER_CHECK_ATTEMPTS=1"',
            'set "RCL_LOGGING_IMPLEMENTATION=rcl_logging_noop"',
            "call install\\local_setup.bat",
            f'"{ROS2_PYTHON}" "{Path(__file__).resolve()}" --inside-ros',
        ]
    )
    command_file.write_text(command, encoding="ascii")
    try:
        append_log("Entering ROS2 Lyrical environment for Stage 2D-3B.")
        completed = subprocess.run(
            [
                "powershell",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(RUN_IN_ROS2),
                "-CommandFile",
                str(command_file),
            ],
            cwd=str(PROJECT_ROOT),
            text=True,
        )
        return int(completed.returncode)
    finally:
        command_file.unlink(missing_ok=True)


def command_output(command: list[str], timeout_s: float = 60.0) -> tuple[int, str]:
    completed = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )
    return int(completed.returncode), completed.stdout


class JsonLineServer:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.queue: queue.Queue[dict[str, Any] | str] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.connected_count = 0

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.queue.put("__stop__")
        try:
            with socket.create_connection((self.host, self.port), timeout=0.1):
                pass
        except OSError:
            pass
        self.thread.join(timeout=1.0)

    def send(self, payload: dict[str, Any]) -> None:
        self.queue.put(payload)

    def disconnect_client(self) -> None:
        self.queue.put("__disconnect__")

    def _run(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            server.settimeout(0.1)
            while not self.stop_event.is_set():
                try:
                    client, _ = server.accept()
                except socket.timeout:
                    continue
                self.connected_count += 1
                with client:
                    while not self.stop_event.is_set():
                        item = self.queue.get()
                        if item == "__stop__":
                            return
                        if item == "__disconnect__":
                            break
                        line = json.dumps(item, ensure_ascii=False, allow_nan=False)
                        try:
                            client.sendall(line.encode("utf-8") + b"\n")
                        except OSError:
                            break


def run_inside_ros() -> int:
    ensure_dirs()
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
    os.environ["RCL_LOGGING_IMPLEMENTATION"] = "rcl_logging_noop"
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_command_gate"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_trajectory_safety"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_kinematics"))

    append_log("Stage 2D-3B offline verification started inside ROS2 environment.")
    urdf_sha = sha256_file(URDF_PATH)
    mapping = read_json(MAPPING_PATH)
    report: dict[str, Any] = {
        "stage": "2D-3B",
        "status": "FAIL",
        "offline_status": "FAIL",
        "real_hardware_status": "FAIL_BLOCKED_NO_COM_PORT",
        "timestamp": iso_timestamp(),
        "environment": {
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "python": sys.executable,
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "ros_log_dir": os.environ.get("ROS_LOG_DIR", ""),
        },
        "model": {
            "urdf": str(URDF_PATH),
            "urdf_sha256": urdf_sha,
            "urdf_sha256_expected": EXPECTED_URDF_SHA256,
            "urdf_sha256_matches_expected": urdf_sha == EXPECTED_URDF_SHA256,
            "joint_names": EXPECTED_JOINT_NAMES,
        },
        "mapping": {
            "path": str(MAPPING_PATH),
            "follower_port": mapping.get("follower_port"),
            "follower_port_source": mapping.get("follower_port_source"),
            "calibration_path": mapping.get("calibration_path"),
            "status": mapping.get("status"),
            "joint_mappings": mapping.get("joint_mappings"),
        },
        "safety": {
            "opened_com_ports": False,
            "real_so101_connected": False,
            "motor_read_requests_sent": 0,
            "torque_enable_written": False,
            "torque_disable_written": False,
            "goal_position_written": False,
            "motion_parameters_written": False,
            "motion_command_sent": False,
            "shadow_execution_started": False,
            "controller_command_topics_published": [],
            "observed_physical_motion": False,
            "test_only_mock_joint_state_used_as_real": False,
        },
        "failures": [],
        "logs": {
            "main_log": str(LOG_PATH),
            "offline_log": str(OFFLINE_LOG_PATH),
            "offline_report": str(OFFLINE_REPORT_PATH),
        },
    }
    if not report["model"]["urdf_sha256_matches_expected"]:
        fail(report, "urdf_sha256", "Frozen URDF hash changed.")

    try:
        verify_ros_bridge_and_gate(report)
        verify_real_server_safe_block(report)
        verify_executables(report)
    except Exception as exc:
        fail(report, "verification_exception", repr(exc))
        append_log(f"ERROR: {exc!r}")

    offline_passed = bool(
        not report.get("failures")
        and report.get("real_hardware_status") == "FAIL_BLOCKED_NO_COM_PORT"
    )
    report["offline_status"] = "PASS" if offline_passed else "FAIL"
    report["status"] = (
        "FAIL_BLOCKED_NO_COM_PORT"
        if offline_passed
        else "FAIL"
    )
    report["stage_pass_criteria"] = {
        "requires_real_10s_200_frames": True,
        "met": False,
        "reason": "No serial COM ports were visible, so real follower read-only sampling was not run.",
    }
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    OFFLINE_REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    append_log(f"Report written: {REPORT_PATH}")
    append_log(f"Stage 2D-3B offline status: {report['offline_status']}")
    append_log(f"Stage 2D-3B real hardware status: {report['real_hardware_status']}")
    return 0 if offline_passed else 2


def verify_ros_bridge_and_gate(report: dict[str, Any]) -> None:
    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from so101_command_gate.command_gate_node import CommandGateNode
    from so101_command_gate.command_gate_validator import load_command_gate_joint_limits
    from so101_command_gate.real_joint_state_bridge_node import RealJointStateBridgeNode

    class Harness(Node):
        def __init__(self) -> None:
            super().__init__("stage_2d3b_harness")
            self.real_joint_states: list[JointState] = []
            self.real_valid: list[bool] = []
            self.real_status: list[dict[str, Any]] = []
            self.gate_status: list[dict[str, Any]] = []
            self.gate_valid: list[bool] = []
            self.safe_valid_pub = self.create_publisher(Bool, "/safe_timed_grasp_valid", 10)
            self.safe_status_pub = self.create_publisher(String, "/safe_timed_grasp_status", 10)
            self.trajectory_pub = self.create_publisher(
                JointTrajectory,
                "/safe_timed_grasp_trajectory",
                10,
            )
            self.mock_joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.create_subscription(
                JointState,
                "/real_joint_states",
                self.real_joint_states.append,
                50,
            )
            self.create_subscription(
                Bool,
                "/real_joint_state_valid",
                lambda msg: self.real_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/real_joint_state_status",
                self.on_real_status,
                50,
            )
            self.create_subscription(
                Bool,
                "/command_gate_valid",
                lambda msg: self.gate_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(String, "/command_gate_status", self.on_gate_status, 50)

        def on_real_status(self, message: String) -> None:
            self.real_status.append(json.loads(message.data))

        def on_gate_status(self, message: String) -> None:
            self.gate_status.append(json.loads(message.data))

        def publish_safe_inputs(self, trajectory: JointTrajectory, plan_id: str) -> None:
            valid = Bool()
            valid.data = True
            status = String()
            status.data = json.dumps(
                {
                    "status": "VALID",
                    "reason": "stage_2d3b_test_valid",
                    "plan_id": plan_id,
                    "active_plan_id": plan_id,
                    "hardware_control_enabled": False,
                    "published_controller_command_topics": [],
                    "timestamp": time.monotonic(),
                },
                allow_nan=False,
            )
            self.safe_valid_pub.publish(valid)
            self.safe_status_pub.publish(status)
            self.trajectory_pub.publish(trajectory)

        def publish_mock_joint_state(self, positions: np.ndarray) -> None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(EXPECTED_JOINT_NAMES)
            msg.position = [float(value) for value in positions.tolist()]
            msg.velocity = [0.0] * len(EXPECTED_JOINT_NAMES)
            self.mock_joint_state_pub.publish(msg)

    def duration(seconds: float) -> Duration:
        msg = Duration()
        msg.sec = int(math.floor(seconds))
        msg.nanosec = int(round((seconds - msg.sec) * 1.0e9))
        return msg

    def make_trajectory(q0: np.ndarray, plan_id: str) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.frame_id = f"base_link;plan_id={plan_id}"
        trajectory.joint_names = list(EXPECTED_JOINT_NAMES)
        for index, offset in enumerate((0.0, 0.02, 0.03)):
            point = JointTrajectoryPoint()
            q = q0 + np.asarray([offset, 0.0, -offset * 0.5, 0.0, 0.0])
            point.positions = [float(value) for value in q.tolist()]
            point.velocities = [0.0] * len(EXPECTED_JOINT_NAMES)
            point.accelerations = [0.0] * len(EXPECTED_JOINT_NAMES)
            point.time_from_start = duration(float(index) * 0.4)
            trajectory.points.append(point)
        return trajectory

    def payload(sequence: int, positions: np.ndarray, **updates: Any) -> dict[str, Any]:
        base = {
            "type": "joint_state",
            "sequence": sequence,
            "timestamp_monotonic_s": time.monotonic(),
            "joint_names": list(EXPECTED_JOINT_NAMES),
            "positions_rad": [float(value) for value in positions.tolist()],
            "source": "real_so101_follower_read_only",
            "read_only": True,
            "torque_state_changed": False,
            "goal_position_written": False,
            "motion_parameters_written": False,
            "motion_command_sent": False,
            "follower_port": "COM4",
            "calibration_path": str(report["mapping"]["calibration_path"]),
            "mapping_status": "offline_test",
        }
        base.update(updates)
        return base

    def spin_for(executor: MultiThreadedExecutor, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_until(
        executor: MultiThreadedExecutor,
        predicate: Any,
        timeout_s: float = 2.0,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def latest(items: list[Any]) -> Any:
        return items[-1] if items else {}

    def seen_real_reason(reason: str) -> bool:
        return any(item.get("reason") == reason for item in harness.real_status)

    def first_real_status_with_reason(reason: str) -> dict[str, Any]:
        for item in reversed(harness.real_status):
            if item.get("reason") == reason:
                return item
        return latest(harness.real_status)

    lower, upper, metadata = load_command_gate_joint_limits(PROJECT_ROOT)
    q0 = np.radians(np.asarray([0.0, -20.0, 30.0, 80.0, 0.0], dtype=np.float64))
    if float(np.min(np.minimum(q0 - lower, upper - q0))) < 0.12:
        q0 = (lower + upper) * 0.5
    report["model"]["joint_limits_rad"] = metadata["joint_limits_rad"]

    rclpy.init()
    cases: list[dict[str, Any]] = []
    server = JsonLineServer("127.0.0.1", 18766)
    server.start()
    bridge = RealJointStateBridgeNode(
        project_root_override=PROJECT_ROOT,
        port_override=18766,
        timeout_s_override=0.30,
    )
    gate = CommandGateNode(
        project_root_override=PROJECT_ROOT,
        current_joint_state_topic_override="/real_joint_states",
        current_joint_state_valid_topic_override="/real_joint_state_valid",
    )
    harness = Harness()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (bridge, gate, harness):
        executor.add_node(node)
    try:
        trajectory = make_trajectory(q0, "stage2d3b-ready")
        seq = 0
        spin_for(executor, 0.25)

        server.send(payload(seq, q0))
        seq += 1
        ready_seen = False
        real_start = len(harness.real_status)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            harness.publish_safe_inputs(trajectory, "stage2d3b-ready")
            server.send(payload(seq, q0))
            seq += 1
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.03)
            if latest(harness.gate_status).get("status") == "READY":
                ready_seen = True
                break
        real_valid_seen = any(
            status.get("status") == "VALID"
            for status in harness.real_status[real_start:]
        )
        cases.append(
            {
                "name": "normal_json_bridge_and_gate_ready",
                "passed": bool(
                    ready_seen
                    and real_valid_seen
                    and harness.real_joint_states
                ),
                "real_valid_seen": real_valid_seen,
                "latest_real_status": latest(harness.real_status),
                "latest_gate_status": latest(harness.gate_status),
            }
        )
        sample_joint_state = {
            "name": list(harness.real_joint_states[-1].name),
            "position": [float(value) for value in harness.real_joint_states[-1].position],
        }
        report["offline_joint_states_sample"] = sample_joint_state

        bad_cases = [
            (
                "wrong_joint_names",
                payload(seq, q0, joint_names=["bad_joint"] + EXPECTED_JOINT_NAMES[1:]),
                "wrong_joint_names",
            ),
            (
                "wrong_length",
                payload(seq + 1, q0, positions_rad=[0.0, 1.0]),
                "wrong_position_length",
            ),
            (
                "nan_inf",
                payload(seq + 2, q0, positions_rad=[0.0, "NaN", 0.0, 0.0, 0.0]),
                "non_finite_position",
            ),
            (
                "urdf_out_of_bounds",
                payload(
                    seq + 3,
                    q0,
                    positions_rad=[float(upper[0] + 0.25), 0.0, 0.0, 0.0, 0.0],
                ),
                "current_joint_state_out_of_bounds",
            ),
            (
                "sequence_regression",
                payload(1, q0),
                "sequence_regression",
            ),
        ]
        for name, item, reason in bad_cases:
            server.send(item)
            seen = wait_until(
                executor,
                lambda expected=reason: seen_real_reason(expected),
                1.0,
            )
            cases.append(
                {
                    "name": name,
                    "passed": bool(seen),
                    "expected_reason": reason,
                    "matching_real_status": first_real_status_with_reason(reason),
                }
            )
        seq += 4

        server.send(payload(seq, q0))
        timeout_sequence = seq
        seq += 1
        wait_until(
            executor,
            lambda: latest(harness.real_status).get("sequence") == timeout_sequence,
            1.0,
        )
        spin_for(executor, 0.45)
        timeout_seen = latest(harness.real_status).get("reason") == "real_joint_state_timeout"
        cases.append(
            {
                "name": "timeout",
                "passed": bool(timeout_seen and (harness.real_valid[-1] is False)),
                "latest_real_status": latest(harness.real_status),
            }
        )

        server.disconnect_client()
        disconnected = wait_until(
            executor,
            lambda: seen_real_reason("tcp_disconnected"),
            1.0,
        )
        cases.append(
            {
                "name": "tcp_disconnect",
                "passed": bool(disconnected),
                "matching_real_status": first_real_status_with_reason("tcp_disconnected"),
            }
        )

        server.send(payload(seq, q0))
        reconnect_sequence = seq
        seq += 1
        reconnected = wait_until(
            executor,
            lambda: latest(harness.real_status).get("sequence") == reconnect_sequence
            and latest(harness.real_status).get("status") == "VALID",
            2.0,
        )
        cases.append(
            {
                "name": "reconnect",
                "passed": bool(reconnected and server.connected_count >= 2),
                "connected_count": server.connected_count,
                "latest_real_status": latest(harness.real_status),
            }
        )

        spin_for(executor, 0.45)
        mock_only_trajectory = make_trajectory(q0 + 0.20, "stage2d3b-mock-isolation")
        deadline = time.monotonic() + 0.75
        while time.monotonic() < deadline:
            harness.publish_safe_inputs(mock_only_trajectory, "stage2d3b-mock-isolation")
            harness.publish_mock_joint_state(q0 + 0.20)
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.03)
        mock_isolation_ok = latest(harness.gate_status).get("status") == "INVALID"
        cases.append(
            {
                "name": "mock_real_topic_isolation",
                "passed": bool(mock_isolation_ok),
                "latest_gate_status": latest(harness.gate_status),
            }
        )

        real_trajectory = make_trajectory(q0, "stage2d3b-real-source")
        first_real_sequence = seq
        ready_after_real_source = False
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            harness.publish_safe_inputs(real_trajectory, "stage2d3b-real-source")
            server.send(payload(seq, q0))
            seq += 1
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.03)
            if latest(harness.gate_status).get("status") == "READY":
                ready_after_real_source = True
                break
        source_valid_propagated = wait_until(
            executor,
            lambda: ready_after_real_source
            or latest(harness.gate_status).get("status") == "READY",
            0.5,
        )
        cases.append(
            {
                "name": "command_gate_reads_real_joint_states",
                "passed": bool(source_valid_propagated),
                "first_real_sequence": first_real_sequence,
                "last_real_sequence": seq - 1,
                "latest_real_status": latest(harness.real_status),
                "latest_gate_status": latest(harness.gate_status),
            }
        )
    finally:
        for node in (bridge, gate, harness):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        server.stop()
        if rclpy.ok():
            rclpy.shutdown()

    report["offline_cases"] = cases
    failed = [item["name"] for item in cases if not item.get("passed")]
    if failed:
        fail(report, "offline_cases", f"One or more 2D-3B offline cases failed: {failed}")


def verify_real_server_safe_block(report: dict[str, Any]) -> None:
    code, output = command_output(
        [
            str(LEROBOT_PYTHON),
            str(READ_ONLY_SERVER),
            "--mapping",
            str(MAPPING_PATH),
            "--follower-port",
            "COM_DOES_NOT_EXIST_2D3B_TEST",
            "--preflight-only",
            "--bind-port",
            "18767",
        ],
        timeout_s=20.0,
    )
    report["real_server_no_com_safe_block"] = {
        "return_code": code,
        "output": output[-4000:],
        "passed": code != 0
        and "READ_ONLY_PREFLIGHT_FAIL" in output
        and "configured_port_missing" in output,
    }
    if not report["real_server_no_com_safe_block"]["passed"]:
        fail(report, "real_server_safe_block", "Read-only server did not fail closed without COM port.")


def verify_executables(report: dict[str, Any]) -> None:
    code, output = command_output(
        [
            "cmd.exe",
            "/d",
            "/s",
            "/c",
            "cd /d "
            + str(ROS2_WS)
            + " && ros2 pkg executables so101_command_gate",
        ],
        timeout_s=20.0,
    )
    report["executables"] = {
        "return_code": code,
        "raw_output": output,
        "command_gate_node": "so101_command_gate command_gate_node" in output,
        "shadow_executor_node": "so101_command_gate shadow_executor_node" in output,
        "mock_joint_state_publisher": (
            "so101_command_gate mock_joint_state_publisher" in output
        ),
        "real_joint_state_bridge_node": (
            "so101_command_gate real_joint_state_bridge_node" in output
        ),
    }
    for executable_name in (
        "command_gate_node",
        "shadow_executor_node",
        "mock_joint_state_publisher",
        "real_joint_state_bridge_node",
    ):
        if not report["executables"][executable_name]:
            fail(report, executable_name, "Executable is not registered.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify Stage 2D-3B real read-only joint state bridge offline."
    )
    parser.add_argument("--inside-ros", action="store_true")
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros()
    return run_outside_ros()


if __name__ == "__main__":
    raise SystemExit(main())
