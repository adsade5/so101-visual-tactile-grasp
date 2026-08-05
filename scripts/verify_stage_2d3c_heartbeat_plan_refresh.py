from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_WS = PROJECT_ROOT / "ros2_ws"
RUN_IN_ROS2 = PROJECT_ROOT / "audit" / "run_in_ros2_lyrical.ps1"
ROS2_PYTHON = Path(r"C:\pixi_ws\.pixi\envs\default\python.exe")
REPORT_DIR = PROJECT_ROOT / "data" / "verification"
REPORT_PATH = REPORT_DIR / "stage_2d3c_heartbeat_refresh_report.json"
LOG_PATH = REPORT_DIR / "stage_2d3c_heartbeat_refresh.log"
ROS_LOG_DIR = REPORT_DIR / "ros_logs_heartbeat_refresh"
URDF_PATH = PROJECT_ROOT / "data" / "robot_model" / "so101" / "so101_new_calib.urdf"
EXPECTED_URDF_SHA256 = "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
EXPECTED_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
FORBIDDEN_CONTROLLER_TOPICS = [
    "/joint_trajectory_controller/joint_trajectory",
    "/joint_trajectory_controller/follow_joint_trajectory",
    "/arm_controller/command",
    "/robot_command",
    "/hardware_command",
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
    if LOG_PATH.exists():
        LOG_PATH.unlink()
    command_file = Path(tempfile.gettempdir()) / (
        f"verify_stage_2d3c_heartbeat_{filename_timestamp()}.cmd"
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
        append_log("Entering ROS2 Lyrical environment for heartbeat refresh verification.")
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


def command_output(command: list[str], timeout_s: float = 240.0) -> tuple[int, str]:
    completed = subprocess.run(
        command,
        cwd=str(ROS2_WS),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )
    return int(completed.returncode), completed.stdout


def run_stage_regression(
    report: dict[str, Any],
    *,
    name: str,
    script: Path,
    report_path: Path,
    timeout_s: float,
) -> None:
    append_log(f"Running {name} regression.")
    code, output = command_output(
        [str(ROS2_PYTHON), "-X", "faulthandler", str(script), "--inside-ros"],
        timeout_s=timeout_s,
    )
    status = "MISSING"
    offline_status = None
    failures: list[dict[str, Any]] = []
    if report_path.is_file():
        child = read_json(report_path)
        status = str(child.get("status"))
        offline_status = child.get("offline_status")
        failures = list(child.get("failures", []))
    passed = bool(code == 0 and not failures and (status == "PASS" or offline_status == "PASS"))
    report[name] = {
        "return_code": code,
        "status": "PASS" if passed else "FAIL",
        "report": str(report_path),
        "report_status": status,
        "offline_status": offline_status,
        "report_failures": failures,
        "log_excerpt": output[-3000:],
        "passed": passed,
    }
    if not passed:
        fail(report, name, f"{name} regression did not pass.")


def run_inside_ros() -> int:
    ensure_dirs()
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
    os.environ["RCL_LOGGING_IMPLEMENTATION"] = "rcl_logging_noop"
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_frame_transform"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_grasp_planner"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_trajectory_safety"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_command_gate"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_kinematics"))

    append_log("Heartbeat refresh verification started inside ROS2 environment.")
    urdf_sha = sha256_file(URDF_PATH)
    report: dict[str, Any] = {
        "stage": "2D-3C heartbeat refresh",
        "timestamp": iso_timestamp(),
        "final_status": "FAIL",
        "root_cause": (
            "object_pose_base_valid/grasp_plan_valid status heartbeats and timed "
            "plan_id-payload pairing could expire independently of live object_pose payloads."
        ),
        "modified_files": [
            "ros2_ws/src/so101_frame_transform/so101_frame_transform/workspace_to_base_node.py",
            "ros2_ws/src/so101_grasp_planner/so101_grasp_planner/visual_grasp_planner_node.py",
            "ros2_ws/src/so101_trajectory_safety/so101_trajectory_safety/timed_trajectory_node.py",
            "ros2_ws/src/so101_kinematics/so101_kinematics/urdf_fk.py",
            "ros2_ws/src/so101_kinematics/so101_kinematics/top_down_ik.py",
            "scripts/verify_stage_2d1_visual_to_ik.py",
            "scripts/verify_stage_2d2_timed_trajectory.py",
            "scripts/verify_stage_2d3c_connection_shadow.py",
            "scripts/verify_stage_2d3c_heartbeat_plan_refresh.py",
        ],
        "heartbeat_topics": [
            "/object_pose_base_valid",
            "/grasp_plan_valid",
            "/grasp_plan_status",
            "/safe_timed_grasp_valid",
            "/safe_timed_grasp_status",
        ],
        "heartbeat_rates": {
            "/object_pose_base_valid": "10 Hz",
            "/grasp_plan_valid": "10 Hz",
            "/grasp_plan_status": "10 Hz",
            "/safe_timed_grasp_valid": "20 Hz",
            "/safe_timed_grasp_status": "20 Hz",
        },
        "payload_topics": [
            "/object_pose_base",
            "/planned_grasp_joint_trajectory",
            "/safe_timed_grasp_trajectory",
        ],
        "model": {
            "urdf": str(URDF_PATH),
            "urdf_sha256": urdf_sha,
            "urdf_sha256_expected": EXPECTED_URDF_SHA256,
            "urdf_sha256_matches_expected": urdf_sha == EXPECTED_URDF_SHA256,
        },
        "safety": {
            "hardware_control_enabled": False,
            "motion_command_sent": False,
            "goal_position_written": False,
            "torque_enable_written": False,
            "torque_disable_written": False,
            "real_controller_topics_published": [],
            "shadow_execution_only": True,
            "forbidden_controller_topics": FORBIDDEN_CONTROLLER_TOPICS,
        },
        "failures": [],
        "logs": {"main_log": str(LOG_PATH), "ros_log_dir": str(ROS_LOG_DIR)},
    }
    if urdf_sha != EXPECTED_URDF_SHA256:
        fail(report, "urdf_sha256", "Frozen URDF hash changed.")
    try:
        verify_with_ros(report)
        run_stage_regression(
            report,
            name="stage_2d2_regression",
            script=PROJECT_ROOT / "scripts" / "verify_stage_2d2_timed_trajectory.py",
            report_path=REPORT_DIR / "stage_2d2_report.json",
            timeout_s=300.0,
        )
        run_stage_regression(
            report,
            name="stage_2d3a_regression",
            script=PROJECT_ROOT / "scripts" / "verify_stage_2d3a_command_gate_shadow.py",
            report_path=REPORT_DIR / "stage_2d3a_report.json",
            timeout_s=420.0,
        )
        run_stage_regression(
            report,
            name="stage_2d3b_offline_regression",
            script=PROJECT_ROOT / "scripts" / "verify_stage_2d3b_real_joint_state_bridge.py",
            report_path=REPORT_DIR / "stage_2d3b_report.json",
            timeout_s=300.0,
        )
        run_stage_regression(
            report,
            name="stage_2d3c_regression",
            script=PROJECT_ROOT / "scripts" / "verify_stage_2d3c_connection_shadow.py",
            report_path=REPORT_DIR / "stage_2d3c_report.json",
            timeout_s=420.0,
        )
    except Exception as exc:
        fail(report, "verification_exception", repr(exc))
        append_log(f"ERROR: {exc!r}")
    finally:
        report["final_status"] = "PASS" if not report.get("failures") else "FAIL"
        REPORT_PATH.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
        append_log(f"Report written: {REPORT_PATH}")
        append_log(f"Heartbeat refresh final status: {report['final_status']}")
    return 0 if report["final_status"] == "PASS" else 2


def verify_with_ros(report: dict[str, Any]) -> None:
    import rclpy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from so101_command_gate.command_gate_node import CommandGateNode
    from so101_command_gate.connection_trajectory_node import ConnectionTrajectoryNode
    from so101_command_gate.shadow_executor_node import ShadowExecutorNode
    from so101_frame_transform.workspace_to_base_node import WorkspaceToBaseNode
    from so101_grasp_planner.visual_grasp_planner_node import VisualGraspPlannerNode
    from so101_trajectory_safety.timed_trajectory_node import (
        DEFAULT_PLANNER_CONFIG_VERSION,
        TimedTrajectoryNode,
        trajectory_hash,
    )

    def duration_from_seconds(seconds: float) -> Duration:
        total_ns = int(round(seconds * 1.0e9))
        message = Duration()
        message.sec = total_ns // 1_000_000_000
        message.nanosec = total_ns % 1_000_000_000
        return message

    def base_to_workspace(base_position: np.ndarray, config: dict[str, Any]) -> np.ndarray:
        tx, ty, tz = [float(value) for value in config["translation_m"]]
        yaw = math.radians(float(config["yaw_deg"]))
        dx = float(base_position[0]) - tx
        dy = float(base_position[1]) - ty
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return np.asarray(
            [cosine * dx + sine * dy, -sine * dx + cosine * dy, float(base_position[2]) - tz],
            dtype=np.float64,
        )

    class FullChainHarness(Node):
        def __init__(self) -> None:
            super().__init__("stage_2d3c_heartbeat_full_chain_harness")
            self.base_valid: list[bool] = []
            self.grasp_valid: list[bool] = []
            self.safe_valid: list[bool] = []
            self.connection_valid: list[bool] = []
            self.gate_valid: list[bool] = []
            self.base_status: list[dict[str, Any]] = []
            self.grasp_status: list[dict[str, Any]] = []
            self.safe_status: list[dict[str, Any]] = []
            self.connection_status: list[dict[str, Any]] = []
            self.gate_status: list[dict[str, Any]] = []
            self.shadow_status: list[dict[str, Any]] = []
            self.safe_trajectories: list[JointTrajectory] = []
            self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", 10)
            self.detected_pub = self.create_publisher(Bool, "/object_detected", 10)
            self.stable_pub = self.create_publisher(Bool, "/object_pose_stable", 10)
            self.real_pub = self.create_publisher(JointState, "/real_joint_states", 10)
            self.real_valid_pub = self.create_publisher(Bool, "/real_joint_state_valid", 10)
            self.start_client = self.create_client(Trigger, "/start_shadow_execution")
            self.create_subscription(Bool, "/object_pose_base_valid", lambda msg: self.base_valid.append(bool(msg.data)), 50)
            self.create_subscription(Bool, "/grasp_plan_valid", lambda msg: self.grasp_valid.append(bool(msg.data)), 50)
            self.create_subscription(Bool, "/safe_timed_grasp_valid", lambda msg: self.safe_valid.append(bool(msg.data)), 50)
            self.create_subscription(Bool, "/connection_plan_valid", lambda msg: self.connection_valid.append(bool(msg.data)), 50)
            self.create_subscription(Bool, "/command_gate_valid", lambda msg: self.gate_valid.append(bool(msg.data)), 50)
            self.create_subscription(String, "/object_pose_base_status", lambda msg: self.base_status.append(self.decode(msg)), 50)
            self.create_subscription(String, "/grasp_plan_status", lambda msg: self.grasp_status.append(self.decode(msg)), 50)
            self.create_subscription(String, "/safe_timed_grasp_status", lambda msg: self.safe_status.append(self.decode(msg)), 50)
            self.create_subscription(String, "/connection_plan_status", lambda msg: self.connection_status.append(self.decode(msg)), 50)
            self.create_subscription(String, "/command_gate_status", lambda msg: self.gate_status.append(self.decode(msg)), 50)
            self.create_subscription(String, "/shadow_execution_status", lambda msg: self.shadow_status.append(self.decode(msg)), 50)
            self.create_subscription(JointTrajectory, "/safe_timed_grasp_trajectory", self.safe_trajectories.append, 50)

        def decode(self, message: String) -> dict[str, Any]:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                return {"status": "INVALID_JSON", "raw": message.data}
            return value if isinstance(value, dict) else {"status": "NOT_OBJECT"}

        def publish_object(self, workspace_position: np.ndarray, detected: bool = True, stable: bool = True) -> None:
            detected_msg = Bool()
            detected_msg.data = bool(detected)
            self.detected_pub.publish(detected_msg)
            stable_msg = Bool()
            stable_msg.data = bool(stable)
            self.stable_pub.publish(stable_msg)
            if detected:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "workspace_plane"
                pose.pose.position.x = float(workspace_position[0])
                pose.pose.position.y = float(workspace_position[1])
                pose.pose.position.z = float(workspace_position[2])
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

        def publish_real_from_safe_start(self) -> None:
            valid = Bool()
            valid.data = True
            self.real_valid_pub.publish(valid)
            if not self.safe_trajectories or not self.safe_trajectories[-1].points:
                return
            point = self.safe_trajectories[-1].points[0]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(EXPECTED_JOINT_NAMES)
            msg.position = [float(value) for value in point.positions]
            msg.velocity = [0.0] * len(EXPECTED_JOINT_NAMES)
            self.real_pub.publish(msg)

    class TimedHarness(Node):
        def __init__(self, name: str) -> None:
            super().__init__(name)
            self.safe_valid: list[bool] = []
            self.safe_status: list[dict[str, Any]] = []
            self.safe_trajectories: list[JointTrajectory] = []
            self.valid_pub = self.create_publisher(Bool, "/grasp_plan_valid", 10)
            self.status_pub = self.create_publisher(String, "/grasp_plan_status", 10)
            self.payload_pub = self.create_publisher(JointTrajectory, "/planned_grasp_joint_trajectory", 10)
            self.create_subscription(Bool, "/safe_timed_grasp_valid", lambda msg: self.safe_valid.append(bool(msg.data)), 50)
            self.create_subscription(String, "/safe_timed_grasp_status", lambda msg: self.safe_status.append(self.decode(msg)), 50)
            self.create_subscription(JointTrajectory, "/safe_timed_grasp_trajectory", self.safe_trajectories.append, 50)

        def decode(self, message: String) -> dict[str, Any]:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                return {"status": "INVALID_JSON", "raw": message.data}
            return value if isinstance(value, dict) else {"status": "NOT_OBJECT"}

        def publish_valid(self, value: bool = True) -> None:
            msg = Bool()
            msg.data = bool(value)
            self.valid_pub.publish(msg)

        def publish_status(self, plan_id: str, hash_value: str, status: str = "VALID") -> None:
            msg = String()
            active = plan_id if status == "VALID" else None
            msg.data = json.dumps(
                {
                    "status": status,
                    "reason": "planned_preview_only" if status == "VALID" else "test_invalid",
                    "plan_id": active,
                    "active_plan_id": active,
                    "latest_generated_plan_id": plan_id,
                    "trajectory_hash": hash_value if status == "VALID" else None,
                    "trajectory_point_count": 3,
                    "trajectory_payload_publish_seq": 1,
                    "trajectory_payload_publish_timestamp": time.monotonic(),
                    "planner_config_version": DEFAULT_PLANNER_CONFIG_VERSION,
                    "planner_heartbeat_seq": 1,
                    "cached_plan_available": status == "VALID",
                    "timestamp": time.monotonic(),
                    "hardware_control_enabled": False,
                    "command_topics_published": [],
                },
                allow_nan=False,
            )
            self.status_pub.publish(msg)

        def publish_payload(self, trajectory: JointTrajectory) -> None:
            self.payload_pub.publish(trajectory)

    def spin_for(executor: MultiThreadedExecutor, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_until(executor: MultiThreadedExecutor, predicate: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def latest(values: list[dict[str, Any]]) -> dict[str, Any]:
        return values[-1] if values else {}

    def make_source_trajectory(plan_id: str, offset: float = 0.0) -> tuple[JointTrajectory, str]:
        q0 = np.radians(np.asarray([0.0, -20.0, 20.0, 70.0, 0.0], dtype=np.float64)) + offset
        q1 = q0 + np.asarray([0.0, 0.0, 0.0, 0.05, 0.0], dtype=np.float64)
        q2 = q0 + np.asarray([0.0, 0.05, 0.0, 0.0, 0.0], dtype=np.float64)
        positions = [q0.tolist(), q1.tolist(), q2.tolist()]
        hash_value = trajectory_hash(EXPECTED_JOINT_NAMES, positions)
        trajectory = JointTrajectory()
        trajectory.header.frame_id = f"base_link;plan_id={plan_id};trajectory_hash={hash_value}"
        trajectory.joint_names = list(EXPECTED_JOINT_NAMES)
        for index, q in enumerate((q0, q1, q2)):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in q]
            point.time_from_start = duration_from_seconds(0.4 * index)
            trajectory.points.append(point)
        return trajectory, hash_value

    def run_direct_timed_case(name: str, body: Any) -> dict[str, Any]:
        timed = TimedTrajectoryNode()
        harness = TimedHarness(f"stage_2d3c_heartbeat_timed_{name}")
        executor = MultiThreadedExecutor(num_threads=2)
        for node in (timed, harness):
            executor.add_node(node)
        try:
            spin_for(executor, 0.25)
            return body(executor, harness)
        finally:
            for node in (timed, harness):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()

    def timed_ready(harness: TimedHarness) -> bool:
        return latest(harness.safe_status).get("status") == "VALID" and bool(harness.safe_valid[-1] if harness.safe_valid else False)

    rclpy.init()
    try:
        config = read_json(PROJECT_ROOT / "config" / "workspace_to_base.json")
        base_a = np.asarray([0.18289733886666235, 0.0, 0.025], dtype=np.float64)
        base_b = base_a + np.asarray([0.006, 0.0, 0.0], dtype=np.float64)
        workspace_a = base_to_workspace(base_a, config)
        workspace_b = base_to_workspace(base_b, config)

        workspace = WorkspaceToBaseNode()
        planner = VisualGraspPlannerNode()
        timed = TimedTrajectoryNode()
        connection = ConnectionTrajectoryNode(project_root_override=PROJECT_ROOT)
        gate = CommandGateNode(
            project_root_override=PROJECT_ROOT,
            current_joint_state_topic_override="/real_joint_states",
            current_joint_state_valid_topic_override="/real_joint_state_valid",
            source_trajectory_topic_override="/connected_safe_timed_grasp_trajectory",
            source_valid_topic_override="/connection_plan_valid",
            source_status_topic_override="/connection_plan_status",
        )
        shadow = ShadowExecutorNode()
        harness = FullChainHarness()
        executor = MultiThreadedExecutor(num_threads=7)
        for node in (workspace, planner, timed, connection, gate, shadow, harness):
            executor.add_node(node)
        try:
            spin_for(executor, 0.5)

            def pump_object(position: np.ndarray, seconds: float, detected: bool = True, stable: bool = True) -> None:
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    harness.publish_object(position, detected=detected, stable=stable)
                    harness.publish_real_from_safe_start()
                    spin_for(executor, 0.06)

            def pump_until(
                position: np.ndarray,
                predicate: Any,
                timeout_s: float,
                detected: bool = True,
                stable: bool = True,
            ) -> bool:
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    harness.publish_object(position, detected=detected, stable=stable)
                    harness.publish_real_from_safe_start()
                    spin_for(executor, 0.06)
                    if predicate():
                        return True
                return False

            def all_valid() -> bool:
                return bool(
                    (harness.base_valid[-1] if harness.base_valid else False)
                    and (harness.grasp_valid[-1] if harness.grasp_valid else False)
                    and (harness.safe_valid[-1] if harness.safe_valid else False)
                    and (harness.connection_valid[-1] if harness.connection_valid else False)
                    and (harness.gate_valid[-1] if harness.gate_valid else False)
                )

            ready_a = pump_until(workspace_a, all_valid, 18.0)
            status_a = latest(harness.safe_status)
            plan_a = status_a.get("active_plan_id")
            hash_a = status_a.get("active_trajectory_hash")
            pump_object(workspace_a, 30.0)
            static_ready = pump_until(workspace_a, all_valid, 3.0)
            static_result = {
                "passed": bool(ready_a and static_ready and latest(harness.safe_status).get("active_plan_id") == plan_a and latest(harness.safe_status).get("active_trajectory_hash") == hash_a),
                "active_plan_id": latest(harness.safe_status).get("active_plan_id"),
                "trajectory_hash": latest(harness.safe_status).get("active_trajectory_hash"),
                "safe_status": latest(harness.safe_status),
                "connection_status": latest(harness.connection_status),
            }

            moved = pump_until(
                workspace_b,
                lambda: all_valid()
                and latest(harness.safe_status).get("active_plan_id") not in (None, plan_a)
                and latest(harness.safe_status).get("active_trajectory_hash") not in (None, hash_a),
                18.0,
            )
            plan_b = latest(harness.safe_status).get("active_plan_id")
            hash_b = latest(harness.safe_status).get("active_trajectory_hash")
            pump_object(workspace_b, 5.0)
            move_stable = pump_until(
                workspace_b,
                lambda: all_valid()
                and latest(harness.safe_status).get("active_plan_id") == plan_b
                and latest(harness.safe_status).get("active_trajectory_hash") == hash_b,
                3.0,
            )
            move_result = {
                "passed": bool(moved and move_stable and plan_b != plan_a and hash_b != hash_a),
                "old_plan_id": plan_a,
                "new_plan_id": plan_b,
                "old_trajectory_hash": hash_a,
                "new_trajectory_hash": hash_b,
                "safe_status": latest(harness.safe_status),
            }

            pump_object(workspace_b, 0.8)
            started = False
            if harness.start_client.wait_for_service(timeout_sec=1.0):
                future = harness.start_client.call_async(Trigger.Request())
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    pump_object(workspace_b, 0.06)
                    if future.done():
                        started = bool(future.result().success)
                        break
            pump_object(workspace_b, 1.0, detected=False, stable=False)
            removed = wait_until(
                executor,
                lambda: latest(harness.safe_status).get("status") == "INVALID"
                and latest(harness.connection_status).get("status") == "INVALID"
                and latest(harness.gate_status).get("status") == "INVALID",
                3.0,
            )
            remove_result = {
                "passed": bool(removed),
                "shadow_started": started,
                "base_valid": harness.base_valid[-1] if harness.base_valid else None,
                "grasp_valid": harness.grasp_valid[-1] if harness.grasp_valid else None,
                "safe_valid": harness.safe_valid[-1] if harness.safe_valid else None,
                "connection_valid": harness.connection_valid[-1] if harness.connection_valid else None,
                "command_gate_valid": harness.gate_valid[-1] if harness.gate_valid else None,
                "shadow_status": latest(harness.shadow_status),
            }

            pump_object(workspace_a, 4.0)
            returned = pump_until(
                workspace_a,
                lambda: all_valid()
                and latest(harness.safe_status).get("active_plan_id") not in (None, plan_b)
                and latest(harness.safe_status).get("active_trajectory_hash") not in (None, hash_b),
                18.0,
            )
            return_result = {
                "passed": bool(returned),
                "return_plan_id": latest(harness.safe_status).get("active_plan_id"),
                "return_trajectory_hash": latest(harness.safe_status).get("active_trajectory_hash"),
                "safe_status": latest(harness.safe_status),
                "connection_status": latest(harness.connection_status),
            }

            report["static_30s_result"] = static_result
            report["move_A_to_B_result"] = move_result
            report["remove_result"] = remove_result
            report["return_result"] = return_result
            report["active_plan_id"] = latest(harness.safe_status).get("active_plan_id")
            report["pending_plan_id"] = latest(harness.safe_status).get("pending_plan_id")
            report["trajectory_hash"] = latest(harness.safe_status).get("active_trajectory_hash")
        finally:
            for node in (workspace, planner, timed, connection, gate, shadow, harness):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()

        def case_status_before_payload(executor: MultiThreadedExecutor, harness: TimedHarness) -> dict[str, Any]:
            trajectory, hash_value = make_source_trajectory("status-before-payload")
            harness.publish_valid(True)
            harness.publish_status("status-before-payload", hash_value)
            spin_for(executor, 0.2)
            harness.publish_payload(trajectory)
            ok = wait_until(executor, lambda: timed_ready(harness), 2.0)
            return {"name": "status_before_payload", "passed": bool(ok), "latest_status": latest(harness.safe_status)}

        def case_payload_before_status(executor: MultiThreadedExecutor, harness: TimedHarness) -> dict[str, Any]:
            trajectory, hash_value = make_source_trajectory("payload-before-status")
            harness.publish_payload(trajectory)
            spin_for(executor, 0.2)
            harness.publish_valid(True)
            harness.publish_status("payload-before-status", hash_value)
            ok = wait_until(executor, lambda: timed_ready(harness), 2.0)
            return {"name": "payload_before_status", "passed": bool(ok), "latest_status": latest(harness.safe_status)}

        def case_stale_pending(executor: MultiThreadedExecutor, harness: TimedHarness) -> dict[str, Any]:
            _, hash_value = make_source_trajectory("stale-pending")
            deadline = time.monotonic() + 1.2
            while time.monotonic() < deadline:
                harness.publish_valid(True)
                harness.publish_status("stale-pending", hash_value)
                spin_for(executor, 0.08)
            ok = wait_until(executor, lambda: latest(harness.safe_status).get("reason") == "pending_plan_timeout", 2.0)
            return {"name": "stale_pending_plan", "passed": bool(ok), "latest_status": latest(harness.safe_status)}

        def case_duplicate(executor: MultiThreadedExecutor, harness: TimedHarness) -> dict[str, Any]:
            trajectory, hash_value = make_source_trajectory("duplicate")
            for _ in range(8):
                harness.publish_valid(True)
                harness.publish_status("duplicate", hash_value)
                harness.publish_payload(trajectory)
                spin_for(executor, 0.08)
            ok = wait_until(executor, lambda: timed_ready(harness), 2.0)
            count = int(latest(harness.safe_status).get("reparameterization_count", -1))
            return {"name": "duplicate_payload_and_status", "passed": bool(ok and count == 1), "reparameterization_count": count, "latest_status": latest(harness.safe_status)}

        def case_hash_mismatch(executor: MultiThreadedExecutor, harness: TimedHarness) -> dict[str, Any]:
            trajectory, hash_value = make_source_trajectory("mismatch")
            wrong_hash = "0" * len(hash_value)
            harness.publish_valid(True)
            harness.publish_status("mismatch", wrong_hash)
            spin_for(executor, 0.2)
            harness.publish_payload(trajectory)
            ok = wait_until(executor, lambda: latest(harness.safe_status).get("reason") == "plan_payload_hash_mismatch", 2.0)
            return {"name": "plan_id_hash_mismatch", "passed": bool(ok), "latest_status": latest(harness.safe_status)}

        def case_heartbeat_stops(executor: MultiThreadedExecutor, harness: TimedHarness) -> dict[str, Any]:
            trajectory, hash_value = make_source_trajectory("heartbeat-stops")
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                harness.publish_valid(True)
                harness.publish_status("heartbeat-stops", hash_value)
                harness.publish_payload(trajectory)
                spin_for(executor, 0.08)
            ready = wait_until(executor, lambda: timed_ready(harness), 2.0)
            spin_for(executor, 1.2)
            invalid = latest(harness.safe_status).get("status") == "INVALID"
            reason = latest(harness.safe_status).get("reason")
            return {"name": "heartbeat_stops", "passed": bool(ready and invalid and "stale" in str(reason)), "latest_status": latest(harness.safe_status)}

        direct_cases = [
            run_direct_timed_case("status_before_payload", case_status_before_payload),
            run_direct_timed_case("payload_before_status", case_payload_before_status),
            run_direct_timed_case("stale_pending", case_stale_pending),
            run_direct_timed_case("duplicate", case_duplicate),
            run_direct_timed_case("hash_mismatch", case_hash_mismatch),
            run_direct_timed_case("heartbeat_stops", case_heartbeat_stops),
        ]
        report["status_before_payload_result"] = direct_cases[0]
        report["payload_before_status_result"] = direct_cases[1]
        report["stale_pending_plan_result"] = direct_cases[2]
        report["duplicate_payload_and_status_result"] = direct_cases[3]
        report["plan_hash_mismatch_result"] = direct_cases[4]
        report["heartbeat_stops_result"] = direct_cases[5]

        all_cases = [
            report["static_30s_result"],
            report["move_A_to_B_result"],
            report["remove_result"],
            report["return_result"],
            *direct_cases,
        ]
        if not all(case.get("passed") for case in all_cases):
            fail(report, "heartbeat_refresh_cases", "One or more heartbeat refresh cases failed.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inside-ros", action="store_true")
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros()
    return run_outside_ros()


if __name__ == "__main__":
    raise SystemExit(main())
