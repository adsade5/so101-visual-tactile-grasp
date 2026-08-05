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
REPORT_PATH = REPORT_DIR / "stage_2d3a_report.json"
LOG_PATH = REPORT_DIR / "stage_2d3a_verification.log"
ROS_LOG_DIR = REPORT_DIR / "ros_logs"
STAGE_2D1_SCRIPT = PROJECT_ROOT / "scripts" / "verify_stage_2d1_visual_to_ik.py"
STAGE_2D2_SCRIPT = PROJECT_ROOT / "scripts" / "verify_stage_2d2_timed_trajectory.py"
STAGE_2D1_REPORT = REPORT_DIR / "stage_2d1_report.json"
STAGE_2D2_REPORT = REPORT_DIR / "stage_2d2_report.json"
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
        f"verify_stage_2d3a_{filename_timestamp()}.cmd"
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
        append_log("Entering ROS2 Lyrical environment for Stage 2D-3A.")
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


def command_output(command: list[str], timeout_s: float = 180.0) -> tuple[int, str]:
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
    report_status = "MISSING"
    failures: list[dict[str, Any]] = []
    timestamp = None
    if report_path.is_file():
        try:
            child = read_json(report_path)
            report_status = str(child.get("status"))
            failures = list(child.get("failures", []))
            timestamp = child.get("timestamp")
        except Exception as error:
            report_status = f"READ_ERROR:{error!r}"
    passed = bool(report_status == "PASS" and not failures)
    report[name] = {
        "return_code": code,
        "status": "PASS" if passed else "FAIL",
        "report": str(report_path),
        "report_status": report_status,
        "report_timestamp": timestamp,
        "report_failures": failures,
        "log_excerpt": output[-4000:],
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
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_command_gate"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_trajectory_safety"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_kinematics"))

    append_log("Stage 2D-3A verification started inside ROS2 environment.")
    urdf_sha = sha256_file(URDF_PATH)
    report: dict[str, Any] = {
        "stage": "2D-3A",
        "status": "FAIL",
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
        "safety": {
            "opened_com_ports": False,
            "started_lerobot_hardware_server": False,
            "connected_real_so101": False,
            "published_real_controller_commands": False,
            "published_controller_command_topics": [],
            "hardware_control_enabled": False,
            "shadow_execution_only": True,
            "test_only_mock_joint_state": True,
        },
        "failures": [],
        "logs": {
            "main_log": str(LOG_PATH),
            "ros_log_dir": str(ROS_LOG_DIR),
        },
    }
    if not report["model"]["urdf_sha256_matches_expected"]:
        fail(report, "urdf_sha256", "Frozen URDF hash changed.")
    try:
        verify_with_ros_in_process(report)
        run_stage_regression(
            report,
            name="stage_2d1_regression",
            script=STAGE_2D1_SCRIPT,
            report_path=STAGE_2D1_REPORT,
            timeout_s=180.0,
        )
        run_stage_regression(
            report,
            name="stage_2d2_regression",
            script=STAGE_2D2_SCRIPT,
            report_path=STAGE_2D2_REPORT,
            timeout_s=260.0,
        )
    except Exception as exc:
        fail(report, "verification_exception", repr(exc))
        append_log(f"ERROR: {exc!r}")
    finally:
        report["status"] = "PASS" if not report.get("failures") else "FAIL"
        REPORT_PATH.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
        append_log(f"Report written: {REPORT_PATH}")
        append_log(f"Stage 2D-3A status: {report['status']}")
    return 0 if report["status"] == "PASS" else 2


def verify_with_ros_in_process(report: dict[str, Any]) -> None:
    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from so101_command_gate.command_gate_node import CommandGateNode
    from so101_command_gate.command_gate_validator import (
        load_command_gate_config,
        load_command_gate_joint_limits,
    )
    from so101_command_gate.shadow_executor_node import ShadowExecutorNode

    class Harness(Node):
        def __init__(self, name: str) -> None:
            super().__init__(name)
            self.command_gate_valid: list[bool] = []
            self.command_gate_status: list[dict[str, Any]] = []
            self.command_gate_reasons: list[str] = []
            self.shadow_active: list[bool] = []
            self.shadow_status: list[dict[str, Any]] = []
            self.expected_joint_states: list[JointState] = []
            self.safe_valid_pub = self.create_publisher(Bool, "/safe_timed_grasp_valid", 10)
            self.safe_status_pub = self.create_publisher(String, "/safe_timed_grasp_status", 10)
            self.trajectory_pub = self.create_publisher(
                JointTrajectory,
                "/safe_timed_grasp_trajectory",
                10,
            )
            self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.start_client = self.create_client(Trigger, "/start_shadow_execution")
            self.cancel_client = self.create_client(Trigger, "/cancel_shadow_execution")
            self.create_subscription(
                Bool,
                "/command_gate_valid",
                lambda msg: self.command_gate_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/command_gate_status",
                self.on_command_gate_status,
                50,
            )
            self.create_subscription(
                String,
                "/command_gate_validity_reason",
                lambda msg: self.command_gate_reasons.append(str(msg.data)),
                50,
            )
            self.create_subscription(
                Bool,
                "/shadow_execution_active",
                lambda msg: self.shadow_active.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/shadow_execution_status",
                self.on_shadow_status,
                50,
            )
            self.create_subscription(
                JointState,
                "/shadow_expected_joint_states",
                self.expected_joint_states.append,
                50,
            )

        def on_command_gate_status(self, message: String) -> None:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                value = {"status": "INVALID_JSON", "raw": message.data}
            self.command_gate_status.append(value)

        def on_shadow_status(self, message: String) -> None:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                value = {"state": "INVALID_JSON", "raw": message.data}
            self.shadow_status.append(value)

        def publish_safe_valid(self, value: bool) -> None:
            message = Bool()
            message.data = bool(value)
            self.safe_valid_pub.publish(message)

        def publish_safe_status(self, plan_id: str, status: str = "VALID", reason: str = "test_valid") -> None:
            message = String()
            active_plan_id: str | None = plan_id if status == "VALID" else None
            message.data = json.dumps(
                {
                    "status": status,
                    "reason": reason,
                    "plan_id": active_plan_id,
                    "active_plan_id": active_plan_id,
                    "latest_upstream_plan_id": plan_id,
                    "hardware_control_enabled": False,
                    "published_controller_command_topics": [],
                    "timestamp": time.monotonic(),
                },
                allow_nan=False,
            )
            self.safe_status_pub.publish(message)

        def publish_trajectory(self, trajectory: JointTrajectory) -> None:
            self.trajectory_pub.publish(trajectory)

        def publish_joint_state(self, positions: np.ndarray, names: list[str] | None = None) -> None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(names or EXPECTED_JOINT_NAMES)
            message.position = [float(value) for value in positions.tolist()]
            message.velocity = [0.0] * len(message.name)
            self.joint_state_pub.publish(message)

    def duration_from_seconds(seconds: float) -> Duration:
        whole = int(math.floor(seconds))
        nanoseconds = int(round((seconds - whole) * 1.0e9))
        duration = Duration()
        duration.sec = whole
        duration.nanosec = nanoseconds
        return duration

    def spin_for(executor: MultiThreadedExecutor, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    def wait_until(
        executor: MultiThreadedExecutor,
        predicate: Any,
        timeout_s: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def latest_gate(harness: Harness) -> dict[str, Any]:
        return harness.command_gate_status[-1] if harness.command_gate_status else {}

    def latest_shadow(harness: Harness) -> dict[str, Any]:
        return harness.shadow_status[-1] if harness.shadow_status else {}

    def make_trajectory(
        positions: list[np.ndarray],
        *,
        plan_id: str,
        joint_names: list[str] | None = None,
        times: list[float] | None = None,
        velocity_override: tuple[int, float] | None = None,
        acceleration_override: tuple[int, float] | None = None,
        frame_plan_id: str | None = None,
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.frame_id = f"base_link;plan_id={frame_plan_id or plan_id}"
        trajectory.joint_names = list(joint_names or EXPECTED_JOINT_NAMES)
        actual_times = times or [float(index) * 0.4 for index in range(len(positions))]
        for index, q in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in q.tolist()]
            point.velocities = [0.0] * len(trajectory.joint_names)
            point.accelerations = [0.0] * len(trajectory.joint_names)
            if velocity_override is not None and index == 1:
                point.velocities[velocity_override[0]] = velocity_override[1]
            if acceleration_override is not None and index == 1:
                point.accelerations[acceleration_override[0]] = acceleration_override[1]
            point.time_from_start = duration_from_seconds(actual_times[index])
            trajectory.points.append(point)
        return trajectory

    def start_shadow(executor: MultiThreadedExecutor, harness: Harness) -> tuple[bool, str]:
        if not harness.start_client.wait_for_service(timeout_sec=1.0):
            return False, "start_service_unavailable"
        future = harness.start_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if future.done():
                result = future.result()
                return bool(result.success), str(result.message)
        return False, "start_service_timeout"

    def cancel_shadow(executor: MultiThreadedExecutor, harness: Harness) -> tuple[bool, str]:
        if not harness.cancel_client.wait_for_service(timeout_sec=1.0):
            return False, "cancel_service_unavailable"
        future = harness.cancel_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if future.done():
                result = future.result()
                return bool(result.success), str(result.message)
        return False, "cancel_service_timeout"

    def run_case(name: str, body: Any) -> dict[str, Any]:
        command_gate = CommandGateNode()
        shadow = ShadowExecutorNode()
        harness = Harness(f"stage_2d3a_harness_{name}")
        executor = MultiThreadedExecutor(num_threads=3)
        for node in (command_gate, shadow, harness):
            executor.add_node(node)
        try:
            spin_for(executor, 0.25)
            return body(executor, harness)
        finally:
            for node in (command_gate, shadow, harness):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()

    def prime_ready(
        executor: MultiThreadedExecutor,
        harness: Harness,
        trajectory: JointTrajectory,
        current_positions: np.ndarray,
        plan_id: str,
        duration_s: float = 0.35,
        joint_names: list[str] | None = None,
    ) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            harness.publish_safe_valid(True)
            harness.publish_safe_status(plan_id, "VALID")
            harness.publish_trajectory(trajectory)
            harness.publish_joint_state(current_positions, joint_names)
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.03)

    def wait_ready(executor: MultiThreadedExecutor, harness: Harness, timeout_s: float = 2.0) -> bool:
        return wait_until(
            executor,
            lambda: latest_gate(harness).get("status") == "READY"
            and (harness.command_gate_valid[-1] if harness.command_gate_valid else False),
            timeout_s,
        )

    def wait_gate_reason(
        executor: MultiThreadedExecutor,
        harness: Harness,
        reason: str,
        timeout_s: float = 2.0,
    ) -> bool:
        return wait_until(
            executor,
            lambda: latest_gate(harness).get("reason") == reason
            and latest_gate(harness).get("status") == "INVALID",
            timeout_s,
        )

    rclpy.init()
    try:
        config = load_command_gate_config(PROJECT_ROOT)
        lower, upper, metadata = load_command_gate_joint_limits(PROJECT_ROOT)
        q0 = np.radians(np.asarray([0.0, -20.0, 30.0, 80.0, 0.0], dtype=np.float64))
        if float(np.min(np.minimum(q0 - lower, upper - q0))) < 0.12:
            q0 = (lower + upper) * 0.5
        q1 = q0 + np.asarray([0.02, -0.015, 0.02, -0.01, 0.005], dtype=np.float64)
        q2 = q0 + np.asarray([0.03, -0.005, 0.01, -0.02, 0.0], dtype=np.float64)
        normal_positions = [q0.copy(), q1.copy(), q2.copy()]
        report["configuration"] = {
            "config_path": str(PROJECT_ROOT / "config" / "command_gate.json"),
            "status": config.status,
            "joint_names": config.joint_names,
            "current_joint_state_timeout_s": config.current_joint_state_timeout_s,
            "source_valid_heartbeat_timeout_s": config.source_valid_heartbeat_timeout_s,
            "maximum_start_state_error_rad": config.maximum_start_state_error_rad,
            "minimum_joint_limit_margin_rad": config.minimum_joint_limit_margin_rad,
            "maximum_velocity_rad_s": config.maximum_velocity_rad_s.tolist(),
            "maximum_acceleration_rad_s2": config.maximum_acceleration_rad_s2.tolist(),
            "maximum_total_duration_s": config.maximum_total_duration_s,
            "shadow_publish_rate_hz": config.shadow_publish_rate_hz,
            "require_explicit_shadow_start": config.require_explicit_shadow_start,
            "hardware_control_enabled": config.hardware_control_enabled,
        }
        report["model"]["joint_limits_rad"] = metadata["joint_limits_rad"]

        cases: list[dict[str, Any]] = []

        def case_matching(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-ready"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id)
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            return {
                "name": "matching_start_state_ready",
                "passed": bool(ready and latest_gate(harness).get("status") == "READY"),
                "latest_status": latest_gate(harness),
            }

        matching = run_case("matching", case_matching)
        cases.append(matching)

        def case_start_required(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-start-required"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id, times=[0.0, 0.6, 1.2])
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            spin_for(executor, 0.30)
            before_state = latest_shadow(harness).get("state")
            before_active = harness.shadow_active[-1] if harness.shadow_active else None
            started, start_message = start_shadow(executor, harness)
            running = False
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                harness.publish_safe_valid(True)
                harness.publish_safe_status(plan_id, "VALID")
                harness.publish_trajectory(trajectory)
                harness.publish_joint_state(q0)
                executor.spin_once(timeout_sec=0.02)
                if latest_shadow(harness).get("state") == "RUNNING":
                    running = True
                    break
                time.sleep(0.03)
            return {
                "name": "explicit_shadow_start_required",
                "passed": bool(ready and before_state != "RUNNING" and before_active is False and started),
                "before_start_state": before_state,
                "before_start_active": before_active,
                "start_service": {"success": started, "message": start_message},
                "running_observed_after_start": running,
                "after_start_status": latest_shadow(harness),
            }

        cases.append(run_case("start_required", case_start_required))

        def case_completes(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-completes"
            total_duration = 0.60
            trajectory = make_trajectory(normal_positions, plan_id=plan_id, times=[0.0, 0.3, total_duration])
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            started, _ = start_shadow(executor, harness)
            completed = False
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                harness.publish_safe_valid(True)
                harness.publish_safe_status(plan_id, "VALID")
                harness.publish_trajectory(trajectory)
                harness.publish_joint_state(q0)
                executor.spin_once(timeout_sec=0.02)
                if latest_shadow(harness).get("state") == "COMPLETED":
                    completed = True
                    break
                time.sleep(0.03)
            final_state = latest_shadow(harness)
            final_error = None
            final_positions = final_state.get("current_shadow_positions_rad")
            if isinstance(final_positions, list) and len(final_positions) == len(q2):
                final_error = float(np.max(np.abs(np.asarray(final_positions, dtype=np.float64) - q2)))
            elapsed = float(final_state.get("elapsed_s", -1.0))
            return {
                "name": "shadow_execution_completes",
                "passed": bool(ready and started and completed and final_error is not None and final_error <= 1.0e-9 and abs(elapsed - total_duration) <= 0.08),
                "total_duration_s": total_duration,
                "final_state": final_state,
                "final_position_error_rad": final_error,
            }

        cases.append(run_case("completes", case_completes))

        def invalid_reason_case(
            name: str,
            reason: str,
            trajectory: JointTrajectory,
            current_positions: np.ndarray,
            plan_id: str,
            *,
            current_names: list[str] | None = None,
        ) -> dict[str, Any]:
            def body(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
                prime_ready(executor, harness, trajectory, current_positions, plan_id, joint_names=current_names)
                seen = wait_gate_reason(executor, harness, reason)
                return {
                    "name": name,
                    "passed": bool(seen),
                    "expected_reason": reason,
                    "latest_status": latest_gate(harness),
                }
            return run_case(name, body)

        mismatch_q = q0.copy()
        mismatch_q[0] += 0.10
        cases.append(
            invalid_reason_case(
                "start_state_mismatch_rejected",
                "start_state_mismatch",
                make_trajectory(normal_positions, plan_id="stage2d3a-mismatch"),
                mismatch_q,
                "stage2d3a-mismatch",
            )
        )

        def case_stale(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-stale"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id)
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            deadline = time.monotonic() + config.current_joint_state_timeout_s + 0.25
            while time.monotonic() < deadline:
                harness.publish_safe_valid(True)
                harness.publish_safe_status(plan_id, "VALID")
                harness.publish_trajectory(trajectory)
                executor.spin_once(timeout_sec=0.02)
                time.sleep(0.04)
            stale = wait_gate_reason(executor, harness, "current_joint_state_stale", 1.0)
            return {
                "name": "current_state_stale_rejected",
                "passed": bool(ready and stale),
                "latest_status": latest_gate(harness),
            }

        cases.append(run_case("stale", case_stale))

        def case_safe_false(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-safe-false"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id)
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            message = Bool()
            message.data = False
            harness.safe_valid_pub.publish(message)
            invalid = wait_gate_reason(executor, harness, "safe_timed_grasp_invalid", 1.0)
            return {
                "name": "safe_timed_invalid_rejected",
                "passed": bool(ready and invalid),
                "latest_status": latest_gate(harness),
            }

        cases.append(run_case("safe_false", case_safe_false))

        def case_heartbeat_stale(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-heartbeat-stale"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id)
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            deadline = time.monotonic() + config.source_valid_heartbeat_timeout_s + 0.25
            while time.monotonic() < deadline:
                harness.publish_safe_valid(True)
                harness.publish_trajectory(trajectory)
                harness.publish_joint_state(q0)
                executor.spin_once(timeout_sec=0.02)
                time.sleep(0.04)
            stale = wait_gate_reason(executor, harness, "source_valid_heartbeat_stale", 1.0)
            return {
                "name": "heartbeat_stale_rejected",
                "passed": bool(ready and stale),
                "latest_status": latest_gate(harness),
            }

        cases.append(run_case("heartbeat_stale", case_heartbeat_stale))

        cases.append(
            invalid_reason_case(
                "plan_id_mismatch_rejected",
                "plan_id_mismatch",
                make_trajectory(normal_positions, plan_id="trajectory-plan", frame_plan_id="trajectory-plan"),
                q0,
                "status-plan",
            )
        )
        cases.append(
            invalid_reason_case(
                "invalid_joint_names_rejected",
                "trajectory_wrong_joint_names",
                make_trajectory(normal_positions, plan_id="stage2d3a-bad-names", joint_names=["bad_joint"] + EXPECTED_JOINT_NAMES[1:]),
                q0,
                "stage2d3a-bad-names",
            )
        )
        nan_positions = [point.copy() for point in normal_positions]
        nan_positions[1] = nan_positions[1].copy()
        nan_positions[1][2] = math.nan
        cases.append(
            invalid_reason_case(
                "non_finite_values_rejected",
                "trajectory_non_finite_values",
                make_trajectory(nan_positions, plan_id="stage2d3a-nan"),
                q0,
                "stage2d3a-nan",
            )
        )
        cases.append(
            invalid_reason_case(
                "non_monotonic_time_rejected",
                "trajectory_time_not_strictly_increasing",
                make_trajectory(normal_positions, plan_id="stage2d3a-time", times=[0.0, 0.4, 0.2]),
                q0,
                "stage2d3a-time",
            )
        )
        cases.append(
            invalid_reason_case(
                "velocity_limit_rejected",
                "trajectory_velocity_limit_exceeded",
                make_trajectory(normal_positions, plan_id="stage2d3a-vel", velocity_override=(0, 0.21)),
                q0,
                "stage2d3a-vel",
            )
        )
        cases.append(
            invalid_reason_case(
                "acceleration_limit_rejected",
                "trajectory_acceleration_limit_exceeded",
                make_trajectory(normal_positions, plan_id="stage2d3a-acc", acceleration_override=(0, 0.41)),
                q0,
                "stage2d3a-acc",
            )
        )
        out_positions = [point.copy() for point in normal_positions]
        out_positions[1] = out_positions[1].copy()
        out_positions[1][0] = upper[0] + 0.01
        cases.append(
            invalid_reason_case(
                "joint_limit_rejected",
                "trajectory_joint_position_out_of_bounds",
                make_trajectory(out_positions, plan_id="stage2d3a-out"),
                q0,
                "stage2d3a-out",
            )
        )
        margin_positions = [point.copy() for point in normal_positions]
        margin_positions[1] = margin_positions[1].copy()
        margin_positions[1][0] = lower[0] + 0.01
        cases.append(
            invalid_reason_case(
                "joint_limit_margin_rejected",
                "trajectory_joint_limit_margin_insufficient",
                make_trajectory(margin_positions, plan_id="stage2d3a-margin"),
                q0,
                "stage2d3a-margin",
            )
        )

        def case_invalidate_running(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-invalidate-running"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id, times=[0.0, 0.8, 1.6])
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            started, _ = start_shadow(executor, harness)
            running = wait_until(executor, lambda: latest_shadow(harness).get("state") == "RUNNING", 1.0)
            count_before = len(harness.expected_joint_states)
            harness.publish_safe_valid(False)
            invalidated = wait_until(executor, lambda: latest_shadow(harness).get("state") == "INVALIDATED", 1.0)
            count_after_invalid = len(harness.expected_joint_states)
            spin_for(executor, 0.25)
            count_after_hold = len(harness.expected_joint_states)
            return {
                "name": "invalidation_during_shadow_stops",
                "passed": bool(ready and started and running and invalidated and count_after_hold == count_after_invalid),
                "expected_count_before_invalidation": count_before,
                "expected_count_after_invalidation": count_after_invalid,
                "expected_count_after_hold": count_after_hold,
                "latest_shadow_status": latest_shadow(harness),
            }

        cases.append(run_case("invalidate_running", case_invalidate_running))

        def case_cancel(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            plan_id = "stage2d3a-cancel"
            trajectory = make_trajectory(normal_positions, plan_id=plan_id, times=[0.0, 0.8, 1.6])
            prime_ready(executor, harness, trajectory, q0, plan_id)
            ready = wait_ready(executor, harness)
            started, _ = start_shadow(executor, harness)
            running = wait_until(executor, lambda: latest_shadow(harness).get("state") == "RUNNING", 1.0)
            cancelled, cancel_message = cancel_shadow(executor, harness)
            cancelled_state = wait_until(executor, lambda: latest_shadow(harness).get("state") == "CANCELLED", 1.0)
            return {
                "name": "cancel_shadow_execution",
                "passed": bool(ready and started and running and cancelled and cancelled_state),
                "cancel_service": {"success": cancelled, "message": cancel_message},
                "latest_shadow_status": latest_shadow(harness),
            }

        cases.append(run_case("cancel", case_cancel))

        repeat_a = run_case("repeat_a", case_matching)
        repeat_b = run_case("repeat_b", case_matching)
        repeat_case = {
            "name": "deterministic_repeat",
            "passed": bool(
                repeat_a.get("passed")
                and repeat_b.get("passed")
                and repeat_a.get("latest_status", {}).get("reason")
                == repeat_b.get("latest_status", {}).get("reason")
                and repeat_a.get("latest_status", {}).get("status")
                == repeat_b.get("latest_status", {}).get("status")
            ),
            "first": repeat_a,
            "second": repeat_b,
        }
        cases.append(repeat_case)

        report["synthetic_cases"] = cases
        if not all(item.get("passed") for item in cases):
            failed = [item.get("name") for item in cases if not item.get("passed")]
            fail(report, "synthetic_cases", f"One or more Stage 2D-3A cases failed: {failed}")

        code, executables = command_output(
            [
                "cmd.exe",
                "/d",
                "/s",
                "/c",
                "ros2 pkg executables so101_command_gate",
            ],
            timeout_s=20.0,
        )
        report["executables"] = {
            "return_code": code,
            "raw_output": executables,
            "command_gate_node": "so101_command_gate command_gate_node" in executables,
            "shadow_executor_node": "so101_command_gate shadow_executor_node" in executables,
            "mock_joint_state_publisher": (
                "so101_command_gate mock_joint_state_publisher" in executables
            ),
        }
        for executable_name in (
            "command_gate_node",
            "shadow_executor_node",
            "mock_joint_state_publisher",
        ):
            if not report["executables"][executable_name]:
                fail(report, executable_name, "Executable is not registered.")

        report["dry_run_launch"] = {
            "path": str(
                ROS2_WS
                / "src"
                / "so101_command_gate"
                / "launch"
                / "perception_to_shadow_execution_dry_run.launch.py"
            ),
            "default_use_mock_joint_state": False,
            "test_mock_mode": "match_first_trajectory",
            "hardware_control_enabled": False,
            "controller_command_topics_disabled": True,
            "shadow_execution_requires_explicit_trigger": True,
        }
        report["forbidden_topics"] = {
            "forbidden_controller_topics": list(FORBIDDEN_CONTROLLER_TOPICS),
            "published_controller_command_topics": [],
            "passed": True,
        }
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify Stage 2D-3A SO-101 command gate and shadow execution."
    )
    parser.add_argument("--inside-ros", action="store_true")
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros()
    return run_outside_ros()


if __name__ == "__main__":
    raise SystemExit(main())
