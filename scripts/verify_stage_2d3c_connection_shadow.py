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
REPORT_PATH = REPORT_DIR / "stage_2d3c_report.json"
LOG_PATH = REPORT_DIR / "stage_2d3c_verification.log"
ROS_LOG_DIR = REPORT_DIR / "ros_logs_2d3c"
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


def run_outside_ros() -> int:
    ensure_dirs()
    if LOG_PATH.exists():
        LOG_PATH.unlink()
    command_file = Path(tempfile.gettempdir()) / (
        f"verify_stage_2d3c_{filename_timestamp()}.cmd"
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
        append_log("Entering ROS2 Lyrical environment for Stage 2D-3C.")
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
    offline_status = None
    failures: list[dict[str, Any]] = []
    if report_path.is_file():
        try:
            child = read_json(report_path)
            report_status = str(child.get("status"))
            offline_status = child.get("offline_status")
            failures = list(child.get("failures", []))
        except Exception as error:
            report_status = f"READ_ERROR:{error!r}"
    if name == "stage_2d3b_offline_regression":
        passed = bool(code == 0 and offline_status == "PASS" and not failures)
    else:
        passed = bool(code == 0 and report_status == "PASS" and not failures)
    report[name] = {
        "return_code": code,
        "status": "PASS" if passed else "FAIL",
        "report": str(report_path),
        "report_status": report_status,
        "offline_status": offline_status,
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

    append_log("Stage 2D-3C verification started inside ROS2 environment.")
    urdf_sha = sha256_file(URDF_PATH)
    report: dict[str, Any] = {
        "stage": "2D-3C",
        "status": "FAIL",
        "timestamp": iso_timestamp(),
        "model": {
            "urdf": str(URDF_PATH),
            "urdf_sha256": urdf_sha,
            "urdf_sha256_expected": EXPECTED_URDF_SHA256,
            "urdf_sha256_matches_expected": urdf_sha == EXPECTED_URDF_SHA256,
            "joint_names": EXPECTED_JOINT_NAMES,
        },
        "safety": {
            "opened_com_ports": False,
            "real_joint_state_read_only": True,
            "torque_enable_written": False,
            "torque_disable_written": False,
            "goal_position_written": False,
            "motion_parameters_written": False,
            "motion_command_sent": False,
            "real_controller_topics_published": [],
            "hardware_control_enabled": False,
            "shadow_execution_only": True,
            "observed_physical_motion": False,
            "forbidden_controller_topics": FORBIDDEN_CONTROLLER_TOPICS,
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
            name="stage_2d3a_regression_includes_2d1_2d2",
            script=PROJECT_ROOT / "scripts" / "verify_stage_2d3a_command_gate_shadow.py",
            report_path=REPORT_DIR / "stage_2d3a_report.json",
            timeout_s=360.0,
        )
        run_stage_regression(
            report,
            name="stage_2d3b_offline_regression",
            script=PROJECT_ROOT / "scripts" / "verify_stage_2d3b_real_joint_state_bridge.py",
            report_path=REPORT_DIR / "stage_2d3b_report.json",
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
        append_log(f"Stage 2D-3C status: {report['status']}")
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
    from so101_command_gate.command_gate_validator import load_command_gate_joint_limits
    from so101_command_gate.connection_parameterizer import (
        load_connection_config,
        parameterize_connection,
    )
    from so101_command_gate.connection_trajectory_node import ConnectionTrajectoryNode
    from so101_command_gate.connection_validator import (
        validate_connection_plan,
        validate_source_inputs,
    )
    from so101_command_gate.shadow_executor_node import ShadowExecutorNode

    class Harness(Node):
        def __init__(self, name: str) -> None:
            super().__init__(name)
            self.connection_valid: list[bool] = []
            self.connection_status: list[dict[str, Any]] = []
            self.connected_trajectories: list[JointTrajectory] = []
            self.command_gate_valid: list[bool] = []
            self.command_gate_status: list[dict[str, Any]] = []
            self.shadow_active: list[bool] = []
            self.shadow_status: list[dict[str, Any]] = []
            self.expected_joint_states: list[JointState] = []
            self.real_state_pub = self.create_publisher(JointState, "/real_joint_states", 10)
            self.real_valid_pub = self.create_publisher(Bool, "/real_joint_state_valid", 10)
            self.safe_valid_pub = self.create_publisher(Bool, "/safe_timed_grasp_valid", 10)
            self.safe_status_pub = self.create_publisher(String, "/safe_timed_grasp_status", 10)
            self.trajectory_pub = self.create_publisher(
                JointTrajectory,
                "/safe_timed_grasp_trajectory",
                10,
            )
            self.start_client = self.create_client(Trigger, "/start_shadow_execution")
            self.create_subscription(
                Bool,
                "/connection_plan_valid",
                lambda msg: self.connection_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(String, "/connection_plan_status", self.on_connection_status, 50)
            self.create_subscription(
                JointTrajectory,
                "/connected_safe_timed_grasp_trajectory",
                self.connected_trajectories.append,
                50,
            )
            self.create_subscription(
                Bool,
                "/command_gate_valid",
                lambda msg: self.command_gate_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(String, "/command_gate_status", self.on_gate_status, 50)
            self.create_subscription(
                Bool,
                "/shadow_execution_active",
                lambda msg: self.shadow_active.append(bool(msg.data)),
                50,
            )
            self.create_subscription(String, "/shadow_execution_status", self.on_shadow_status, 50)
            self.create_subscription(
                JointState,
                "/shadow_expected_joint_states",
                self.expected_joint_states.append,
                50,
            )

        def decode(self, message: String) -> dict[str, Any]:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                return {"status": "INVALID_JSON", "raw": message.data}
            return value if isinstance(value, dict) else {"status": "NOT_OBJECT"}

        def on_connection_status(self, message: String) -> None:
            self.connection_status.append(self.decode(message))

        def on_gate_status(self, message: String) -> None:
            self.command_gate_status.append(self.decode(message))

        def on_shadow_status(self, message: String) -> None:
            self.shadow_status.append(self.decode(message))

        def publish_real_state(self, positions: np.ndarray, valid: bool = True) -> None:
            valid_message = Bool()
            valid_message.data = bool(valid)
            self.real_valid_pub.publish(valid_message)
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(EXPECTED_JOINT_NAMES)
            message.position = [float(value) for value in positions.tolist()]
            message.velocity = [0.0] * len(EXPECTED_JOINT_NAMES)
            self.real_state_pub.publish(message)

        def publish_safe_inputs(
            self,
            trajectory: JointTrajectory,
            plan_id: str,
            valid: bool = True,
            status: str = "VALID",
        ) -> None:
            valid_message = Bool()
            valid_message.data = bool(valid)
            self.safe_valid_pub.publish(valid_message)
            status_message = String()
            active_plan_id: str | None = plan_id if status == "VALID" else None
            status_message.data = json.dumps(
                {
                    "status": status,
                    "reason": "test_valid" if status == "VALID" else "test_invalid",
                    "plan_id": active_plan_id,
                    "active_plan_id": active_plan_id,
                    "latest_upstream_plan_id": plan_id,
                    "hardware_control_enabled": False,
                    "published_controller_command_topics": [],
                    "timestamp": time.monotonic(),
                },
                allow_nan=False,
            )
            self.safe_status_pub.publish(status_message)
            self.trajectory_pub.publish(trajectory)

    def duration_from_seconds(seconds: float) -> Duration:
        total_nanoseconds = int(round(seconds * 1.0e9))
        duration = Duration()
        duration.sec = total_nanoseconds // 1_000_000_000
        duration.nanosec = total_nanoseconds % 1_000_000_000
        return duration

    def seconds_from_point(point: JointTrajectoryPoint) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1.0e-9

    def make_trajectory(
        positions: list[np.ndarray],
        *,
        plan_id: str,
        times: list[float] | None = None,
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.frame_id = f"base_link;plan_id={plan_id}"
        trajectory.joint_names = list(EXPECTED_JOINT_NAMES)
        actual_times = times or [0.0, 0.40, 0.80]
        for index, q in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in q.tolist()]
            point.velocities = [0.0] * len(EXPECTED_JOINT_NAMES)
            point.accelerations = [0.0] * len(EXPECTED_JOINT_NAMES)
            point.time_from_start = duration_from_seconds(actual_times[index])
            trajectory.points.append(point)
        return trajectory

    def status_for(plan_id: str, status: str = "VALID") -> dict[str, Any]:
        active_plan_id: str | None = plan_id if status == "VALID" else None
        return {
            "status": status,
            "reason": "test_valid" if status == "VALID" else "test_invalid",
            "plan_id": active_plan_id,
            "active_plan_id": active_plan_id,
            "latest_upstream_plan_id": plan_id,
            "hardware_control_enabled": False,
            "published_controller_command_topics": [],
            "timestamp": time.monotonic(),
        }

    def joint_state(positions: np.ndarray, names: list[str] | None = None) -> JointState:
        message = JointState()
        message.name = list(names or EXPECTED_JOINT_NAMES)
        message.position = [float(value) for value in positions.tolist()]
        return message

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

    def latest_connection(harness: Harness) -> dict[str, Any]:
        return harness.connection_status[-1] if harness.connection_status else {}

    def latest_gate(harness: Harness) -> dict[str, Any]:
        return harness.command_gate_status[-1] if harness.command_gate_status else {}

    def latest_shadow(harness: Harness) -> dict[str, Any]:
        return harness.shadow_status[-1] if harness.shadow_status else {}

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

    def prime(
        executor: MultiThreadedExecutor,
        harness: Harness,
        trajectory: JointTrajectory,
        current: np.ndarray,
        plan_id: str,
        *,
        real_valid: bool = True,
        safe_valid: bool = True,
        status: str = "VALID",
        duration_s: float = 0.45,
    ) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            harness.publish_real_state(current, real_valid)
            harness.publish_safe_inputs(trajectory, plan_id, safe_valid, status)
            spin_for(executor, 0.06)

    def run_node_case(name: str, body: Any) -> dict[str, Any]:
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
        harness = Harness(f"stage_2d3c_harness_{name}")
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (connection, gate, shadow, harness):
            executor.add_node(node)
        try:
            spin_for(executor, 0.25)
            return body(executor, harness)
        finally:
            for node in (connection, gate, shadow, harness):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()

    def offline_plan_case(
        *,
        current: np.ndarray,
        trajectory: JointTrajectory,
        plan_id: str,
        real_valid: bool = True,
        safe_valid: bool = True,
        status: str = "VALID",
        current_age_s: float = 0.0,
        heartbeat_age_s: float = 0.0,
    ) -> tuple[Any, Any]:
        result, current_positions, upstream_plan_id = validate_source_inputs(
            real_joint_state_valid=real_valid,
            safe_timed_valid=safe_valid,
            source_status=status_for(plan_id, status),
            source_trajectory=trajectory,
            current_joint_state=joint_state(current),
            current_joint_state_age_s=current_age_s,
            source_valid_heartbeat_age_s=heartbeat_age_s,
            config=connection_config,
            lower=lower,
            upper=upper,
            project_root=PROJECT_ROOT,
        )
        if not result.valid or current_positions is None or upstream_plan_id is None:
            return result, None
        plan = parameterize_connection(
            q_current=current_positions,
            source_trajectory=trajectory,
            upstream_plan_id=upstream_plan_id,
            config=connection_config,
        )
        plan_result = validate_connection_plan(
            plan=plan,
            config=connection_config,
            lower=lower,
            upper=upper,
            project_root=PROJECT_ROOT,
        )
        return plan_result, plan

    rclpy.init()
    try:
        connection_config = load_connection_config(PROJECT_ROOT)
        lower, upper, metadata = load_command_gate_joint_limits(PROJECT_ROOT)
        q_current = np.radians(np.asarray([3.0, -18.0, 18.0, 68.0, 2.0], dtype=np.float64))
        q_start = np.radians(np.asarray([0.0, -20.0, 20.0, 70.0, 0.0], dtype=np.float64))
        q_mid = np.radians(np.asarray([0.0, -20.0, 20.0, 80.0, 0.0], dtype=np.float64))
        q_end = np.radians(np.asarray([0.0, -15.0, 20.0, 70.0, 0.0], dtype=np.float64))
        base_trajectory = make_trajectory([q_start, q_mid, q_end], plan_id="stage2d3c-valid")
        report["configuration"] = {
            "config_path": str(PROJECT_ROOT / "config" / "connection_trajectory.json"),
            "status": connection_config.status,
            "joint_names": connection_config.joint_names,
            "sample_rate_hz": connection_config.sample_rate_hz,
            "maximum_connection_velocity_rad_s": connection_config.maximum_connection_velocity_rad_s.tolist(),
            "maximum_connection_acceleration_rad_s2": connection_config.maximum_connection_acceleration_rad_s2.tolist(),
            "minimum_connection_duration_s": connection_config.minimum_connection_duration_s,
            "maximum_connection_duration_s": connection_config.maximum_connection_duration_s,
            "minimum_joint_limit_margin_rad": connection_config.minimum_joint_limit_margin_rad,
            "minimum_tcp_z_m": connection_config.minimum_tcp_z_m,
        }
        report["model"]["joint_limits_rad"] = metadata["joint_limits_rad"]

        synthetic_cases: list[dict[str, Any]] = []
        valid_result, valid_plan = offline_plan_case(
            current=q_current,
            trajectory=base_trajectory,
            plan_id="stage2d3c-valid",
        )
        if valid_plan is not None:
            times = [seconds_from_point(point) for point in valid_plan.combined_trajectory.points]
            positions = np.asarray(
                [point.positions for point in valid_plan.combined_trajectory.points],
                dtype=np.float64,
            )
            final_metrics = {
                "q_current_rad": valid_plan.q_current.tolist(),
                "q_start_rad": valid_plan.q_start.tolist(),
                "per_joint_start_error_rad": valid_plan.per_joint_start_error_rad.tolist(),
                "connection_duration_s": valid_plan.connection_duration_s,
                "connection_point_count": valid_plan.connection_point_count,
                "combined_trajectory_point_count": len(valid_plan.combined_trajectory.points),
                "combined_total_duration_s": times[-1],
                "maximum_connection_velocity_rad_s_observed": valid_plan.maximum_connection_velocity_rad_s_observed.tolist(),
                "maximum_connection_acceleration_rad_s2_observed": valid_plan.maximum_connection_acceleration_rad_s2_observed.tolist(),
                "minimum_joint_limit_margin_rad": valid_result.minimum_joint_limit_margin_rad,
                "minimum_tcp_z_m_observed": valid_result.minimum_tcp_z_m_observed,
                "tcp_x_range_m_observed": valid_result.tcp_x_range_m_observed,
                "tcp_y_range_m_observed": valid_result.tcp_y_range_m_observed,
                "time_strictly_increasing": bool(np.all(np.diff(times) > 0.0)),
                "first_point_matches_q_current": bool(np.max(np.abs(positions[0] - q_current)) <= 1.0e-10),
                "connection_endpoint_matches_q_start": bool(np.max(np.abs(positions[valid_plan.connection_point_count - 1] - q_start)) <= 1.0e-10),
            }
        else:
            final_metrics = {}
        report["final_metrics"] = final_metrics
        synthetic_cases.append(
            {
                "name": "A.valid_connection",
                "passed": bool(valid_result.valid and valid_plan is not None and final_metrics.get("first_point_matches_q_current") and final_metrics.get("connection_endpoint_matches_q_start")),
                "reason": valid_result.reason,
                "metrics": final_metrics,
            }
        )

        zero_result, zero_plan = offline_plan_case(
            current=q_start.copy(),
            trajectory=base_trajectory,
            plan_id="stage2d3c-valid",
        )
        synthetic_cases.append(
            {
                "name": "B.zero_connection",
                "passed": bool(zero_result.valid and zero_plan is not None and zero_plan.connection_duration_s >= 1.0),
                "reason": zero_result.reason,
                "connection_duration_s": None if zero_plan is None else zero_plan.connection_duration_s,
                "point_count": None if zero_plan is None else len(zero_plan.combined_trajectory.points),
            }
        )
        invalid_specs = [
            ("C.joint_state_stale", "current_joint_state_stale", {"current_age_s": 0.40}),
            ("D.real_joint_state_invalid", "real_joint_state_invalid", {"real_valid": False}),
            ("E.safe_trajectory_invalid", "safe_trajectory_invalid", {"safe_valid": False}),
            ("F.current_joint_out_of_bounds", "current_joint_out_of_bounds", {"current": lower - np.asarray([0.1, 0, 0, 0, 0], dtype=np.float64)}),
            ("G.joint_margin_insufficient", "joint_margin_insufficient", {"current": lower + np.asarray([0.01, 0.20, 0.20, 0.20, 0.20], dtype=np.float64)}),
            ("H.connection_duration_too_long", "connection_duration_too_long", {"current": np.asarray([1.65, q_current[1], q_current[2], q_current[3], q_current[4]], dtype=np.float64)}),
            ("I.tcp_below_minimum_z", "tcp_below_minimum_z", {"current": np.radians(np.asarray([0.0, -35.0, 70.0, 60.0, 0.0], dtype=np.float64))}),
            ("J.tcp_outside_workspace", "tcp_outside_workspace", {"current": np.zeros(5, dtype=np.float64)}),
            ("L.non_finite_input", "non_finite_input", {"current": np.asarray([math.nan, q_current[1], q_current[2], q_current[3], q_current[4]], dtype=np.float64)}),
        ]
        for name, expected_reason, overrides in invalid_specs:
            current = overrides.pop("current", q_current)
            result, _ = offline_plan_case(
                current=current,
                trajectory=base_trajectory,
                plan_id="stage2d3c-valid",
                **overrides,
            )
            synthetic_cases.append(
                {
                    "name": name,
                    "passed": bool((not result.valid) and result.reason == expected_reason),
                    "expected_reason": expected_reason,
                    "actual_reason": result.reason,
                }
            )

        changed_trajectory = make_trajectory([q_start, q_mid, q_end], plan_id="stage2d3c-valid2")
        changed_result, changed_plan = offline_plan_case(
            current=q_current + np.asarray([0.04, 0.0, 0.0, 0.0, 0.0], dtype=np.float64),
            trajectory=changed_trajectory,
            plan_id="stage2d3c-valid2",
        )
        repeat_result, repeat_plan = offline_plan_case(
            current=q_current,
            trajectory=base_trajectory,
            plan_id="stage2d3c-valid",
        )
        synthetic_cases.append(
            {
                "name": "K.plan_id_changes",
                "passed": bool(
                    valid_plan is not None
                    and changed_plan is not None
                    and changed_result.valid
                    and changed_plan.connection_plan_id != valid_plan.connection_plan_id
                ),
                "original_connection_plan_id": None if valid_plan is None else valid_plan.connection_plan_id,
                "changed_connection_plan_id": None if changed_plan is None else changed_plan.connection_plan_id,
            }
        )
        synthetic_cases.append(
            {
                "name": "M.deterministic_repeat",
                "passed": bool(
                    valid_plan is not None
                    and repeat_plan is not None
                    and repeat_result.valid
                    and repeat_plan.connection_plan_id == valid_plan.connection_plan_id
                ),
                "first_connection_plan_id": None if valid_plan is None else valid_plan.connection_plan_id,
                "second_connection_plan_id": None if repeat_plan is None else repeat_plan.connection_plan_id,
            }
        )

        def case_ready(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            prime(executor, harness, base_trajectory, q_current, "stage2d3c-valid")
            connection_ready = wait_until(
                executor,
                lambda: latest_connection(harness).get("status") == "VALID"
                and bool(harness.connection_valid[-1] if harness.connection_valid else False),
                2.0,
            )
            gate_ready = wait_until(
                executor,
                lambda: latest_gate(harness).get("status") == "READY"
                and bool(harness.command_gate_valid[-1] if harness.command_gate_valid else False),
                2.0,
            )
            return {
                "name": "N.command_gate_ready_with_connected_trajectory",
                "passed": bool(connection_ready and gate_ready),
                "connection_status": latest_connection(harness),
                "command_gate_status": latest_gate(harness),
            }

        synthetic_cases.append(run_node_case("ready", case_ready))

        def case_start_required(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            prime(executor, harness, base_trajectory, q_current, "stage2d3c-valid")
            ready = wait_until(executor, lambda: latest_gate(harness).get("status") == "READY", 2.0)
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                harness.publish_real_state(q_current, True)
                harness.publish_safe_inputs(base_trajectory, "stage2d3c-valid", True, "VALID")
                spin_for(executor, 0.06)
            before_state = latest_shadow(harness).get("state")
            before_active = harness.shadow_active[-1] if harness.shadow_active else None
            started, start_message = start_shadow(executor, harness)
            running = False
            deadline = time.monotonic() + 1.2
            while time.monotonic() < deadline:
                harness.publish_real_state(q_current, True)
                harness.publish_safe_inputs(base_trajectory, "stage2d3c-valid", True, "VALID")
                spin_for(executor, 0.06)
                if latest_shadow(harness).get("state") == "RUNNING":
                    running = True
                    break
            return {
                "name": "O.explicit_shadow_start_required",
                "passed": bool(ready and before_state != "RUNNING" and before_active is False and started),
                "before_start_state": before_state,
                "before_start_active": before_active,
                "start_service": {"success": started, "message": start_message},
                "running_observed_after_start": running,
                "after_start_status": latest_shadow(harness),
            }

        synthetic_cases.append(run_node_case("start_required", case_start_required))

        def case_completes(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            prime(executor, harness, base_trajectory, q_current, "stage2d3c-valid")
            ready = wait_until(executor, lambda: latest_gate(harness).get("status") == "READY", 2.0)
            started, _ = start_shadow(executor, harness)
            deadline = time.monotonic() + 5.0
            completed = False
            while time.monotonic() < deadline:
                harness.publish_real_state(q_current, True)
                harness.publish_safe_inputs(base_trajectory, "stage2d3c-valid", True, "VALID")
                spin_for(executor, 0.06)
                if latest_shadow(harness).get("state") == "COMPLETED":
                    completed = True
                    break
            return {
                "name": "P.full_shadow_completes",
                "passed": bool(ready and started and completed),
                "latest_shadow_status": latest_shadow(harness),
                "expected_joint_state_count": len(harness.expected_joint_states),
            }

        synthetic_cases.append(run_node_case("completes", case_completes))

        def case_remove_object(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            prime(executor, harness, base_trajectory, q_current, "stage2d3c-valid")
            ready = wait_until(executor, lambda: latest_gate(harness).get("status") == "READY", 2.0)
            started, _ = start_shadow(executor, harness)
            running = wait_until(executor, lambda: latest_shadow(harness).get("state") == "RUNNING", 1.0)
            deadline = time.monotonic() + 2.2
            while time.monotonic() < deadline:
                harness.publish_real_state(q_current, True)
                harness.publish_safe_inputs(base_trajectory, "stage2d3c-valid", False, "INVALID")
                spin_for(executor, 0.06)
                if latest_shadow(harness).get("state") == "INVALIDATED":
                    break
            return {
                "name": "Q.remove_object_during_connection",
                "passed": bool(
                    ready
                    and started
                    and running
                    and latest_gate(harness).get("status") == "INVALID"
                    and latest_shadow(harness).get("state") == "INVALIDATED"
                ),
                "connection_status": latest_connection(harness),
                "command_gate_status": latest_gate(harness),
                "shadow_status": latest_shadow(harness),
            }

        synthetic_cases.append(run_node_case("remove_object", case_remove_object))

        def case_real_disconnect(executor: MultiThreadedExecutor, harness: Harness) -> dict[str, Any]:
            prime(executor, harness, base_trajectory, q_current, "stage2d3c-valid")
            ready = wait_until(executor, lambda: latest_gate(harness).get("status") == "READY", 2.0)
            started, _ = start_shadow(executor, harness)
            running = wait_until(executor, lambda: latest_shadow(harness).get("state") == "RUNNING", 1.0)
            deadline = time.monotonic() + 2.2
            while time.monotonic() < deadline:
                harness.publish_real_state(q_current, False)
                harness.publish_safe_inputs(base_trajectory, "stage2d3c-valid", True, "VALID")
                spin_for(executor, 0.06)
                if latest_shadow(harness).get("state") == "INVALIDATED":
                    break
            return {
                "name": "R.real_joint_state_disconnect_during_shadow",
                "passed": bool(
                    ready
                    and started
                    and running
                    and latest_gate(harness).get("status") == "INVALID"
                    and latest_shadow(harness).get("state") == "INVALIDATED"
                ),
                "connection_status": latest_connection(harness),
                "command_gate_status": latest_gate(harness),
                "shadow_status": latest_shadow(harness),
            }

        synthetic_cases.append(run_node_case("real_disconnect", case_real_disconnect))

        report["synthetic_cases"] = synthetic_cases
        if not all(item.get("passed") for item in synthetic_cases):
            failed = [item.get("name") for item in synthetic_cases if not item.get("passed")]
            fail(report, "synthetic_cases", f"One or more Stage 2D-3C cases failed: {failed}")

        code, executables = command_output(
            ["cmd.exe", "/d", "/s", "/c", "ros2 pkg executables so101_command_gate"],
            timeout_s=20.0,
        )
        report["executables"] = {
            "return_code": code,
            "raw_output": executables,
            "connection_trajectory_node": "so101_command_gate connection_trajectory_node" in executables,
            "command_gate_node": "so101_command_gate command_gate_node" in executables,
            "shadow_executor_node": "so101_command_gate shadow_executor_node" in executables,
        }
        for executable_name in ("connection_trajectory_node", "command_gate_node", "shadow_executor_node"):
            if not report["executables"][executable_name]:
                fail(report, executable_name, "Executable is not registered.")
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
