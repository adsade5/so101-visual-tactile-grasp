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
REPORT_PATH = REPORT_DIR / "stage_2d2_report.json"
LOG_PATH = REPORT_DIR / "stage_2d2_verification.log"
ROS_LOG_DIR = REPORT_DIR / "ros_logs"
STAGE_2D1_REPORT = REPORT_DIR / "stage_2d1_report.json"
STAGE_2D1_SCRIPT = PROJECT_ROOT / "scripts" / "verify_stage_2d1_visual_to_ik.py"
URDF_PATH = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)
EXPECTED_URDF_SHA256 = (
    "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
)
EXPECTED_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
PLANNER_CONFIG_VERSION = "visual_grasp_planner_v1"
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


def trajectory_hash(
    joint_names: list[str],
    positions: list[np.ndarray],
    planner_config_version: str = PLANNER_CONFIG_VERSION,
) -> str:
    def canonical_number(value: float) -> float | str:
        numeric = float(value)
        if math.isfinite(numeric):
            return round(numeric, 12)
        if math.isnan(numeric):
            return "nan"
        return "inf" if numeric > 0.0 else "-inf"

    payload = {
        "joint_names": list(joint_names),
        "positions": [
            [canonical_number(float(value)) for value in np.asarray(point, dtype=float).tolist()]
            for point in positions
        ],
        "waypoint_count": len(positions),
        "planner_config_version": planner_config_version,
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return value


def run_outside_ros() -> int:
    ensure_dirs()
    if LOG_PATH.exists():
        LOG_PATH.unlink()

    command_file = Path(tempfile.gettempdir()) / (
        f"verify_stage_2d2_{filename_timestamp()}.cmd"
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
        append_log("Entering ROS2 Lyrical environment for Stage 2D-2.")
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


def command_output(command: list[str], timeout_s: float = 90.0) -> tuple[int, str]:
    completed = subprocess.run(
        command,
        cwd=str(ROS2_WS),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )
    return int(completed.returncode), completed.stdout


def fail(report: dict[str, Any], item: str, detail: str) -> None:
    report.setdefault("failures", []).append({"item": item, "detail": detail})


def duration_to_seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def trajectory_to_arrays(message: Any) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[float]]:
    positions = np.asarray([point.positions for point in message.points], dtype=np.float64)
    velocities = np.asarray([point.velocities for point in message.points], dtype=np.float64)
    accelerations = np.asarray([point.accelerations for point in message.points], dtype=np.float64)
    times = [duration_to_seconds(point.time_from_start) for point in message.points]
    return positions, velocities, accelerations, times


def build_good_paths(lower: np.ndarray, upper: np.ndarray) -> dict[str, list[np.ndarray]]:
    q0 = np.radians(np.asarray([0.0, -20.0, 30.0, 80.0, 0.0], dtype=np.float64))
    margin = np.minimum(q0 - lower, upper - q0)
    if float(np.min(margin)) < 0.09:
        q0 = (lower + upper) * 0.5
    q1 = q0 + np.asarray([0.02, -0.015, 0.025, -0.02, 0.01], dtype=np.float64)
    q2 = q1 + np.asarray([0.015, 0.01, -0.02, 0.015, -0.005], dtype=np.float64)
    zero = [q0.copy(), q0.copy(), q1.copy()]
    too_long_step = np.asarray([0.12, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    q_long_1 = q0 + too_long_step
    if np.any(q_long_1 > upper - 0.06):
        q_long_1 = q0 - too_long_step
    too_long = [
        (q0.copy() if index % 2 == 0 else q_long_1.copy())
        for index in range(50)
    ]
    return {
        "normal": [q0.copy(), q1.copy(), q2.copy(), q0.copy()],
        "zero_segment": zero,
        "too_long": too_long,
    }


def verify_output_trajectory(
    trajectory: Any,
    status: dict[str, Any],
    source_positions: list[np.ndarray],
    velocity_limits: np.ndarray,
    acceleration_limits: np.ndarray,
) -> tuple[bool, dict[str, Any]]:
    positions, velocities, accelerations, times = trajectory_to_arrays(trajectory)
    velocity_observed = np.max(np.abs(velocities), axis=0)
    acceleration_observed = np.max(np.abs(accelerations), axis=0)
    source_waypoint_times = [
        float(value)
        for value in status.get("source_waypoint_times_s", [])
    ]
    boundary_stop_checks: list[dict[str, Any]] = []
    for waypoint_index, waypoint_time in enumerate(source_waypoint_times):
        matches = [
            index
            for index, value in enumerate(times)
            if abs(value - waypoint_time) <= 1.0e-8
        ]
        if not matches:
            boundary_stop_checks.append(
                {
                    "waypoint_index": waypoint_index,
                    "time_s": waypoint_time,
                    "passed": False,
                    "reason": "waypoint_time_missing",
                }
            )
            continue
        index = matches[0]
        boundary_stop_checks.append(
            {
                "waypoint_index": waypoint_index,
                "time_s": waypoint_time,
                "passed": bool(
                    np.max(np.abs(velocities[index])) <= 1.0e-10
                    and np.max(np.abs(accelerations[index])) <= 1.0e-10
                ),
                "max_abs_velocity": float(np.max(np.abs(velocities[index]))),
                "max_abs_acceleration": float(np.max(np.abs(accelerations[index]))),
            }
        )

    checks = {
        "joint_names": list(trajectory.joint_names),
        "point_count": len(trajectory.points),
        "source_waypoint_count": len(source_positions),
        "first_time_s": times[0] if times else None,
        "last_time_s": times[-1] if times else None,
        "time_strictly_increasing": all(
            times[index] > times[index - 1]
            for index in range(1, len(times))
        ),
        "all_positions_finite": bool(np.all(np.isfinite(positions))),
        "all_velocities_finite": bool(np.all(np.isfinite(velocities))),
        "all_accelerations_finite": bool(np.all(np.isfinite(accelerations))),
        "velocity_observed": velocity_observed.tolist(),
        "acceleration_observed": acceleration_observed.tolist(),
        "velocity_within_limits": bool(
            np.all(velocity_observed <= velocity_limits + 1.0e-9)
        ),
        "acceleration_within_limits": bool(
            np.all(acceleration_observed <= acceleration_limits + 1.0e-9)
        ),
        "boundary_stop_checks": boundary_stop_checks,
        "output_more_dense_than_source": len(trajectory.points) > len(source_positions),
        "first_position_matches_source": bool(
            np.max(np.abs(positions[0] - source_positions[0])) <= 1.0e-10
        ),
        "last_position_matches_source": bool(
            np.max(np.abs(positions[-1] - source_positions[-1])) <= 1.0e-10
        ),
    }
    passed = bool(
        list(trajectory.joint_names) == EXPECTED_JOINT_NAMES
        and checks["point_count"] > len(source_positions)
        and checks["first_time_s"] == 0.0
        and checks["time_strictly_increasing"]
        and checks["all_positions_finite"]
        and checks["all_velocities_finite"]
        and checks["all_accelerations_finite"]
        and checks["velocity_within_limits"]
        and checks["acceleration_within_limits"]
        and checks["first_position_matches_source"]
        and checks["last_position_matches_source"]
        and all(item["passed"] for item in boundary_stop_checks)
    )
    return passed, checks


def run_stage_2d1_regression(report: dict[str, Any]) -> None:
    append_log("Running Stage 2D-1 synthetic verifier regression as a hard gate.")
    code, output = command_output(
        [
            str(ROS2_PYTHON),
            "-X",
            "faulthandler",
            str(STAGE_2D1_SCRIPT),
            "--inside-ros",
        ],
        timeout_s=180.0,
    )
    stage_2d1_status = "UNKNOWN"
    stage_2d1_timestamp = None
    stage_2d1_failures: list[dict[str, Any]] = []
    if STAGE_2D1_REPORT.is_file():
        try:
            stage_2d1_report = read_json(STAGE_2D1_REPORT)
            stage_2d1_status = str(stage_2d1_report.get("status"))
            stage_2d1_timestamp = stage_2d1_report.get("timestamp")
            stage_2d1_failures = list(stage_2d1_report.get("failures", []))
        except Exception as error:
            stage_2d1_status = f"READ_ERROR:{error!r}"
    report["stage_2d1_regression"] = {
        "return_code": code,
        "status": "PASS" if stage_2d1_status == "PASS" and not stage_2d1_failures else "FAIL",
        "report": str(STAGE_2D1_REPORT),
        "report_status": stage_2d1_status,
        "report_timestamp": stage_2d1_timestamp,
        "report_failures": stage_2d1_failures,
        "log_excerpt": output[-4000:],
        "passed": stage_2d1_status == "PASS" and not stage_2d1_failures,
    }
    if not report["stage_2d1_regression"]["passed"]:
        fail(report, "stage_2d1_regression", "Stage 2D-1 synthetic regression did not pass.")


def verify_with_ros_in_process(report: dict[str, Any]) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    sys.path.insert(0, str(ROS2_WS / "src" / "so101_kinematics"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_trajectory_safety"))
    from so101_trajectory_safety.minimum_jerk_parameterizer import parameterize_path
    from so101_trajectory_safety.timed_trajectory_node import TimedTrajectoryNode
    from so101_trajectory_safety.trajectory_validator import (
        load_joint_limits,
        load_safety_config,
        validate_source_path,
    )

    class Observer(Node):
        def __init__(self) -> None:
            super().__init__("so101_stage_2d2_observer")
            self.status_messages: list[dict[str, Any]] = []
            self.valid_messages: list[bool] = []
            self.reason_messages: list[str] = []
            self.trajectories: list[JointTrajectory] = []
            self.source_hash_by_plan: dict[str, str] = {}
            self.source_point_count_by_plan: dict[str, int] = {}
            self.plan_valid_pub = self.create_publisher(Bool, "/grasp_plan_valid", 10)
            self.source_pub = self.create_publisher(
                JointTrajectory,
                "/planned_grasp_joint_trajectory",
                10,
            )
            self.source_status_pub = self.create_publisher(
                String,
                "/grasp_plan_status",
                10,
            )
            self.create_subscription(
                Bool,
                "/safe_timed_grasp_valid",
                lambda msg: self.valid_messages.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/safe_timed_grasp_validity_reason",
                lambda msg: self.reason_messages.append(str(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/safe_timed_grasp_status",
                self.on_status,
                50,
            )
            self.create_subscription(
                JointTrajectory,
                "/safe_timed_grasp_trajectory",
                self.trajectories.append,
                50,
            )

        def on_status(self, message: String) -> None:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                value = {"status": "INVALID_JSON", "raw": message.data}
            self.status_messages.append(value)

        def publish_plan_valid(self, value: bool) -> None:
            message = Bool()
            message.data = bool(value)
            self.plan_valid_pub.publish(message)

        def publish_source_status(
            self,
            plan_id: str,
            status: str = "VALID",
            reason: str = "planned_preview_only",
            trajectory_republished: bool = False,
            hash_value: str | None = None,
            point_count: int | None = None,
        ) -> None:
            source_hash = hash_value or self.source_hash_by_plan.get(plan_id)
            source_point_count = point_count or self.source_point_count_by_plan.get(plan_id)
            message = String()
            message.data = json.dumps(
                {
                    "status": status,
                    "reason": reason,
                    "plan_id": plan_id,
                    "active_plan_id": plan_id if status == "VALID" else None,
                    "latest_generated_plan_id": plan_id,
                    "trajectory_hash": source_hash if status == "VALID" else None,
                    "trajectory_point_count": source_point_count,
                    "trajectory_payload_publish_seq": 1 if trajectory_republished else 0,
                    "trajectory_payload_publish_timestamp": time.monotonic() if trajectory_republished else None,
                    "planner_config_version": PLANNER_CONFIG_VERSION,
                    "source": "stage_2d2_test",
                    "trajectory_republished": trajectory_republished,
                    "timestamp": time.monotonic(),
                },
                allow_nan=False,
            )
            self.source_status_pub.publish(message)

        def publish_trajectory(
            self,
            plan_id: str,
            joint_names: list[str],
            positions: list[np.ndarray],
        ) -> None:
            hash_value = trajectory_hash(joint_names, positions)
            self.source_hash_by_plan[plan_id] = hash_value
            self.source_point_count_by_plan[plan_id] = len(positions)
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.header.frame_id = f"base_link;plan_id={plan_id};trajectory_hash={hash_value}"
            trajectory.joint_names = list(joint_names)
            for index, q in enumerate(positions):
                point = JointTrajectoryPoint()
                point.positions = [float(value) for value in np.asarray(q).tolist()]
                point.time_from_start.sec = index
                point.time_from_start.nanosec = 0
                trajectory.points.append(point)
            self.source_pub.publish(trajectory)

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

    def force_status_tick(node: Any, executor: MultiThreadedExecutor) -> None:
        node.publish_status()
        executor.spin_once(timeout_sec=0.02)

    def latest_status(observer: Observer) -> dict[str, Any]:
        return observer.status_messages[-1] if observer.status_messages else {}

    def latest_valid(observer: Observer) -> bool | None:
        return observer.valid_messages[-1] if observer.valid_messages else None

    def latest_matching_trajectory(
        observer: Observer,
        output_point_count: int,
    ) -> JointTrajectory | None:
        for trajectory in reversed(observer.trajectories):
            if len(trajectory.points) == output_point_count:
                return trajectory
        return None

    def statuses_since(observer: Observer, start_index: int) -> list[dict[str, Any]]:
        return observer.status_messages[start_index:]

    def valids_since(observer: Observer, start_index: int) -> list[bool]:
        return observer.valid_messages[start_index:]

    def publish_heartbeat_for(
        executor: MultiThreadedExecutor,
        safety_node: Any,
        observer: Observer,
        plan_id: str,
        duration_s: float,
        period_s: float = 0.1,
    ) -> None:
        deadline = time.monotonic() + duration_s
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            observer.publish_plan_valid(True)
            observer.publish_source_status(plan_id)
            spin_for(executor, 0.03)
            force_status_tick(safety_node, executor)
            next_publish += period_s
            while time.monotonic() < min(next_publish, deadline):
                executor.spin_once(timeout_sec=0.02)

    def publish_case(
        executor: MultiThreadedExecutor,
        safety_node: Any,
        observer: Observer,
        positions: list[np.ndarray],
        plan_id: str,
        joint_names: list[str] | None = None,
    ) -> None:
        source_joint_names = joint_names or EXPECTED_JOINT_NAMES
        hash_value = trajectory_hash(source_joint_names, positions)
        observer.source_hash_by_plan[plan_id] = hash_value
        observer.source_point_count_by_plan[plan_id] = len(positions)
        observer.publish_plan_valid(True)
        observer.publish_source_status(
            plan_id,
            trajectory_republished=True,
            hash_value=hash_value,
            point_count=len(positions),
        )
        spin_for(executor, 0.08)
        observer.publish_trajectory(plan_id, source_joint_names, positions)
        force_status_tick(safety_node, executor)

    config = load_safety_config(PROJECT_ROOT)
    lower, upper, metadata = load_joint_limits(PROJECT_ROOT)
    paths = build_good_paths(lower, upper)
    report["configuration"] = {
        "config_path": str(PROJECT_ROOT / "config" / "trajectory_safety.json"),
        "status": config.status,
        "joint_names": config.joint_names,
        "maximum_velocity_rad_s": config.maximum_velocity_rad_s.tolist(),
        "maximum_acceleration_rad_s2": config.maximum_acceleration_rad_s2.tolist(),
        "sample_rate_hz": config.sample_rate_hz,
        "minimum_segment_duration_s": config.minimum_segment_duration_s,
        "maximum_segment_duration_s": config.maximum_segment_duration_s,
        "maximum_total_duration_s": config.maximum_total_duration_s,
        "maximum_input_adjacent_delta_rad": config.maximum_input_adjacent_delta_rad,
        "minimum_joint_limit_margin_rad": config.minimum_joint_limit_margin_rad,
        "input_stale_timeout_s": config.input_stale_timeout_s,
        "source_valid_heartbeat_timeout_s": config.source_valid_heartbeat_timeout_s,
        "trajectory_payload_timeout_before_first_plan_s": (
            config.trajectory_payload_timeout_before_first_plan_s
        ),
        "require_periodic_trajectory_republish": (
            config.require_periodic_trajectory_republish
        ),
        "stop_at_each_source_waypoint": config.stop_at_each_source_waypoint,
    }
    report["model"]["joint_limits_rad"] = metadata["joint_limits_rad"]

    validation = validate_source_path(
        EXPECTED_JOINT_NAMES,
        [point.tolist() for point in paths["normal"]],
        config,
        lower,
        upper,
    )
    direct_a = parameterize_path(validation.positions, config)
    direct_b = parameterize_path(validation.positions, config)
    direct_repeat_passed = bool(
        direct_a.success
        and direct_b.success
        and len(direct_a.points) == len(direct_b.points)
        and all(
            abs(a.time_from_start_s - b.time_from_start_s) <= 1.0e-12
            and np.max(np.abs(a.positions - b.positions)) <= 1.0e-12
            and np.max(np.abs(a.velocities - b.velocities)) <= 1.0e-12
            and np.max(np.abs(a.accelerations - b.accelerations)) <= 1.0e-12
            for a, b in zip(direct_a.points, direct_b.points, strict=True)
        )
    )
    report["direct_parameterizer"] = {
        "deterministic_repeat_passed": direct_repeat_passed,
        "normal_success": direct_a.success,
        "normal_reason": direct_a.reason,
        "normal_output_point_count": len(direct_a.points),
        "normal_total_duration_s": direct_a.total_duration_s,
        "normal_max_velocity_rad_s_observed": direct_a.maximum_velocity_rad_s_observed,
        "normal_max_acceleration_rad_s2_observed": direct_a.maximum_acceleration_rad_s2_observed,
    }
    if not direct_repeat_passed:
        fail(report, "direct_parameterizer", "Minimum-jerk output was not deterministic.")

    rclpy.init()
    observer = Observer()
    safety_node = TimedTrajectoryNode()
    executor = MultiThreadedExecutor(num_threads=4)
    for ros_node in (observer, safety_node):
        executor.add_node(ros_node)

    try:
        spin_for(executor, 0.6)
        report["node_execution"] = {
            "mode": "in_process_rclpy_executor",
            "nodes_started": ["so101_stage_2d2_observer", "timed_trajectory_node"],
            "camera_started": False,
            "hardware_started": False,
            "controller_command_publishers_started": False,
        }

        topic_names = sorted(
            name
            for name, _types in observer.get_topic_names_and_types()
        )
        forbidden_seen = [
            topic
            for topic in FORBIDDEN_CONTROLLER_TOPICS
            if topic in topic_names
        ]
        report["ros_graph_safety"] = {
            "forbidden_controller_topics": FORBIDDEN_CONTROLLER_TOPICS,
            "forbidden_topics_seen": forbidden_seen,
            "passed": not forbidden_seen,
            "topic_names": topic_names,
        }
        if forbidden_seen:
            fail(report, "ros_graph_safety", f"Forbidden command topics seen: {forbidden_seen}")

        cases: list[dict[str, Any]] = []
        plan_a = "stage2d2-plan-a"
        plan_b = "stage2d2-plan-b"
        plan_zero = "stage2d2-plan-zero"

        normal_trajectory_count_before = len(observer.trajectories)
        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        normal_ready = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "VALID"
            and len(observer.trajectories) > normal_trajectory_count_before,
            timeout_s=5.0,
        )
        normal_status = latest_status(observer)
        normal_trajectory = latest_matching_trajectory(
            observer,
            int(normal_status.get("output_point_count", 0)),
        )
        normal_output_passed = False
        normal_checks: dict[str, Any] = {}
        if normal_trajectory is not None:
            normal_output_passed, normal_checks = verify_output_trajectory(
                normal_trajectory,
                normal_status,
                paths["normal"],
                config.maximum_velocity_rad_s,
                config.maximum_acceleration_rad_s2,
            )
        normal_case = {
            "name": "normal_valid_trajectory",
            "passed": bool(
                normal_ready
                and latest_valid(observer) is True
                and normal_status.get("status") == "VALID"
                and normal_status.get("reason") == "time_parameterized_preview_valid"
                and normal_status.get("limit_profile_status")
                == "provisional_software_preview_limits"
                and normal_status.get("hardware_control_enabled") is False
                and normal_status.get("published_controller_command_topics") == []
                and normal_output_passed
            ),
            "latest_status": normal_status,
            "output_checks": normal_checks,
        }
        cases.append(normal_case)
        if not normal_case["passed"]:
            fail(report, "normal_valid_trajectory", f"Normal trajectory failed: {normal_status}")

        zero_trajectory_count_before = len(observer.trajectories)
        publish_case(executor, safety_node, observer, paths["zero_segment"], plan_zero)
        zero_ready = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "VALID"
            and latest_status(observer).get("source_waypoint_count") == len(paths["zero_segment"])
            and len(observer.trajectories) > zero_trajectory_count_before,
            timeout_s=5.0,
        )
        zero_status = latest_status(observer)
        zero_trajectory = latest_matching_trajectory(
            observer,
            int(zero_status.get("output_point_count", 0)),
        )
        zero_output_passed = False
        zero_checks: dict[str, Any] = {}
        if zero_trajectory is not None:
            zero_output_passed, zero_checks = verify_output_trajectory(
                zero_trajectory,
                zero_status,
                paths["zero_segment"],
                config.maximum_velocity_rad_s,
                config.maximum_acceleration_rad_s2,
            )
        zero_case = {
            "name": "zero_displacement_segment",
            "passed": bool(zero_ready and zero_status.get("status") == "VALID" and zero_output_passed),
            "latest_status": zero_status,
            "output_checks": zero_checks,
        }
        cases.append(zero_case)
        if not zero_case["passed"]:
            fail(report, "zero_displacement_segment", f"Zero segment failed: {zero_status}")

        def invalid_case(name: str, positions: list[np.ndarray], expected_reason: str, joint_names: list[str] | None = None) -> None:
            status_start = len(observer.status_messages)
            valid_start = len(observer.valid_messages)
            publish_case(
                executor,
                safety_node,
                observer,
                positions,
                f"stage2d2-invalid-{name}",
                joint_names,
            )
            wait_until(
                executor,
                lambda: any(
                    status.get("status") == "INVALID"
                    and status.get("reason") == expected_reason
                    for status in statuses_since(observer, status_start)
                ),
                timeout_s=1.0,
            )
            matching_statuses = [
                status
                for status in statuses_since(observer, status_start)
                if status.get("status") == "INVALID"
                and status.get("reason") == expected_reason
            ]
            status = matching_statuses[-1] if matching_statuses else latest_status(observer)
            valid_values = valids_since(observer, valid_start)
            item = {
                "name": name,
                "passed": bool(
                    matching_statuses
                    and any(value is False for value in valid_values)
                    and status.get("status") == "INVALID"
                    and status.get("reason") == expected_reason
                ),
                "expected_reason": expected_reason,
                "matched_status": status,
                "latest_status": latest_status(observer),
            }
            cases.append(item)
            if not item["passed"]:
                fail(report, name, f"Expected {expected_reason}, got {status}")

        wrong_names = ["bad_joint"] + EXPECTED_JOINT_NAMES[1:]
        invalid_case("wrong_names", paths["normal"], "wrong_joint_names", wrong_names)
        wrong_order = EXPECTED_JOINT_NAMES[1:] + [EXPECTED_JOINT_NAMES[0]]
        invalid_case("wrong_order", paths["normal"], "wrong_joint_order", wrong_order)
        nan_path = [point.copy() for point in paths["normal"]]
        nan_path[1] = nan_path[1].copy()
        nan_path[1][2] = math.nan
        invalid_case("nan_inf_position", nan_path, "non_finite_position")
        out_path = [point.copy() for point in paths["normal"]]
        out_path[1] = out_path[1].copy()
        out_path[1][0] = lower[0] - 0.01
        invalid_case("out_of_bounds", out_path, "joint_position_out_of_bounds")
        margin_path = [point.copy() for point in paths["normal"]]
        margin_path[1] = margin_path[1].copy()
        margin_path[1][0] = lower[0] + 0.03
        invalid_case("margin_insufficient", margin_path, "joint_limit_margin_insufficient")
        jump_path = [paths["normal"][0].copy(), paths["normal"][0].copy()]
        jump_path[1][0] += 0.151
        invalid_case("adjacent_jump_too_large", jump_path, "input_adjacent_delta_exceeds_limit")
        invalid_case("total_duration_too_long", paths["too_long"], "total_duration_exceeds_limit")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "VALID",
            timeout_s=5.0,
        )
        false_start = time.monotonic()
        observer.publish_plan_valid(False)
        force_status_tick(safety_node, executor)
        false_invalidated = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "INVALID"
            and latest_status(observer).get("reason") == "grasp_plan_valid_false",
            timeout_s=0.70,
        )
        false_elapsed = time.monotonic() - false_start
        false_case = {
            "name": "explicit_valid_false_invalidates",
            "passed": bool(false_invalidated and latest_valid(observer) is False and false_elapsed <= 0.70),
            "elapsed_s": false_elapsed,
            "latest_status": latest_status(observer),
        }
        cases.append(false_case)
        if not false_case["passed"]:
            fail(report, "explicit_valid_false_invalidates", f"False invalidation failed: {false_case}")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(executor, lambda: latest_status(observer).get("status") == "VALID", timeout_s=5.0)
        event_start_status = len(observer.status_messages)
        event_start_valid = len(observer.valid_messages)
        event_start_recompute = int(latest_status(observer).get("reparameterization_count", -1))
        publish_heartbeat_for(executor, safety_node, observer, plan_a, duration_s=3.1)
        event_statuses = statuses_since(observer, event_start_status)
        event_valids = valids_since(observer, event_start_valid)
        event_latest = latest_status(observer)
        event_case = {
            "name": "event_driven_trajectory_cached_hold",
            "passed": bool(
                event_valids
                and all(event_valids)
                and event_latest.get("status") == "VALID"
                and event_latest.get("using_cached_timed_trajectory") is True
                and event_latest.get("reparameterization_count") == event_start_recompute
            ),
            "duration_s": 3.1,
            "false_count": sum(1 for value in event_valids if not value),
            "reparameterization_count_before": event_start_recompute,
            "reparameterization_count_after": event_latest.get("reparameterization_count"),
            "latest_status": event_latest,
            "status_count": len(event_statuses),
        }
        cases.append(event_case)
        if not event_case["passed"]:
            fail(report, "event_driven_trajectory_cached_hold", f"Cached hold failed: {event_case}")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(executor, lambda: latest_status(observer).get("status") == "VALID", timeout_s=5.0)
        spin_for(executor, 0.25)
        static_start_valid = len(observer.valid_messages)
        static_start_recompute = int(latest_status(observer).get("reparameterization_count", -1))
        publish_heartbeat_for(executor, safety_node, observer, plan_a, duration_s=5.1)
        static_valids = valids_since(observer, static_start_valid)
        static_latest = latest_status(observer)
        static_case = {
            "name": "static_object_valid_for_five_seconds",
            "passed": bool(
                static_valids
                and all(static_valids)
                and static_latest.get("status") == "VALID"
                and static_latest.get("reparameterization_count") == static_start_recompute
            ),
            "duration_s": 5.1,
            "false_count": sum(1 for value in static_valids if not value),
            "reparameterization_count_before": static_start_recompute,
            "reparameterization_count_after": static_latest.get("reparameterization_count"),
            "latest_status": static_latest,
        }
        cases.append(static_case)
        if not static_case["passed"]:
            fail(report, "static_object_valid_for_five_seconds", f"Static hold failed: {static_case}")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(executor, lambda: latest_status(observer).get("status") == "VALID", timeout_s=5.0)
        heartbeat_start = time.monotonic()
        heartbeat_invalidated = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "INVALID"
            and latest_status(observer).get("reason") == "upstream_valid_heartbeat_stale",
            timeout_s=config.source_valid_heartbeat_timeout_s + 0.4,
        )
        heartbeat_elapsed = time.monotonic() - heartbeat_start
        heartbeat_case = {
            "name": "heartbeat_stopped_invalidates",
            "passed": bool(
                heartbeat_invalidated
                and latest_valid(observer) is False
                and heartbeat_elapsed <= config.source_valid_heartbeat_timeout_s + 0.4
            ),
            "elapsed_s": heartbeat_elapsed,
            "latest_status": latest_status(observer),
        }
        cases.append(heartbeat_case)
        if not heartbeat_case["passed"]:
            fail(report, "heartbeat_stopped_invalidates", f"Heartbeat stale failed: {heartbeat_case}")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(executor, lambda: latest_status(observer).get("status") == "VALID", timeout_s=5.0)
        reparam_before = int(latest_status(observer).get("reparameterization_count", -1))
        publish_heartbeat_for(executor, safety_node, observer, plan_a, duration_s=0.5)
        after_hold = int(latest_status(observer).get("reparameterization_count", -1))
        shifted_b = [point.copy() for point in paths["normal"]]
        shifted_b[1] = shifted_b[1] + np.asarray([0.01, 0.0, 0.0, 0.0, 0.0])
        publish_case(executor, safety_node, observer, shifted_b, plan_b)
        wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "VALID"
            and latest_status(observer).get("active_plan_id") == plan_b,
            timeout_s=5.0,
        )
        reparam_after = int(latest_status(observer).get("reparameterization_count", -1))
        new_plan_case = {
            "name": "new_plan_reparameterizes_once",
            "passed": bool(after_hold == reparam_before and reparam_after == after_hold + 1),
            "reparameterization_count_before_hold": reparam_before,
            "reparameterization_count_after_hold": after_hold,
            "reparameterization_count_after_plan_b": reparam_after,
            "latest_status": latest_status(observer),
        }
        cases.append(new_plan_case)
        if not new_plan_case["passed"]:
            fail(report, "new_plan_reparameterizes_once", f"New plan reparameterization failed: {new_plan_case}")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(executor, lambda: latest_status(observer).get("status") == "VALID", timeout_s=5.0)
        observer.publish_plan_valid(True)
        observer.publish_source_status(
            "stage2d2-plan-c",
            hash_value=trajectory_hash(EXPECTED_JOINT_NAMES, shifted_b),
            point_count=len(shifted_b),
        )
        force_status_tick(safety_node, executor)
        waiting_seen = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "INVALID"
            and latest_status(observer).get("reason") == "waiting_for_matching_source_trajectory",
            timeout_s=1.0,
        )
        publish_case(executor, safety_node, observer, shifted_b, "stage2d2-plan-c")
        recovered = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "VALID"
            and latest_status(observer).get("active_plan_id") == "stage2d2-plan-c",
            timeout_s=5.0,
        )
        waiting_case = {
            "name": "new_plan_waiting_for_trajectory",
            "passed": bool(recovered),
            "waiting_status_seen": waiting_seen,
            "hash_cache_reused_without_waiting": bool(recovered and not waiting_seen),
            "recovered": recovered,
            "latest_status": latest_status(observer),
        }
        cases.append(waiting_case)
        if not waiting_case["passed"]:
            fail(report, "new_plan_waiting_for_trajectory", f"New plan waiting failed: {waiting_case}")

        publish_case(executor, safety_node, observer, paths["normal"], plan_a)
        wait_until(executor, lambda: latest_status(observer).get("status") == "VALID", timeout_s=5.0)
        planner_stale_start = time.monotonic()
        observer.publish_plan_valid(False)
        force_status_tick(safety_node, executor)
        planner_invalidated = wait_until(
            executor,
            lambda: latest_status(observer).get("status") == "INVALID"
            and latest_status(observer).get("reason") == "grasp_plan_valid_false",
            timeout_s=0.70,
        )
        planner_stale_case = {
            "name": "planner_pose_stale_invalidates",
            "passed": bool(planner_invalidated),
            "elapsed_s": time.monotonic() - planner_stale_start,
            "first_invalid_node": "/grasp_plan_valid",
            "first_invalid_reason": "grasp_plan_valid_false",
            "latest_status": latest_status(observer),
        }
        cases.append(planner_stale_case)
        if not planner_stale_case["passed"]:
            fail(report, "planner_pose_stale_invalidates", f"Planner stale sequence failed: {planner_stale_case}")

        report["synthetic_cases"] = cases
        if not all(item["passed"] for item in cases):
            fail(report, "synthetic_cases", "One or more Stage 2D-2 synthetic cases failed.")

        code, executables = command_output(
            [
                "cmd.exe",
                "/d",
                "/s",
                "/c",
                (
                    "ros2 pkg executables so101_trajectory_safety & "
                    "ros2 pkg executables so101_grasp_planner"
                ),
            ],
            timeout_s=20.0,
        )
        report["executables"] = {
            "return_code": code,
            "raw_output": executables,
            "timed_trajectory_node": (
                "so101_trajectory_safety timed_trajectory_node" in executables
            ),
            "visual_grasp_planner_node": (
                "so101_grasp_planner visual_grasp_planner_node" in executables
            ),
        }
        if not report["executables"]["timed_trajectory_node"]:
            fail(report, "timed_trajectory_node_executable", "Executable is not registered.")

        report["live_vision_to_safe_timed_grasp_validation"] = {
            "status": "NOT_RUN",
            "reason": (
                "Physical camera and object-motion validation is not faked by this "
                "offline verifier."
            ),
            "launch_command": (
                "ros2 launch so101_trajectory_safety "
                "perception_to_safe_timed_grasp_dry_run.launch.py"
            ),
        }
    finally:
        for ros_node in (safety_node, observer):
            executor.remove_node(ros_node)
            ros_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def run_inside_ros() -> int:
    ensure_dirs()
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
    os.environ["RCL_LOGGING_IMPLEMENTATION"] = "rcl_logging_noop"
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_trajectory_safety"))
    sys.path.insert(0, str(ROS2_WS / "src" / "so101_kinematics"))
    append_log("Stage 2D-2 verification started inside ROS2 environment.")
    urdf_sha = sha256_file(URDF_PATH)
    report: dict[str, Any] = {
        "stage": "2D-2",
        "status": "FAIL",
        "timestamp": iso_timestamp(),
        "environment": {
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "python": sys.executable,
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "zenoh_router_check_attempts": os.environ.get(
                "ZENOH_ROUTER_CHECK_ATTEMPTS",
                "",
            ),
            "rcl_logging_implementation": os.environ.get(
                "RCL_LOGGING_IMPLEMENTATION",
                "",
            ),
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
            "limit_profile_status": "provisional_software_preview_limits",
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
        run_stage_2d1_regression(report)
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
        append_log(f"Stage 2D-2 status: {report['status']}")
    return 0 if report["status"] == "PASS" else 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify SO-101 Stage 2D-2 timed grasp trajectory safety."
    )
    parser.add_argument("--inside-ros", action="store_true")
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros()
    return run_outside_ros()


if __name__ == "__main__":
    raise SystemExit(main())
