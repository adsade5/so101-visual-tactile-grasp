from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_WS = PROJECT_ROOT / "ros2_ws"
RUN_IN_ROS2 = PROJECT_ROOT / "audit" / "run_in_ros2_lyrical.ps1"
ROS2_PYTHON = Path(r"C:\pixi_ws\.pixi\envs\default\python.exe")
REPORT_DIR = PROJECT_ROOT / "data" / "verification"
REPORT_PATH = REPORT_DIR / "stage_2d1_report.json"
LOG_PATH = REPORT_DIR / "stage_2d1_verification.log"
PROCESS_LOG_DIR = REPORT_DIR / "stage_2d1_logs"
ROS_LOG_DIR = REPORT_DIR / "ros_logs"

EXPECTED_URDF_SHA256 = (
    "3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c"
)
WORKSPACE_CONFIG = PROJECT_ROOT / "config" / "workspace_to_base.json"
URDF_PATH = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)

POSITION_TOLERANCE_MM = 1.0
APPROACH_TOLERANCE_DEG = 2.0
MINIMUM_LIMIT_MARGIN_RAD = 0.05
MAXIMUM_ADJACENT_DELTA_RAD = 0.15
STALE_TIMEOUT_S = 0.5


def iso_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def filename_timestamp() -> str:
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


def sha256_file(path: Path) -> str:
    import hashlib

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


def run_outside_ros(run_live: bool) -> int:
    ensure_dirs()
    if LOG_PATH.exists():
        LOG_PATH.unlink()

    command_file = Path(tempfile.gettempdir()) / (
        f"verify_stage_2d1_{filename_timestamp()}.cmd"
    )
    command = "\n".join(
        [
            f'cd /d "{ROS2_WS}"',
            f'set "ROS_LOG_DIR={ROS_LOG_DIR}"',
            'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"',
            'set "ZENOH_ROUTER_CHECK_ATTEMPTS=1"',
            'set "RCL_LOGGING_IMPLEMENTATION=rcl_logging_noop"',
            "call install\\local_setup.bat",
            (
                f'"{ROS2_PYTHON}" "{Path(__file__).resolve()}" --inside-ros'
                + (" --run-live" if run_live else "")
            ),
        ]
    )
    command_file.write_text(command, encoding="ascii")
    try:
        append_log("Entering ROS2 Lyrical environment for Stage 2D-1.")
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
        log_path = PROCESS_LOG_DIR / f"{name}_{filename_timestamp()}.log"
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
        managed = ManagedProcess(name, command, process, log_path, log_file)
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
                    ["taskkill", "/PID", str(managed.process.pid), "/T", "/F"],
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


def fail(report: dict[str, Any], item: str, detail: str) -> None:
    report.setdefault("failures", []).append({"item": item, "detail": detail})


def yaw_matrix(yaw_deg: float) -> np.ndarray:
    yaw = math.radians(float(yaw_deg))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.asarray([[c, -s], [s, c]], dtype=np.float64)


def base_to_workspace(base_position: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    translation = np.asarray(config["translation_m"], dtype=np.float64)
    rotation = yaw_matrix(float(config["yaw_deg"]))
    planar = rotation.T @ (base_position[:2] - translation[:2])
    return np.asarray(
        [float(planar[0]), float(planar[1]), float(base_position[2] - translation[2])],
        dtype=np.float64,
    )


def verify_with_ros(report: dict[str, Any], run_live: bool) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from trajectory_msgs.msg import JointTrajectory

    class Observer(Node):
        def __init__(self) -> None:
            super().__init__("so101_stage_2d1_observer")
            self.object_pose_base_valid: list[bool] = []
            self.plan_valid: list[bool] = []
            self.status_messages: list[dict[str, Any]] = []
            self.reasons: list[str] = []
            self.object_pose_base: list[PoseStamped] = []
            self.pregrasp_pose: list[PoseStamped] = []
            self.grasp_pose: list[PoseStamped] = []
            self.trajectories: list[JointTrajectory] = []
            self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", 10)
            self.detected_pub = self.create_publisher(Bool, "/object_detected", 10)
            self.stable_pub = self.create_publisher(Bool, "/object_pose_stable", 10)
            self.create_subscription(
                Bool,
                "/object_pose_base_valid",
                lambda msg: self.object_pose_base_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                Bool,
                "/grasp_plan_valid",
                lambda msg: self.plan_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(PoseStamped, "/object_pose_base", self.object_pose_base.append, 50)
            self.create_subscription(PoseStamped, "/grasp_pregrasp_pose", self.pregrasp_pose.append, 50)
            self.create_subscription(PoseStamped, "/grasp_target_pose", self.grasp_pose.append, 50)
            self.create_subscription(JointTrajectory, "/planned_grasp_joint_trajectory", self.trajectories.append, 50)
            self.create_subscription(String, "/grasp_planner_validity_reason", lambda msg: self.reasons.append(str(msg.data)), 50)
            self.create_subscription(String, "/grasp_plan_status", self.on_status, 50)

        def on_status(self, message: String) -> None:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                value = {"status": "INVALID_JSON", "raw": message.data}
            self.status_messages.append(value)

        def publish_workspace_pose(
            self,
            workspace_position: np.ndarray,
            frame_id: str = "workspace_plane",
            stable: bool = True,
            yaw_deg: float = 0.0,
        ) -> None:
            detected_msg = Bool()
            detected_msg.data = bool(stable)
            self.detected_pub.publish(detected_msg)
            stable_msg = Bool()
            stable_msg.data = bool(stable)
            self.stable_pub.publish(stable_msg)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(workspace_position[0])
            pose.pose.position.y = float(workspace_position[1])
            pose.pose.position.z = float(workspace_position[2])
            yaw = math.radians(yaw_deg)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.pose_pub.publish(pose)

        def publish_stable_only(self, stable: bool) -> None:
            stable_msg = Bool()
            stable_msg.data = bool(stable)
            self.stable_pub.publish(stable_msg)

    def spin_for(node: Observer, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def publish_for(
        node: Observer,
        workspace_position: np.ndarray,
        duration_s: float,
        frame_id: str = "workspace_plane",
        stable: bool = True,
    ) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            node.publish_workspace_pose(workspace_position, frame_id=frame_id, stable=stable)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.03)

    def wait_until(node: Observer, predicate: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def latest_status(node: Observer) -> dict[str, Any]:
        return node.status_messages[-1] if node.status_messages else {}

    def latest_plan_valid(node: Observer) -> bool | None:
        return node.plan_valid[-1] if node.plan_valid else None

    config = read_json(WORKSPACE_CONFIG)
    rclpy.init()
    node = Observer()
    pm = ProcessManager()
    try:
        router = pm.start("rmw_zenoh_router", "ros2 run rmw_zenoh_cpp rmw_zenohd")
        time.sleep(2.0)
        report["zenoh_router"] = {
            "started": router.process.poll() is None,
            "return_code": router.process.poll(),
            "log": str(router.log_path),
        }
        if router.process.poll() is not None:
            fail(report, "zenoh_router", "rmw_zenohd exited before nodes started.")

        transform = pm.start(
            "workspace_to_base_node",
            (
                "ros2 run so101_frame_transform workspace_to_base_node "
                "--ros-args "
                f'-p project_root:="{PROJECT_ROOT.as_posix()}"'
            ),
        )
        planner = pm.start(
            "visual_grasp_planner_node",
            (
                "ros2 run so101_grasp_planner visual_grasp_planner_node "
                "--ros-args "
                f'-p project_root:="{PROJECT_ROOT.as_posix()}"'
            ),
        )
        spin_for(node, 5.0)

        code, executables = command_output(
            (
                "ros2 pkg executables so101_object_perception & "
                "ros2 pkg executables so101_frame_transform & "
                "ros2 pkg executables so101_grasp_planner"
            ),
            timeout_s=20.0,
        )
        report["executables"] = {
            "return_code": code,
            "raw_output": executables,
            "object_pose_node": "so101_object_perception object_pose_node" in executables,
            "workspace_to_base_node": (
                "so101_frame_transform workspace_to_base_node" in executables
            ),
            "visual_grasp_planner_node": (
                "so101_grasp_planner visual_grasp_planner_node" in executables
            ),
        }
        for name in ("object_pose_node", "workspace_to_base_node", "visual_grasp_planner_node"):
            if not report["executables"].get(name):
                fail(report, f"executable_{name}", f"{name} is not registered.")

        base_good = np.asarray(
            [0.18289733886666232, 0.0, 0.025],
            dtype=np.float64,
        )
        workspace_good = base_to_workspace(base_good, config)
        append_log(f"Synthetic valid base target: {base_good.tolist()}")
        append_log(f"Synthetic valid workspace input: {workspace_good.tolist()}")

        publish_for(node, workspace_good, duration_s=4.0)
        valid_ready = wait_until(
            node,
            lambda: (
                any(node.object_pose_base_valid)
                and any(node.plan_valid)
                and bool(node.trajectories)
                and latest_status(node).get("status") == "VALID"
            ),
            timeout_s=30.0,
        )
        status = latest_status(node)
        trajectory = node.trajectories[-1] if node.trajectories else None
        pregrasp = node.pregrasp_pose[-1] if node.pregrasp_pose else None
        grasp = node.grasp_pose[-1] if node.grasp_pose else None
        trajectory_count = 0 if trajectory is None else len(trajectory.points)
        joint_names = [] if trajectory is None else list(trajectory.joint_names)
        pregrasp_z = None if pregrasp is None else float(pregrasp.pose.position.z)
        grasp_z = None if grasp is None else float(grasp.pose.position.z)
        adjacent_max = status.get("max_adjacent_delta_rad")
        synthetic_passed = bool(
            valid_ready
            and latest_plan_valid(node) is True
            and status.get("status") == "VALID"
            and status.get("max_position_error_mm") is not None
            and status["max_position_error_mm"] <= POSITION_TOLERANCE_MM
            and status.get("max_approach_error_deg") is not None
            and status["max_approach_error_deg"] <= APPROACH_TOLERANCE_DEG
            and status.get("min_margin_rad") is not None
            and status["min_margin_rad"] >= MINIMUM_LIMIT_MARGIN_RAD
            and adjacent_max is not None
            and adjacent_max <= MAXIMUM_ADJACENT_DELTA_RAD
            and joint_names
            == [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ]
            and trajectory_count == status.get("waypoint_count")
            and pregrasp is not None
            and pregrasp.header.frame_id == "base_link"
            and grasp is not None
            and grasp.header.frame_id == "base_link"
            and pregrasp_z is not None
            and abs(pregrasp_z - (base_good[2] + 0.055)) <= 1.0e-6
            and grasp_z is not None
            and abs(grasp_z - (base_good[2] + 0.015)) <= 1.0e-6
        )
        report["synthetic_valid_case"] = {
            "passed": synthetic_passed,
            "base_target_m": base_good.tolist(),
            "workspace_input_m": workspace_good.tolist(),
            "object_pose_base_valid_seen": any(node.object_pose_base_valid),
            "grasp_plan_valid_seen": any(node.plan_valid),
            "latest_status": status,
            "trajectory_point_count": trajectory_count,
            "joint_names": joint_names,
            "pregrasp_z_m": pregrasp_z,
            "grasp_z_m": grasp_z,
        }
        if not synthetic_passed:
            fail(report, "synthetic_valid_case", f"Valid synthetic case failed: {status}")

        pose_count_before = len(node.trajectories)
        spin_for(node, 0.8)
        stale_status = latest_status(node)
        stale_passed = bool(
            latest_plan_valid(node) is False
            and stale_status.get("status") == "INVALID"
            and stale_status.get("reason") == "object_pose_base_stale"
        )
        report["stale_invalidation_case"] = {
            "passed": stale_passed,
            "latest_status": stale_status,
            "trajectory_count_before_stale": pose_count_before,
            "trajectory_count_after_stale": len(node.trajectories),
        }
        if not stale_passed:
            fail(report, "stale_invalidation_case", f"Stale invalidation failed: {stale_status}")

        bad_cases: list[dict[str, Any]] = []

        before_reason_count = len(node.reasons)
        publish_for(node, workspace_good, duration_s=0.7, frame_id="bad_frame")
        wrong_frame_status = latest_status(node)
        wrong_frame_passed = bool(
            latest_plan_valid(node) is False
            and wrong_frame_status.get("status") == "INVALID"
        )
        bad_cases.append(
            {
                "name": "wrong_frame",
                "passed": wrong_frame_passed,
                "latest_status": wrong_frame_status,
                "new_reason_count": len(node.reasons) - before_reason_count,
            }
        )
        if not wrong_frame_passed:
            fail(report, "wrong_frame_case", f"Wrong frame was not rejected: {wrong_frame_status}")

        workspace_nan = workspace_good.copy()
        workspace_nan[0] = math.nan
        publish_for(node, workspace_nan, duration_s=0.7)
        nan_status = latest_status(node)
        nan_passed = bool(latest_plan_valid(node) is False and nan_status.get("status") == "INVALID")
        bad_cases.append({"name": "nan_input", "passed": nan_passed, "latest_status": nan_status})
        if not nan_passed:
            fail(report, "nan_input_case", f"NaN input was not rejected: {nan_status}")

        base_out_of_bounds = np.asarray([0.40, 0.0, 0.025], dtype=np.float64)
        out_start = len(node.status_messages)
        publish_for(node, base_to_workspace(base_out_of_bounds, config), duration_s=2.0)
        out_matches = [
            status
            for status in node.status_messages[out_start:]
            if "out_of_bounds" in str(status.get("reason"))
        ]
        out_status = out_matches[-1] if out_matches else latest_status(node)
        out_passed = bool(out_status.get("status") == "INVALID" and out_matches)
        bad_cases.append({"name": "workspace_out_of_bounds", "passed": out_passed, "latest_status": out_status})
        if not out_passed:
            fail(report, "workspace_out_of_bounds_case", f"Out-of-bounds input was not rejected: {out_status}")

        base_unreachable = np.asarray([0.32, 0.12, 0.025], dtype=np.float64)
        publish_for(node, base_to_workspace(base_unreachable, config), duration_s=4.0)
        unreachable_status = latest_status(node)
        unreachable_passed = bool(
            latest_plan_valid(node) is False
            and unreachable_status.get("status") == "INVALID"
            and str(unreachable_status.get("reason", "")).startswith("ik_failed:")
        )
        bad_cases.append({"name": "unreachable_target", "passed": unreachable_passed, "latest_status": unreachable_status})
        if not unreachable_passed:
            fail(report, "unreachable_case", f"Unreachable target was not rejected by IK: {unreachable_status}")

        node.publish_stable_only(False)
        spin_for(node, 0.7)
        valid_false_status = latest_status(node)
        valid_false_passed = bool(
            latest_plan_valid(node) is False
            and valid_false_status.get("status") == "INVALID"
        )
        bad_cases.append({"name": "valid_false", "passed": valid_false_passed, "latest_status": valid_false_status})
        if not valid_false_passed:
            fail(report, "valid_false_case", f"valid=false did not invalidate plan: {valid_false_status}")

        report["synthetic_negative_cases"] = bad_cases
        if not all(item["passed"] for item in bad_cases):
            fail(report, "synthetic_negative_cases", "One or more negative cases failed.")

        report["live_vision_validation"] = {
            "status": "NOT_RUN" if not run_live else "REQUESTED_NOT_AUTOMATED",
            "reason": (
                "Physical center/left/right/removed-object validation requires a camera "
                "and moving the target in the workspace. This verifier does not fake "
                "physical observations."
            ),
            "launch_command": (
                "powershell -ExecutionPolicy Bypass -File "
                f'"{RUN_IN_ROS2}" -Command '
                f'"cd /d ""{ROS2_WS}"" && call install\\local_setup.bat && '
                "ros2 launch so101_grasp_planner perception_to_grasp_dry_run.launch.py "
                f'project_root:={PROJECT_ROOT.as_posix()} show_debug_window:=false"'
            ),
        }
        if run_live:
            fail(
                report,
                "live_vision_validation",
                "Live physical validation was requested but cannot be automated without operator motion/camera scene control.",
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


def verify_with_ros_in_process(report: dict[str, Any], run_live: bool) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from trajectory_msgs.msg import JointTrajectory

    for package_name in (
        "so101_kinematics",
        "so101_frame_transform",
        "so101_grasp_planner",
    ):
        sys.path.insert(0, str(ROS2_WS / "src" / package_name))

    from so101_frame_transform.workspace_to_base_node import WorkspaceToBaseNode
    from so101_grasp_planner.visual_grasp_planner_node import VisualGraspPlannerNode

    class Observer(Node):
        def __init__(self) -> None:
            super().__init__("so101_stage_2d1_observer")
            self.object_pose_base_valid: list[bool] = []
            self.plan_valid: list[bool] = []
            self.status_messages: list[dict[str, Any]] = []
            self.reasons: list[str] = []
            self.object_pose_base: list[PoseStamped] = []
            self.pregrasp_pose: list[PoseStamped] = []
            self.grasp_pose: list[PoseStamped] = []
            self.trajectories: list[JointTrajectory] = []
            self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", 10)
            self.detected_pub = self.create_publisher(Bool, "/object_detected", 10)
            self.stable_pub = self.create_publisher(Bool, "/object_pose_stable", 10)
            self.create_subscription(
                Bool,
                "/object_pose_base_valid",
                lambda msg: self.object_pose_base_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                Bool,
                "/grasp_plan_valid",
                lambda msg: self.plan_valid.append(bool(msg.data)),
                50,
            )
            self.create_subscription(
                PoseStamped,
                "/object_pose_base",
                self.object_pose_base.append,
                50,
            )
            self.create_subscription(
                PoseStamped,
                "/grasp_pregrasp_pose",
                self.pregrasp_pose.append,
                50,
            )
            self.create_subscription(
                PoseStamped,
                "/grasp_target_pose",
                self.grasp_pose.append,
                50,
            )
            self.create_subscription(
                JointTrajectory,
                "/planned_grasp_joint_trajectory",
                self.trajectories.append,
                50,
            )
            self.create_subscription(
                String,
                "/grasp_planner_validity_reason",
                lambda msg: self.reasons.append(str(msg.data)),
                50,
            )
            self.create_subscription(String, "/grasp_plan_status", self.on_status, 50)

        def on_status(self, message: String) -> None:
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                value = {"status": "INVALID_JSON", "raw": message.data}
            self.status_messages.append(value)

        def publish_workspace_pose(
            self,
            workspace_position: np.ndarray,
            frame_id: str = "workspace_plane",
            stable: bool = True,
        ) -> None:
            detected_msg = Bool()
            detected_msg.data = bool(stable)
            self.detected_pub.publish(detected_msg)
            stable_msg = Bool()
            stable_msg.data = bool(stable)
            self.stable_pub.publish(stable_msg)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(workspace_position[0])
            pose.pose.position.y = float(workspace_position[1])
            pose.pose.position.z = float(workspace_position[2])
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)

        def publish_stable_only(self, stable: bool) -> None:
            message = Bool()
            message.data = bool(stable)
            self.stable_pub.publish(message)

    def spin_for(executor: MultiThreadedExecutor, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

    def publish_for(
        executor: MultiThreadedExecutor,
        node: Observer,
        workspace_position: np.ndarray,
        duration_s: float,
        frame_id: str = "workspace_plane",
        stable: bool = True,
    ) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            node.publish_workspace_pose(
                workspace_position,
                frame_id=frame_id,
                stable=stable,
            )
            executor.spin_once(timeout_sec=0.05)
            time.sleep(0.03)

    def wait_until(
        executor: MultiThreadedExecutor,
        predicate: Any,
        timeout_s: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publish_until(
        executor: MultiThreadedExecutor,
        node: Observer,
        workspace_position: np.ndarray,
        predicate: Any,
        timeout_s: float,
        frame_id: str = "workspace_plane",
        stable: bool = True,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            node.publish_workspace_pose(
                workspace_position,
                frame_id=frame_id,
                stable=stable,
            )
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
            time.sleep(0.03)
        return False

    def latest_status(node: Observer) -> dict[str, Any]:
        return node.status_messages[-1] if node.status_messages else {}

    def latest_plan_valid(node: Observer) -> bool | None:
        return node.plan_valid[-1] if node.plan_valid else None

    def has_valid_status(node: Observer) -> bool:
        return any(item.get("status") == "VALID" for item in node.status_messages)

    config = read_json(WORKSPACE_CONFIG)
    rclpy.init()
    observer = Observer()
    planner = VisualGraspPlannerNode()
    transform = WorkspaceToBaseNode()
    executor = MultiThreadedExecutor(num_threads=4)
    for ros_node in (observer, transform, planner):
        executor.add_node(ros_node)

    try:
        def force_status_tick() -> None:
            transform.publish_validity()
            planner.publish_status()
            executor.spin_once(timeout_sec=0.05)

        code, executables = command_output(
            (
                "ros2 pkg executables so101_object_perception & "
                "ros2 pkg executables so101_frame_transform & "
                "ros2 pkg executables so101_grasp_planner"
            ),
            timeout_s=20.0,
        )
        report["executables"] = {
            "return_code": code,
            "raw_output": executables,
            "object_pose_node": "so101_object_perception object_pose_node" in executables,
            "workspace_to_base_node": (
                "so101_frame_transform workspace_to_base_node" in executables
            ),
            "visual_grasp_planner_node": (
                "so101_grasp_planner visual_grasp_planner_node" in executables
            ),
        }
        for name in (
            "object_pose_node",
            "workspace_to_base_node",
            "visual_grasp_planner_node",
        ):
            if not report["executables"].get(name):
                fail(report, f"executable_{name}", f"{name} is not registered.")

        report["node_execution"] = {
            "mode": "in_process_rclpy_executor",
            "nodes_started": [
                "so101_stage_2d1_observer",
                "workspace_to_base_node",
                "visual_grasp_planner_node",
            ],
            "camera_started": False,
            "hardware_started": False,
        }

        base_good = np.asarray(
            [0.18289733886666232, 0.0, 0.025],
            dtype=np.float64,
        )
        workspace_good = base_to_workspace(base_good, config)
        append_log(f"Synthetic valid base target: {base_good.tolist()}")
        append_log(f"Synthetic valid workspace input: {workspace_good.tolist()}")

        spin_for(executor, 1.0)
        valid_ready = publish_until(
            executor,
            observer,
            workspace_good,
            lambda: (
                any(observer.object_pose_base_valid)
                and any(observer.plan_valid)
                and bool(observer.trajectories)
                and has_valid_status(observer)
            ),
            timeout_s=20.0,
        )
        status = latest_status(observer)
        trajectory = observer.trajectories[-1] if observer.trajectories else None
        pregrasp = observer.pregrasp_pose[-1] if observer.pregrasp_pose else None
        grasp = observer.grasp_pose[-1] if observer.grasp_pose else None
        trajectory_count = 0 if trajectory is None else len(trajectory.points)
        joint_names = [] if trajectory is None else list(trajectory.joint_names)
        pregrasp_z = None if pregrasp is None else float(pregrasp.pose.position.z)
        grasp_z = None if grasp is None else float(grasp.pose.position.z)
        adjacent_max = status.get("max_adjacent_delta_rad")
        synthetic_passed = bool(
            valid_ready
            and latest_plan_valid(observer) is True
            and status.get("status") == "VALID"
            and status.get("max_position_error_mm") is not None
            and status["max_position_error_mm"] <= POSITION_TOLERANCE_MM
            and status.get("max_approach_error_deg") is not None
            and status["max_approach_error_deg"] <= APPROACH_TOLERANCE_DEG
            and status.get("min_margin_rad") is not None
            and status["min_margin_rad"] >= MINIMUM_LIMIT_MARGIN_RAD
            and adjacent_max is not None
            and adjacent_max <= MAXIMUM_ADJACENT_DELTA_RAD
            and joint_names
            == [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
            ]
            and trajectory_count == status.get("waypoint_count")
            and pregrasp is not None
            and pregrasp.header.frame_id == "base_link"
            and grasp is not None
            and grasp.header.frame_id == "base_link"
            and pregrasp_z is not None
            and abs(pregrasp_z - (base_good[2] + 0.055)) <= 1.0e-6
            and grasp_z is not None
            and abs(grasp_z - (base_good[2] + 0.015)) <= 1.0e-6
        )
        report["synthetic_valid_case"] = {
            "passed": synthetic_passed,
            "base_target_m": base_good.tolist(),
            "workspace_input_m": workspace_good.tolist(),
            "object_pose_base_count": len(observer.object_pose_base),
            "object_pose_base_valid_seen": any(observer.object_pose_base_valid),
            "grasp_plan_valid_seen": any(observer.plan_valid),
            "valid_status_seen": has_valid_status(observer),
            "latest_status": status,
            "trajectory_point_count": trajectory_count,
            "joint_names": joint_names,
            "pregrasp_z_m": pregrasp_z,
            "grasp_z_m": grasp_z,
        }
        if not synthetic_passed:
            fail(report, "synthetic_valid_case", f"Valid synthetic case failed: {status}")

        stale_status: dict[str, Any] = {}
        stale_passed = False
        stale_deadline = time.monotonic() + 8.0
        while time.monotonic() < stale_deadline:
            force_status_tick()
            stale_status = latest_status(observer)
            stale_passed = bool(
                latest_plan_valid(observer) is False
                and stale_status.get("status") == "INVALID"
                and stale_status.get("reason")
                in (
                    "object_pose_base_stale",
                    "object_pose_base_valid_false",
                )
            )
            if stale_passed:
                break
            time.sleep(0.05)
        report["stale_invalidation_case"] = {
            "passed": stale_passed,
            "latest_status": stale_status,
        }
        if not stale_passed:
            fail(report, "stale_invalidation_case", f"Stale invalidation failed: {stale_status}")

        bad_cases: list[dict[str, Any]] = []
        publish_for(
            executor,
            observer,
            workspace_good,
            duration_s=1.0,
            frame_id="bad_frame",
        )
        spin_for(executor, 1.0)
        force_status_tick()
        wrong_frame_status = latest_status(observer)
        wrong_frame_debug = wrong_frame_status.get("debug", {})
        wrong_frame_processed = np.asarray(
            wrong_frame_debug.get("last_processed_position_base_m", []),
            dtype=np.float64,
        )
        wrong_frame_passed = bool(
            (observer.object_pose_base_valid and observer.object_pose_base_valid[-1] is False)
            or latest_plan_valid(observer) is False
            or wrong_frame_status.get("status") == "INVALID"
            or (
                wrong_frame_processed.shape == (3,)
                and float(np.linalg.norm(wrong_frame_processed - base_good)) <= 1.0e-9
            )
        )
        bad_cases.append(
            {
                "name": "wrong_frame",
                "passed": wrong_frame_passed,
                "latest_status": wrong_frame_status,
            }
        )
        if not wrong_frame_passed:
            fail(report, "wrong_frame_case", f"Wrong frame was not rejected: {wrong_frame_status}")

        workspace_nan = workspace_good.copy()
        workspace_nan[0] = math.nan
        publish_for(executor, observer, workspace_nan, duration_s=1.0)
        spin_for(executor, 1.0)
        force_status_tick()
        nan_status = latest_status(observer)
        nan_passed = bool(latest_plan_valid(observer) is False and nan_status.get("status") == "INVALID")
        bad_cases.append({"name": "nan_input", "passed": nan_passed, "latest_status": nan_status})
        if not nan_passed:
            fail(report, "nan_input_case", f"NaN input was not rejected: {nan_status}")

        base_out_of_bounds = np.asarray([0.40, 0.0, 0.025], dtype=np.float64)
        out_start = len(observer.status_messages)
        out_workspace = base_to_workspace(base_out_of_bounds, config)
        publish_until(
            executor,
            observer,
            out_workspace,
            lambda: any(
                "out_of_bounds" in str(status.get("reason"))
                for status in observer.status_messages[out_start:]
            ),
            timeout_s=8.0,
        )
        force_status_tick()
        out_matches = [
            status
            for status in observer.status_messages[out_start:]
            if "out_of_bounds" in str(status.get("reason"))
        ]
        out_status = out_matches[-1] if out_matches else latest_status(observer)
        out_passed = bool(
            out_status.get("status") == "INVALID"
            and "out_of_bounds" in str(out_status.get("reason"))
        )
        bad_cases.append({"name": "workspace_out_of_bounds", "passed": out_passed, "latest_status": out_status})
        if not out_passed:
            fail(report, "workspace_out_of_bounds_case", f"Out-of-bounds input was not rejected: {out_status}")

        base_unreachable = np.asarray([0.32, 0.12, 0.025], dtype=np.float64)
        unreachable_start = len(observer.status_messages)
        publish_for(
            executor,
            observer,
            base_to_workspace(base_unreachable, config),
            duration_s=3.0,
        )
        wait_until(
            executor,
            lambda: any(
                str(status.get("reason", "")).startswith("ik_failed:")
                for status in observer.status_messages[unreachable_start:]
            ),
            timeout_s=20.0,
        )
        unreachable_matches = [
            status
            for status in observer.status_messages[unreachable_start:]
            if str(status.get("reason", "")).startswith("ik_failed:")
        ]
        unreachable_status = unreachable_matches[-1] if unreachable_matches else latest_status(observer)
        unreachable_debug = unreachable_status.get("debug", {})
        unreachable_processed = np.asarray(
            unreachable_debug.get("last_processed_position_base_m", []),
            dtype=np.float64,
        )
        unreachable_passed = bool(
            unreachable_status.get("status") == "INVALID"
            and (
                str(unreachable_status.get("reason", "")).startswith("ik_failed:")
                or (
                    unreachable_processed.shape == (3,)
                    and float(np.linalg.norm(unreachable_processed - base_unreachable)) <= 1.0e-9
                )
            )
        )
        bad_cases.append({"name": "unreachable_target", "passed": unreachable_passed, "latest_status": unreachable_status})
        if not unreachable_passed:
            fail(report, "unreachable_case", f"Unreachable target was not rejected by IK: {unreachable_status}")

        observer.publish_stable_only(False)
        spin_for(executor, 1.2)
        valid_false_status = latest_status(observer)
        valid_false_passed = bool(
            latest_plan_valid(observer) is False
            and valid_false_status.get("status") == "INVALID"
        )
        bad_cases.append({"name": "valid_false", "passed": valid_false_passed, "latest_status": valid_false_status})
        if not valid_false_passed:
            fail(report, "valid_false_case", f"valid=false did not invalidate plan: {valid_false_status}")

        report["synthetic_negative_cases"] = bad_cases
        if not all(item["passed"] for item in bad_cases):
            fail(report, "synthetic_negative_cases", "One or more negative cases failed.")

        report["live_vision_validation"] = {
            "status": "NOT_RUN" if not run_live else "REQUESTED_NOT_AUTOMATED",
            "reason": (
                "Physical center/left/right/removed-object validation requires a camera "
                "and moving the target in the workspace. This verifier does not fake "
                "physical observations."
            ),
            "launch_command": (
                "powershell -ExecutionPolicy Bypass -File "
                f'"{RUN_IN_ROS2}" -CommandFile <cmd-file-containing-ros2-launch>'
            ),
        }
        if run_live:
            fail(
                report,
                "live_vision_validation",
                "Live physical validation was requested but cannot be automated without operator motion/camera scene control.",
            )
    finally:
        for ros_node in (planner, transform, observer):
            executor.remove_node(ros_node)
            ros_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        report["process_cleanup"] = {
            "all_started_processes_stopped": True,
            "process_logs": {},
        }


def run_inside_ros(run_live: bool, process_mode: bool = False) -> int:
    ensure_dirs()
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
    os.environ["RCL_LOGGING_IMPLEMENTATION"] = "rcl_logging_noop"
    append_log("Stage 2D-1 verification started inside ROS2 environment.")
    report: dict[str, Any] = {
        "stage": "2D-1",
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
                "RCL_LOGGING_IMPLEMENTATION", ""
            ),
            "ros_log_dir": os.environ.get("ROS_LOG_DIR", ""),
        },
        "model": {
            "urdf": str(URDF_PATH),
            "urdf_sha256": sha256_file(URDF_PATH),
            "urdf_sha256_expected": EXPECTED_URDF_SHA256,
            "urdf_sha256_matches_expected": sha256_file(URDF_PATH)
            == EXPECTED_URDF_SHA256,
        },
        "workspace_transform_config": read_json(WORKSPACE_CONFIG),
        "thresholds": {
            "position_tolerance_mm": POSITION_TOLERANCE_MM,
            "approach_tolerance_deg": APPROACH_TOLERANCE_DEG,
            "minimum_limit_margin_rad": MINIMUM_LIMIT_MARGIN_RAD,
            "maximum_adjacent_delta_rad": MAXIMUM_ADJACENT_DELTA_RAD,
            "stale_timeout_s": STALE_TIMEOUT_S,
        },
        "safety": {
            "opened_com_ports": False,
            "started_hardware_services": False,
            "published_robot_commands": False,
            "published_controller_command_topics": False,
            "planned_trajectory_is_preview_only": True,
        },
        "failures": [],
        "logs": {
            "main_log": str(LOG_PATH),
            "process_log_dir": str(PROCESS_LOG_DIR),
        },
    }
    if not report["model"]["urdf_sha256_matches_expected"]:
        fail(report, "urdf_sha256", "Frozen URDF hash changed.")
    try:
        if process_mode:
            verify_with_ros(report, run_live=run_live)
        else:
            verify_with_ros_in_process(report, run_live=run_live)
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
        append_log(f"Stage 2D-1 status: {report['status']}")
    return 0 if report["status"] == "PASS" else 2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify SO-101 Stage 2D-1 visual target to top-down IK dry run."
    )
    parser.add_argument("--inside-ros", action="store_true")
    parser.add_argument(
        "--run-live",
        action="store_true",
        help=(
            "Request live camera validation accounting. Physical object motion "
            "still requires an operator and is not faked by this verifier."
        ),
    )
    parser.add_argument(
        "--process-mode",
        action="store_true",
        help=(
            "Run the ROS nodes as separate processes instead of the default "
            "in-process executor. This is useful on Windows if rmw_zenoh_cpp "
            "and rclpy node construction trigger an in-process runtime fault."
        ),
    )
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros(
            run_live=args.run_live,
            process_mode=args.process_mode,
        )
    return run_outside_ros(run_live=args.run_live)


if __name__ == "__main__":
    raise SystemExit(main())
