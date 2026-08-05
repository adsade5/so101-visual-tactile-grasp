from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS2_WS = PROJECT_ROOT / "ros2_ws"
RUN_IN_ROS2 = PROJECT_ROOT / "audit" / "run_in_ros2_lyrical.ps1"
ROS2_PYTHON = Path(r"C:\pixi_ws\.pixi\envs\default\python.exe")
REPORT_DIR = PROJECT_ROOT / "data" / "verification"
REPORT_PATH = REPORT_DIR / "stage_2d2_live_hold_report.json"
LOG_PATH = REPORT_DIR / "stage_2d2_live_hold.log"
ROS_LOG_DIR = REPORT_DIR / "ros_logs"


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


def run_outside_ros(duration_s: float, removal_window_s: float) -> int:
    ensure_dirs()
    if LOG_PATH.exists():
        LOG_PATH.unlink()
    command_file = Path(tempfile.gettempdir()) / (
        f"monitor_stage_2d2_live_{filename_timestamp()}.cmd"
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
                f'"{ROS2_PYTHON}" "{Path(__file__).resolve()}" --inside-ros '
                f"--duration-s {duration_s:.3f} "
                f"--removal-window-s {removal_window_s:.3f}"
            ),
        ]
    )
    command_file.write_text(command, encoding="ascii")
    try:
        append_log("Entering ROS2 Lyrical environment for live Stage 2D-2 monitor.")
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


def run_inside_ros(duration_s: float, removal_window_s: float) -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class Monitor(Node):
        def __init__(self) -> None:
            super().__init__("stage_2d2_live_validity_monitor")
            self.start = time.monotonic()
            self.events: list[dict[str, Any]] = []
            self.last_status: dict[str, Any] = {}
            self.create_subscription(
                Bool,
                "/object_pose_stable",
                lambda msg: self.record_bool("/object_pose_stable", bool(msg.data)),
                50,
            )
            self.create_subscription(
                Bool,
                "/object_pose_base_valid",
                lambda msg: self.record_bool("/object_pose_base_valid", bool(msg.data)),
                50,
            )
            self.create_subscription(
                Bool,
                "/grasp_plan_valid",
                lambda msg: self.record_bool("/grasp_plan_valid", bool(msg.data)),
                50,
            )
            self.create_subscription(
                Bool,
                "/safe_timed_grasp_valid",
                lambda msg: self.record_bool("/safe_timed_grasp_valid", bool(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/grasp_plan_status",
                lambda msg: self.record_string("/grasp_plan_status", str(msg.data)),
                50,
            )
            self.create_subscription(
                String,
                "/safe_timed_grasp_status",
                lambda msg: self.record_string("/safe_timed_grasp_status", str(msg.data)),
                50,
            )

        def elapsed(self) -> float:
            return time.monotonic() - self.start

        def record_bool(self, topic: str, value: bool) -> None:
            self.events.append(
                {"t_s": self.elapsed(), "topic": topic, "type": "bool", "value": value}
            )
            append_log(f"{self.elapsed():.3f}s {topic}={value}")

        def record_string(self, topic: str, value: str) -> None:
            parsed: dict[str, Any] | None = None
            try:
                maybe = json.loads(value)
                if isinstance(maybe, dict):
                    parsed = maybe
            except json.JSONDecodeError:
                parsed = None
            if topic == "/safe_timed_grasp_status" and parsed is not None:
                self.last_status = parsed
            self.events.append(
                {
                    "t_s": self.elapsed(),
                    "topic": topic,
                    "type": "string",
                    "parsed": parsed,
                }
            )

    ensure_dirs()
    os.environ["ROS_LOG_DIR"] = str(ROS_LOG_DIR)
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
    os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "1"
    os.environ["RCL_LOGGING_IMPLEMENTATION"] = "rcl_logging_noop"
    append_log("Stage 2D-2 live hold monitor started.")
    rclpy.init()
    monitor = Monitor()
    try:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.05)
        hold_topics = [
            "/object_pose_stable",
            "/object_pose_base_valid",
            "/grasp_plan_valid",
            "/safe_timed_grasp_valid",
        ]
        false_events = [
            event
            for event in monitor.events
            if event.get("topic") in hold_topics and event.get("value") is False
        ]
        topic_counts = {
            topic: sum(1 for event in monitor.events if event.get("topic") == topic)
            for topic in hold_topics + ["/grasp_plan_status", "/safe_timed_grasp_status"]
        }
        reparameterization_counts = [
            event.get("parsed", {}).get("reparameterization_count")
            for event in monitor.events
            if event.get("topic") == "/safe_timed_grasp_status"
            and isinstance(event.get("parsed"), dict)
            and event.get("parsed", {}).get("status") == "VALID"
        ]
        report = {
            "stage": "2D-2-live-hold",
            "status": "PASS" if not false_events and all(topic_counts[t] > 0 for t in hold_topics) else "FAIL",
            "timestamp": iso_timestamp(),
            "duration_s": duration_s,
            "removal_window_s": removal_window_s,
            "topic_counts": topic_counts,
            "false_events": false_events,
            "first_false_event": false_events[0] if false_events else None,
            "last_safe_status": monitor.last_status,
            "reparameterization_counts_seen": reparameterization_counts,
            "reparameterization_count_changed": (
                len(set(reparameterization_counts)) > 1
                if reparameterization_counts
                else None
            ),
            "safety": {
                "opened_com_ports": False,
                "started_lerobot_hardware_server": False,
                "connected_real_so101": False,
                "published_real_controller_commands": False,
                "published_controller_command_topics": [],
                "hardware_control_enabled": False,
            },
            "notes": (
                "This monitor only observes ROS topics. It does not move the object "
                "or fake live camera observations."
            ),
        }
        REPORT_PATH.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),
            encoding="utf-8",
        )
        append_log(f"Report written: {REPORT_PATH}")
        append_log(f"Live hold monitor status: {report['status']}")
        return 0 if report["status"] == "PASS" else 2
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor live Stage 2D-2 validity hold.")
    parser.add_argument("--inside-ros", action="store_true")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--removal-window-s", type=float, default=0.0)
    args = parser.parse_args()
    if args.inside_ros:
        return run_inside_ros(args.duration_s, args.removal_window_s)
    return run_outside_ros(args.duration_s, args.removal_window_s)


if __name__ == "__main__":
    raise SystemExit(main())
