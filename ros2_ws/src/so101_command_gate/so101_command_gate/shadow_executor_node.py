from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from .command_gate_validator import (
    DEFAULT_PROJECT_ROOT,
    FORBIDDEN_CONTROLLER_TOPICS,
    load_command_gate_config,
    seconds_from_duration,
)


class ShadowExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("shadow_executor_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.project_root = Path(str(self.get_parameter("project_root").value)).resolve()
        self.config = load_command_gate_config(self.project_root)

        self.command_gate_valid = False
        self.command_gate_status: dict[str, Any] | None = None
        self.command_gate_status_raw: str | None = None
        self.candidate_trajectory: JointTrajectory | None = None
        self.candidate_signature: str | None = None

        self.state = "IDLE"
        self.reason = "waiting_for_command_gate_ready"
        self.active_plan_id: str | None = None
        self.running_signature: str | None = None
        self.started_monotonic_s: float | None = None
        self.elapsed_s = 0.0
        self.total_duration_s = 0.0
        self.current_shadow_positions_rad: list[float] = []
        self.current_shadow_velocities_rad_s: list[float] = []

        self.expected_joint_state_publisher = self.create_publisher(
            JointState,
            "/shadow_expected_joint_states",
            10,
        )
        self.active_publisher = self.create_publisher(
            Bool,
            "/shadow_execution_active",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/shadow_execution_status",
            10,
        )

        self.create_subscription(
            Bool,
            "/command_gate_valid",
            self.handle_command_gate_valid,
            10,
        )
        self.create_subscription(
            String,
            "/command_gate_status",
            self.handle_command_gate_status,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/shadow_command_candidate_trajectory",
            self.handle_candidate_trajectory,
            10,
        )

        self.create_service(
            Trigger,
            "/start_shadow_execution",
            self.handle_start_shadow_execution,
        )
        self.create_service(
            Trigger,
            "/cancel_shadow_execution",
            self.handle_cancel_shadow_execution,
        )

        period_s = 1.0 / self.config.shadow_publish_rate_hz
        self.create_timer(period_s, self.tick)
        self.get_logger().info(
            "SO-101 shadow executor started | status=shadow_execution_only | "
            "requires explicit /start_shadow_execution trigger | "
            "hardware_control_enabled=False | controller command topics disabled"
        )

    def handle_command_gate_valid(self, message: Bool) -> None:
        self.command_gate_valid = bool(message.data)
        if self.state == "RUNNING" and not self.command_gate_valid:
            self.invalidate("command_gate_invalidated")

    def handle_command_gate_status(self, message: String) -> None:
        self.command_gate_status_raw = str(message.data)
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.command_gate_status = None
            if self.state == "RUNNING":
                self.invalidate("command_gate_status_invalid_json")
            return
        if not isinstance(value, dict):
            self.command_gate_status = None
            if self.state == "RUNNING":
                self.invalidate("command_gate_status_not_object")
            return
        previous_plan_id = None if self.command_gate_status is None else self.command_gate_status.get("plan_id")
        self.command_gate_status = value
        plan_id = value.get("plan_id")
        if self.state == "RUNNING":
            if value.get("status") != "READY":
                self.invalidate(str(value.get("reason", "command_gate_not_ready")))
            elif self.active_plan_id is not None and plan_id != self.active_plan_id:
                self.invalidate("plan_id_changed")
        elif self.state in ("IDLE", "READY") and self.command_gate_valid and value.get("status") == "READY":
            self.state = "READY"
            self.reason = "shadow_execution_candidate_ready"
        elif self.state == "READY" and value.get("status") != "READY":
            self.state = "IDLE"
            self.reason = str(value.get("reason", "command_gate_not_ready"))
        if previous_plan_id is not None and plan_id != previous_plan_id and self.state == "RUNNING":
            self.invalidate("plan_id_changed")

    def handle_candidate_trajectory(self, message: JointTrajectory) -> None:
        signature = trajectory_signature(message)
        if self.state == "RUNNING" and self.running_signature is not None and signature != self.running_signature:
            self.invalidate("trajectory_replaced")
            return
        self.candidate_trajectory = message
        self.candidate_signature = signature

    def handle_start_shadow_execution(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if not self.command_gate_valid:
            response.success = False
            response.message = "command_gate_not_valid"
            return response
        if self.command_gate_status is None or self.command_gate_status.get("status") != "READY":
            response.success = False
            response.message = "command_gate_not_ready"
            return response
        if self.candidate_trajectory is None or self.candidate_signature is None:
            response.success = False
            response.message = "missing_shadow_candidate_trajectory"
            return response
        self.active_plan_id = str(self.command_gate_status.get("plan_id"))
        self.running_signature = self.candidate_signature
        self.started_monotonic_s = time.monotonic()
        self.elapsed_s = 0.0
        self.total_duration_s = trajectory_total_duration_s(self.candidate_trajectory)
        self.state = "RUNNING"
        self.reason = "shadow_execution_running"
        response.success = True
        response.message = "shadow_execution_started"
        return response

    def handle_cancel_shadow_execution(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if self.state == "RUNNING":
            self.state = "CANCELLED"
            self.reason = "cancel_shadow_execution_called"
        elif self.state == "READY":
            self.state = "CANCELLED"
            self.reason = "cancel_shadow_execution_called"
        response.success = True
        response.message = self.reason
        return response

    def invalidate(self, reason: str) -> None:
        if self.state == "RUNNING":
            self.state = "INVALIDATED"
            self.reason = reason
            self.started_monotonic_s = None

    def tick(self) -> None:
        try:
            if self.state == "RUNNING":
                self.tick_running()
        except Exception as error:
            self.state = "FAILED"
            self.reason = repr(error)
            self.started_monotonic_s = None
        self.publish_status()

    def tick_running(self) -> None:
        if self.started_monotonic_s is None or self.candidate_trajectory is None:
            self.invalidate("internal_missing_running_state")
            return
        if not self.command_gate_valid:
            self.invalidate("command_gate_invalidated")
            return
        elapsed = time.monotonic() - self.started_monotonic_s
        self.elapsed_s = max(0.0, elapsed)
        if self.total_duration_s <= 0.0 or not math.isfinite(self.total_duration_s):
            self.state = "FAILED"
            self.reason = "invalid_total_duration"
            return
        if self.elapsed_s >= self.total_duration_s:
            positions, velocities = sample_trajectory(
                self.candidate_trajectory,
                self.total_duration_s,
            )
            self.current_shadow_positions_rad = positions
            self.current_shadow_velocities_rad_s = velocities
            self.publish_expected_joint_state()
            self.elapsed_s = self.total_duration_s
            self.state = "COMPLETED"
            self.reason = "shadow_execution_completed"
            self.started_monotonic_s = None
            return
        positions, velocities = sample_trajectory(
            self.candidate_trajectory,
            self.elapsed_s,
        )
        self.current_shadow_positions_rad = positions
        self.current_shadow_velocities_rad_s = velocities
        self.publish_expected_joint_state()

    def publish_expected_joint_state(self) -> None:
        if self.candidate_trajectory is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.candidate_trajectory.joint_names)
        message.position = [float(value) for value in self.current_shadow_positions_rad]
        message.velocity = [float(value) for value in self.current_shadow_velocities_rad_s]
        self.expected_joint_state_publisher.publish(message)

    def publish_status(self) -> None:
        active_message = Bool()
        active_message.data = self.state == "RUNNING"
        self.active_publisher.publish(active_message)

        total = self.total_duration_s
        progress = 0.0 if total <= 0.0 else min(1.0, max(0.0, self.elapsed_s / total))
        now = self.get_clock().now().to_msg()
        payload = {
            "state": self.state,
            "reason": self.reason,
            "plan_id": self.active_plan_id,
            "elapsed_s": self.elapsed_s,
            "total_duration_s": total,
            "progress_ratio": progress,
            "current_shadow_positions_rad": self.current_shadow_positions_rad,
            "hardware_control_enabled": False,
            "command_topics_published": [],
            "shadow_execution_only": True,
            "timestamp": float(now.sec) + float(now.nanosec) * 1.0e-9,
            "disabled_controller_command_topics": list(FORBIDDEN_CONTROLLER_TOPICS),
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.status_publisher.publish(message)


def trajectory_total_duration_s(trajectory: JointTrajectory) -> float:
    if not trajectory.points:
        return 0.0
    return seconds_from_duration(trajectory.points[-1].time_from_start)


def trajectory_signature(trajectory: JointTrajectory) -> str:
    payload = {
        "frame_id": trajectory.header.frame_id,
        "joint_names": list(trajectory.joint_names),
        "points": [
            {
                "positions": [round(float(value), 12) for value in point.positions],
                "velocities": [round(float(value), 12) for value in point.velocities],
                "accelerations": [round(float(value), 12) for value in point.accelerations],
                "time_from_start_s": round(seconds_from_duration(point.time_from_start), 12),
            }
            for point in trajectory.points
        ],
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sample_trajectory(
    trajectory: JointTrajectory,
    elapsed_s: float,
) -> tuple[list[float], list[float]]:
    if not trajectory.points:
        return [], []
    times = [seconds_from_duration(point.time_from_start) for point in trajectory.points]
    if elapsed_s <= times[0]:
        first = trajectory.points[0]
        return list(first.positions), list(first.velocities)
    if elapsed_s >= times[-1]:
        last = trajectory.points[-1]
        return list(last.positions), list(last.velocities)
    for index in range(1, len(times)):
        if elapsed_s <= times[index]:
            previous = trajectory.points[index - 1]
            current = trajectory.points[index]
            t0 = times[index - 1]
            t1 = times[index]
            ratio = 0.0 if t1 <= t0 else (elapsed_s - t0) / (t1 - t0)
            p0 = np.asarray(previous.positions, dtype=np.float64)
            p1 = np.asarray(current.positions, dtype=np.float64)
            v0 = np.asarray(previous.velocities, dtype=np.float64)
            v1 = np.asarray(current.velocities, dtype=np.float64)
            positions = p0 + ratio * (p1 - p0)
            velocities = v0 + ratio * (v1 - v0)
            return positions.tolist(), velocities.tolist()
    last = trajectory.points[-1]
    return list(last.positions), list(last.velocities)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ShadowExecutorNode | None = None
    try:
        node = ShadowExecutorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
