from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory

from .command_gate_validator import (
    DEFAULT_PROJECT_ROOT,
    FORBIDDEN_CONTROLLER_TOPICS,
    CommandGateConfig,
    GateValidationResult,
    extract_status_plan_id,
    load_command_gate_config,
    load_command_gate_joint_limits,
    make_invalid,
    validate_command_candidate,
)


STATUS_PERIOD_S = 0.05


class CommandGateNode(Node):
    def __init__(
        self,
        *,
        project_root_override: str | Path | None = None,
        current_joint_state_topic_override: str | None = None,
        current_joint_state_valid_topic_override: str | None = None,
        source_trajectory_topic_override: str | None = None,
        source_valid_topic_override: str | None = None,
        source_status_topic_override: str | None = None,
    ) -> None:
        super().__init__("command_gate_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.declare_parameter("current_joint_state_topic", "/joint_states")
        self.declare_parameter("current_joint_state_valid_topic", "")
        self.declare_parameter("source_trajectory_topic", "/safe_timed_grasp_trajectory")
        self.declare_parameter("source_valid_topic", "/safe_timed_grasp_valid")
        self.declare_parameter("source_status_topic", "/safe_timed_grasp_status")
        self.project_root = Path(
            str(project_root_override or self.get_parameter("project_root").value)
        ).resolve()
        self.current_joint_state_topic = str(
            current_joint_state_topic_override
            or self.get_parameter("current_joint_state_topic").value
        )
        self.current_joint_state_valid_topic = str(
            current_joint_state_valid_topic_override
            or self.get_parameter("current_joint_state_valid_topic").value
        )
        self.source_trajectory_topic = str(
            source_trajectory_topic_override
            or self.get_parameter("source_trajectory_topic").value
        )
        self.source_valid_topic = str(
            source_valid_topic_override
            or self.get_parameter("source_valid_topic").value
        )
        self.source_status_topic = str(
            source_status_topic_override
            or self.get_parameter("source_status_topic").value
        )

        self.config: CommandGateConfig = load_command_gate_config(self.project_root)
        self.lower, self.upper, self.model_metadata = load_command_gate_joint_limits(
            self.project_root
        )

        self.safe_timed_valid = False
        self.last_safe_timed_status: dict[str, Any] | None = None
        self.last_safe_timed_status_raw: str | None = None
        self.last_valid_status_time: float | None = None
        self.last_trajectory: JointTrajectory | None = None
        self.last_joint_state: JointState | None = None
        self.last_joint_state_time: float | None = None
        self.current_joint_state_source_valid = True
        self.last_current_joint_state_valid_time: float | None = None
        self.last_validation: GateValidationResult | None = None

        self.valid_publisher = self.create_publisher(Bool, "/command_gate_valid", 10)
        self.status_publisher = self.create_publisher(String, "/command_gate_status", 10)
        self.reason_publisher = self.create_publisher(
            String,
            "/command_gate_validity_reason",
            10,
        )
        self.candidate_publisher = self.create_publisher(
            JointTrajectory,
            "/shadow_command_candidate_trajectory",
            10,
        )

        self.create_subscription(
            JointTrajectory,
            self.source_trajectory_topic,
            self.handle_safe_timed_trajectory,
            10,
        )
        self.create_subscription(
            Bool,
            self.source_valid_topic,
            self.handle_safe_timed_valid,
            10,
        )
        self.create_subscription(
            String,
            self.source_status_topic,
            self.handle_safe_timed_status,
            10,
        )
        self.create_subscription(
            JointState,
            self.current_joint_state_topic,
            self.handle_joint_state,
            10,
        )
        if self.current_joint_state_valid_topic:
            self.current_joint_state_source_valid = False
            self.create_subscription(
                Bool,
                self.current_joint_state_valid_topic,
                self.handle_current_joint_state_valid,
                10,
            )
        self.create_timer(STATUS_PERIOD_S, self.publish_status)

        self.get_logger().info(
            "SO-101 command gate started | status=shadow_execution_only | "
            "hardware_control_enabled=False | controller command topics disabled | "
            f"maximum_start_state_error_rad={self.config.maximum_start_state_error_rad:.3f}"
        )
        self.get_logger().info(
            "Shadow candidate publisher only: /shadow_command_candidate_trajectory. "
            f"Disabled real command topics: {FORBIDDEN_CONTROLLER_TOPICS}"
        )
        self.get_logger().info(
            "Command gate current state source | "
            f"topic={self.current_joint_state_topic} | "
            f"valid_topic={self.current_joint_state_valid_topic or '<none>'}"
        )
        self.get_logger().info(
            "Command gate upstream source | "
            f"trajectory={self.source_trajectory_topic} | "
            f"valid={self.source_valid_topic} | "
            f"status={self.source_status_topic}"
        )

    def handle_safe_timed_trajectory(self, message: JointTrajectory) -> None:
        self.last_trajectory = message

    def handle_safe_timed_valid(self, message: Bool) -> None:
        self.safe_timed_valid = bool(message.data)

    def handle_safe_timed_status(self, message: String) -> None:
        self.last_safe_timed_status_raw = str(message.data)
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.last_safe_timed_status = None
            return
        if not isinstance(value, dict):
            self.last_safe_timed_status = None
            return
        self.last_safe_timed_status = value
        if value.get("status") == "VALID":
            self.last_valid_status_time = time.monotonic()

    def handle_joint_state(self, message: JointState) -> None:
        self.last_joint_state = message
        self.last_joint_state_time = time.monotonic()

    def handle_current_joint_state_valid(self, message: Bool) -> None:
        self.current_joint_state_source_valid = bool(message.data)
        self.last_current_joint_state_valid_time = time.monotonic()

    def current_joint_state_age_s(self) -> float | None:
        if self.last_joint_state_time is None:
            return None
        return time.monotonic() - self.last_joint_state_time

    def source_valid_heartbeat_age_s(self) -> float | None:
        if self.last_valid_status_time is None:
            return None
        return time.monotonic() - self.last_valid_status_time

    def make_timestamp(self) -> float:
        now = self.get_clock().now().to_msg()
        return float(now.sec) + float(now.nanosec) * 1.0e-9

    def evaluate(self) -> GateValidationResult:
        if self.current_joint_state_valid_topic and not self.current_joint_state_source_valid:
            return make_invalid(
                "current_joint_state_source_invalid",
                extract_status_plan_id(self.last_safe_timed_status),
                self.current_joint_state_age_s(),
                self.source_valid_heartbeat_age_s(),
            )
        return validate_command_candidate(
            safe_timed_valid=self.safe_timed_valid,
            safe_timed_status=self.last_safe_timed_status,
            trajectory=self.last_trajectory,
            joint_state=self.last_joint_state,
            current_joint_state_age_s=self.current_joint_state_age_s(),
            source_valid_heartbeat_age_s=self.source_valid_heartbeat_age_s(),
            config=self.config,
            lower=self.lower,
            upper=self.upper,
        )

    def publish_status(self) -> None:
        validation = self.evaluate()
        self.last_validation = validation
        payload = validation.status_dict(self.config, self.make_timestamp())
        payload["safe_timed_status_raw"] = self.last_safe_timed_status_raw

        valid_message = Bool()
        valid_message.data = validation.ready
        self.valid_publisher.publish(valid_message)

        status_message = String()
        status_message.data = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
        )
        self.status_publisher.publish(status_message)

        reason_message = String()
        reason_message.data = str(payload["reason"])
        self.reason_publisher.publish(reason_message)

        if validation.ready and self.last_trajectory is not None:
            self.candidate_publisher.publish(self.last_trajectory)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: CommandGateNode | None = None
    try:
        node = CommandGateNode()
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
