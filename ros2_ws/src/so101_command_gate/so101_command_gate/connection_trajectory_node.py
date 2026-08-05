from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory

from .command_gate_validator import (
    DEFAULT_PROJECT_ROOT,
    FORBIDDEN_CONTROLLER_TOPICS,
    load_command_gate_joint_limits,
    reorder_joint_state_positions,
)
from .connection_parameterizer import (
    ConnectionPlan,
    load_connection_config,
    parameterize_connection,
    trajectory_signature,
)
from .connection_validator import (
    ConnectionValidationResult,
    parse_status_json,
    validate_connection_plan,
    validate_source_inputs,
)


STATUS_PERIOD_S = 0.05
CURRENT_REPLAN_THRESHOLD_RAD = 0.03


class ConnectionTrajectoryNode(Node):
    def __init__(self, *, project_root_override: str | Path | None = None) -> None:
        super().__init__("connection_trajectory_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.declare_parameter("current_joint_state_topic", "/real_joint_states")
        self.declare_parameter("current_joint_state_valid_topic", "/real_joint_state_valid")
        self.declare_parameter("source_trajectory_topic", "/safe_timed_grasp_trajectory")
        self.declare_parameter("source_valid_topic", "/safe_timed_grasp_valid")
        self.declare_parameter("source_status_topic", "/safe_timed_grasp_status")

        self.project_root = Path(
            str(project_root_override or self.get_parameter("project_root").value)
        ).resolve()
        self.current_joint_state_topic = str(self.get_parameter("current_joint_state_topic").value)
        self.current_joint_state_valid_topic = str(
            self.get_parameter("current_joint_state_valid_topic").value
        )
        self.source_trajectory_topic = str(self.get_parameter("source_trajectory_topic").value)
        self.source_valid_topic = str(self.get_parameter("source_valid_topic").value)
        self.source_status_topic = str(self.get_parameter("source_status_topic").value)

        self.config = load_connection_config(self.project_root)
        self.lower, self.upper, self.model_metadata = load_command_gate_joint_limits(
            self.project_root
        )

        self.real_joint_state_valid = False
        self.safe_timed_valid = False
        self.last_joint_state: JointState | None = None
        self.last_joint_state_time: float | None = None
        self.last_source_status_raw: str | None = None
        self.last_source_status: dict[str, Any] | None = None
        self.last_valid_status_time: float | None = None
        self.last_source_trajectory: JointTrajectory | None = None

        self.last_plan: ConnectionPlan | None = None
        self.last_plan_source_signature: str | None = None
        self.last_plan_current_positions: np.ndarray | None = None
        self.last_validation: ConnectionValidationResult | None = None

        self.valid_publisher = self.create_publisher(Bool, "/connection_plan_valid", 10)
        self.status_publisher = self.create_publisher(String, "/connection_plan_status", 10)
        self.reason_publisher = self.create_publisher(
            String,
            "/connection_plan_validity_reason",
            10,
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/connected_safe_timed_grasp_trajectory",
            10,
        )

        self.create_subscription(
            JointState,
            self.current_joint_state_topic,
            self.handle_joint_state,
            10,
        )
        self.create_subscription(
            Bool,
            self.current_joint_state_valid_topic,
            self.handle_current_joint_state_valid,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            self.source_trajectory_topic,
            self.handle_source_trajectory,
            10,
        )
        self.create_subscription(
            Bool,
            self.source_valid_topic,
            self.handle_source_valid,
            10,
        )
        self.create_subscription(
            String,
            self.source_status_topic,
            self.handle_source_status,
            10,
        )
        self.create_timer(STATUS_PERIOD_S, self.publish_status)

        self.get_logger().info(
            "SO-101 connection trajectory node started | "
            "REAL JOINT STATE READ-ONLY | CONNECTION TRAJECTORY SHADOW ONLY | "
            "HARDWARE MOTION DISABLED | NO CONTROLLER COMMAND TOPICS"
        )
        self.get_logger().info(
            "Connection inputs | "
            f"current={self.current_joint_state_topic} | "
            f"current_valid={self.current_joint_state_valid_topic} | "
            f"source_trajectory={self.source_trajectory_topic} | "
            f"source_valid={self.source_valid_topic} | "
            f"source_status={self.source_status_topic}"
        )

    def handle_joint_state(self, message: JointState) -> None:
        self.last_joint_state = message
        self.last_joint_state_time = time.monotonic()

    def handle_current_joint_state_valid(self, message: Bool) -> None:
        self.real_joint_state_valid = bool(message.data)

    def handle_source_trajectory(self, message: JointTrajectory) -> None:
        self.last_source_trajectory = message

    def handle_source_valid(self, message: Bool) -> None:
        self.safe_timed_valid = bool(message.data)

    def handle_source_status(self, message: String) -> None:
        self.last_source_status_raw = str(message.data)
        self.last_source_status = parse_status_json(self.last_source_status_raw)
        if self.last_source_status is not None and self.last_source_status.get("status") == "VALID":
            self.last_valid_status_time = time.monotonic()

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

    def evaluate(self) -> tuple[ConnectionValidationResult, ConnectionPlan | None]:
        source_result, current_positions, upstream_plan_id = validate_source_inputs(
            real_joint_state_valid=self.real_joint_state_valid,
            safe_timed_valid=self.safe_timed_valid,
            source_status=self.last_source_status,
            source_trajectory=self.last_source_trajectory,
            current_joint_state=self.last_joint_state,
            current_joint_state_age_s=self.current_joint_state_age_s(),
            source_valid_heartbeat_age_s=self.source_valid_heartbeat_age_s(),
            config=self.config,
            lower=self.lower,
            upper=self.upper,
            project_root=self.project_root,
        )
        if not source_result.valid:
            self.last_plan = None
            self.last_plan_source_signature = None
            self.last_plan_current_positions = None
            return source_result, None

        assert current_positions is not None
        assert upstream_plan_id is not None
        assert self.last_source_trajectory is not None
        source_signature = trajectory_signature(self.last_source_trajectory)
        should_replan = self.last_plan is None
        if self.last_plan is not None:
            should_replan = should_replan or self.last_plan.upstream_plan_id != upstream_plan_id
            should_replan = should_replan or self.last_plan_source_signature != source_signature
            if self.last_plan_current_positions is None:
                should_replan = True
            else:
                current_delta = float(
                    np.max(np.abs(current_positions - self.last_plan_current_positions))
                )
                should_replan = should_replan or current_delta > CURRENT_REPLAN_THRESHOLD_RAD

        if should_replan:
            try:
                plan = parameterize_connection(
                    q_current=current_positions.copy(),
                    source_trajectory=self.last_source_trajectory,
                    upstream_plan_id=upstream_plan_id,
                    config=self.config,
                )
            except Exception as error:
                self.last_plan = None
                self.last_plan_source_signature = None
                self.last_plan_current_positions = None
                return (
                    ConnectionValidationResult(
                        False,
                        "connection_parameterization_failed:" + repr(error),
                        upstream_plan_id,
                    ),
                    None,
                )
            self.last_plan = plan
            self.last_plan_source_signature = source_signature
            self.last_plan_current_positions = current_positions.copy()

        assert self.last_plan is not None
        plan_result = validate_connection_plan(
            plan=self.last_plan,
            config=self.config,
            lower=self.lower,
            upper=self.upper,
            project_root=self.project_root,
        )
        if not plan_result.valid:
            return plan_result, None
        return plan_result, self.last_plan

    def status_payload(
        self,
        validation: ConnectionValidationResult,
        plan: ConnectionPlan | None,
    ) -> dict[str, Any]:
        status = "VALID" if validation.valid else "INVALID"
        if plan is None:
            payload: dict[str, Any] = {
                "status": status,
                "reason": validation.reason,
                "upstream_plan_id": validation.upstream_plan_id,
                "connection_plan_id": None,
                "plan_id": None,
                "active_plan_id": None,
                "current_joint_positions_rad": [],
                "target_start_positions_rad": [],
                "per_joint_start_error_rad": [],
                "maximum_start_error_rad": None,
                "connection_duration_s": 0.0,
                "connection_point_count": 0,
                "original_trajectory_point_count": 0,
                "combined_trajectory_point_count": 0,
                "maximum_connection_velocity_rad_s_observed": [0.0] * 5,
                "maximum_connection_acceleration_rad_s2_observed": [0.0] * 5,
                "minimum_joint_limit_margin_rad": validation.minimum_joint_limit_margin_rad,
                "minimum_tcp_z_m_observed": validation.minimum_tcp_z_m_observed,
                "tcp_x_range_m_observed": validation.tcp_x_range_m_observed,
                "tcp_y_range_m_observed": validation.tcp_y_range_m_observed,
                "time_strictly_increasing": validation.time_strictly_increasing,
            }
        else:
            max_error = float(np.max(plan.per_joint_start_error_rad))
            payload = {
                "status": status,
                "reason": (
                    "connection_and_grasp_shadow_candidate_valid"
                    if validation.valid
                    else validation.reason
                ),
                "upstream_plan_id": plan.upstream_plan_id,
                "latest_upstream_plan_id": plan.upstream_plan_id,
                "connection_plan_id": plan.connection_plan_id,
                "plan_id": plan.connection_plan_id,
                "active_plan_id": plan.connection_plan_id,
                "current_joint_positions_rad": [float(value) for value in plan.q_current],
                "target_start_positions_rad": [float(value) for value in plan.q_start],
                "per_joint_start_error_rad": [
                    float(value) for value in plan.per_joint_start_error_rad
                ],
                "maximum_start_error_rad": max_error,
                "connection_duration_s": plan.connection_duration_s,
                "connection_point_count": plan.connection_point_count,
                "original_trajectory_point_count": plan.original_trajectory_point_count,
                "combined_trajectory_point_count": len(plan.combined_trajectory.points),
                "maximum_connection_velocity_rad_s_observed": [
                    float(value) for value in plan.maximum_connection_velocity_rad_s_observed
                ],
                "maximum_connection_acceleration_rad_s2_observed": [
                    float(value) for value in plan.maximum_connection_acceleration_rad_s2_observed
                ],
                "minimum_joint_limit_margin_rad": validation.minimum_joint_limit_margin_rad,
                "minimum_tcp_z_m_observed": validation.minimum_tcp_z_m_observed,
                "tcp_x_range_m_observed": validation.tcp_x_range_m_observed,
                "tcp_y_range_m_observed": validation.tcp_y_range_m_observed,
                "time_strictly_increasing": validation.time_strictly_increasing,
            }
        payload.update(
            {
                "current_joint_state_age_s": self.current_joint_state_age_s(),
                "source_valid_heartbeat_age_s": self.source_valid_heartbeat_age_s(),
                "shadow_connection_preview_only": True,
                "hardware_control_enabled": False,
                "command_topics_published": [],
                "real_controller_topics_published": [],
                "disabled_controller_command_topics": list(FORBIDDEN_CONTROLLER_TOPICS),
                "timestamp": self.make_timestamp(),
                "status_profile": self.config.status,
                "source_status_raw": self.last_source_status_raw,
            }
        )
        return payload

    def publish_status(self) -> None:
        validation, plan = self.evaluate()
        self.last_validation = validation

        valid_message = Bool()
        valid_message.data = validation.valid
        self.valid_publisher.publish(valid_message)

        payload = self.status_payload(validation, plan)
        status_message = String()
        status_message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.status_publisher.publish(status_message)

        reason_message = String()
        reason_message.data = str(payload["reason"])
        self.reason_publisher.publish(reason_message)

        if validation.valid and plan is not None:
            self.trajectory_publisher.publish(plan.combined_trajectory)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ConnectionTrajectoryNode | None = None
    try:
        node = ConnectionTrajectoryNode()
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
