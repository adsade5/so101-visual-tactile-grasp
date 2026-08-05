from __future__ import annotations

import json
import hashlib
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .minimum_jerk_parameterizer import (
    TimedTrajectoryResult,
    parameterize_path,
)
from .trajectory_validator import (
    DEFAULT_PROJECT_ROOT,
    LIMIT_PROFILE_STATUS,
    TrajectorySafetyConfig,
    load_joint_limits,
    load_safety_config,
    validate_source_path,
)


STATUS_PERIOD_S = 0.05
THROTTLED_LOG_PERIOD_S = 2.0
DEFAULT_PLANNER_CONFIG_VERSION = "visual_grasp_planner_v1"
COMMAND_TOPIC_NAMES = [
    "/joint_trajectory_controller/joint_trajectory",
    "/joint_trajectory_controller/follow_joint_trajectory",
    "/arm_controller/command",
    "/robot_command",
    "/hardware_command",
]
INPUT_VALIDATION_REASONS = {
    "wrong_joint_names",
    "wrong_joint_order",
    "fewer_than_two_waypoints",
    "wrong_position_length",
    "positions_not_numeric",
    "wrong_position_shape",
    "non_finite_position",
    "joint_position_out_of_bounds",
    "joint_limit_margin_insufficient",
    "input_adjacent_delta_exceeds_limit",
    "segment_duration_exceeds_limit",
    "total_duration_exceeds_limit",
    "parameterization_iteration_limit",
}


def trajectory_hash(
    joint_names: list[str],
    positions: list[list[float]],
    planner_config_version: str = DEFAULT_PLANNER_CONFIG_VERSION,
) -> str:
    def canonical_number(value: float) -> float | str:
        numeric = float(value)
        if math.isfinite(numeric):
            return round(numeric, 12)
        if math.isnan(numeric):
            return "nan"
        return "inf" if numeric > 0.0 else "-inf"

    payload = {
        "joint_names": [str(value) for value in joint_names],
        "positions": [
            [canonical_number(float(value)) for value in point]
            for point in positions
        ],
        "waypoint_count": len(positions),
        "planner_config_version": planner_config_version,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def duration_from_seconds(seconds: float) -> Duration:
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    message = Duration()
    message.sec = whole
    message.nanosec = nanoseconds
    return message


class TimedTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("timed_trajectory_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.project_root = Path(
            str(self.get_parameter("project_root").value)
        ).resolve()

        self.config = load_safety_config(self.project_root)
        self.lower, self.upper, self.model_metadata = load_joint_limits(self.project_root)

        self.plan_valid = False
        self.last_source_trajectory_time: float | None = None
        self.last_upstream_valid_heartbeat_time: float | None = None
        self.last_upstream_status_heartbeat_time: float | None = None
        self.last_parameterized_plan_time: float | None = None
        self.latest_upstream_plan_id: str | None = None
        self.active_plan_id: str | None = None
        self.active_trajectory_hash: str | None = None
        self.active_payload_time: float | None = None
        self.pending_plan_id: str | None = None
        self.pending_trajectory_hash: str | None = None
        self.pending_status_time: float | None = None
        self.pending_payload_time: float | None = None
        self.latest_payload_hash: str | None = None
        self.latest_payload_time: float | None = None
        self.payload_cache: dict[str, tuple[JointTrajectory, float]] = {}
        self.last_source_waypoint_count = 0
        self.last_output_point_count = 0
        self.last_margin: float | None = None
        self.last_adjacent_delta: float | None = None
        self.last_result: TimedTrajectoryResult | None = None
        self.last_trajectory: JointTrajectory | None = None
        self.last_grasp_plan_status: str | None = None
        self.last_grasp_plan_status_json: dict[str, Any] | None = None
        self.last_input_validation_error_time: float | None = None
        self.reparameterization_count = 0
        self.cached_timed_trajectory_hit_count = 0
        self.timed_trajectory_recomputed = False
        self.using_cached_timed_trajectory = False
        self.last_log_by_key: dict[str, float] = {}
        self.last_status: dict[str, Any] = self.make_status(
            status="INVALID",
            reason="waiting_for_grasp_plan",
        )

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/safe_timed_grasp_trajectory",
            10,
        )
        self.valid_publisher = self.create_publisher(
            Bool,
            "/safe_timed_grasp_valid",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/safe_timed_grasp_status",
            10,
        )
        self.reason_publisher = self.create_publisher(
            String,
            "/safe_timed_grasp_validity_reason",
            10,
        )

        self.create_subscription(
            JointTrajectory,
            "/planned_grasp_joint_trajectory",
            self.handle_source_trajectory,
            10,
        )
        self.create_subscription(
            Bool,
            "/grasp_plan_valid",
            self.handle_plan_valid,
            10,
        )
        self.create_subscription(
            String,
            "/grasp_plan_status",
            self.handle_grasp_plan_status,
            10,
        )
        self.create_timer(STATUS_PERIOD_S, self.publish_status)

        self.get_logger().info(
            "SO-101 timed trajectory safety preview started | "
            "hardware_control_enabled=False | "
            "controller_topics_disabled=True | "
            f"limit_profile_status={LIMIT_PROFILE_STATUS} | "
            f"maximum_velocity_rad_s={self.config.maximum_velocity_rad_s.tolist()} | "
            f"maximum_acceleration_rad_s2={self.config.maximum_acceleration_rad_s2.tolist()} | "
            f"sample_rate_hz={self.config.sample_rate_hz:.1f}"
        )
        self.get_logger().info(
            "Publishing preview only on /safe_timed_grasp_trajectory; "
            "no controller command publishers are created."
        )

    def handle_plan_valid(self, message: Bool) -> None:
        self.plan_valid = bool(message.data)
        if not self.plan_valid:
            self.invalidate("grasp_plan_valid_false")
            self.throttled_log(
                "grasp_plan_valid_false",
                "Upstream grasp plan explicitly invalidated.",
            )
            return
        self.last_upstream_valid_heartbeat_time = time.monotonic()
        self.try_promote_pending_plan()

    def handle_grasp_plan_status(self, message: String) -> None:
        self.last_grasp_plan_status = str(message.data)
        self.last_upstream_status_heartbeat_time = time.monotonic()
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.last_grasp_plan_status_json = None
            return
        if not isinstance(value, dict):
            self.last_grasp_plan_status_json = None
            return
        self.last_grasp_plan_status_json = value
        if value.get("status") != "VALID":
            return
        plan_id = value.get("plan_id")
        status_hash = value.get("trajectory_hash")
        if plan_id is None or status_hash is None:
            return
        plan_id = str(plan_id)
        status_hash = str(status_hash)
        self.last_upstream_valid_heartbeat_time = time.monotonic()
        if self.latest_upstream_plan_id != plan_id:
            self.get_logger().info(
                "Upstream grasp plan heartbeat changed | "
                f"old={self.latest_upstream_plan_id} | new={plan_id}"
            )
        self.latest_upstream_plan_id = plan_id
        self.pending_plan_id = plan_id
        self.pending_trajectory_hash = status_hash
        self.pending_status_time = time.monotonic()
        self.pending_payload_time = (
            self.payload_cache.get(status_hash, (None, None))[1]
            if status_hash in self.payload_cache
            else None
        )
        self.try_promote_pending_plan()

    def handle_source_trajectory(self, message: JointTrajectory) -> None:
        self.last_source_trajectory_time = time.monotonic()
        self.latest_payload_time = self.last_source_trajectory_time
        payload_hash = self.compute_payload_hash(message)
        self.latest_payload_hash = payload_hash
        self.payload_cache[payload_hash] = (message, self.last_source_trajectory_time)
        if len(self.payload_cache) > 8:
            oldest = sorted(
                self.payload_cache.items(),
                key=lambda item: item[1][1],
            )[0][0]
            self.payload_cache.pop(oldest, None)
        if payload_hash == self.pending_trajectory_hash:
            self.pending_payload_time = self.last_source_trajectory_time
        self.try_promote_pending_plan()

    def try_promote_pending_plan(self) -> None:
        if not self.plan_valid:
            return
        if (
            self.last_upstream_valid_heartbeat_time is None
            or time.monotonic() - self.last_upstream_valid_heartbeat_time
            > self.config.source_valid_heartbeat_timeout_s
        ):
            return
        if (
            self.last_upstream_status_heartbeat_time is None
            or time.monotonic() - self.last_upstream_status_heartbeat_time
            > self.config.source_valid_heartbeat_timeout_s
        ):
            return
        if self.pending_plan_id is None or self.pending_trajectory_hash is None:
            return
        cached = self.payload_cache.get(self.pending_trajectory_hash)
        if cached is None:
            return
        message, payload_time = cached
        self.pending_payload_time = payload_time
        if (
            self.active_plan_id == self.pending_plan_id
            and self.active_trajectory_hash == self.pending_trajectory_hash
            and self.last_trajectory is not None
            and self.last_result is not None
        ):
            return
        self.timed_trajectory_recomputed = False
        self.using_cached_timed_trajectory = False
        raw_positions = [
            [float(value) for value in point.positions]
            for point in message.points
        ]
        self.last_source_waypoint_count = len(raw_positions)

        validation = validate_source_path(
            joint_names=list(message.joint_names),
            raw_positions=raw_positions,
            config=self.config,
            lower=self.lower,
            upper=self.upper,
        )
        self.last_margin = validation.minimum_joint_limit_margin_rad
        self.last_adjacent_delta = validation.maximum_input_adjacent_delta_rad

        if not validation.valid or validation.positions is None:
            self.invalidate(validation.reason)
            self.last_input_validation_error_time = time.monotonic()
            return

        result = parameterize_path(validation.positions, self.config)
        self.last_result = result
        self.last_output_point_count = len(result.points)
        if not result.success:
            self.invalidate(result.reason, preserve_result=True)
            self.last_input_validation_error_time = time.monotonic()
            return

        self.active_plan_id = self.pending_plan_id
        self.active_trajectory_hash = self.pending_trajectory_hash
        self.active_payload_time = payload_time
        self.last_trajectory = self.make_trajectory(
            result,
            self.active_plan_id,
            self.active_trajectory_hash,
        )
        self.last_parameterized_plan_time = time.monotonic()
        self.reparameterization_count += 1
        self.timed_trajectory_recomputed = True
        self.using_cached_timed_trajectory = False
        self.last_status = self.make_status(
            status="VALID",
            reason="time_parameterized_preview_valid",
        )
        self.get_logger().info(
            "Safe timed trajectory reparameterized | "
            f"plan_id={self.active_plan_id} | "
            f"trajectory_hash={self.active_trajectory_hash} | "
            f"source_waypoints={self.last_source_waypoint_count} | "
            f"output_points={self.last_output_point_count} | "
            f"reparameterization_count={self.reparameterization_count}"
        )
        self.trajectory_publisher.publish(self.last_trajectory)
        self.publish_status()

    def compute_payload_hash(self, message: JointTrajectory) -> str:
        raw_positions = [
            [float(value) for value in point.positions]
            for point in message.points
        ]
        planner_config_version = DEFAULT_PLANNER_CONFIG_VERSION
        if isinstance(self.last_grasp_plan_status_json, dict):
            planner_config_version = str(
                self.last_grasp_plan_status_json.get(
                    "planner_config_version",
                    DEFAULT_PLANNER_CONFIG_VERSION,
                )
            )
        return trajectory_hash(
            list(message.joint_names),
            raw_positions,
            planner_config_version,
        )

    def make_trajectory(
        self,
        result: TimedTrajectoryResult,
        plan_id: str,
        trajectory_hash_value: str,
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = (
            f"base_link;plan_id={plan_id};trajectory_hash={trajectory_hash_value}"
        )
        trajectory.joint_names = list(self.config.joint_names)
        for source in result.points:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in source.positions.tolist()]
            point.velocities = [float(value) for value in source.velocities.tolist()]
            point.accelerations = [
                float(value)
                for value in source.accelerations.tolist()
            ]
            point.time_from_start = duration_from_seconds(
                source.time_from_start_s
            )
            trajectory.points.append(point)
        return trajectory

    def invalidate(
        self,
        reason: str,
        preserve_result: bool = False,
        clear_result: bool | None = None,
        clear_trajectory: bool = True,
    ) -> None:
        if clear_trajectory:
            self.last_trajectory = None
        if clear_result is None:
            clear_result = not preserve_result
        if not preserve_result:
            clear_result = True
        if clear_result:
            self.last_result = None
            self.last_output_point_count = 0
        self.timed_trajectory_recomputed = False
        self.using_cached_timed_trajectory = False
        self.last_status = self.make_status(status="INVALID", reason=reason)

    def throttled_log(self, key: str, message: str) -> None:
        now = time.monotonic()
        last = self.last_log_by_key.get(key, 0.0)
        if now - last >= THROTTLED_LOG_PERIOD_S:
            self.get_logger().info(message)
            self.last_log_by_key[key] = now

    def make_status(self, status: str, reason: str) -> dict[str, Any]:
        result = self.last_result
        now_monotonic = time.monotonic()
        now_msg = self.get_clock().now().to_msg()
        source_age = (
            None
            if self.last_source_trajectory_time is None
            else now_monotonic - self.last_source_trajectory_time
        )
        heartbeat_age = (
            None
            if self.last_upstream_valid_heartbeat_time is None
            else now_monotonic - self.last_upstream_valid_heartbeat_time
        )
        status_heartbeat_age = (
            None
            if self.last_upstream_status_heartbeat_time is None
            else now_monotonic - self.last_upstream_status_heartbeat_time
        )
        parameterized_age = (
            None
            if self.last_parameterized_plan_time is None
            else now_monotonic - self.last_parameterized_plan_time
        )
        active_payload_age = (
            None
            if self.active_payload_time is None
            else now_monotonic - self.active_payload_time
        )
        pending_payload_age = (
            None
            if self.pending_payload_time is None
            else now_monotonic - self.pending_payload_time
        )
        maximum_velocity = (
            [0.0] * len(self.config.joint_names)
            if result is None
            else result.maximum_velocity_rad_s_observed
        )
        maximum_acceleration = (
            [0.0] * len(self.config.joint_names)
            if result is None
            else result.maximum_acceleration_rad_s2_observed
        )
        return {
            "status": status,
            "reason": reason,
            "plan_id": self.active_plan_id,
            "active_plan_id": self.active_plan_id,
            "pending_plan_id": self.pending_plan_id,
            "latest_upstream_plan_id": self.latest_upstream_plan_id,
            "active_trajectory_hash": self.active_trajectory_hash,
            "pending_trajectory_hash": self.pending_trajectory_hash,
            "latest_payload_hash": self.latest_payload_hash,
            "source_waypoint_count": self.last_source_waypoint_count,
            "output_point_count": self.last_output_point_count if status == "VALID" else 0,
            "total_duration_s": 0.0 if result is None else result.total_duration_s,
            "sample_rate_hz": self.config.sample_rate_hz,
            "maximum_velocity_rad_s_observed": maximum_velocity,
            "maximum_acceleration_rad_s2_observed": maximum_acceleration,
            "velocity_limits_rad_s": [
                float(value)
                for value in self.config.maximum_velocity_rad_s.tolist()
            ],
            "acceleration_limits_rad_s2": [
                float(value)
                for value in self.config.maximum_acceleration_rad_s2.tolist()
            ],
            "minimum_joint_limit_margin_rad": self.last_margin,
            "maximum_input_adjacent_delta_rad": self.last_adjacent_delta,
            "time_strictly_increasing": (
                False if result is None else result.time_strictly_increasing
            ),
            "all_positions_finite": False if result is None else result.all_positions_finite,
            "all_velocities_finite": False if result is None else result.all_velocities_finite,
            "all_accelerations_finite": (
                False if result is None else result.all_accelerations_finite
            ),
            "limit_profile_status": LIMIT_PROFILE_STATUS,
            "stop_at_each_source_waypoint": self.config.stop_at_each_source_waypoint,
            "timestamp": now_msg.sec + now_msg.nanosec * 1.0e-9,
            "hardware_control_enabled": False,
            "published_controller_command_topics": [],
            "using_cached_timed_trajectory": self.using_cached_timed_trajectory,
            "timed_trajectory_recomputed": self.timed_trajectory_recomputed,
            "reparameterization_count": self.reparameterization_count,
            "cached_timed_trajectory_hit_count": self.cached_timed_trajectory_hit_count,
            "last_source_trajectory_age_s": source_age,
            "last_upstream_valid_heartbeat_age_s": heartbeat_age,
            "upstream_valid_heartbeat_age_s": heartbeat_age,
            "upstream_status_heartbeat_age_s": status_heartbeat_age,
            "active_payload_age_s": active_payload_age,
            "pending_payload_age_s": pending_payload_age,
            "latest_payload_age_s": (
                None
                if self.latest_payload_time is None
                else now_monotonic - self.latest_payload_time
            ),
            "last_parameterized_plan_age_s": parameterized_age,
            "source_waypoint_times_s": (
                [] if result is None else result.source_waypoint_times_s
            ),
            "segment_durations_s": [] if result is None else result.segment_durations_s,
            "input_stale_timeout_s": self.config.input_stale_timeout_s,
            "source_valid_heartbeat_timeout_s": (
                self.config.source_valid_heartbeat_timeout_s
            ),
            "trajectory_payload_timeout_before_first_plan_s": (
                self.config.trajectory_payload_timeout_before_first_plan_s
            ),
            "require_periodic_trajectory_republish": (
                self.config.require_periodic_trajectory_republish
            ),
            "source_plan_status_raw": self.last_grasp_plan_status,
            "disabled_controller_command_topics": list(COMMAND_TOPIC_NAMES),
        }

    def publish_status(self) -> None:
        preserving_recent_input_error = bool(
            self.last_status.get("status") == "INVALID"
            and self.last_status.get("reason") in INPUT_VALIDATION_REASONS
            and self.last_input_validation_error_time is not None
            and time.monotonic() - self.last_input_validation_error_time <= 0.5
        )
        if preserving_recent_input_error:
            pass
        elif not self.plan_valid:
            self.invalidate("grasp_plan_valid_false")
        elif self.last_upstream_valid_heartbeat_time is None:
            self.invalidate("waiting_for_source_valid_heartbeat")
        elif (
            self.pending_status_time is not None
            and self.pending_plan_id is not None
            and (
                self.active_plan_id != self.pending_plan_id
                or self.active_trajectory_hash != self.pending_trajectory_hash
            )
            and time.monotonic() - self.pending_status_time
            > self.config.source_valid_heartbeat_timeout_s
        ):
            self.pending_plan_id = None
            self.pending_trajectory_hash = None
            self.pending_status_time = None
            self.pending_payload_time = None
            self.invalidate(
                "pending_plan_timeout",
                clear_result=False,
                clear_trajectory=True,
            )
        elif (
            time.monotonic() - self.last_upstream_valid_heartbeat_time
            > self.config.source_valid_heartbeat_timeout_s
        ):
            self.invalidate("upstream_valid_heartbeat_stale")
            self.throttled_log(
                "upstream_valid_heartbeat_stale",
                "Upstream valid heartbeat stale; safe timed trajectory invalidated.",
            )
        elif self.last_upstream_status_heartbeat_time is None:
            self.invalidate("waiting_for_source_status_heartbeat")
        elif (
            time.monotonic() - self.last_upstream_status_heartbeat_time
            > self.config.source_valid_heartbeat_timeout_s
        ):
            self.invalidate("upstream_status_heartbeat_stale")
            self.throttled_log(
                "upstream_status_heartbeat_stale",
                "Upstream status heartbeat stale; safe timed trajectory invalidated.",
            )
        elif (
            self.pending_trajectory_hash is not None
            and self.latest_payload_hash is not None
            and self.latest_payload_time is not None
            and self.pending_status_time is not None
            and self.latest_payload_time >= self.pending_status_time
            and self.pending_trajectory_hash not in self.payload_cache
            and self.latest_payload_hash != self.pending_trajectory_hash
        ):
            self.invalidate(
                "plan_payload_hash_mismatch",
                clear_result=False,
                clear_trajectory=True,
            )
        elif self.last_trajectory is None or self.last_result is None:
            if self.latest_upstream_plan_id is not None and (
                self.active_plan_id is not None
                and self.latest_upstream_plan_id != self.active_plan_id
            ):
                self.invalidate(
                    "waiting_for_matching_source_trajectory",
                    clear_result=False,
                    clear_trajectory=True,
                )
            else:
                self.invalidate("waiting_for_source_trajectory")
        elif (
            self.latest_upstream_plan_id is not None
            and self.active_plan_id is not None
            and self.latest_upstream_plan_id != self.active_plan_id
        ):
            self.invalidate(
                "waiting_for_matching_source_trajectory",
                clear_result=False,
                clear_trajectory=True,
            )
        elif (
            self.pending_trajectory_hash is not None
            and self.active_trajectory_hash is not None
            and self.pending_trajectory_hash != self.active_trajectory_hash
        ):
            self.invalidate(
                "waiting_for_matching_source_trajectory",
                clear_result=False,
                clear_trajectory=True,
            )
        elif (
            self.config.require_periodic_trajectory_republish
            and self.last_source_trajectory_time is not None
            and time.monotonic() - self.last_source_trajectory_time
            > self.config.input_stale_timeout_s
        ):
            self.invalidate("source_trajectory_stale")
        else:
            if self.timed_trajectory_recomputed:
                self.using_cached_timed_trajectory = False
            else:
                self.using_cached_timed_trajectory = True
                self.cached_timed_trajectory_hit_count += 1
            self.last_status = self.make_status(
                status="VALID",
                reason="time_parameterized_preview_valid",
            )
            self.throttled_log(
                "cached_timed_trajectory_hold",
                f"Using cached timed trajectory for plan_id={self.active_plan_id}",
            )

        valid_message = Bool()
        valid_message.data = self.last_status.get("status") == "VALID"
        self.valid_publisher.publish(valid_message)

        status_message = String()
        status_message.data = json.dumps(
            self.last_status,
            ensure_ascii=False,
            allow_nan=False,
        )
        self.status_publisher.publish(status_message)

        reason_message = String()
        reason_message.data = str(self.last_status.get("reason", ""))
        self.reason_publisher.publish(reason_message)

        self.timed_trajectory_recomputed = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: TimedTrajectoryNode | None = None
    try:
        node = TimedTrajectoryNode()
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
