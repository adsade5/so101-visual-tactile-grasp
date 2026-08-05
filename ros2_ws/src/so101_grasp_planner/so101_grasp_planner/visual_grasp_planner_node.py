from __future__ import annotations

import json
import hashlib
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_kinematics.top_down_ik import (
    APPROACH_TOLERANCE_DEG,
    BASE_LINK,
    MAXIMUM_ADJACENT_DELTA_RAD,
    MINIMUM_LIMIT_MARGIN_RAD,
    POSITION_TOLERANCE_MM,
    RANDOM_SEED,
    REFERENCE_Q_DEG,
    TARGET_APPROACH_AXIS_BASE,
    TIP_LINK,
    create_default_solver,
    format_float_list,
    generate_vertical_joint_path,
    pose_data,
)
from so101_kinematics.urdf_fk import ARM_JOINT_NAMES


DEFAULT_PROJECT_ROOT = (
    "E:/PycharmProjects/Embodied_AI/"
    "LeRobot_Project/so101_visual_tactile_grasp"
)

WORKSPACE_X_BOUNDS_M = (0.12, 0.32)
WORKSPACE_Y_BOUNDS_M = (-0.12, 0.12)
OBJECT_Z_BOUNDS_M = (0.015, 0.040)
STALE_TIMEOUT_S = 0.5
STATUS_PERIOD_S = 0.1
OBJECT_REPLAN_POSITION_THRESHOLD_M = 0.001
THROTTLED_LOG_PERIOD_S = 2.0
PLANNER_CONFIG_VERSION = "visual_grasp_planner_v1"
DEFAULT_REFERENCE_POSITION_M = np.asarray(
    [0.18289733886666237, -1.0074442143860504e-05, 0.05442612789124232],
    dtype=np.float64,
)
DEFAULT_REFERENCE_APPROACH_AXIS = np.asarray(
    [9.155950952010694e-06, 6.193363591682789e-06, -0.9999999999389054],
    dtype=np.float64,
)


def finite_values(values: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def quaternion_is_valid(values: list[float]) -> bool:
    if not finite_values(values):
        return False
    norm = math.sqrt(sum(float(value) * float(value) for value in values))
    return norm > 1.0e-6


def top_down_quaternion_xyzw() -> tuple[float, float, float, float]:
    return (0.0, 1.0, 0.0, 0.0)


def set_pose_position(
    message: PoseStamped,
    position: np.ndarray,
) -> None:
    message.pose.position.x = float(position[0])
    message.pose.position.y = float(position[1])
    message.pose.position.z = float(position[2])
    qx, qy, qz, qw = top_down_quaternion_xyzw()
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw


def duration_from_seconds(seconds: float) -> Any:
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1.0e9))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return whole, nanoseconds


def trajectory_hash(
    joint_names: list[str],
    positions: list[list[float]],
    planner_config_version: str = PLANNER_CONFIG_VERSION,
) -> str:
    payload = {
        "joint_names": [str(value) for value in joint_names],
        "positions": [
            [round(float(value), 12) for value in point]
            for point in positions
        ],
        "waypoint_count": len(positions),
        "planner_config_version": planner_config_version,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


class VisualGraspPlannerNode(Node):
    def __init__(self) -> None:
        precomputed_project_root = Path(DEFAULT_PROJECT_ROOT).resolve()
        precomputed_solver, precomputed_model = create_default_solver(
            precomputed_project_root
        )
        precomputed_reference_q_rad = np.radians(REFERENCE_Q_DEG)
        precomputed_reference_position_m = DEFAULT_REFERENCE_POSITION_M.copy()
        precomputed_reference_approach_axis = DEFAULT_REFERENCE_APPROACH_AXIS.copy()

        super().__init__("visual_grasp_planner_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.declare_parameter("pregrasp_clearance_m", 0.055)
        self.declare_parameter("grasp_clearance_m", 0.015)
        self.declare_parameter("vertical_step_m", 0.005)
        self.declare_parameter("minimum_tcp_z_m", 0.035)

        self.project_root = Path(
            str(self.get_parameter("project_root").value)
        ).resolve()
        self.pregrasp_clearance_m = float(
            self.get_parameter("pregrasp_clearance_m").value
        )
        self.grasp_clearance_m = float(
            self.get_parameter("grasp_clearance_m").value
        )
        self.vertical_step_m = float(
            self.get_parameter("vertical_step_m").value
        )
        self.minimum_tcp_z_m = float(
            self.get_parameter("minimum_tcp_z_m").value
        )

        if self.vertical_step_m <= 0.0:
            raise ValueError("vertical_step_m must be positive")

        self.config = self.load_workspace_transform_config()
        self.solver = precomputed_solver
        self.model = precomputed_model
        self.reference_q_rad = precomputed_reference_q_rad
        self.reference_position_m = precomputed_reference_position_m
        self.reference_approach_axis = precomputed_reference_approach_axis
        if self.project_root != precomputed_project_root:
            self.solver, self.model = create_default_solver(self.project_root)
            self.reference_q_rad = np.radians(REFERENCE_Q_DEG)
            self.reference_position_m, _, self.reference_approach_axis = pose_data(
                self.solver.fk,
                self.reference_q_rad,
            )

        self.object_pose_valid = False
        self.last_pose_received_time: float | None = None
        self.last_object_pose_base_valid_time: float | None = None
        self.last_pose_stamp_key: tuple[int, int, str] | None = None
        self.latest_pose_message: PoseStamped | None = None
        self.pose_received_count = 0
        self.valid_true_count = 0
        self.plan_attempt_count = 0
        self.planner_heartbeat_seq = 0
        self.plan_id: str | None = None
        self.trajectory_hash: str | None = None
        self.latest_generated_plan_id: str | None = None
        self.trajectory_payload_publish_seq = 0
        self.trajectory_payload_publish_timestamp: float | None = None
        self.cached_plan_available = False
        self.trajectory_needs_publish = False
        self.last_trajectory_republished = False
        self.last_planned_object_position_m: np.ndarray | None = None
        self.last_log_by_key: dict[str, float] = {}
        self.last_processed_position: list[float] | None = None
        self.last_waypoint_count = 0
        self.last_max_position_error_mm: float | None = None
        self.last_max_approach_error_deg: float | None = None
        self.last_min_margin_rad: float | None = None
        self.last_max_adjacent_delta_rad: float | None = None
        self.last_status: dict[str, Any] = self.make_status(
            status="INVALID",
            reason="waiting_for_object_pose_base",
        )
        self.last_trajectory: JointTrajectory | None = None
        self.last_pregrasp_pose: PoseStamped | None = None
        self.last_grasp_pose: PoseStamped | None = None

        self.plan_valid_publisher = self.create_publisher(
            Bool,
            "/grasp_plan_valid",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/grasp_plan_status",
            10,
        )
        self.pregrasp_publisher = self.create_publisher(
            PoseStamped,
            "/grasp_pregrasp_pose",
            10,
        )
        self.target_publisher = self.create_publisher(
            PoseStamped,
            "/grasp_target_pose",
            10,
        )
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            "/planned_grasp_joint_trajectory",
            10,
        )
        self.reason_publisher = self.create_publisher(
            String,
            "/grasp_planner_validity_reason",
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/object_pose_base",
            self.handle_object_pose,
            10,
        )
        self.create_subscription(
            Bool,
            "/object_pose_base_valid",
            self.handle_object_valid,
            10,
        )
        self.create_timer(STATUS_PERIOD_S, self.publish_status)

        self.get_logger().info(
            "SO-101 visual grasp planner dry-run started | "
            f"hardware_disabled=True | frame={BASE_LINK} | "
            f"calibration_status={self.config.get('calibration_status')} | "
            f"z_assumption={self.config.get('z_assumption')} | "
            f"pregrasp_clearance_m={self.pregrasp_clearance_m:.3f} | "
            f"grasp_clearance_m={self.grasp_clearance_m:.3f} | "
            f"vertical_step_m={self.vertical_step_m:.3f}"
        )
        self.get_logger().info(
            "Publishing planning preview only on /planned_grasp_joint_trajectory; "
            "no robot command or controller topic is used."
        )

    def load_workspace_transform_config(self) -> dict[str, Any]:
        config_path = self.project_root / "config" / "workspace_to_base.json"
        if not config_path.is_file():
            raise FileNotFoundError(f"Workspace transform config not found: {config_path}")
        value = json.loads(config_path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError(f"Expected JSON object in {config_path}")
        return value

    def stamp_key(self, message: PoseStamped) -> tuple[int, int, str]:
        return (
            int(message.header.stamp.sec),
            int(message.header.stamp.nanosec),
            str(message.header.frame_id),
        )

    def make_status(
        self,
        status: str,
        reason: str,
        object_position: list[float] | None = None,
        waypoint_count: int = 0,
        max_position_error_mm: float | None = None,
        max_approach_error_deg: float | None = None,
        min_margin_rad: float | None = None,
        max_adjacent_delta_rad: float | None = None,
    ) -> dict[str, Any]:
        now_monotonic = time.monotonic()
        now_msg = self.get_clock().now().to_msg()
        last_object_pose_age = (
            None
            if self.last_pose_received_time is None
            else now_monotonic - self.last_pose_received_time
        )
        last_object_valid_age = (
            None
            if self.last_object_pose_base_valid_time is None
            else now_monotonic - self.last_object_pose_base_valid_time
        )
        return {
            "status": status,
            "reason": reason,
            "plan_id": self.plan_id,
            "active_plan_id": self.plan_id if status == "VALID" else None,
            "latest_generated_plan_id": self.latest_generated_plan_id,
            "trajectory_hash": self.trajectory_hash if status == "VALID" else None,
            "trajectory_point_count": self.last_waypoint_count if status == "VALID" else 0,
            "trajectory_payload_publish_seq": self.trajectory_payload_publish_seq,
            "trajectory_payload_publish_timestamp": self.trajectory_payload_publish_timestamp,
            "planner_config_version": PLANNER_CONFIG_VERSION,
            "planner_heartbeat_seq": self.planner_heartbeat_seq,
            "last_object_pose_age_s": last_object_pose_age,
            "last_object_pose_base_valid_age_s": last_object_valid_age,
            "cached_plan_available": self.cached_plan_available,
            "trajectory_republished": self.last_trajectory_republished,
            "object_position_base_m": object_position,
            "waypoint_count": waypoint_count,
            "max_position_error_mm": max_position_error_mm,
            "max_approach_error_deg": max_approach_error_deg,
            "min_margin_rad": min_margin_rad,
            "max_adjacent_delta_rad": max_adjacent_delta_rad,
            "calibration_status": self.config.get("calibration_status"),
            "z_assumption": self.config.get("z_assumption"),
            "timestamp": now_msg.sec + now_msg.nanosec * 1.0e-9,
            "time_parameterization_status": "nominal_preview_only",
            "hardware_control_enabled": False,
            "command_topics_published": [],
            "debug": {
                "pose_received_count": self.pose_received_count,
                "valid_true_count": self.valid_true_count,
                "plan_attempt_count": self.plan_attempt_count,
                "last_processed_position_base_m": self.last_processed_position,
            },
        }

    def invalidate(
        self,
        reason: str,
        object_position: list[float] | None = None,
    ) -> None:
        if reason != "object_pose_base_stale":
            self.last_pose_received_time = time.monotonic()

        self.cached_plan_available = False
        self.trajectory_needs_publish = False
        self.last_trajectory = None
        self.last_pregrasp_pose = None
        self.last_grasp_pose = None
        self.last_waypoint_count = 0
        self.last_max_position_error_mm = None
        self.last_max_approach_error_deg = None
        self.last_min_margin_rad = None
        self.last_max_adjacent_delta_rad = None
        self.last_status = self.make_status(
            status="INVALID",
            reason=reason,
            object_position=object_position,
        )
        self.throttled_log(f"invalid:{reason}", f"Planner invalid: {reason}")

    def handle_object_valid(self, message: Bool) -> None:
        self.object_pose_valid = bool(message.data)
        if not self.object_pose_valid:
            self.invalidate("object_pose_base_valid_false")
            return

        self.valid_true_count += 1
        self.last_object_pose_base_valid_time = time.monotonic()

        if (
            self.latest_pose_message is not None
            and self.last_pose_received_time is not None
            and time.monotonic() - self.last_pose_received_time <= STALE_TIMEOUT_S
        ):
            self.process_object_pose(self.latest_pose_message)

    def handle_object_pose(self, message: PoseStamped) -> None:
        now = time.monotonic()
        self.last_pose_received_time = now
        self.latest_pose_message = message
        self.pose_received_count += 1

        if not self.object_pose_valid:
            self.invalidate("waiting_for_object_pose_base_valid")
            return

        self.process_object_pose(message)

    def process_object_pose(self, message: PoseStamped) -> None:
        key = self.stamp_key(message)
        if self.last_pose_stamp_key == key:
            return
        self.last_pose_stamp_key = key

        if message.header.frame_id != BASE_LINK:
            self.invalidate(f"wrong_frame:{message.header.frame_id}")
            return

        position = message.pose.position
        orientation = message.pose.orientation
        object_position = [
            float(position.x),
            float(position.y),
            float(position.z),
        ]
        self.last_processed_position = object_position
        values = object_position + [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ]
        if not finite_values(values):
            self.invalidate("non_finite_pose", object_position)
            return
        if not quaternion_is_valid(values[3:]):
            self.invalidate("invalid_quaternion", object_position)
            return
        x, y, z = object_position
        if not (WORKSPACE_X_BOUNDS_M[0] <= x <= WORKSPACE_X_BOUNDS_M[1]):
            self.invalidate("object_x_out_of_bounds", object_position)
            return
        if not (WORKSPACE_Y_BOUNDS_M[0] <= y <= WORKSPACE_Y_BOUNDS_M[1]):
            self.invalidate("object_y_out_of_bounds", object_position)
            return
        if not (OBJECT_Z_BOUNDS_M[0] <= z <= OBJECT_Z_BOUNDS_M[1]):
            self.invalidate("object_z_out_of_bounds", object_position)
            return

        object_array = np.asarray(object_position, dtype=np.float64)
        try:
            if self.should_replan(object_array):
                self.plan_for_object(message, object_array)
            elif self.cached_plan_available:
                self.last_status = self.make_status(
                    status="VALID",
                    reason="planned_preview_only",
                    object_position=object_position,
                    waypoint_count=self.last_waypoint_count,
                    max_position_error_mm=self.last_max_position_error_mm,
                    max_approach_error_deg=self.last_max_approach_error_deg,
                    min_margin_rad=self.last_min_margin_rad,
                    max_adjacent_delta_rad=self.last_max_adjacent_delta_rad,
                )
                self.throttled_log(
                    "planner_cached_hold",
                    f"Planner heartbeat keeps cached plan valid: {self.plan_id}",
                )
        except Exception as error:
            self.invalidate(
                f"planner_exception:{error!r}",
                object_position,
            )

    def should_replan(self, object_position: np.ndarray) -> bool:
        if not self.cached_plan_available or self.last_trajectory is None:
            return True
        if self.last_planned_object_position_m is None:
            return True
        delta = float(np.linalg.norm(object_position - self.last_planned_object_position_m))
        return delta > OBJECT_REPLAN_POSITION_THRESHOLD_M

    def make_plan_id(
        self,
        object_position: np.ndarray,
        pregrasp: np.ndarray,
        grasp: np.ndarray,
        q_path_rad: list[np.ndarray],
    ) -> str:
        payload = {
            "joint_names": list(ARM_JOINT_NAMES),
            "object_position_base_m": [
                round(float(value), 9)
                for value in object_position.tolist()
            ],
            "pregrasp_base_m": [
                round(float(value), 9)
                for value in pregrasp.tolist()
            ],
            "grasp_base_m": [
                round(float(value), 9)
                for value in grasp.tolist()
            ],
            "positions": [
                [round(float(value), 9) for value in q.tolist()]
                for q in q_path_rad
            ],
        }
        text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(text.encode("utf-8")).hexdigest()[:24]

    def throttled_log(self, key: str, message: str) -> None:
        now = time.monotonic()
        last = self.last_log_by_key.get(key, 0.0)
        if now - last >= THROTTLED_LOG_PERIOD_S:
            self.get_logger().info(message)
            self.last_log_by_key[key] = now

    def interpolate_segment(
        self,
        start: np.ndarray,
        end: np.ndarray,
        include_start: bool,
    ) -> list[np.ndarray]:
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(math.ceil(distance / self.vertical_step_m)))
        points: list[np.ndarray] = []
        first_index = 0 if include_start else 1
        for index in range(first_index, steps + 1):
            alpha = index / steps
            points.append(start + alpha * (end - start))
        return points

    def build_tcp_targets(
        self,
        object_position: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
        pregrasp = object_position + np.asarray(
            [0.0, 0.0, self.pregrasp_clearance_m],
            dtype=np.float64,
        )
        grasp = object_position + np.asarray(
            [0.0, 0.0, self.grasp_clearance_m],
            dtype=np.float64,
        )
        for name, target in (("pregrasp", pregrasp), ("grasp", grasp)):
            if float(target[2]) < self.minimum_tcp_z_m:
                raise ValueError(
                    f"{name}_tcp_z_below_minimum:{target[2]:.6f}"
                )
        targets: list[np.ndarray] = []
        targets.append(pregrasp.copy())
        targets.extend(
            self.interpolate_segment(
                pregrasp,
                grasp,
                include_start=False,
            )
        )
        targets.extend(
            self.interpolate_segment(
                grasp,
                pregrasp,
                include_start=False,
            )
        )
        return pregrasp, grasp, targets

    def make_pose(
        self,
        source: PoseStamped,
        position: np.ndarray,
    ) -> PoseStamped:
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = BASE_LINK
        set_pose_position(output, position)
        return output

    def make_trajectory(
        self,
        q_path_rad: list[np.ndarray],
    ) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = BASE_LINK
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        for index, q in enumerate(q_path_rad):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in q.tolist()]
            seconds, nanoseconds = duration_from_seconds(0.25 * index)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            trajectory.points.append(point)
        return trajectory

    def plan_for_object(
        self,
        message: PoseStamped,
        object_position: np.ndarray,
    ) -> None:
        self.plan_attempt_count += 1
        object_position_list = format_float_list(object_position)
        try:
            pregrasp, grasp, targets = self.build_tcp_targets(object_position)
        except ValueError as error:
            self.invalidate(str(error), object_position_list)
            return

        reference_to_pregrasp = self.interpolate_segment(
            self.reference_position_m,
            pregrasp,
            include_start=True,
        )
        solve_targets = [*reference_to_pregrasp, *targets[1:]]
        payload_start_index = len(reference_to_pregrasp) - 1
        path = generate_vertical_joint_path(
            solver=self.solver,
            target_positions_m=solve_targets,
            reference_q_rad=self.reference_q_rad,
            position_tolerance_mm=POSITION_TOLERANCE_MM,
            approach_tolerance_deg=APPROACH_TOLERANCE_DEG,
            minimum_limit_margin_rad=MINIMUM_LIMIT_MARGIN_RAD,
            maximum_adjacent_delta_rad=MAXIMUM_ADJACENT_DELTA_RAD,
            rng=np.random.default_rng(RANDOM_SEED),
        )
        if not path.success:
            self.invalidate(
                f"ik_failed:{path.failure_reason}",
                object_position_list,
            )
            return

        payload_q_path = path.q_path_rad[payload_start_index:]
        if not payload_q_path:
            self.invalidate("ik_failed:no_payload_waypoints", object_position_list)
            return

        self.last_object_pose_base_valid_time = time.monotonic()
        self.last_pregrasp_pose = self.make_pose(message, pregrasp)
        self.last_grasp_pose = self.make_pose(message, grasp)
        self.last_trajectory = self.make_trajectory(payload_q_path)
        self.last_waypoint_count = len(payload_q_path)
        self.last_max_position_error_mm = path.max_position_error_mm
        self.last_max_approach_error_deg = path.max_approach_error_deg
        self.last_min_margin_rad = path.min_margin_rad
        self.last_max_adjacent_delta_rad = path.max_adjacent_delta_rad
        self.plan_id = self.make_plan_id(
            object_position=object_position,
            pregrasp=pregrasp,
            grasp=grasp,
            q_path_rad=payload_q_path,
        )
        self.latest_generated_plan_id = self.plan_id
        self.trajectory_hash = trajectory_hash(
            list(ARM_JOINT_NAMES),
            [
                [float(value) for value in q.tolist()]
                for q in payload_q_path
            ],
        )
        self.last_trajectory.header.frame_id = (
            f"{BASE_LINK};plan_id={self.plan_id};trajectory_hash={self.trajectory_hash}"
        )
        self.cached_plan_available = True
        self.trajectory_needs_publish = True
        self.last_planned_object_position_m = object_position.copy()
        self.last_pose_received_time = time.monotonic()
        self.last_status = self.make_status(
            status="VALID",
            reason="planned_preview_only",
            object_position=object_position_list,
            waypoint_count=self.last_waypoint_count,
            max_position_error_mm=self.last_max_position_error_mm,
            max_approach_error_deg=self.last_max_approach_error_deg,
            min_margin_rad=self.last_min_margin_rad,
            max_adjacent_delta_rad=self.last_max_adjacent_delta_rad,
        )
        self.get_logger().info(
            "New SO-101 grasp preview plan created | "
            f"plan_id={self.plan_id} | "
            f"waypoints={len(path.q_path_rad)} | "
            f"plan_attempt_count={self.plan_attempt_count}"
        )

    def publish_status(self) -> None:
        self.planner_heartbeat_seq += 1
        if (
            self.last_pose_received_time is None
            or time.monotonic() - self.last_pose_received_time > STALE_TIMEOUT_S
        ):
            self.invalidate("object_pose_base_stale")
        elif (
            self.last_object_pose_base_valid_time is None
            or time.monotonic() - self.last_object_pose_base_valid_time > STALE_TIMEOUT_S
        ):
            self.invalidate("object_pose_base_valid_false")

        valid_now = self.last_status.get("status") == "VALID"
        will_publish_trajectory = bool(
            valid_now
            and self.last_trajectory is not None
            and self.trajectory_needs_publish
        )
        if will_publish_trajectory:
            self.trajectory_payload_publish_seq += 1
            now_msg = self.get_clock().now().to_msg()
            self.trajectory_payload_publish_timestamp = (
                now_msg.sec + now_msg.nanosec * 1.0e-9
            )
        self.last_trajectory_republished = will_publish_trajectory
        if valid_now:
            self.last_status = self.make_status(
                status="VALID",
                reason="planned_preview_only",
                object_position=self.last_processed_position,
                waypoint_count=self.last_waypoint_count,
                max_position_error_mm=self.last_max_position_error_mm,
                max_approach_error_deg=self.last_max_approach_error_deg,
                min_margin_rad=self.last_min_margin_rad,
                max_adjacent_delta_rad=self.last_max_adjacent_delta_rad,
            )

        valid_message = Bool()
        valid_message.data = valid_now
        self.plan_valid_publisher.publish(valid_message)

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

        if valid_message.data:
            if self.last_pregrasp_pose is not None:
                self.pregrasp_publisher.publish(self.last_pregrasp_pose)
            if self.last_grasp_pose is not None:
                self.target_publisher.publish(self.last_grasp_pose)
            if will_publish_trajectory:
                self.trajectory_publisher.publish(self.last_trajectory)
                self.trajectory_needs_publish = False
                self.throttled_log(
                    "trajectory_event_publish",
                    f"Published event-driven source trajectory for plan_id={self.plan_id}",
                )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: VisualGraspPlannerNode | None = None
    try:
        node = VisualGraspPlannerNode()
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
