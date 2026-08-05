from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
HARDWARE_CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_hardware.json"
GRASP_CONFIG_PATH = PROJECT_ROOT / "config" / "mvp_grasp.yaml"
INTEGRATED_SNAPSHOT_PATH = PROJECT_ROOT / "data" / "runtime" / "mvp_last_integrated_grasp_snapshot.json"
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
for package_path in (
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))
if str(PROJECT_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from mvp_descend_from_pregrasp import (  # noqa: E402
    ARM_JOINT_NAMES,
    DescentConfig,
    FrozenPregrasp,
    create_model,
    plan_segmented_descent,
    validate_fresh_joint_state,
)
from mvp_move_to_pregrasp import (  # noqa: E402
    MoveConfig,
    SNAPSHOT_PATH,
    atomic_write_json,
    estimated_duration_s,
    final_joint_error,
    joint_delta,
    make_pregrasp_snapshot,
    status_is_accepted,
    target_within_urdf_limits,
    validate_joint_contract,
)


CONFIRM_PHRASE = "VISUAL_GRASP"
GRIPPER_LOGICAL_KEY = "gripper.pos"
GRIPPER_LOGICAL_NAME = "gripper"
WRIST_ROLL_LOGICAL_KEY = "wrist_roll.pos"


@dataclass(frozen=True)
class GraspConfig:
    total_descent_m: float = 0.07
    descent_waypoint_drop_m: tuple[float, ...] = (0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07)
    pregrasp_max_abs_joint_delta_rad: float = 1.00
    max_abs_joint_delta_per_descent_waypoint_rad: float = 0.25
    position_tolerance_m: float = 0.008
    approach_tolerance_deg: float = 5.0
    max_xy_error_from_waypoint_m: float = 0.010
    minimum_actual_z_drop_per_waypoint_m: float = 0.004
    minimum_total_actual_z_drop_m: float = 0.050
    arm_final_joint_tolerance_rad: float = 0.035
    speed_rad_s: float = 0.06
    max_speed_rad_s: float = 0.08
    execute_service_timeout_s: float = 120.0
    inter_waypoint_hold_s: float = 0.3
    joint_state_max_age_s: float = 1.0
    object_pose_max_age_s: float = 2.0
    pregrasp_target_max_age_s: float = 2.0
    gripper_state_max_age_s: float = 1.0
    gripper_target_max_age_s: float = 2.0
    gripper_open_target_pos: float | None = None
    gripper_open_target_verified: bool = False
    gripper_close_target_source: str = "initial_gripper_position"
    gripper_close_hold_s: float = 1.0
    gripper_interpolation_enabled: bool = True
    gripper_only_motion_duration_s: float = 2.0
    gripper_open_ramp_fraction: tuple[float, ...] = (0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.00)


@dataclass(frozen=True)
class StampedGripperState:
    position: float
    received_monotonic_s: float


def parse_yaml_scalar(value: str) -> float | bool | None | str:
    text = value.strip()
    if text.lower() == "null":
        return None
    if text.lower() == "true":
        return True
    if text.lower() == "false":
        return False
    try:
        return float(text)
    except ValueError:
        return text


def load_grasp_config(path: Path | None = None) -> GraspConfig:
    config_path = path or GRASP_CONFIG_PATH
    if not config_path.is_file():
        return GraspConfig()
    values: dict[str, Any] = {}
    current_list_key: str | None = None
    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.endswith(":"):
            current_list_key = stripped[:-1].strip()
            values[current_list_key] = []
            continue
        if stripped.startswith("-") and current_list_key is not None:
            item = parse_yaml_scalar(stripped[1:].strip())
            values[current_list_key].append(item)
            continue
        current_list_key = None
        if ":" not in stripped:
            continue
        key, raw_value = stripped.split(":", 1)
        values[key.strip()] = parse_yaml_scalar(raw_value)
    return GraspConfig(
        total_descent_m=float(values.get("total_descent_m", 0.07)),
        descent_waypoint_drop_m=tuple(float(v) for v in values.get("descent_waypoint_drop_m", [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07])),
        pregrasp_max_abs_joint_delta_rad=float(values.get("pregrasp_max_abs_joint_delta_rad", 1.00)),
        max_abs_joint_delta_per_descent_waypoint_rad=float(values.get("max_abs_joint_delta_per_descent_waypoint_rad", 0.25)),
        position_tolerance_m=float(values.get("position_tolerance_m", 0.008)),
        approach_tolerance_deg=float(values.get("approach_tolerance_deg", 5.0)),
        max_xy_error_from_waypoint_m=float(values.get("max_xy_error_from_waypoint_m", 0.010)),
        minimum_actual_z_drop_per_waypoint_m=float(values.get("minimum_actual_z_drop_per_waypoint_m", 0.004)),
        minimum_total_actual_z_drop_m=float(values.get("minimum_total_actual_z_drop_m", 0.050)),
        arm_final_joint_tolerance_rad=float(values.get("arm_final_joint_tolerance_rad", 0.035)),
        speed_rad_s=float(values.get("speed_rad_s", 0.06)),
        max_speed_rad_s=float(values.get("max_speed_rad_s", 0.08)),
        execute_service_timeout_s=float(values.get("execute_service_timeout_s", 120.0)),
        inter_waypoint_hold_s=float(values.get("inter_waypoint_hold_s", 0.3)),
        joint_state_max_age_s=float(values.get("joint_state_max_age_s", 1.0)),
        object_pose_max_age_s=float(values.get("object_pose_max_age_s", 2.0)),
        pregrasp_target_max_age_s=float(values.get("pregrasp_target_max_age_s", 2.0)),
        gripper_state_max_age_s=float(values.get("gripper_state_max_age_s", 1.0)),
        gripper_target_max_age_s=float(values.get("gripper_target_max_age_s", 2.0)),
        gripper_open_target_pos=None
        if values.get("gripper_open_target_pos") is None
        else float(values.get("gripper_open_target_pos")),
        gripper_open_target_verified=bool(values.get("gripper_open_target_verified", False)),
        gripper_close_target_source=str(values.get("gripper_close_target_source", "initial_gripper_position")),
        gripper_close_hold_s=float(values.get("gripper_close_hold_s", 1.0)),
        gripper_interpolation_enabled=bool(values.get("gripper_interpolation_enabled", True)),
        gripper_only_motion_duration_s=float(values.get("gripper_only_motion_duration_s", 2.0)),
        gripper_open_ramp_fraction=tuple(float(v) for v in values.get("gripper_open_ramp_fraction", [0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.00])),
    )


def load_motor_mapping(
    hardware_config_path: Path = HARDWARE_CONFIG_PATH,
) -> dict[str, Any]:
    hardware = json.loads(hardware_config_path.read_text(encoding="utf-8"))
    calibration_path = Path(str(hardware["calibration_path"]))
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    wrist_roll_id = int(calibration["wrist_roll"]["id"])
    gripper_id = int(calibration["gripper"]["id"])
    return {
        "calibration_path": str(calibration_path),
        "id_5_name": next((name for name, entry in calibration.items() if int(entry["id"]) == 5), None),
        "id_6_name": next((name for name, entry in calibration.items() if int(entry["id"]) == 6), None),
        "gripper_hardware_id": gripper_id,
        "wrist_roll_hardware_id": wrist_roll_id,
        "gripper_logical_key": GRIPPER_LOGICAL_KEY,
        "wrist_roll_logical_key": WRIST_ROLL_LOGICAL_KEY,
        "motor_mapping_verified": wrist_roll_id == 5 and gripper_id == 6,
        "gripper_calibration_range": [0.0, 100.0],
    }


def gripper_target_in_range(value: float | None) -> bool:
    return value is not None and math.isfinite(float(value)) and 0.0 <= float(value) <= 100.0


def validate_gripper_open_config(config: GraspConfig) -> tuple[bool, str]:
    if config.gripper_open_target_pos is None or not config.gripper_open_target_verified:
        return False, "gripper_open_target_not_configured_or_unverified"
    if not gripper_target_in_range(config.gripper_open_target_pos):
        return False, "gripper_target_out_of_calibration_range"
    return True, "ok"


def build_gripper_ramp_targets(
    initial_gripper_position: float,
    open_target: float | None,
    fractions: tuple[float, ...],
) -> list[float | None]:
    if open_target is None:
        return [None for _ in fractions]
    return [
        float(initial_gripper_position + fraction * (float(open_target) - initial_gripper_position))
        for fraction in fractions
    ]


def validate_gripper_ramp_targets(targets: list[float | None]) -> bool:
    return all(value is not None and gripper_target_in_range(value) for value in targets)


def make_descent_config(config: GraspConfig) -> DescentConfig:
    return DescentConfig(
        waypoint_drop_m=config.descent_waypoint_drop_m,
        start_pregrasp_joint_tolerance_rad=0.10,
        max_abs_joint_delta_per_waypoint_rad=config.max_abs_joint_delta_per_descent_waypoint_rad,
        position_tolerance_m=config.position_tolerance_m,
        approach_tolerance_deg=config.approach_tolerance_deg,
        max_xy_error_from_waypoint_m=config.max_xy_error_from_waypoint_m,
        minimum_actual_z_drop_per_waypoint_m=config.minimum_actual_z_drop_per_waypoint_m,
        minimum_total_actual_z_drop_m=config.minimum_total_actual_z_drop_m,
        final_joint_tolerance_rad=config.arm_final_joint_tolerance_rad,
        speed_rad_s=config.speed_rad_s,
        max_speed_rad_s=config.max_speed_rad_s,
        execute_service_timeout_s=config.execute_service_timeout_s,
        inter_waypoint_hold_s=config.inter_waypoint_hold_s,
        joint_state_max_age_s=config.joint_state_max_age_s,
        pregrasp_target_max_age_s=config.pregrasp_target_max_age_s,
    )


def build_integrated_plan_summary(
    *,
    mode: str,
    config: GraspConfig,
    motor_mapping: dict[str, Any],
    object_pose_base: list[float],
    pregrasp_pose_base: list[float],
    current_joint_positions_rad: list[float],
    pregrasp_joint_target_rad: list[float],
    initial_gripper_position: float,
    compute_message: str,
) -> dict[str, Any]:
    model = create_model()
    frozen = FrozenPregrasp(
        object_pose_base=object_pose_base,
        pregrasp_pose_base=pregrasp_pose_base,
        pregrasp_joint_target_rad=pregrasp_joint_target_rad,
        solution_type=None,
        selected_offset_m=None,
        position_error_m=None,
        approach_error_deg=None,
    )
    pregrasp_delta = joint_delta(current_joint_positions_rad, pregrasp_joint_target_rad)
    descent_config = make_descent_config(config)
    descent_plan = plan_segmented_descent(
        model=model,
        frozen=frozen,
        current_joint_positions_rad=pregrasp_joint_target_rad,
        config=descent_config,
    )
    ramp_targets = build_gripper_ramp_targets(
        initial_gripper_position,
        config.gripper_open_target_pos,
        config.gripper_open_ramp_fraction,
    )
    all_gripper_targets_valid = validate_gripper_ramp_targets(ramp_targets)
    open_valid, open_reason = validate_gripper_open_config(config)
    waypoints = []
    for index, waypoint in enumerate(descent_plan.waypoints):
        waypoints.append(
            {
                "index": waypoint.index,
                "requested_xyz_m": waypoint.requested_xyz_m,
                "actual_fk_xyz_m": waypoint.actual_fk_xyz_m,
                "arm_joint_target_rad": waypoint.selected_joint_target_rad,
                "arm_joint_delta_rad": waypoint.joint_delta_from_previous_rad,
                "maximum_abs_arm_joint_delta_rad": waypoint.maximum_abs_joint_delta_rad,
                "position_error_m": waypoint.position_error_m,
                "approach_error_deg": waypoint.approach_error_deg,
                "xy_error_m": waypoint.xy_error_m,
                "actual_z_drop_from_previous_m": waypoint.actual_z_drop_from_previous_m,
                "gripper_ramp_fraction": config.gripper_open_ramp_fraction[index],
                "gripper_target_position": ramp_targets[index],
            }
        )
    estimated_pregrasp = estimated_duration_s(
        current_joint_positions_rad,
        pregrasp_joint_target_rad,
        config.speed_rad_s,
    )
    estimated_descent = 0.0
    previous = list(pregrasp_joint_target_rad)
    for waypoint in descent_plan.waypoints:
        estimated_descent += estimated_duration_s(previous, waypoint.selected_joint_target_rad, config.speed_rad_s)
        previous = waypoint.selected_joint_target_rad
    reason = (
        "integrated_visual_grasp_plan_ready"
        if descent_plan.success and all_gripper_targets_valid and open_valid
        else (open_reason if not open_valid else descent_plan.reason)
    )
    return {
        "success": bool(descent_plan.success and all_gripper_targets_valid and open_valid),
        "reason": reason,
        "mode": mode,
        "object_pose_base": object_pose_base,
        "pregrasp_pose_base": pregrasp_pose_base,
        "current_joint_positions_rad": current_joint_positions_rad,
        "pregrasp_joint_target_rad": pregrasp_joint_target_rad,
        "pregrasp_max_abs_joint_delta_rad": float(pregrasp_delta["maximum_abs_joint_delta_rad"]),
        "pregrasp_limit_rad": config.pregrasp_max_abs_joint_delta_rad,
        "compute_response_message": compute_message,
        "initial_gripper_position": float(initial_gripper_position),
        "gripper_motor_logical_name": GRIPPER_LOGICAL_NAME,
        "gripper_motor_hardware_id": motor_mapping["gripper_hardware_id"],
        "wrist_roll_motor_hardware_id": motor_mapping["wrist_roll_hardware_id"],
        "gripper_open_target_position": config.gripper_open_target_pos,
        "gripper_open_target_verified": bool(config.gripper_open_target_verified),
        "gripper_close_target_source": config.gripper_close_target_source,
        "gripper_close_target_position": float(initial_gripper_position),
        "waypoint_count": len(waypoints),
        "descent_waypoints": waypoints,
        "total_requested_z_drop_m": config.total_descent_m,
        "total_actual_z_drop_m": descent_plan.total_actual_z_drop_m,
        "all_arm_waypoints_valid": bool(descent_plan.success),
        "all_gripper_targets_valid": bool(all_gripper_targets_valid),
        "estimated_pregrasp_duration_s": estimated_pregrasp,
        "estimated_descent_duration_s": estimated_descent,
        "estimated_gripper_close_duration_s": config.gripper_only_motion_duration_s,
        "estimated_total_duration_s": estimated_pregrasp + estimated_descent + config.gripper_only_motion_duration_s,
        "live_visual_used_before_motion": True,
        "live_visual_required_after_motion": False,
        "all_motion_planned_before_execute": True,
        "hardware_command_sent": False,
    }


def pose_to_list(msg: Any) -> list[float]:
    return [
        float(msg.pose.position.x),
        float(msg.pose.position.y),
        float(msg.pose.position.z),
    ]


def json_print(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False))


class VisualGraspNode:
    def __init__(self, config: GraspConfig) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, String
        from std_srvs.srv import Trigger

        class _Node(Node):
            pass

        self.rclpy = rclpy
        self.JointState = JointState
        self.Float64 = Float64
        self.Trigger = Trigger
        self.node = _Node("mvp_visual_grasp")
        self.config = config
        self.latest_joint_state = None
        self.latest_gripper_state: StampedGripperState | None = None
        self.latest_object_pose: PoseStamped | None = None
        self.latest_object_pose_time = 0.0
        self.latest_pregrasp_target = None
        self.latest_pregrasp_target_time = 0.0
        self.latest_pregrasp_pose: PoseStamped | None = None
        self.pregrasp_valid = False
        self.pregrasp_status = ""
        self.tcp_connected = False
        self.tcp_status = "unknown"
        self.node.create_subscription(JointState, "/mvp/joint_states", self._joint_state_cb, 10)
        self.node.create_subscription(Float64, "/mvp/gripper_state", self._gripper_state_cb, 10)
        self.node.create_subscription(PoseStamped, "/object_pose_base", self._object_pose_cb, 10)
        self.node.create_subscription(JointState, "/mvp/pregrasp_joint_target", self._pregrasp_target_cb, 10)
        self.node.create_subscription(PoseStamped, "/mvp/pregrasp_pose", self._pregrasp_pose_cb, 10)
        self.node.create_subscription(Bool, "/mvp/pregrasp_valid", self._pregrasp_valid_cb, 10)
        self.node.create_subscription(String, "/mvp/pregrasp_status", self._pregrasp_status_cb, 10)
        self.node.create_subscription(Bool, "/mvp/tcp_connected", self._tcp_connected_cb, 10)
        self.node.create_subscription(String, "/mvp/tcp_status", self._tcp_status_cb, 10)
        self.arm_target_pub = self.node.create_publisher(JointState, "/mvp/joint_target", 10)
        self.gripper_target_pub = self.node.create_publisher(Float64, "/mvp/gripper_target", 10)
        self.compute_client = self.node.create_client(Trigger, "/mvp/compute_pregrasp")
        self.execute_client = self.node.create_client(Trigger, "/mvp/execute_target")

    def _joint_state_cb(self, msg: Any) -> None:
        from mvp_descend_from_pregrasp import StampedJointState

        self.latest_joint_state = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )

    def _gripper_state_cb(self, msg: Any) -> None:
        self.latest_gripper_state = StampedGripperState(float(msg.data), time.monotonic())

    def _object_pose_cb(self, msg: Any) -> None:
        self.latest_object_pose = msg
        self.latest_object_pose_time = time.monotonic()

    def _pregrasp_target_cb(self, msg: Any) -> None:
        from mvp_descend_from_pregrasp import StampedJointState

        self.latest_pregrasp_target = StampedJointState(
            names=tuple(str(name) for name in msg.name),
            positions_rad=tuple(float(value) for value in msg.position),
            received_monotonic_s=time.monotonic(),
        )
        self.latest_pregrasp_target_time = self.latest_pregrasp_target.received_monotonic_s

    def _pregrasp_pose_cb(self, msg: Any) -> None:
        self.latest_pregrasp_pose = msg

    def _pregrasp_valid_cb(self, msg: Any) -> None:
        self.pregrasp_valid = bool(msg.data)

    def _pregrasp_status_cb(self, msg: Any) -> None:
        self.pregrasp_status = str(msg.data)

    def _tcp_connected_cb(self, msg: Any) -> None:
        self.tcp_connected = bool(msg.data)

    def _tcp_status_cb(self, msg: Any) -> None:
        self.tcp_status = str(msg.data)

    def spin_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call_trigger(self, client: Any, timeout_s: float) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=3.0):
            return False, "service_unavailable"
        future = client.call_async(self.Trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            return False, "execute_service_timeout" if timeout_s >= 100.0 else "service_timeout"
        response = future.result()
        return bool(response.success), str(response.message)

    def publish_arm_target_once(self, target_rad: list[float]) -> None:
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(value) for value in target_rad]
        self.arm_target_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def publish_gripper_target_once(self, target_pos: float) -> None:
        msg = self.Float64()
        msg.data = float(target_pos)
        self.gripper_target_pub.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.2)

    def destroy(self) -> None:
        self.node.destroy_node()


def run(args: argparse.Namespace) -> int:
    config = load_grasp_config()
    motor_mapping = load_motor_mapping()
    if not motor_mapping["motor_mapping_verified"]:
        json_print({"success": False, "reason": "gripper_motor_mapping_mismatch", **motor_mapping})
        return 2
    open_valid, open_reason = validate_gripper_open_config(config)
    if args.execute and not open_valid:
        json_print(
            {
                "success": False,
                "reason": open_reason,
                "mode": "execute",
                "gripper_open_target_position": config.gripper_open_target_pos,
                "gripper_open_target_verified": config.gripper_open_target_verified,
                "hardware_command_sent": False,
            }
        )
        return 2

    import rclpy

    model = create_model()
    rclpy.init()
    node = VisualGraspNode(config)
    try:
        if not node.spin_until(
            lambda: validate_fresh_joint_state(
                node.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=config.joint_state_max_age_s,
            )[0],
            10.0,
        ):
            json_print({"success": False, "reason": "joint_state_unavailable_or_stale"})
            return 3
        if not node.spin_until(
            lambda: node.latest_gripper_state is not None
            and time.monotonic() - node.latest_gripper_state.received_monotonic_s <= config.gripper_state_max_age_s,
            10.0,
        ):
            json_print({"success": False, "reason": "gripper_state_unavailable_or_stale"})
            return 4
        if not node.tcp_connected or node.tcp_status != "connected":
            json_print({"success": False, "reason": "tcp_not_connected"})
            return 5
        if not node.spin_until(
            lambda: node.latest_object_pose is not None
            and time.monotonic() - node.latest_object_pose_time <= config.object_pose_max_age_s,
            config.object_pose_max_age_s,
        ):
            json_print({"success": False, "reason": "object_pose_unavailable_or_stale"})
            return 6

        assert node.latest_joint_state is not None
        assert node.latest_gripper_state is not None
        assert node.latest_object_pose is not None
        current = [float(value) for value in node.latest_joint_state.positions_rad]
        initial_gripper = float(node.latest_gripper_state.position)
        compute_started = time.monotonic()
        compute_success, compute_message = node.call_trigger(node.compute_client, 10.0)
        if not compute_success:
            json_print({"success": False, "reason": "compute_pregrasp_failed", "message": compute_message})
            return 7
        if not node.spin_until(
            lambda: node.latest_pregrasp_target is not None
            and node.latest_pregrasp_target_time >= compute_started
            and node.latest_pregrasp_pose is not None
            and time.monotonic() - node.latest_pregrasp_target.received_monotonic_s <= config.pregrasp_target_max_age_s,
            config.pregrasp_target_max_age_s,
        ):
            json_print({"success": False, "reason": "pregrasp_target_unavailable_or_stale"})
            return 8
        assert node.latest_pregrasp_target is not None
        assert node.latest_pregrasp_pose is not None
        valid_target, target_reason = validate_joint_contract(node.latest_pregrasp_target.names, node.latest_pregrasp_target.positions_rad)
        if not valid_target:
            json_print({"success": False, "reason": target_reason})
            return 9
        if not node.pregrasp_valid or not status_is_accepted(node.pregrasp_status, compute_message):
            json_print({"success": False, "reason": "pregrasp_not_ready", "status": node.pregrasp_status})
            return 10
        frozen_target = [float(value) for value in node.latest_pregrasp_target.positions_rad]
        if not target_within_urdf_limits(model, frozen_target):
            json_print({"success": False, "reason": "joint_limit_failed"})
            return 11
        pregrasp_delta = joint_delta(current, frozen_target)
        if float(pregrasp_delta["maximum_abs_joint_delta_rad"]) > config.pregrasp_max_abs_joint_delta_rad:
            json_print({"success": False, "reason": "pregrasp_joint_delta_exceeded"})
            return 12

        object_pose_base = pose_to_list(node.latest_object_pose)
        pregrasp_pose_base = pose_to_list(node.latest_pregrasp_pose)
        created_at_unix_s = time.time()
        planned_snapshot = make_pregrasp_snapshot(
            snapshot_state="planned",
            created_at_unix_s=created_at_unix_s,
            object_pose_base=object_pose_base,
            pregrasp_pose_base=pregrasp_pose_base,
            frozen_target_rad=frozen_target,
            compute_message=compute_message,
            compute_pregrasp_success=True,
            pregrasp_valid=node.pregrasp_valid,
            pregrasp_status=node.pregrasp_status,
            hardware_command_sent=False,
            execute_response_message=None,
            motion_completed=None,
            final_joint_positions_rad=None,
            final_errors=None,
            config=MoveConfig(),
            tcp_connected_after_motion=None,
            tcp_status_after_motion=None,
        )
        atomic_write_json(SNAPSHOT_PATH, planned_snapshot)

        summary = build_integrated_plan_summary(
            mode="plan_only" if args.plan_only or not args.execute else "execute",
            config=config,
            motor_mapping=motor_mapping,
            object_pose_base=object_pose_base,
            pregrasp_pose_base=pregrasp_pose_base,
            current_joint_positions_rad=current,
            pregrasp_joint_target_rad=frozen_target,
            initial_gripper_position=initial_gripper,
            compute_message=compute_message,
        )
        atomic_write_json(
            INTEGRATED_SNAPSHOT_PATH,
            {
                "schema_version": 1,
                "stage": "MVP-4D-INTEGRATED-VISUAL-GRASP",
                "created_at_unix_s": created_at_unix_s,
                "updated_at_unix_s": time.time(),
                "snapshot_state": "planned",
                **summary,
            },
        )
        if args.plan_only or not args.execute:
            json_print(summary)
            return 0 if summary["success"] else 13

        if args.confirm != CONFIRM_PHRASE:
            summary.update({"success": False, "reason": "wrong_confirmation", "required_confirm": CONFIRM_PHRASE})
            json_print(summary)
            return 2
        if not summary["success"]:
            json_print(summary)
            return 13

        execute_count = 0
        arm_publish_count = 0
        gripper_publish_count = 0
        node.publish_arm_target_once(frozen_target)
        arm_publish_count += 1
        pregrasp_success, pregrasp_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
        execute_count += 1
        if not pregrasp_success:
            summary.update({"success": False, "reason": pregrasp_message, "hardware_command_sent": True})
            json_print(summary)
            return 14
        for waypoint in summary["descent_waypoints"]:
            node.publish_arm_target_once(waypoint["arm_joint_target_rad"])
            arm_publish_count += 1
            node.publish_gripper_target_once(float(waypoint["gripper_target_position"]))
            gripper_publish_count += 1
            waypoint_success, waypoint_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
            execute_count += 1
            if not waypoint_success:
                summary.update({"success": False, "reason": waypoint_message, "hardware_command_sent": True})
                json_print(summary)
                return 15
            time.sleep(config.inter_waypoint_hold_s)
        if not node.spin_until(
            lambda: validate_fresh_joint_state(
                node.latest_joint_state,
                now_monotonic_s=time.monotonic(),
                max_age_s=config.joint_state_max_age_s,
            )[0],
            3.0,
        ):
            summary.update({"success": False, "reason": "final_arm_state_unavailable", "hardware_command_sent": True})
            json_print(summary)
            return 16
        assert node.latest_joint_state is not None
        hold_arm = [float(value) for value in node.latest_joint_state.positions_rad]
        gripper_before_close = None if node.latest_gripper_state is None else float(node.latest_gripper_state.position)
        node.publish_arm_target_once(hold_arm)
        arm_publish_count += 1
        node.publish_gripper_target_once(initial_gripper)
        gripper_publish_count += 1
        close_success, close_message = node.call_trigger(node.execute_client, config.execute_service_timeout_s)
        execute_count += 1
        time.sleep(config.gripper_close_hold_s)
        gripper_final = None if node.latest_gripper_state is None else float(node.latest_gripper_state.position)
        gripper_error = None if gripper_final is None else abs(gripper_final - initial_gripper)
        summary.update(
            {
                "success": bool(close_success),
                "reason": "grasp_close_attempt_completed" if close_success else "grasp_close_attempt_failed",
                "close_execute_response_message": close_message,
                "hardware_command_sent": True,
                "arm_target_publish_count": arm_publish_count,
                "gripper_target_publish_count": gripper_publish_count,
                "execute_call_count": execute_count,
                "gripper_initial_position": initial_gripper,
                "gripper_open_target_position": config.gripper_open_target_pos,
                "gripper_position_before_close": gripper_before_close,
                "gripper_close_target_position": initial_gripper,
                "gripper_final_position": gripper_final,
                "gripper_final_error": gripper_error,
                "gripper_close_command_completed": bool(close_success),
                "gripper_close_target_reached": None if gripper_error is None else gripper_error <= 1.0,
                "possible_object_blocking_gripper": None if gripper_error is None else gripper_error > 1.0,
                "object_may_be_grasped": None,
            }
        )
        json_print(summary)
        return 0 if close_success else 17
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Integrated SO-101 visual pregrasp, 7 cm descent, and gripper close.")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.execute and args.plan_only:
        print("--plan-only and --execute are mutually exclusive", file=sys.stderr)
        return 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
