from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .urdf_fk import (
    ARM_JOINT_NAMES,
    UrdfForwardKinematics,
    rotation_to_quaternion_xyzw,
)


DEFAULT_PROJECT_ROOT = (
    "E:/PycharmProjects/Embodied_AI/"
    "LeRobot_Project/"
    "so101_visual_tactile_grasp"
)


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(
            f"JSON file not found: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as file:
        value = json.load(file)

    if not isinstance(value, dict):
        raise ValueError(
            f"Expected JSON object: {path}"
        )

    return value


class ForwardKinematicsNode(Node):
    def __init__(self) -> None:
        super().__init__("so101_fk_node")

        self.declare_parameter(
            "project_root",
            DEFAULT_PROJECT_ROOT,
        )

        project_root = Path(
            str(
                self.get_parameter(
                    "project_root"
                ).value
            )
        ).resolve()

        config_path = (
            project_root
            / "config"
            / "kinematics.json"
        )

        urdf_path = (
            project_root
            / "data"
            / "robot_model"
            / "so101"
            / "so101_new_calib.urdf"
        )

        config = load_json(config_path)

        if (
            config.get("joint_position_unit")
            != "radian"
        ):
            raise ValueError(
                "joint_position_unit must be radian"
            )

        self.base_link = str(
            config["base_link"]
        )

        self.tip_link = str(
            config["tip_link"]
        )

        self.joint_state_topic = str(
            config["joint_state_topic"]
        )

        self.pose_topic = str(
            config[
                "end_effector_pose_topic"
            ]
        )

        self.valid_topic = str(
            config["fk_valid_topic"]
        )

        self.stale_timeout_s = float(
            config["stale_timeout_s"]
        )

        status_rate_hz = float(
            config[
                "status_publish_rate_hz"
            ]
        )

        if self.stale_timeout_s <= 0:
            raise ValueError(
                "stale_timeout_s must be positive"
            )

        if status_rate_hz <= 0:
            raise ValueError(
                "status_publish_rate_hz "
                "must be positive"
            )

        aliases_value = config[
            "joint_aliases"
        ]

        if not isinstance(
            aliases_value,
            dict,
        ):
            raise ValueError(
                "joint_aliases must be an object"
            )

        self.joint_aliases: dict[
            str,
            list[str],
        ] = {}

        for canonical_name in ARM_JOINT_NAMES:
            aliases = aliases_value.get(
                canonical_name
            )

            if (
                not isinstance(aliases, list)
                or not aliases
            ):
                raise ValueError(
                    "Missing aliases for "
                    f"{canonical_name}"
                )

            parsed_aliases = [
                str(alias)
                for alias in aliases
            ]

            if canonical_name not in parsed_aliases:
                parsed_aliases.insert(
                    0,
                    canonical_name,
                )

            self.joint_aliases[
                canonical_name
            ] = parsed_aliases

        self.solver = (
            UrdfForwardKinematics(
                urdf_path=urdf_path,
                base_link=self.base_link,
                tip_link=self.tip_link,
            )
        )

        self.solver.validate_expected_chain()

        self.pose_publisher = (
            self.create_publisher(
                PoseStamped,
                self.pose_topic,
                10,
            )
        )

        self.valid_publisher = (
            self.create_publisher(
                Bool,
                self.valid_topic,
                10,
            )
        )

        self.joint_subscription = (
            self.create_subscription(
                JointState,
                self.joint_state_topic,
                self.handle_joint_state,
                20,
            )
        )

        self.status_timer = self.create_timer(
            1.0 / status_rate_hz,
            self.publish_valid_status,
        )

        self.current_valid = False
        self.last_valid_time = 0.0
        self.last_log_time = 0.0
        self.last_warning_time = 0.0

        self.resolved_source_names: (
            dict[str, str] | None
        ) = None

        self.get_logger().info(
            "SO-101 FK node started"
        )

        self.get_logger().info(
            f"URDF: {urdf_path}"
        )

        self.get_logger().info(
            f"Chain: {self.base_link} "
            f"-> {self.tip_link}"
        )

        self.get_logger().info(
            f"Input: {self.joint_state_topic}"
        )

        self.get_logger().info(
            f"Output: {self.pose_topic}"
        )

        self.get_logger().info(
            "No robot commands are produced "
            "by this node."
        )

    def warn_throttled(
        self,
        message: str,
        period_s: float = 2.0,
    ) -> None:
        now = time.monotonic()

        if (
            now - self.last_warning_time
            < period_s
        ):
            return

        self.get_logger().warning(message)
        self.last_warning_time = now

    def reject_message(
        self,
        reason: str,
    ) -> None:
        self.current_valid = False

        self.warn_throttled(
            f"Rejected /joint_states: {reason}"
        )

    def resolve_joint_sources(
        self,
        available_names: list[str],
    ) -> dict[str, str]:
        available_set = set(
            available_names
        )

        resolved: dict[str, str] = {}

        for canonical_name in ARM_JOINT_NAMES:
            aliases = self.joint_aliases[
                canonical_name
            ]

            source_name = next(
                (
                    alias
                    for alias in aliases
                    if alias in available_set
                ),
                None,
            )

            if source_name is None:
                raise ValueError(
                    "missing joint "
                    f"{canonical_name}; "
                    f"accepted aliases={aliases}"
                )

            resolved[
                canonical_name
            ] = source_name

        return resolved

    def handle_joint_state(
        self,
        message: JointState,
    ) -> None:
        if not message.name:
            self.reject_message(
                "name array is empty"
            )
            return

        if len(message.name) != len(
            message.position
        ):
            self.reject_message(
                "name and position lengths "
                f"differ: {len(message.name)} "
                f"vs {len(message.position)}"
            )
            return

        if len(set(message.name)) != len(
            message.name
        ):
            self.reject_message(
                "duplicate joint names"
            )
            return

        positions = np.asarray(
            message.position,
            dtype=np.float64,
        )

        if not np.all(
            np.isfinite(positions)
        ):
            self.reject_message(
                "position contains NaN or Inf"
            )
            return

        try:
            resolved = (
                self.resolve_joint_sources(
                    list(message.name)
                )
            )

        except ValueError as error:
            self.reject_message(str(error))
            return

        if (
            self.resolved_source_names
            != resolved
        ):
            self.resolved_source_names = (
                resolved
            )

            self.get_logger().info(
                "Resolved joint mapping: "
                f"{resolved}"
            )

        name_to_index = {
            name: index
            for index, name
            in enumerate(message.name)
        }

        joint_positions: dict[
            str,
            float,
        ] = {}

        for canonical_name in ARM_JOINT_NAMES:
            source_name = resolved[
                canonical_name
            ]

            source_index = name_to_index[
                source_name
            ]

            joint_positions[
                canonical_name
            ] = float(
                message.position[
                    source_index
                ]
            )

        try:
            transform = self.solver.compute(
                joint_positions
            )

        except ValueError as error:
            self.reject_message(
                f"FK failed: {error}"
            )
            return

        if not np.all(
            np.isfinite(transform)
        ):
            self.reject_message(
                "FK result contains NaN or Inf"
            )
            return

        rotation = transform[:3, :3]
        position = transform[:3, 3]

        quaternion = (
            rotation_to_quaternion_xyzw(
                rotation
            )
        )

        if not np.all(
            np.isfinite(quaternion)
        ):
            self.reject_message(
                "quaternion contains NaN or Inf"
            )
            return

        pose = PoseStamped()

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.header.frame_id = (
            self.base_link
        )

        pose.pose.position.x = float(
            position[0]
        )

        pose.pose.position.y = float(
            position[1]
        )

        pose.pose.position.z = float(
            position[2]
        )

        pose.pose.orientation.x = float(
            quaternion[0]
        )

        pose.pose.orientation.y = float(
            quaternion[1]
        )

        pose.pose.orientation.z = float(
            quaternion[2]
        )

        pose.pose.orientation.w = float(
            quaternion[3]
        )

        self.pose_publisher.publish(pose)

        self.current_valid = True
        self.last_valid_time = (
            time.monotonic()
        )

        now = time.monotonic()

        if (
            now - self.last_log_time
            >= 1.0
        ):
            joint_degrees = {
                name: math.degrees(value)
                for name, value
                in joint_positions.items()
            }

            self.get_logger().info(
                "FK valid | "
                f"position_m=("
                f"{position[0]:.4f}, "
                f"{position[1]:.4f}, "
                f"{position[2]:.4f}) | "
                f"q_deg={joint_degrees}"
            )

            self.last_log_time = now

    def publish_valid_status(
        self,
    ) -> None:
        now = time.monotonic()

        if (
            self.current_valid
            and now - self.last_valid_time
            > self.stale_timeout_s
        ):
            self.current_valid = False

            self.warn_throttled(
                "Joint state became stale; "
                "FK output invalid",
                period_s=1.0,
            )

        message = Bool()
        message.data = bool(
            self.current_valid
        )

        self.valid_publisher.publish(
            message
        )


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node: (
        ForwardKinematicsNode | None
    ) = None

    try:
        node = ForwardKinematicsNode()
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