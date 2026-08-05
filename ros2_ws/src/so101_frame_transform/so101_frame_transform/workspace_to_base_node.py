from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


DEFAULT_PROJECT_ROOT = Path(
    r"E:\PycharmProjects\Embodied_AI"
    r"\LeRobot_Project"
    r"\so101_visual_tactile_grasp"
)


def finite_values(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion

    norm = math.sqrt(
        x * x + y * y + z * z + w * w
    )

    if norm <= 1.0e-12:
        raise ValueError("Quaternion has zero norm")

    return (
        x / norm,
        y / norm,
        z / norm,
        w / norm,
    )


def quaternion_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second

    return normalize_quaternion(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def yaw_quaternion(
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    return (
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


class WorkspaceToBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("workspace_to_base_node")

        self.declare_parameter(
            "project_root",
            str(DEFAULT_PROJECT_ROOT),
        )

        project_root = Path(
            str(
                self.get_parameter(
                    "project_root"
                ).value
            )
        )

        config_path = (
            project_root
            / "config"
            / "workspace_to_base.json"
        )

        if not config_path.is_file():
            raise FileNotFoundError(
                f"Config not found: {config_path}"
            )

        config = json.loads(
            config_path.read_text(
                encoding="utf-8"
            )
        )

        self.source_frame = str(
            config["source_frame"]
        )

        self.target_frame = str(
            config["target_frame"]
        )

        translation = [
            float(value)
            for value in config[
                "translation_m"
            ]
        ]

        if (
            len(translation) != 3
            or not finite_values(translation)
        ):
            raise ValueError(
                "translation_m must contain "
                "three finite values"
            )

        self.tx, self.ty, self.tz = translation

        self.yaw_rad = math.radians(
            float(config["yaw_deg"])
        )

        self.stale_timeout_s = float(
            config.get(
                "stale_timeout_s",
                0.5,
            )
        )

        self.rotation_quaternion = (
            yaw_quaternion(self.yaw_rad)
        )

        self.stable = False
        self.detected = False
        self.transform_valid_heartbeat_seq = 0
        self.last_object_pose_payload_time: float | None = None
        self.last_object_detected_heartbeat_time: float | None = None
        self.last_object_pose_stable_heartbeat_time: float | None = None
        self.last_valid_time: float | None = None

        self.pose_publisher = (
            self.create_publisher(
                PoseStamped,
                "/object_pose_base",
                10,
            )
        )

        self.valid_publisher = (
            self.create_publisher(
                Bool,
                "/object_pose_base_valid",
                10,
            )
        )
        self.status_publisher = (
            self.create_publisher(
                String,
                "/object_pose_base_status",
                10,
            )
        )

        self.create_subscription(
            PoseStamped,
            "/object_pose",
            self.handle_pose,
            10,
        )

        self.create_subscription(
            Bool,
            "/object_detected",
            self.handle_detected,
            10,
        )

        self.create_subscription(
            Bool,
            "/object_pose_stable",
            self.handle_stable,
            10,
        )

        self.create_timer(
            0.1,
            self.publish_validity,
        )

        self.get_logger().info(
            "Workspace-to-base transform started | "
            f"{self.source_frame} -> "
            f"{self.target_frame} | "
            f"translation=({self.tx:.3f}, "
            f"{self.ty:.3f}, {self.tz:.3f}) m | "
            f"yaw={math.degrees(self.yaw_rad):.3f} deg | "
            f"status={config.get('calibration_status')}"
        )

    def handle_stable(
        self,
        message: Bool,
    ) -> None:
        self.stable = bool(message.data)
        self.last_object_pose_stable_heartbeat_time = time.monotonic()

        if not self.stable:
            self.last_valid_time = None

    def handle_detected(
        self,
        message: Bool,
    ) -> None:
        self.detected = bool(message.data)
        self.last_object_detected_heartbeat_time = time.monotonic()

        if not self.detected:
            self.last_valid_time = None

    def handle_pose(
        self,
        message: PoseStamped,
    ) -> None:
        self.last_object_pose_payload_time = time.monotonic()

        if not self.detected or not self.stable:
            return

        if (
            message.header.frame_id
            != self.source_frame
        ):
            self.get_logger().warning(
                "Rejected pose with frame_id="
                f"{message.header.frame_id}"
            )
            return

        position = message.pose.position
        orientation = message.pose.orientation

        values = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]

        if not finite_values(values):
            self.get_logger().warning(
                "Rejected non-finite pose"
            )
            return

        cosine = math.cos(self.yaw_rad)
        sine = math.sin(self.yaw_rad)

        output = PoseStamped()
        output.header = message.header
        output.header.frame_id = self.target_frame

        output.pose.position.x = (
            cosine * position.x
            - sine * position.y
            + self.tx
        )

        output.pose.position.y = (
            sine * position.x
            + cosine * position.y
            + self.ty
        )

        output.pose.position.z = (
            position.z + self.tz
        )

        input_quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        transformed_quaternion = (
            quaternion_multiply(
                self.rotation_quaternion,
                input_quaternion,
            )
        )

        (
            output.pose.orientation.x,
            output.pose.orientation.y,
            output.pose.orientation.z,
            output.pose.orientation.w,
        ) = transformed_quaternion

        self.pose_publisher.publish(output)
        self.last_valid_time = time.monotonic()

    def age_s(self, timestamp: float | None) -> float | None:
        if timestamp is None:
            return None
        return time.monotonic() - timestamp

    def input_fresh(self, timestamp: float | None) -> bool:
        age = self.age_s(timestamp)
        return bool(age is not None and age <= self.stale_timeout_s)

    def publish_validity(self) -> None:
        self.transform_valid_heartbeat_seq += 1
        valid = bool(
            self.detected
            and
            self.stable
            and self.input_fresh(self.last_object_pose_payload_time)
            and self.input_fresh(self.last_object_detected_heartbeat_time)
            and self.input_fresh(self.last_object_pose_stable_heartbeat_time)
            and self.last_valid_time is not None
        )

        message = Bool()
        message.data = valid

        self.valid_publisher.publish(message)

        now_msg = self.get_clock().now().to_msg()
        status_message = String()
        status_message.data = json.dumps(
            {
                "status": "VALID" if valid else "INVALID",
                "reason": (
                    "workspace_to_base_transform_valid"
                    if valid
                    else "workspace_to_base_input_stale_or_invalid"
                ),
                "last_object_pose_payload_age_s": self.age_s(
                    self.last_object_pose_payload_time
                ),
                "last_object_detected_heartbeat_age_s": self.age_s(
                    self.last_object_detected_heartbeat_time
                ),
                "last_object_pose_stable_heartbeat_age_s": self.age_s(
                    self.last_object_pose_stable_heartbeat_time
                ),
                "last_transform_output_age_s": self.age_s(self.last_valid_time),
                "transform_valid_heartbeat_seq": self.transform_valid_heartbeat_seq,
                "detected": self.detected,
                "stable": self.stable,
                "source_frame": self.source_frame,
                "target_frame": self.target_frame,
                "timestamp": now_msg.sec + now_msg.nanosec * 1.0e-9,
            },
            ensure_ascii=False,
            allow_nan=False,
        )
        self.status_publisher.publish(status_message)


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node = WorkspaceToBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
