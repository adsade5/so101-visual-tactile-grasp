from __future__ import annotations

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


class TfFkConsistencyChecker(Node):
    def __init__(self) -> None:
        super().__init__(
            "tf_fk_consistency_checker"
        )

        self.declare_parameter(
            "base_link",
            "base_link",
        )

        self.declare_parameter(
            "tip_link",
            "gripper_frame_link",
        )

        self.declare_parameter(
            "pose_topic",
            "/end_effector_pose",
        )

        self.declare_parameter(
            "maximum_position_error_mm",
            0.5,
        )

        self.declare_parameter(
            "maximum_orientation_error_deg",
            0.2,
        )

        self.base_link = str(
            self.get_parameter(
                "base_link"
            ).value
        )

        self.tip_link = str(
            self.get_parameter(
                "tip_link"
            ).value
        )

        pose_topic = str(
            self.get_parameter(
                "pose_topic"
            ).value
        )

        self.maximum_position_error_mm = (
            float(
                self.get_parameter(
                    "maximum_position_error_mm"
                ).value
            )
        )

        self.maximum_orientation_error_deg = (
            float(
                self.get_parameter(
                    "maximum_orientation_error_deg"
                ).value
            )
        )

        self.tf_buffer = Buffer(
            cache_time=Duration(
                seconds=5.0
            )
        )

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.pose_subscription = (
            self.create_subscription(
                PoseStamped,
                pose_topic,
                self.handle_pose,
                20,
            )
        )

        self.result_publisher = (
            self.create_publisher(
                Bool,
                "/fk_tf_consistent",
                10,
            )
        )

        self.latest_pose: (
            PoseStamped | None
        ) = None

        self.latest_pose_time = 0.0

        self.position_errors_mm: deque[
            float
        ] = deque(maxlen=50)

        self.orientation_errors_deg: deque[
            float
        ] = deque(maxlen=50)

        self.last_log_time = 0.0

        self.timer = self.create_timer(
            0.1,
            self.check_consistency,
        )

        self.get_logger().info(
            "TF/FK consistency checker started"
        )

        self.get_logger().info(
            f"Comparing {pose_topic} with "
            f"TF {self.base_link} -> {self.tip_link}"
        )

        self.get_logger().info(
            "Strict checking should first be "
            "performed with static zero joint angles."
        )

    def handle_pose(
        self,
        message: PoseStamped,
    ) -> None:
        if (
            message.header.frame_id
            != self.base_link
        ):
            self.get_logger().warning(
                "Pose frame mismatch: "
                f"{message.header.frame_id}"
            )

            return

        self.latest_pose = message
        self.latest_pose_time = (
            time.monotonic()
        )

    @staticmethod
    def normalized_quaternion(
        values: np.ndarray,
    ) -> np.ndarray:
        norm = float(
            np.linalg.norm(values)
        )

        if norm <= 1e-12:
            raise ValueError(
                "Quaternion has zero norm"
            )

        return values / norm

    @staticmethod
    def quaternion_error_degrees(
        first: np.ndarray,
        second: np.ndarray,
    ) -> float:
        first_normalized = (
            TfFkConsistencyChecker
            .normalized_quaternion(first)
        )

        second_normalized = (
            TfFkConsistencyChecker
            .normalized_quaternion(second)
        )

        dot_product = float(
            np.dot(
                first_normalized,
                second_normalized,
            )
        )

        # q和-q代表相同旋转。
        dot_product = abs(dot_product)

        dot_product = float(
            np.clip(
                dot_product,
                -1.0,
                1.0,
            )
        )

        return math.degrees(
            2.0 * math.acos(dot_product)
        )

    def publish_result(
        self,
        value: bool,
    ) -> None:
        message = Bool()
        message.data = bool(value)

        self.result_publisher.publish(
            message
        )

    def check_consistency(self) -> None:
        if self.latest_pose is None:
            self.publish_result(False)
            return

        if (
            time.monotonic()
            - self.latest_pose_time
            > 0.5
        ):
            self.publish_result(False)
            return

        try:
            transform = (
                self.tf_buffer.lookup_transform(
                    self.base_link,
                    self.tip_link,
                    Time(),
                    timeout=Duration(
                        seconds=0.05
                    ),
                )
            )

        except TransformException:
            self.publish_result(False)
            return

        pose = self.latest_pose.pose

        pose_position = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ],
            dtype=np.float64,
        )

        tf_position = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )

        pose_quaternion = np.asarray(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        )

        tf_quaternion = np.asarray(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=np.float64,
        )

        if not np.all(
            np.isfinite(pose_position)
        ):
            self.publish_result(False)
            return

        if not np.all(
            np.isfinite(tf_position)
        ):
            self.publish_result(False)
            return

        position_error_mm = float(
            np.linalg.norm(
                pose_position - tf_position
            )
            * 1000.0
        )

        try:
            orientation_error_deg = (
                self.quaternion_error_degrees(
                    pose_quaternion,
                    tf_quaternion,
                )
            )

        except ValueError:
            self.publish_result(False)
            return

        self.position_errors_mm.append(
            position_error_mm
        )

        self.orientation_errors_deg.append(
            orientation_error_deg
        )

        enough_samples = (
            len(self.position_errors_mm) >= 10
        )

        maximum_position_error = max(
            self.position_errors_mm
        )

        maximum_orientation_error = max(
            self.orientation_errors_deg
        )

        consistent = (
            enough_samples
            and maximum_position_error
            <= self.maximum_position_error_mm
            and maximum_orientation_error
            <= self.maximum_orientation_error_deg
        )

        self.publish_result(consistent)

        now = time.monotonic()

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                "TF/FK comparison | "
                f"current_position_error="
                f"{position_error_mm:.6f} mm | "
                f"current_orientation_error="
                f"{orientation_error_deg:.6f} deg | "
                f"window_max_position="
                f"{maximum_position_error:.6f} mm | "
                f"window_max_orientation="
                f"{maximum_orientation_error:.6f} deg | "
                f"consistent={consistent}"
            )

            self.last_log_time = now


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node = TfFkConsistencyChecker()

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