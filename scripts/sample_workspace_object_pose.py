from __future__ import annotations

import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


SAMPLE_COUNT = 30
TIMEOUT_S = 20.0
EXPECTED_FRAME = "workspace_plane"


class WorkspacePoseSampler(Node):
    def __init__(self) -> None:
        super().__init__(
            "workspace_pose_sampler"
        )

        self.stable = False
        self.samples: list[
            tuple[float, float, float]
        ] = []

        self.create_subscription(
            Bool,
            "/object_pose_stable",
            self.handle_stable,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/object_pose",
            self.handle_pose,
            20,
        )

    def handle_stable(
        self,
        message: Bool,
    ) -> None:
        self.stable = bool(message.data)

    def handle_pose(
        self,
        message: PoseStamped,
    ) -> None:
        if not self.stable:
            return

        if (
            message.header.frame_id
            != EXPECTED_FRAME
        ):
            return

        values = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )

        if not all(
            math.isfinite(value)
            for value in values
        ):
            return

        if len(self.samples) < SAMPLE_COUNT:
            self.samples.append(values)


def main() -> int:
    rclpy.init()

    node = WorkspacePoseSampler()

    start_time = time.monotonic()

    try:
        while (
            rclpy.ok()
            and len(node.samples)
            < SAMPLE_COUNT
            and time.monotonic()
            - start_time
            < TIMEOUT_S
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    if len(node.samples) < SAMPLE_COUNT:
        print(
            json.dumps(
                {
                    "status": "FAIL",
                    "reason": (
                        "Not enough stable "
                        "object-pose samples"
                    ),
                    "sample_count": len(
                        node.samples
                    ),
                    "required": SAMPLE_COUNT,
                },
                indent=2,
            )
        )

        return 2

    xs = [
        value[0]
        for value in node.samples
    ]

    ys = [
        value[1]
        for value in node.samples
    ]

    zs = [
        value[2]
        for value in node.samples
    ]

    result = {
        "status": "PASS",
        "sample_count": len(
            node.samples
        ),
        "frame_id": EXPECTED_FRAME,
        "workspace_x_m": statistics.mean(
            xs
        ),
        "workspace_y_m": statistics.mean(
            ys
        ),
        "workspace_z_m": statistics.mean(
            zs
        ),
        "std_x_mm": statistics.pstdev(
            xs
        )
        * 1000.0,
        "std_y_mm": statistics.pstdev(
            ys
        )
        * 1000.0,
        "std_z_mm": statistics.pstdev(
            zs
        )
        * 1000.0,
    }

    print(
        json.dumps(
            result,
            indent=2,
        )
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())