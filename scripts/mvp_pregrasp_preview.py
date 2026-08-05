from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
for package_path in (
    ROS_SRC / "so101_mvp_control",
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_control.pregrasp_planner import ARM_JOINT_NAMES, create_model, validate_fk_result


class PregraspPreviewClient(Node):
    def __init__(self, timeout_s: float) -> None:
        super().__init__("mvp_pregrasp_preview_client")
        self.timeout_s = float(timeout_s)
        self.object_pose: PoseStamped | None = None
        self.pregrasp_pose: PoseStamped | None = None
        self.joint_target: JointState | None = None
        self.create_subscription(
            PoseStamped,
            "/object_pose_base",
            self.handle_object_pose,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/mvp/pregrasp_pose",
            self.handle_pregrasp_pose,
            10,
        )
        self.create_subscription(
            JointState,
            "/mvp/pregrasp_joint_target",
            self.handle_joint_target,
            10,
        )
        self.compute_client = self.create_client(Trigger, "/mvp/compute_pregrasp")

    def handle_object_pose(self, message: PoseStamped) -> None:
        self.object_pose = message

    def handle_pregrasp_pose(self, message: PoseStamped) -> None:
        self.pregrasp_pose = message

    def handle_joint_target(self, message: JointState) -> None:
        self.joint_target = message

    def spin_until(self, predicate, label: str) -> bool:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        print(f"TIMEOUT waiting_for={label}")
        return False

    def call_compute(self) -> bool:
        if not self.compute_client.wait_for_service(timeout_sec=self.timeout_s):
            print("TIMEOUT waiting_for=/mvp/compute_pregrasp")
            return False
        future = self.compute_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                print(f"compute_pregrasp_success={response.success}")
                print(f"compute_pregrasp_message={response.message}")
                return bool(response.success)
        print("TIMEOUT waiting_for=compute_pregrasp_response")
        return False


def pose_xyz(message: PoseStamped) -> np.ndarray:
    return np.asarray(
        [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ],
        dtype=np.float64,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = PregraspPreviewClient(timeout_s=args.timeout_s)
    try:
        if not node.spin_until(lambda: node.object_pose is not None, "/object_pose_base"):
            return 1
        if not node.call_compute():
            return 1
        if not node.spin_until(
            lambda: node.pregrasp_pose is not None,
            "/mvp/pregrasp_pose",
        ):
            return 1
        if not node.spin_until(
            lambda: node.joint_target is not None,
            "/mvp/pregrasp_joint_target",
        ):
            return 1

        assert node.object_pose is not None
        assert node.pregrasp_pose is not None
        assert node.joint_target is not None

        if tuple(node.joint_target.name) != ARM_JOINT_NAMES:
            print(f"BAD_JOINT_ORDER names={list(node.joint_target.name)}")
            return 1
        if len(node.joint_target.position) != 5:
            print(f"BAD_JOINT_COUNT count={len(node.joint_target.position)}")
            return 1
        if not all(math.isfinite(float(value)) for value in node.joint_target.position):
            print("BAD_JOINT_VALUE non_finite")
            return 1

        object_position = pose_xyz(node.object_pose)
        pregrasp_position = pose_xyz(node.pregrasp_pose)
        q_rad = np.asarray([float(value) for value in node.joint_target.position], dtype=np.float64)
        model = create_model(PROJECT_ROOT)
        fk_ok, position_error_m, approach_error_deg, _ = validate_fk_result(
            model,
            q_rad,
            pregrasp_position,
            position_tolerance_m=0.002,
            approach_tolerance_deg=5.0,
        )

        print(f"object_pose_base_m={object_position.tolist()}")
        print(f"pregrasp_pose_base_m={pregrasp_position.tolist()}")
        print(f"pregrasp_delta_z_m={float(pregrasp_position[2] - object_position[2]):.6f}")
        print(f"pregrasp_joint_names={list(node.joint_target.name)}")
        print(f"pregrasp_joint_positions_rad={q_rad.tolist()}")
        print(f"ik_fk_position_error_m={position_error_m:.6f}")
        print(f"ik_fk_approach_error_deg={approach_error_deg:.3f}")
        print(f"ik_fk_validation={fk_ok}")
        print("NO_HARDWARE_COMMAND_SENT")
        return 0 if fk_ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
