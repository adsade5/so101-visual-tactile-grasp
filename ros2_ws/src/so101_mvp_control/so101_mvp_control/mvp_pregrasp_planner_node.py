from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .pregrasp_planner import (
    ARM_JOINT_NAMES,
    DEFAULT_APPROACH_TOLERANCE_DEG,
    DEFAULT_POSITION_TOLERANCE_M,
    JointStateSnapshot,
    PoseSnapshot,
    PregraspPlan,
    compute_pregrasp_plan,
    create_model,
    top_down_quaternion_xyzw,
)


DEFAULT_PROJECT_ROOT = (
    "E:/PycharmProjects/Embodied_AI/"
    "LeRobot_Project/so101_visual_tactile_grasp"
)


class MvpPregraspPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("mvp_pregrasp_planner_node")

        self.declare_parameter("project_root", DEFAULT_PROJECT_ROOT)
        self.declare_parameter("object_pose_topic", "/object_pose_base")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_object_pose_age_s", 1.0)
        self.declare_parameter("pregrasp_height_m", 0.08)
        self.declare_parameter("use_joint_state_seed", True)
        self.declare_parameter("joint_state_topic", "/mvp/joint_states")
        self.declare_parameter("max_joint_state_age_s", 1.0)
        self.declare_parameter("position_tolerance_m", DEFAULT_POSITION_TOLERANCE_M)
        self.declare_parameter(
            "approach_tolerance_deg",
            DEFAULT_APPROACH_TOLERANCE_DEG,
        )

        self.project_root = Path(str(self.get_parameter("project_root").value))
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.max_object_pose_age_s = float(
            self.get_parameter("max_object_pose_age_s").value
        )
        self.pregrasp_height_m = float(
            self.get_parameter("pregrasp_height_m").value
        )
        self.use_joint_state_seed = bool(
            self.get_parameter("use_joint_state_seed").value
        )
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.max_joint_state_age_s = float(
            self.get_parameter("max_joint_state_age_s").value
        )
        self.position_tolerance_m = float(
            self.get_parameter("position_tolerance_m").value
        )
        self.approach_tolerance_deg = float(
            self.get_parameter("approach_tolerance_deg").value
        )

        self.model = create_model(self.project_root)
        self.latest_object_pose: PoseSnapshot | None = None
        self.latest_joint_state: JointStateSnapshot | None = None
        self.current_plan: PregraspPlan | None = None

        self.pregrasp_pose_publisher = self.create_publisher(
            PoseStamped,
            "/mvp/pregrasp_pose",
            10,
        )
        self.joint_target_publisher = self.create_publisher(
            JointState,
            "/mvp/pregrasp_joint_target",
            10,
        )
        self.valid_publisher = self.create_publisher(
            Bool,
            "/mvp/pregrasp_valid",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/mvp/pregrasp_status",
            10,
        )

        self.create_subscription(
            PoseStamped,
            self.object_pose_topic,
            self.handle_object_pose,
            10,
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.handle_joint_state,
            10,
        )
        self.create_service(
            Trigger,
            "/mvp/compute_pregrasp",
            self.handle_compute_pregrasp,
        )
        self.create_service(
            Trigger,
            "/mvp/clear_pregrasp",
            self.handle_clear_pregrasp,
        )

        self.publish_valid(False)
        self.publish_status("waiting_for_object_pose")
        self.get_logger().info(
            "MVP pregrasp planner preview started | "
            f"object_pose_topic={self.object_pose_topic} | "
            f"base_frame={self.base_frame} | "
            f"pregrasp_height_m={self.pregrasp_height_m:.3f}"
        )

    def handle_object_pose(self, message: PoseStamped) -> None:
        position = np.asarray(
            [
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
            ],
            dtype=np.float64,
        )
        self.latest_object_pose = PoseSnapshot(
            frame_id=str(message.header.frame_id),
            position_m=position,
            received_monotonic_s=time.monotonic(),
        )

    def handle_joint_state(self, message: JointState) -> None:
        positions = np.asarray([float(value) for value in message.position], dtype=np.float64)
        self.latest_joint_state = JointStateSnapshot(
            names=tuple(str(name) for name in message.name),
            positions_rad=positions,
            received_monotonic_s=time.monotonic(),
        )

    def handle_compute_pregrasp(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        plan = compute_pregrasp_plan(
            model=self.model,
            object_pose=self.latest_object_pose,
            joint_state=self.latest_joint_state,
            base_frame=self.base_frame,
            now_monotonic_s=time.monotonic(),
            max_object_pose_age_s=self.max_object_pose_age_s,
            pregrasp_height_m=self.pregrasp_height_m,
            use_joint_state_seed=self.use_joint_state_seed,
            max_joint_state_age_s=self.max_joint_state_age_s,
            position_tolerance_m=self.position_tolerance_m,
            approach_tolerance_deg=self.approach_tolerance_deg,
        )
        self.current_plan = plan if plan.success else None

        if not plan.success:
            self.publish_valid(False)
            self.publish_status(plan.reason)
            response.success = False
            response.message = plan.reason
            return response

        self.publish_pregrasp_plan(plan)
        self.publish_valid(True)
        self.publish_status("pregrasp_ready")

        assert plan.pregrasp_position_m is not None
        response.success = True
        response.message = (
            "pregrasp_ready "
            f"x={plan.pregrasp_position_m[0]:.6f} "
            f"y={plan.pregrasp_position_m[1]:.6f} "
            f"z={plan.pregrasp_position_m[2]:.6f} "
            f"position_error_m={plan.position_error_m:.6f} "
            f"approach_error_deg={plan.approach_error_deg:.3f}"
        )
        return response

    def handle_clear_pregrasp(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        self.current_plan = None
        self.publish_valid(False)
        self.publish_status("cleared")
        response.success = True
        response.message = "cleared"
        return response

    def publish_pregrasp_plan(self, plan: PregraspPlan) -> None:
        if plan.pregrasp_position_m is None or plan.joint_positions_rad is None:
            raise ValueError("Cannot publish incomplete pregrasp plan")

        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(plan.pregrasp_position_m[0])
        pose.pose.position.y = float(plan.pregrasp_position_m[1])
        pose.pose.position.z = float(plan.pregrasp_position_m[2])
        qx, qy, qz, qw = top_down_quaternion_xyzw()
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pregrasp_pose_publisher.publish(pose)

        target = JointState()
        target.header.stamp = now
        target.name = list(ARM_JOINT_NAMES)
        target.position = [float(value) for value in plan.joint_positions_rad.tolist()]
        self.joint_target_publisher.publish(target)

        self.get_logger().info(
            "pregrasp_ready | "
            f"seed_source={plan.seed_source} | "
            f"position_error_m={plan.position_error_m:.6f} | "
            f"approach_error_deg={plan.approach_error_deg:.3f}"
        )

    def publish_valid(self, valid: bool) -> None:
        message = Bool()
        message.data = bool(valid)
        self.valid_publisher.publish(message)

    def publish_status(self, status: str) -> None:
        message = String()
        message.data = str(status)
        self.status_publisher.publish(message)

    def status_payload(self) -> str:
        payload = {
            "node": "mvp_pregrasp_planner_node",
            "object_pose_topic": self.object_pose_topic,
            "base_frame": self.base_frame,
            "pregrasp_height_m": self.pregrasp_height_m,
            "hardware_control_enabled": False,
            "command_topics_published": [],
        }
        return json.dumps(payload, sort_keys=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MvpPregraspPlannerNode | None = None
    try:
        node = MvpPregraspPlannerNode()
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
