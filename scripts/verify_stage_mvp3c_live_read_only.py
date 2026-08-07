from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "verification" / "stage_mvp3c_live_ros_result.json"
ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


class LiveReadOnlyVerifier(Node):
    def __init__(self, sample_count: int) -> None:
        super().__init__("mvp3c_live_read_only_verifier")
        self.sample_count = int(sample_count)
        self.joint_samples: list[tuple[float, JointState]] = []
        self.gripper_sample: Float64 | None = None
        self.target_pub = self.create_publisher(JointState, "/mvp/joint_target", 10)
        self.create_subscription(JointState, "/mvp/joint_states", self._joint_cb, 10)
        self.create_subscription(Float64, "/mvp/gripper_state", self._gripper_cb, 10)
        self.execute_client = self.create_client(Trigger, "/mvp/execute_target")

    def _joint_cb(self, msg: JointState) -> None:
        self.joint_samples.append((time.monotonic(), msg))

    def _gripper_cb(self, msg: Float64) -> None:
        self.gripper_sample = msg

    def wait_for_samples(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if len(self.joint_samples) >= self.sample_count and self.gripper_sample is not None:
                return
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_current_as_target(self) -> None:
        sample = self.joint_samples[-1][1]
        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = list(sample.name)
        target.position = list(sample.position)
        for _ in range(5):
            self.target_pub.publish(target)
            rclpy.spin_once(self, timeout_sec=0.05)

    def call_execute_target(self) -> dict[str, Any]:
        if not self.execute_client.wait_for_service(timeout_sec=3.0):
            return {"success": None, "message": "execute_service_unavailable"}
        future = self.execute_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return {"success": None, "message": "execute_service_timeout"}
        result = future.result()
        return {"success": bool(result.success), "message": str(result.message)}


def joint_sample_to_dict(msg: JointState) -> dict[str, Any]:
    return {
        "name": list(msg.name),
        "position": [float(value) for value in msg.position],
    }


def compute_rate(samples: list[tuple[float, JointState]]) -> float:
    if len(samples) < 2:
        return 0.0
    elapsed = samples[-1][0] - samples[0][0]
    if elapsed <= 0.0:
        return 0.0
    return (len(samples) - 1) / elapsed


def main() -> int:
    parser = argparse.ArgumentParser(description="MVP-3C live read-only ROS2 bridge verifier.")
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    rclpy.init()
    node = LiveReadOnlyVerifier(args.samples)
    try:
        node.wait_for_samples(args.timeout_s)
        joint_state_received = len(node.joint_samples) >= args.samples
        gripper_state_received = node.gripper_sample is not None
        rate = compute_rate(node.joint_samples[: args.samples])
        latest = node.joint_samples[-1][1] if node.joint_samples else JointState()
        order_valid = list(latest.name) == ARM_JOINT_NAMES
        positions = [float(value) for value in latest.position]
        all_finite = len(positions) == len(ARM_JOINT_NAMES) and all(math.isfinite(value) for value in positions)

        execute_result = {"success": None, "message": "not_called"}
        after_sample = None
        if joint_state_received:
            node.publish_current_as_target()
            execute_result = node.call_execute_target()
            deadline = time.monotonic() + 3.0
            before_count = len(node.joint_samples)
            while rclpy.ok() and time.monotonic() < deadline and len(node.joint_samples) <= before_count:
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.joint_samples:
                after_sample = joint_sample_to_dict(node.joint_samples[-1][1])

        report = {
            "joint_state_received": joint_state_received,
            "joint_state_message_count": len(node.joint_samples),
            "joint_state_sample": joint_sample_to_dict(latest),
            "joint_state_order_valid": order_valid,
            "all_joint_values_finite": all_finite,
            "joint_state_publish_rate_observed_hz": rate,
            "publish_rate_valid": 4.0 <= rate <= 6.0,
            "gripper_state_received": gripper_state_received,
            "gripper_state_sample": None
            if node.gripper_sample is None
            else {"data": float(node.gripper_sample.data)},
            "execute_service_success": execute_result["success"],
            "execute_service_message": execute_result["message"],
            "execute_target_rejection": {
                "success": execute_result["success"],
                "message": execute_result["message"],
                "valid": execute_result["success"] is False
                and "hardware_motion_disabled" in execute_result["message"],
            },
            "after_execute_joint_state_sample": after_sample,
            "physical_motion_command_requested": False,
        }
        failure_reasons = []
        if not joint_state_received:
            failure_reasons.append("insufficient_joint_state_samples")
        if len(node.joint_samples) == 0:
            failure_reasons.append("joint_state_timeout")
        if not gripper_state_received:
            failure_reasons.append("gripper_state_timeout")
        if not order_valid:
            failure_reasons.append("joint_order_invalid")
        if not all_finite:
            failure_reasons.append("joint_values_not_finite")
        if not report["publish_rate_valid"]:
            failure_reasons.append("publish_rate_out_of_range")
        if execute_result["success"] is None:
            failure_reasons.append(str(execute_result["message"]))
        elif execute_result["success"] is not False:
            failure_reasons.append("unexpected_execute_success")
        elif "hardware_motion_disabled" not in execute_result["message"]:
            failure_reasons.append("unexpected_execute_rejection_message")
        report["failure_reasons"] = failure_reasons
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
        ok = (
            joint_state_received
            and gripper_state_received
            and order_valid
            and all_finite
            and report["publish_rate_valid"]
            and report["execute_target_rejection"]["valid"]
        )
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
