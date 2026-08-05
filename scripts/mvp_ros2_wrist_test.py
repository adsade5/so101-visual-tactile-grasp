from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any


ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def plan_only(delta_deg: float, speed_rad_s: float) -> int:
    payload = {
        "mode": "plan_only",
        "opens_com4": False,
        "sends_motion": False,
        "required_for_execution": ["--execute", "--confirm", "MVP_MOVE"],
        "joint": "wrist_roll",
        "delta_deg": float(delta_deg),
        "delta_rad": math.radians(float(delta_deg)),
        "speed_rad_s": float(speed_rad_s),
        "topics": {
            "read_state": "/mvp/joint_states",
            "publish_target": "/mvp/joint_target",
            "execute_service": "/mvp/execute_target",
            "stop_service": "/mvp/stop",
        },
    }
    print(json.dumps(payload, indent=2))
    return 0


def execute(delta_deg: float, speed_rad_s: float, confirm: str) -> int:
    if confirm != "MVP_MOVE":
        print("Refusing execution: --confirm MVP_MOVE is required", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    class WristTestNode(Node):
        def __init__(self) -> None:
            super().__init__("mvp_ros2_wrist_test")
            self.latest: JointState | None = None
            self.create_subscription(JointState, "/mvp/joint_states", self._state_cb, 10)
            self.target_pub = self.create_publisher(JointState, "/mvp/joint_target", 10)
            self.execute_client = self.create_client(Trigger, "/mvp/execute_target")
            self.stop_client = self.create_client(Trigger, "/mvp/stop")

        def _state_cb(self, msg: JointState) -> None:
            if list(msg.name) == ARM_JOINT_NAMES and len(msg.position) >= len(ARM_JOINT_NAMES):
                self.latest = msg

    rclpy.init()
    node = WristTestNode()
    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and node.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest is None:
            print("No /mvp/joint_states sample received", file=sys.stderr)
            return 3
        if not node.execute_client.wait_for_service(timeout_sec=3.0):
            print("/mvp/execute_target service unavailable", file=sys.stderr)
            return 4

        current = [float(value) for value in node.latest.position[: len(ARM_JOINT_NAMES)]]
        target = list(current)
        target[4] += math.radians(delta_deg)
        result = publish_and_execute(node, target)
        if not result["success"]:
            print(json.dumps(result, indent=2), file=sys.stderr)
            return 5
        result_back = publish_and_execute(node, current)
        print(json.dumps({"forward": result, "return": result_back}, indent=2))
        return 0 if result_back["success"] else 6
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def publish_and_execute(node: Any, target: list[float]) -> dict[str, Any]:
    import rclpy
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = list(ARM_JOINT_NAMES)
    msg.position = list(target)
    for _ in range(5):
        node.target_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    future = node.execute_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done() or future.result() is None:
        return {"success": False, "reason": "execute_service_timeout"}
    response = future.result()
    return {"success": bool(response.success), "reason": str(response.message)}


def main() -> int:
    parser = argparse.ArgumentParser(description="Future supervised ROS2 wrist-roll MVP motion acceptance helper.")
    parser.add_argument("--plan-only", action="store_true", help="Print the planned action without ROS or hardware motion.")
    parser.add_argument("--execute", action="store_true", help="Send the command through ROS2 services.")
    parser.add_argument("--confirm", default="", help="Must be MVP_MOVE for --execute.")
    parser.add_argument("--delta-deg", type=float, default=2.0)
    parser.add_argument("--speed-rad-s", type=float, default=0.04)
    args = parser.parse_args()

    if args.plan_only or not args.execute:
        return plan_only(args.delta_deg, args.speed_rad_s)
    return execute(args.delta_deg, args.speed_rad_s, args.confirm)


if __name__ == "__main__":
    raise SystemExit(main())
