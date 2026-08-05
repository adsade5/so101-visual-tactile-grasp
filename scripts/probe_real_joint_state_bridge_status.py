from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from so101_command_gate.real_joint_state_bridge_node import RealJointStateBridgeNode


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--duration-s", type=float, default=4.0)
    args = parser.parse_args()

    rclpy.init()
    bridge = RealJointStateBridgeNode(
        project_root_override=args.project_root,
        host_override=args.host,
        port_override=args.port,
        timeout_s_override=0.5,
    )
    harness = Node("real_joint_state_bridge_probe")
    statuses: list[dict[str, Any]] = []
    diagnostics: list[dict[str, Any]] = []
    valids: list[bool] = []
    harness.create_subscription(
        String,
        "/real_joint_state_status",
        lambda message: statuses.append(json.loads(message.data)),
        10,
    )
    harness.create_subscription(
        String,
        "/real_joint_state_diagnostic",
        lambda message: diagnostics.append(json.loads(message.data)),
        10,
    )
    harness.create_subscription(
        Bool,
        "/real_joint_state_valid",
        lambda message: valids.append(bool(message.data)),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(harness)
    try:
        deadline = time.monotonic() + float(args.duration_s)
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.remove_node(bridge)
        executor.remove_node(harness)
        bridge.destroy_node()
        harness.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

    result = {
        "status_count": len(statuses),
        "diagnostic_count": len(diagnostics),
        "valid_samples_tail": valids[-10:],
        "latest_status": statuses[-1] if statuses else None,
        "latest_diagnostic": diagnostics[-1] if diagnostics else None,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    latest_status = result["latest_status"]
    if not isinstance(latest_status, dict):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
