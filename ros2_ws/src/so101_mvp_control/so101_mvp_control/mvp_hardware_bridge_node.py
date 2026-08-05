from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MvpHardwareBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mvp_hardware_bridge_node")
        self.create_service(Trigger, "mvp_hardware_stop", self.handle_stop)
        self._log_skeleton_status()

    def _log_skeleton_status(self) -> None:
        logger = self.get_logger()
        logger.info("SO101 MVP control skeleton started")
        logger.info("hardware motion disabled")
        logger.info("no legacy command gate")
        logger.info("no shadow executor")
        logger.info("mvp_hardware_bridge_node does not connect to TCP or serial in MVP-0")

    def handle_stop(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        response.success = True
        response.message = "Stage MVP-0 skeleton stop acknowledged; no hardware was connected"
        return response


def main() -> None:
    rclpy.init()
    node = MvpHardwareBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

