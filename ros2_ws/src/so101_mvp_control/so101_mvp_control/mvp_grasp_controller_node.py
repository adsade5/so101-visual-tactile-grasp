from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MvpGraspControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("mvp_grasp_controller_node")
        self.create_service(Trigger, "start_mvp_grasp", self.handle_start)
        self._log_skeleton_status()

    def _log_skeleton_status(self) -> None:
        logger = self.get_logger()
        logger.info("SO101 MVP control skeleton started")
        logger.info("hardware motion disabled")
        logger.info("no legacy command gate")
        logger.info("no shadow executor")
        logger.info("mvp_grasp_controller_node waits for explicit future start service")

    def handle_start(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        response.success = False
        response.message = "Stage MVP-0 skeleton only; hardware motion disabled"
        return response


def main() -> None:
    rclpy.init()
    node = MvpGraspControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

