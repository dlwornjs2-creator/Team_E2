"""Executable entry point for the modular Any6D robot controller."""

from __future__ import annotations

from typing import Optional

import rclpy

import DR_init

from .config import DEFAULT_CONFIG
from .control_node import RobotControlNode


def main(args=None) -> None:
    config = DEFAULT_CONFIG
    rclpy.init(args=args)
    DR_init.__dsr__id = config.robot.robot_id
    DR_init.__dsr__model = config.robot.robot_model

    controller: Optional[RobotControlNode] = None
    try:
        controller = RobotControlNode(config)
        DR_init.__dsr__node = controller

        # DSR_ROBOT2 must be imported after DR_init receives id/model/node.
        try:
            import DSR_ROBOT2 as dsr_api
        except ImportError as error:
            controller.get_logger().error(f"DSR_ROBOT2 import failed: {error}")
            raise

        controller.bind_dsr_api(dsr_api)
        controller.initialize_hardware()
        controller.run()

    except KeyboardInterrupt:
        pass
    finally:
        if controller is not None:
            controller.shutdown_hardware()
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
