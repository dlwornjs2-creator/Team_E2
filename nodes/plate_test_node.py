"""접시 파트 단독 테스트 노드."""

import rclpy
import DR_init

from plate import config as cfg

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("plate_test_node", namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR API 등록 후에 controller 를 import 해야 한다
    from plate.controller import PlateController

    controller = PlateController(node)

    try:
        if not controller.setup():
            return

        controller.move_home()
        controller.pick_plate()

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()