"""접시 파트 단독 테스트 노드.

실행:  ros2 run dishwashing_robot plate_test
"""

import rclpy
import DR_init

from plate import config as cfg

from plate.waypoints import (
    DISH1_VIA_J, DISH1_WASH_START_J,
    DISH2_VIA_J, DISH2_WASH_START_J,
)

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("plate_test_node", namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR 노드 등록 후에 import 해야 서비스 클라이언트가 정상 생성된다
    from plate.controller import PlateController

    controller = PlateController(node)

    try:
        if not controller.setup():
            return

        # --- 전체 시나리오 ---
        # controller.run_plate_task()

        controller.move_home()
        controller.pick_plate()

        # 세척 제외 — 경유점 이동만 확인
        controller.move_to_wash_start(DISH1_VIA_J, DISH1_WASH_START_J, "dish1")
        controller.leave_wash_area(DISH1_VIA_J, "dish1")
        controller.move_to_wash_start(DISH2_VIA_J, DISH2_WASH_START_J, "dish2")
        controller.leave_wash_area(DISH2_VIA_J, "dish2")

        # controller.place_plate()
        controller.move_home()

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()