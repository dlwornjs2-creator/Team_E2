"""접시 파트 단독 테스트 노드.

실행:  ros2 run dishwashing_robot plate_test

아래 main() 안에서 실행할 단계만 주석 해제해 사용한다.
처음 도는 동작은 반드시 작은 값 / 저속으로 확인할 것.
"""

import rclpy
import DR_init

from plate import config as cfg

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("plate_test_node", namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = node

    # DSR 노드 등록 후에 import 해야 서비스 클라이언트가 정상 생성된다
    from DSR_ROBOT2 import movej
    from plate.controller import PlateController
    from plate.waypoints import (
        WASH1_APPROACH_J, WASH1_START_J,
        WASH2_APPROACH_J, WASH2_START_J,
    )

    controller = PlateController(node)

    try:
        if not controller.setup():
            return

        # ---------------------------------------------------------------
        # [1] 전과정   <- 현재 활성화
        #     Pick -> Wash(위치1,2) -> Rotate -> Wash(위치1,2)
        #       -> Place -> Home
        # ---------------------------------------------------------------
        controller.run_plate_task()

        # ---------------------------------------------------------------
        # [2] 전과정 (회전 없이 1회 세척만)
        # ---------------------------------------------------------------
        # controller.run_plate_task(rotate=False)

        # ---------------------------------------------------------------
        # [3] 세척 위치 1 만
        # ---------------------------------------------------------------
        # controller.wash_face(WASH1_APPROACH_J, WASH1_START_J,
        #                      cfg.WASH1_START_ANGLE, cfg.WASH1_FORCE_SIGN,
        #                      label="wash pos 1")

        # ---------------------------------------------------------------
        # [4] 세척 위치 2 만
        # ---------------------------------------------------------------
        # controller.wash_face(WASH2_APPROACH_J, WASH2_START_J,
        #                      cfg.WASH2_START_ANGLE, cfg.WASH2_FORCE_SIGN,
        #                      label="wash pos 2")

        # ---------------------------------------------------------------
        # [5] 음식물 확인 / 배출만 테스트
        # ---------------------------------------------------------------
        # from plate import food_check
        # food_check.run_food_check(node.get_logger())

        # ---------------------------------------------------------------
        # [6] 접시 회전(재파지)
        # ---------------------------------------------------------------
        # controller.rotate_plate(steps=1)

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()