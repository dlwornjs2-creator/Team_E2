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
        PLACE_VIA_J,
    )

    controller = PlateController(node)

    try:
        if not controller.setup():
            return

        # ---------------------------------------------------------------
        # [1] 전과정   <- 현재 활성화
        # ---------------------------------------------------------------
        controller.run_plate_task()

        # ---------------------------------------------------------------
        # [2] 파지 -> 회전 -> 배치 경유점까지만 (좌표 따기용)
        # ---------------------------------------------------------------
        # controller.move_home()
        # controller.pick_plate()
        # controller.rotate_plate()
        # movej(PLACE_VIA_J, vel=cfg.VEL_J, acc=cfg.ACC_J)

        # ---------------------------------------------------------------
        # [3] 파지 -> 회전 -> 배치
        # ---------------------------------------------------------------
        # controller.move_home()
        # controller.pick_plate()
        # controller.rotate_plate()
        # controller.place_plate()
        # controller.move_home()

        # ---------------------------------------------------------------
        # [4] 세척 위치 1 (앞면, 원호) 만
        # ---------------------------------------------------------------
        # controller.wash_face(WASH1_APPROACH_J, WASH1_START_J,
        #                      cfg.WASH1_START_ANGLE, cfg.WASH1_FORCE_SIGN,
        #                      label="wash pos 1", sweep=False)

        # ---------------------------------------------------------------
        # [5] 세척 위치 2 (뒷면, 직선 쓸기) 만
        # ---------------------------------------------------------------
        # controller.wash_face(WASH2_APPROACH_J, WASH2_START_J,
        #                      cfg.WASH2_START_ANGLE, cfg.WASH2_FORCE_SIGN,
        #                      label="wash pos 2", sweep=True)

        # ---------------------------------------------------------------
        # [6] 접시 회전(재파지)만
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