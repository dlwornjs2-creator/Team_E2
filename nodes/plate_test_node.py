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
    from plate.controller import PlateController
    from plate.waypoints import (
        DISH1_VIA_J, DISH1_WASH_START_J,
        DISH2_VIA_J, DISH2_WASH_START_J,
    )

    controller = PlateController(node)

    try:
        if not controller.setup():
            return

        # ---------------------------------------------------------------
        # [1] dish2 위치 -> 배치 확인   <- 현재 활성화
        #     sol 7 -> sol 2 로 J1 이 크게 회전하는 구간이 핵심.
        #     처음에는 접시 없이 빈 그리퍼로, 저속으로 볼 것.
        # ---------------------------------------------------------------
        # ---------------------------------------------------------------
        # 임시 테스트: 전과정에서 세척 동작만 빼고 경로만 확인
        # ---------------------------------------------------------------
        controller.move_home()
        controller.pick_plate()

        # dish1 세척 위치 왕복 (wash_arcs 는 호출 안 함)
        controller.move_to_wash_start(DISH1_VIA_J, DISH1_WASH_START_J, "dish1")
        controller.leave_wash_area(DISH1_VIA_J, "dish1")

        # dish2 세척 위치 왕복
        controller.move_to_wash_start(DISH2_VIA_J, DISH2_WASH_START_J, "dish2")
        controller.leave_wash_area(DISH2_VIA_J, "dish2")

        controller.place_plate()
        controller.move_home()

        # ---------------------------------------------------------------
        # [2] 파지 -> 배치
        # ---------------------------------------------------------------
        # controller.move_home()
        # controller.pick_plate()
        # controller.place_plate()

        # ---------------------------------------------------------------
        # [3] dish1 세척 (동작 확인 완료)
        # ---------------------------------------------------------------
        # controller.wash_face(cfg.DISH1_COORD, DISH1_VIA_J,
        #                      DISH1_WASH_START_J, cfg.DISH1_START_ANGLE,
        #                      "dish1 (front)")

        # ---------------------------------------------------------------
        # [4] dish2 세척 (수세미 교체 후 재조정 예정)
        # ---------------------------------------------------------------
        # controller.wash_face(cfg.DISH2_COORD, DISH2_VIA_J,
        #                      DISH2_WASH_START_J, cfg.DISH2_START_ANGLE,
        #                      "dish2 (back)")

        # ---------------------------------------------------------------
        # [5] 전체 시나리오 (Pick -> Wash -> Place -> Home)
        # ---------------------------------------------------------------
        # controller.run_plate_task()

        controller.move_home()

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