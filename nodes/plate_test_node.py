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
        # [1] 접시 회전(재파지)만 테스트   <- 현재 활성화
        #     릴리즈 안전 -> 릴리즈 -> open -> 릴리즈 안전
        #       -> 그랩 안전 -> 그랩 -> close -> 그랩 안전
        #     1회 = 약 60도. 처음에는 steps=1 로 한 칸만 확인할 것.
        # ---------------------------------------------------------------

        # 한 칸이 확인되면 반바퀴(3회)로:
        # controller.rotate_plate()          # config.ROTATE_STEPS = 3

        # ---------------------------------------------------------------
        # [2] 파지 -> 회전 -> 배치
        # ---------------------------------------------------------------
        controller.move_home()
        controller.pick_plate()
        controller.rotate_plate()
        # controller.place_plate()

        # ---------------------------------------------------------------
        # [3] 전과정 (세척 포함)
        # ---------------------------------------------------------------
        # controller.run_plate_task()

        # ---------------------------------------------------------------
        # [4] dish1 세척만 (수세미 교체로 좌표 재티칭 필요)
        # ---------------------------------------------------------------
        # controller.wash_face(cfg.DISH1_COORD, DISH1_VIA_J,
        #                      DISH1_WASH_START_J, cfg.DISH1_START_ANGLE,
        #                      "dish1 (front)")

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