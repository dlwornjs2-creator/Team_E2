"""back_ui 진입점. `ros2 run back_ui node`로 띄운다.

첫 번째 진짜 토픽 구독: `/<robot_name>/joint_states`(sensor_msgs/JointState,
기본 robot_name=`dsr01`). doosan-robot2 드라이버가 로봇 이름으로 네임스페이스
잡아서 내는 진짜 관절값이다 — 이름 없는 `/joint_states`는 실제 목록에서
따로 존재하긴 하지만(용도 불명) 이쪽이 아니다(2026-08-05 실제
`ros2 topic list`로 확인 후 정정). image/voice/main 노드는 아직 팀원이 안
만들었고 db는 서비스만 낸다 — 그래서 robot.links만 실제 값이고 나머지는
아직 정적 스냅샷(state_store.py 참고)이다.

관절 각도 -> 링크 위치/자세 계산은 robot_fk.py(순수 계산, rclpy 없음)가
한다. HTTP 서버·front_ui 쪽은 스키마가 안 바뀌는 한 손댈 필요 없다
(CLAUDE.md: "back_ui가 준비되면 ros2 run back_ui node를 띄우기만 하면 되고
front_ui 쪽은 손대지 않는다").
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from back_ui.frame_store import FrameStore
from back_ui.http_server import start_server
from back_ui.robot_fk import compute_links
from back_ui.state_store import StateStore

# 관절 콜백을 이 간격보다 자주 처리하지 않는다. joint_state_broadcaster는
# 보통 컨트롤러 제어 주기(수백 Hz~1kHz)로 퍼블리시하는데, front_ui는 가장
# 빨라도 0.1초(POLL_MONITOR)에 한 번만 본다 — 그 사이에 들어오는 메시지를
# 전부 FK 계산해봐야 아무도 안 본다. rclpy 콜백은 spin()과 같은 스레드에서
# 도는데, HTTP 서버 스레드랑 GIL을 나눠 쓰다 보니 콜백이 너무 잦으면 HTTP
# 응답이 밀리는 것처럼 느껴진다(2026-08-05 "렉 걸린다" 보고 이후 추가).
_JOINT_UPDATE_MIN_INTERVAL = 0.1


class BackUiNode(Node):
    def __init__(self):
        super().__init__("back_ui")

        self.state_store = StateStore()
        self.frame_store = FrameStore()
        self._httpd = start_server(self.state_store, self.frame_store)

        addr, port = self._httpd.server_address
        self.get_logger().info(f"back_ui HTTP 서버 시작: http://{addr}:{port}")

        # 로봇 이름이 바뀌거나(다른 팀/다른 대수) 나중에 여러 대를 다뤄야
        # 하는 상황을 대비해 하드코딩 대신 파라미터로 뺐다.
        # 실행 시 바꾸려면: ros2 run back_ui node --ros-args -p robot_name:=dsr02
        self.declare_parameter("robot_name", "dsr01")
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        joint_states_topic = f"/{robot_name}/joint_states"

        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 10
        )
        self.get_logger().info(f"구독 시작: {joint_states_topic}")
        self._last_joint_update = 0.0

        # TODO(다음 단계): image/voice/main 노드가 생기면 각자 토픽을 구독해서
        # system/objects/zones/recent_tasks도 여기처럼 채운다.

    def _on_joint_states(self, msg: JointState):
        """관절 각도 -> base_link 기준 링크 pos/rpy로 바꿔서 저장소에 반영.

        msg.name/msg.position은 순서가 정해져 있지 않고(그리퍼 관절이 섞여
        있을 수도 있다) robot_fk.compute_links가 이름으로 찾아 쓰므로
        그대로 dict로 묶어서 넘긴다.

        `_JOINT_UPDATE_MIN_INTERVAL`보다 자주 들어온 메시지는 그냥 버린다
        (연산 자체는 가볍지만, 퍼블리시가 너무 잦으면 이 콜백이 spin()
        스레드를 계속 붙잡아서 HTTP 서버 스레드가 GIL을 못 받아 응답이
        밀린다 — "렉" 원인으로 의심되는 부분).
        """
        now = time.monotonic()
        if now - self._last_joint_update < _JOINT_UPDATE_MIN_INTERVAL:
            return
        self._last_joint_update = now

        angles = dict(zip(msg.name, msg.position))
        self.state_store.update_robot_links(compute_links(angles))

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BackUiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
