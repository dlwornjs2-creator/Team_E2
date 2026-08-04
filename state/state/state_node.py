"""상태 관리 노드 — 2단계.

부팅 시 다른 노드 준비 확인 + 음성 명령을 받아 탐색 액션으로 넘긴다.

순서도 대응:
  시작 -> State = LOAD -> 각 노드에 상태 확인 요청
       -> 성공? -> No 면 재시도 / Yes 면 State = IDLE -> 대기
       -> 명령 수신 -> 상태 확인 -> IDLE 이면 State = RUN + 액션 발행
                                  아니면 거절

상태
  LOAD  부팅 중. 다른 노드 준비를 기다리는 중
  IDLE  대기. 이 상태에서만 명령을 받는다
  RUN   작업 중. 새 명령은 거절한다

제공
  service  /state/target_search  interfaces/TargetSearch  음성 노드의 명령 접수
  topic    /state/current        std_msgs/String          현재 상태 (JSON)

사용
  service  /<노드>/init          interfaces/NodeInit      각 노드의 준비 확인
  action   /item/search          interfaces/Search        탐색 실행 지시
"""

import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from interfaces.action import Search
from interfaces.srv import NodeInit, TargetSearch


# ----------------------------------------------------------------------
# 상태 정의
# ----------------------------------------------------------------------
LOAD = 'LOAD'
IDLE = 'IDLE'
RUN = 'RUN'


class StateNode(Node):

    def __init__(self):
        super().__init__('state_node')

        # 확인할 노드 목록. 노드가 늘어나면 여기에 추가한다
        self.declare_parameter('targets', ['db'])
        self.declare_parameter('wait_timeout', 5.0)     # 서비스 등장 대기 [s]
        self.declare_parameter('retry_period', 2.0)     # 재시도 간격 [s]
        self.declare_parameter('max_retries', 0)        # 0 이면 무한 재시도
        self.declare_parameter('search_action', 'item/search')
        self.declare_parameter('goal_accept_timeout', 3.0)

        self.targets = list(self.get_parameter('targets').value)
        self.wait_timeout = self.get_parameter('wait_timeout').value
        self.retry_period = self.get_parameter('retry_period').value
        self.max_retries = self.get_parameter('max_retries').value
        self.search_action = self.get_parameter('search_action').value
        self.goal_accept_timeout = self.get_parameter('goal_accept_timeout').value

        self.state = LOAD
        self.ready = {name: False for name in self.targets}
        self.current_goal = None        # 진행 중인 탐색 goal handle

        cb = ReentrantCallbackGroup()

        # 현재 상태 발행. 늦게 뜨는 노드도 마지막 상태를 바로 받도록 TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, 'state/current',
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.init_clients = {
            name: self.create_client(NodeInit, f'{name}/init',
                                     callback_group=cb)
            for name in self.targets
        }

        # 음성 노드가 부르는 명령 접수 창구
        self.create_service(
            TargetSearch, 'state/target_search',
            self.on_target_search, callback_group=cb)

        # 실제 탐색을 시킬 액션 클라이언트
        self.search_cli = ActionClient(
            self, Search, self.search_action, callback_group=cb)

        self.publish_state()
        self.get_logger().info(f'확인 대상: {self.targets}')
        self.startup_check()

    # ------------------------------------------------------------------
    def set_state(self, new_state, reason=''):
        old = self.state
        self.state = new_state
        self.publish_state(reason)
        if old != new_state:
            self.get_logger().info(
                f'상태 전이: {old} -> {new_state}'
                + (f' ({reason})' if reason else ''))

    def publish_state(self, reason=''):
        msg = String()
        msg.data = json.dumps(
            {'state': self.state, 'ready': self.ready, 'reason': reason},
            ensure_ascii=False)
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------
    # 부팅 확인 (순서도의 LOAD 구간)
    # ------------------------------------------------------------------
    def startup_check(self):
        """전부 준비될 때까지 재시도한다. 되면 IDLE 로 넘어간다."""
        attempt = 0

        while rclpy.ok():
            attempt += 1

            # 아직 준비 안 된 노드만 다시 물어본다
            for name in self.targets:
                if not self.ready[name]:
                    self.ready[name] = self.check_one(name)

            if all(self.ready.values()):
                self.set_state(IDLE, '전체 노드 준비 완료')
                return

            not_ready = [n for n, ok in self.ready.items() if not ok]

            if self.max_retries and attempt >= self.max_retries:
                self.get_logger().error(
                    f'{attempt}회 시도 후 포기 — 미준비: {not_ready}')
                return

            self.get_logger().warn(
                f'[{attempt}회차] 미준비: {not_ready} — '
                f'{self.retry_period}초 뒤 재시도')
            self.publish_state('재시도 대기')
            time.sleep(self.retry_period)

    # ------------------------------------------------------------------
    # 명령 접수 (게이트키퍼)
    # ------------------------------------------------------------------
    def on_target_search(self, request, response):
        """음성 노드의 탐색 명령. 상태를 보고 받을지 거절할지 판단한다."""
        name = request.target_name.strip()
        label = request.class_label.strip()
        self.get_logger().info(f"명령 수신: target='{name}' class='{label}'")

        if not name and not label:
            response.success = False
            response.message = 'target_name 과 class_label 이 모두 비어 있습니다'
            self.get_logger().warn(f'거절: {response.message}')
            return response

        if self.state != IDLE:
            response.success = False
            response.message = f'지금은 {self.state} 상태라 명령을 받을 수 없습니다'
            self.get_logger().warn(f'거절: {response.message}')
            return response

        if not self.search_cli.server_is_ready():
            response.success = False
            response.message = f'탐색 노드 없음 (/{self.search_action})'
            self.get_logger().error(f'거절: {response.message}')
            return response

        handle = self.send_search_goal(name, label)
        if handle is None:
            response.success = False
            response.message = '탐색 노드가 명령을 받지 않았습니다'
            self.get_logger().error(f'거절: {response.message}')
            return response

        # 여기서 응답은 "접수 완료" 까지만. 결과는 액션으로 따로 돌아온다
        self.current_goal = handle
        self.set_state(RUN, f"탐색 시작: {name or label}")

        response.success = True
        response.message = f"'{name or label}' 탐색을 시작합니다"
        return response

    # ------------------------------------------------------------------
    # 탐색 액션 발행
    # ------------------------------------------------------------------
    def send_search_goal(self, name, label):
        """탐색 goal 을 보낸다. 수락되면 goal handle, 아니면 None."""
        goal = Search.Goal()
        goal.target_name = name
        goal.class_label = label

        send_future = self.search_cli.send_goal_async(
            goal, feedback_callback=self.on_search_feedback)

        # 서비스 콜백 안이라 spin_until_future_complete 를 쓰면 교착이 생긴다
        deadline = time.time() + self.goal_accept_timeout
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.02)

        if not send_future.done():
            self.get_logger().error('탐색 goal 응답 시간 초과')
            return None

        handle = send_future.result()
        if not handle.accepted:
            return None

        # 결과가 오면 IDLE 로 되돌린다
        handle.get_result_async().add_done_callback(self.on_search_result)
        return handle

    def on_search_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  탐색 중: {fb.step} ({fb.progress * 100:.0f}%)')

    def on_search_result(self, future):
        """탐색이 끝나면 호출된다. 성공/실패/취소 모두 여기로 온다."""
        self.current_goal = None

        try:
            res = future.result().result
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f'탐색 결과 수신 실패: {e}')
            self.set_state(IDLE, '탐색 실패')
            return

        if res.success:
            self.get_logger().info(f'탐색 완료: {res.message} @ {res.location}')
        else:
            self.get_logger().warn(f'탐색 실패: {res.message}')

        self.set_state(IDLE, '탐색 종료')

    # ------------------------------------------------------------------
    def check_one(self, name):
        """노드 하나에 준비 확인을 보낸다. 준비됐으면 True."""
        cli = self.init_clients[name]

        if not cli.wait_for_service(timeout_sec=self.wait_timeout):
            self.get_logger().warn(f'[{name}] 서비스 없음 (/{name}/init)')
            return False

        req = NodeInit.Request()
        req.request = json.dumps({'node': 'state_node'}, ensure_ascii=False)

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.wait_timeout)

        if not future.done():
            self.get_logger().warn(f'[{name}] 응답 시간 초과')
            return False

        res = future.result()
        if not res.success:
            self.get_logger().warn(f'[{name}] 준비 안 됨 — {res.message}')
            return False

        self.get_logger().info(f'[{name}] {res.message}')

        # 노드별 상태 JSON. 내용은 노드마다 다르므로 로그로만 남긴다
        try:
            status = json.loads(res.response) if res.response else {}
            if status:
                self.get_logger().info(f'       {status}')
        except json.JSONDecodeError:
            self.get_logger().warn(f'[{name}] 상태 JSON 파싱 실패')

        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateNode()
        # 서비스 콜백 안에서 액션 goal 을 보내야 해서 멀티스레드가 필요하다
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'state_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()