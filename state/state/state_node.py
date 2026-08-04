"""상태 관리 노드 — 1단계.

지금은 부팅 시 다른 노드가 올라왔는지 확인하는 것까지만 한다.
명령 처리는 아직 없다.

순서도 대응:
  시작 -> State = LOAD -> 각 노드에 상태 확인 요청
       -> 성공? -> No 면 재시도 / Yes 면 State = IDLE -> 대기

상태
  LOAD  부팅 중. 다른 노드 준비를 기다리는 중
  IDLE  대기. 명령을 받을 수 있는 상태
  RUN   작업 중. (아직 사용하지 않음)

발행
  /state/current  std_msgs/String  현재 상태 (JSON)

사용
  /<노드>/init    interfaces/NodeInit  각 노드의 준비 확인
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from interfaces.srv import NodeInit


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

        self.targets = list(self.get_parameter('targets').value)
        self.wait_timeout = self.get_parameter('wait_timeout').value
        self.retry_period = self.get_parameter('retry_period').value
        self.max_retries = self.get_parameter('max_retries').value

        self.state = LOAD
        self.ready = {name: False for name in self.targets}

        # 현재 상태 발행. 늦게 뜨는 노드도 마지막 상태를 바로 받도록 TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, 'state/current',
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.init_clients = {
            name: self.create_client(NodeInit, f'{name}/init')
            for name in self.targets
        }

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
        rclpy.spin(node)
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