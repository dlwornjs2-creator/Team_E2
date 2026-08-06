"""back_ui 진입점. `ros2 run back_ui node`로 실행한다.

GitHub ui 브랜치의 `/dsr01/joint_states` + FK + HTTP 구조를 그대로 유지하고,
어제 검증한 `/ui/task_state` JSON 구독만 추가했다.
"""

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from back_ui.frame_store import FrameStore
from back_ui.http_server import start_server
from back_ui.robot_fk import compute_links
from back_ui.state_store import StateStore

_JOINT_UPDATE_MIN_INTERVAL = 0.1


class BackUiNode(Node):
    def __init__(self):
        super().__init__("back_ui")

        # GitHub 기존 구조
        self.state_store = StateStore()
        self.frame_store = FrameStore()
        self._httpd = start_server(self.state_store, self.frame_store)

        addr, port = self._httpd.server_address
        self.get_logger().info(
            f"back_ui HTTP 서버 시작: http://{addr}:{port}"
        )

        self.declare_parameter("robot_name", "dsr01")
        robot_name = (
            self.get_parameter("robot_name")
            .get_parameter_value()
            .string_value
        )
        joint_states_topic = f"/{robot_name}/joint_states"

        self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            10,
        )
        self.get_logger().info(f"구독 시작: {joint_states_topic}")
        self._last_joint_update = 0.0

        # 어제 테스트한 UI 상태 토픽 추가
        self.create_subscription(
            String,
            "/ui/task_state",
            self._on_task_state,
            10,
        )
        self.get_logger().info("구독 시작: /ui/task_state")

    def _on_joint_states(self, msg: JointState):
        now = time.monotonic()
        if now - self._last_joint_update < _JOINT_UPDATE_MIN_INTERVAL:
            return

        self._last_joint_update = now
        angles = dict(zip(msg.name, msg.position))
        self.state_store.update_robot_links(compute_links(angles))

    def _on_task_state(self, msg: String):
        """String 안의 JSON을 task 상태로 반영한다."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"/ui/task_state JSON 형식 오류: {error}")
            return

        if not isinstance(data, dict):
            self.get_logger().error(
                "/ui/task_state 최상위 JSON은 객체여야 합니다."
            )
            return

        self.state_store.update_task(
            task_id=data.get("task_id"),
            target_id=data.get("target_id"),
            target_name=data.get("target_name"),
            voice_command=data.get("voice_command"),
            status=data.get("status"),
            stage=data.get("stage"),
            action=data.get("action"),
            action_reason=data.get("action_reason"),
            elapsed_sec=data.get("elapsed_sec"),
            current_zone=data.get("current_zone"),
            detections=data.get("detections"),
        )

        self.get_logger().info(
            "UI 상태 갱신: "
            f"action={data.get('action')}, "
            f"reason={data.get('action_reason')}"
        )

    def destroy_node(self):
        self._httpd.shutdown()
        self._httpd.server_close()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
