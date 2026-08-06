"""back_ui 진입점. `ros2 run back_ui node`로 실행한다.

GitHub ui 브랜치의 `/dsr01/joint_states` + FK + HTTP 구조를 유지하고,
`/ui/task_state` JSON 구독과 RealSense 컬러 영상 구독을 추가한다.
"""

import json
import time

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from back_ui.frame_store import FrameStore
from back_ui.http_server import start_server
from back_ui.robot_fk import compute_links
from back_ui.state_store import StateStore


_JOINT_UPDATE_MIN_INTERVAL = 0.1


class BackUiNode(Node):
    def __init__(self):
        super().__init__("back_ui")

        # 상태 저장소와 이미지 저장소
        self.state_store = StateStore()
        self.frame_store = FrameStore()

        # HTTP 서버 시작
        self._httpd = start_server(
            self.state_store,
            self.frame_store,
        )

        addr, port = self._httpd.server_address
        self.get_logger().info(
            f"back_ui HTTP 서버 시작: http://{addr}:{port}"
        )

        # -----------------------------------------------------
        # RealSense 컬러 영상 구독
        # -----------------------------------------------------
        self._bridge = CvBridge()

        self._image_subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._on_image,
            10,
        )

        self.get_logger().info(
            "카메라 구독 시작: "
            "/camera/camera/color/image_raw"
        )

        # -----------------------------------------------------
        # 로봇 joint_states 구독
        # -----------------------------------------------------
        self.declare_parameter(
            "robot_name",
            "dsr01",
        )

        robot_name = (
            self.get_parameter("robot_name")
            .get_parameter_value()
            .string_value
        )

        joint_states_topic = (
            f"/{robot_name}/joint_states"
        )

        self._joint_subscription = (
            self.create_subscription(
                JointState,
                joint_states_topic,
                self._on_joint_states,
                10,
            )
        )

        self.get_logger().info(
            f"구독 시작: {joint_states_topic}"
        )

        self._last_joint_update = 0.0

        # -----------------------------------------------------
        # UI 작업 상태 토픽 구독
        # -----------------------------------------------------
        self._task_subscription = (
            self.create_subscription(
                String,
                "/ui/task_state",
                self._on_task_state,
                10,
            )
        )

        self.get_logger().info(
            "구독 시작: /ui/task_state"
        )

    def _on_image(self, msg: Image):
        """ROS Image를 JPEG로 변환해 HTTP 프레임 저장소에 넣는다."""
        try:
            # sensor_msgs/Image → OpenCV BGR 이미지
            frame = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            # OpenCV 이미지 → JPEG
            success, jpeg = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    80,
                ],
            )

            if not success:
                self.get_logger().warning(
                    "JPEG 인코딩에 실패했습니다."
                )
                return

            # /frame.jpg에서 사용할 최신 이미지 저장
            self.frame_store.set_latest(
                jpeg.tobytes()
            )

            # FrameStore의 현재 frame_id 읽기
            frame_id, _ = (
                self.frame_store.get_latest()
            )

            # /state의 frame_id 갱신
            self.state_store.update_frame_id(
                frame_id
            )

            # UI가 카메라 영상을 표시하도록 상태 갱신
            self.state_store.update_system(
                camera_connected=True
            )

        except Exception as error:
            self.get_logger().error(
                f"카메라 이미지 처리 오류: {error}"
            )

    def _on_joint_states(
        self,
        msg: JointState,
    ):
        """관절 상태를 이용해 3D 로봇 링크 좌표를 갱신한다."""
        now = time.monotonic()

        if (
            now - self._last_joint_update
            < _JOINT_UPDATE_MIN_INTERVAL
        ):
            return

        self._last_joint_update = now

        angles = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        links = compute_links(angles)

        self.state_store.update_robot_links(
            links
        )

    def _on_task_state(
        self,
        msg: String,
    ):
        """String 안의 JSON을 UI task 상태로 반영한다."""
        try:
            data = json.loads(msg.data)

        except json.JSONDecodeError as error:
            self.get_logger().error(
                "/ui/task_state JSON 형식 오류: "
                f"{error}"
            )
            return

        if not isinstance(data, dict):
            self.get_logger().error(
                "/ui/task_state 최상위 JSON은 "
                "객체여야 합니다."
            )
            return

        self.state_store.update_task(
            task_id=data.get("task_id"),
            target_id=data.get("target_id"),
            target_name=data.get("target_name"),
            voice_command=data.get(
                "voice_command"
            ),
            status=data.get("status"),
            stage=data.get("stage"),
            action=data.get("action"),
            action_reason=data.get(
                "action_reason"
            ),
            elapsed_sec=data.get(
                "elapsed_sec"
            ),
            current_zone=data.get(
                "current_zone"
            ),
            detections=data.get(
                "detections"
            ),
        )

        self.get_logger().info(
            "UI 상태 갱신: "
            f"action={data.get('action')}, "
            f"reason={data.get('action_reason')}"
        )

    def destroy_node(self):
        """ROS 노드 종료 시 HTTP 서버도 함께 종료한다."""
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