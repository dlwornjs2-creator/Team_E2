"""Request/response bridge for the future Any6D detector node."""

from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import SearchConfig
from .models import RobotTask


class DetectionClient:
    """Publish targeted detection requests and wait for matching results."""

    def __init__(self, node: Node, config: SearchConfig) -> None:
        self.node = node
        self.config = config
        self._lock = threading.Lock()
        self._responses: dict[str, bool] = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = node.create_publisher(
            String, config.detection_request_topic, qos
        )
        self.subscription = node.create_subscription(
            String,
            config.detection_result_topic,
            self._on_result,
            qos,
        )

    def _on_result(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            request_id = str(payload["request_id"]).strip()
            detected = payload["detected"]
            if not request_id or not isinstance(detected, bool):
                raise ValueError("request_id와 boolean detected가 필요합니다")
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.node.get_logger().error(
                f"Invalid Any6D detection result: {error}"
            )
            return
        with self._lock:
            self._responses[request_id] = detected

    def request_detection(
        self,
        task: RobotTask,
        zone: int,
        timeout_sec: float,
    ) -> bool:
        request_id = f"{task.task_id}:zone-{zone}"
        with self._lock:
            self._responses.pop(request_id, None)

        request = String()
        request.data = json.dumps(
            {
                "request_id": request_id,
                "task_id": task.task_id,
                "search_zone": zone,
                "name": task.name,
                "class_label": task.class_label,
            },
            ensure_ascii=False,
        )
        self.publisher.publish(request)

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                if request_id in self._responses:
                    return self._responses.pop(request_id)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().warning(
            f"Any6D detection timeout: request_id={request_id}"
        )
        return False
