"""ROS 2 service client for the Any6D detector node."""

from __future__ import annotations

import json
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from interfaces.srv import DetectObject
from rclpy.node import Node

from .config import SearchConfig
from .models import DetectionResult, RobotTask


class DetectionClient:
    """Request target or landmark detection through one ROS 2 service."""

    def __init__(self, node: Node, config: SearchConfig) -> None:
        self.node = node
        self.config = config
        self.client = node.create_client(DetectObject, config.detection_service)
        self._sequence = 0

    @property
    def sequence(self) -> int:
        return self._sequence

    def request_detection(
        self,
        task: RobotTask,
        zone: int,
        timeout_sec: float,
        *,
        request_kind: str = "target",
        candidates: tuple[tuple[str, str], ...] = (),
    ) -> DetectionResult:
        if not self.client.wait_for_service(
            timeout_sec=self.config.detection_service_wait_timeout_sec
        ):
            self.node.get_logger().warning(
                f"Any6D service unavailable: {self.config.detection_service}"
            )
            return DetectionResult(False, None)

        suffix = "" if request_kind == "target" else f":{request_kind}"
        request_id = f"{task.task_id}:zone-{zone}{suffix}"
        object_name = task.class_label or task.name
        payload: dict[str, Any] = {
            "request_id": request_id,
            "request_type": request_kind,
            "task_id": task.task_id,
            "search_zone": zone,
            "object_name": object_name,
            "name": object_name,
            "class_label": object_name,
        }
        if candidates:
            payload["candidate_targets"] = [
                {
                    "object_name": class_label,
                    "name": class_label,
                    "class_label": class_label,
                }
                for name, class_label in candidates
            ]

        request = DetectObject.Request()
        request.request = json.dumps(payload, ensure_ascii=False)
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, max(0.0, remaining)),
            )

        if not future.done():
            future.cancel()
            self.node.get_logger().warning(
                f"Any6D service timeout: request_id={request_id}"
            )
            return DetectionResult(False, None)
        if future.exception() is not None:
            self.node.get_logger().error(
                f"Any6D service failed: {future.exception()}"
            )
            return DetectionResult(False, None)

        response = future.result()
        if response is None or not response.success:
            message = response.message if response else "empty response"
            self.node.get_logger().warning(f"Any6D detection failed: {message}")
            return DetectionResult(False, None)
        return self._parse_response(response.response, request_id)

    def _parse_response(self, raw: str, request_id: str) -> DetectionResult:
        try:
            payload = json.loads(raw) if raw else {}
            detected = payload["detected"]
            if not isinstance(detected, bool):
                raise ValueError("detected must be boolean")
            response_id = str(payload.get("request_id", request_id))
            if response_id != request_id:
                raise ValueError(
                    f"request_id mismatch: {response_id} != {request_id}"
                )
            pose = self._parse_pose(payload.get("pose"))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.node.get_logger().error(
                f"Invalid Any6D service response: {error}"
            )
            return DetectionResult(False, None)

        self._sequence += 1
        return DetectionResult(
            detected=detected,
            pose=pose,
            detected_name=str(payload.get("detected_name", "")),
            detected_class_label=str(
                payload.get("detected_class_label", "")
            ),
        )

    def _parse_pose(self, data: Any) -> Optional[PoseStamped]:
        if data is None:
            return None
        if not isinstance(data, dict):
            raise ValueError("pose must be an object")
        position = data.get("position", {})
        orientation = data.get("orientation", {})
        pose = PoseStamped()
        pose.header.frame_id = str(data.get("frame_id", "")).strip()
        if not pose.header.frame_id:
            raise ValueError("pose.frame_id is required")
        stamp = data.get("stamp", {})
        pose.header.stamp.sec = int(stamp.get("sec", 0))
        pose.header.stamp.nanosec = int(stamp.get("nanosec", 0))
        pose.pose.position.x = float(position["x"])
        pose.pose.position.y = float(position["y"])
        pose.pose.position.z = float(position["z"])
        pose.pose.orientation.x = float(orientation["x"])
        pose.pose.orientation.y = float(orientation["y"])
        pose.pose.orientation.z = float(orientation["z"])
        pose.pose.orientation.w = float(orientation["w"])
        return pose
