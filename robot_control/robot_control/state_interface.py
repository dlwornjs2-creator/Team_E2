"""State-node request, readiness, task queue, and result interface."""

from __future__ import annotations

import json
import threading
from collections import deque
from typing import Any, Callable, Optional

from interfaces.srv import NodeInit
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from .config import InterfaceConfig
from .models import RobotTask, TaskOutcome


StatusSupplier = Callable[[], dict[str, Any]]
AcceptanceGuard = Callable[[], Optional[str]]


class StateInterface:
    def __init__(
        self,
        node: Node,
        config: InterfaceConfig,
        *,
        status_supplier: StatusSupplier,
        acceptance_guard: AcceptanceGuard,
    ) -> None:
        self.node = node
        self.config = config
        self.status_supplier = status_supplier
        self.acceptance_guard = acceptance_guard
        self._ready = False
        self._lock = threading.Lock()
        self._pending: deque[RobotTask] = deque()
        self._active: Optional[RobotTask] = None
        self._known_task_ids: set[str] = set()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.init_service = node.create_service(
            NodeInit,
            config.control_init_service,
            self._on_init_request,
        )
        self.task_subscription = node.create_subscription(
            String,
            config.state_task_topic,
            self._on_task_request,
            qos,
        )
        self.result_publisher = node.create_publisher(
            String,
            config.control_result_topic,
            qos,
        )

    @property
    def ready(self) -> bool:
        return self._ready

    def mark_ready(self) -> None:
        self._ready = True

    def _queue_status(self) -> tuple[Optional[str], int]:
        with self._lock:
            active_id = self._active.task_id if self._active else None
            return active_id, len(self._pending)

    def _on_init_request(self, request, response):
        try:
            payload = json.loads(request.request) if request.request.strip() else {}
            if not isinstance(payload, dict):
                raise ValueError("request JSON must be an object")
        except (json.JSONDecodeError, ValueError) as error:
            response.success = False
            response.response = "{}"
            response.message = f"초기화 요청 JSON 오류: {error}"
            return response

        active_id, pending_count = self._queue_status()
        status = self.status_supplier()
        status.update(
            {
                "ready": self._ready,
                "node": self.node.get_fully_qualified_name(),
                "active_task_id": active_id,
                "pending_tasks": pending_count,
            }
        )
        requester = str(payload.get("node", "state_node"))
        response.success = self._ready
        response.response = json.dumps(status, ensure_ascii=False)
        response.message = (
            f"제어 노드 준비 완료 / 요청자={requester}"
            if self._ready
            else f"제어 노드 초기화 중 / 요청자={requester}"
        )
        return response

    def _parse_task(self, raw_json: str) -> RobotTask:
        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError as error:
            raise ValueError(f"JSON 파싱 실패: {error}") from error
        if not isinstance(payload, dict):
            raise ValueError("요청은 JSON 객체여야 합니다")

        command = str(payload.get("command", "pick")).strip().lower()
        if command not in {"pick", "search", "search_and_pick"}:
            raise ValueError(f"지원하지 않는 command: {command}")

        name = str(
            payload.get("name") or payload.get("target_name") or ""
        ).strip()
        class_label = str(payload.get("class_label") or "").strip()
        if not name and not class_label:
            raise ValueError("name(target_name) 또는 class_label이 필요합니다")

        task_id = str(payload.get("task_id") or "").strip()
        if not task_id:
            task_id = f"task-{self.node.get_clock().now().nanoseconds}"
        requested_by = str(payload.get("requested_by", "state_node")).strip()
        return RobotTask(
            task_id=task_id,
            name=name,
            class_label=class_label,
            requested_by=requested_by,
            command=command,
        )

    def _on_task_request(self, message: String) -> None:
        try:
            task = self._parse_task(message.data)
        except ValueError as error:
            self.node.get_logger().error(f"Rejected state request: {error}")
            self.publish_event(
                None,
                status="completed",
                success=False,
                outcome=TaskOutcome.REJECTED,
                message=str(error),
            )
            return

        rejection = ""
        with self._lock:
            if not self._ready:
                rejection = "제어 노드 초기화가 완료되지 않았습니다"
            elif task.task_id in self._known_task_ids:
                rejection = f"중복 task_id입니다: {task.task_id}"
            elif len(self._pending) >= self.config.max_pending_tasks:
                rejection = "작업 대기열이 가득 찼습니다"
            else:
                rejection = self.acceptance_guard() or ""

            if not rejection:
                self._known_task_ids.add(task.task_id)
                self._pending.append(task)
                queue_size = len(self._pending)

        if rejection:
            self.node.get_logger().warning(
                f"Rejected task {task.task_id}: {rejection}"
            )
            self.publish_event(
                task,
                status="completed",
                success=False,
                outcome=TaskOutcome.REJECTED,
                message=rejection,
            )
            return

        self.publish_event(
            task,
            status="accepted",
            success=True,
            outcome=TaskOutcome.QUEUED,
            message="작업 요청을 접수했습니다",
            extra={"queue_size": queue_size},
        )

    def publish_event(
        self,
        task: Optional[RobotTask],
        *,
        status: str,
        success: bool,
        outcome: str | TaskOutcome,
        message: str,
        extra: Optional[dict[str, Any]] = None,
    ) -> None:
        outcome_value = (
            outcome.value if isinstance(outcome, TaskOutcome) else outcome
        )
        payload: dict[str, Any] = {
            "task_id": task.task_id if task else None,
            "status": status,
            "success": success,
            "outcome": outcome_value,
            "message": message,
            "stamp_ns": self.node.get_clock().now().nanoseconds,
        }
        if task:
            payload["target"] = {
                "name": task.name,
                "class_label": task.class_label,
            }
            payload["command"] = task.command
            payload["requested_by"] = task.requested_by
        if extra:
            payload.update(extra)

        result = String()
        result.data = json.dumps(payload, ensure_ascii=False)
        self.result_publisher.publish(result)

    def take_next_task(self) -> Optional[RobotTask]:
        with self._lock:
            if self._active is not None or not self._pending:
                return None
            self._active = self._pending.popleft()
            return self._active

    def finish_task(self, task: RobotTask) -> None:
        with self._lock:
            if self._active == task:
                self._active = None
