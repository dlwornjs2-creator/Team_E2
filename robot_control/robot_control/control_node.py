"""Top-level ROS 2 control node and task orchestrator."""

from __future__ import annotations

from typing import Any, Optional

import rclpy
from rclpy.node import Node

from .config import AppConfig, DEFAULT_CONFIG
from .db_client import DBClient
from .models import (
    DBLookupError,
    GripperError,
    PoseValidationError,
    RobotTask,
    TaskOutcome,
)
from .motion_executor import MotionExecutor
from .pose_provider import PoseProvider, create_pose_provider
from .state_interface import StateInterface


class RobotControlNode(Node):
    def __init__(self, config: AppConfig = DEFAULT_CONFIG) -> None:
        super().__init__(
            "robot_control_any6d",
            namespace=config.robot.robot_id,
        )
        self.config = config
        self.motion: Optional[MotionExecutor] = None
        self.pose_provider: PoseProvider = create_pose_provider(
            self,
            config.pose,
            enable_motion=config.robot.enable_motion,
        )
        self.db = DBClient(self, config.interface)
        self.state = StateInterface(
            self,
            config.interface,
            status_supplier=self._status_payload,
            acceptance_guard=self._acceptance_guard,
        )

    def bind_dsr_api(self, dsr_api) -> None:
        """Create the hardware layer after DR_init has received this node."""
        self.motion = MotionExecutor(
            self,
            dsr_api,
            self.config.robot,
            self.config.motion,
            self.config.gripper,
        )

    def initialize_hardware(self) -> None:
        if self.motion is None:
            raise RuntimeError("DSR_ROBOT2 is not bound")
        self.motion.configure()
        self.motion.initialize()
        self.state.mark_ready()
        interface = self.config.interface
        self.get_logger().info(
            f"Control ready: init={interface.control_init_service}, "
            f"request={interface.state_task_topic}, "
            f"result={interface.control_result_topic}"
        )

    def _status_payload(self) -> dict[str, Any]:
        return {
            "motion_enabled": self.config.robot.enable_motion,
            "robot_busy": self.motion.busy if self.motion else False,
            "holding_object": (
                self.motion.holding_object if self.motion else False
            ),
            "target_input_mode": self.config.pose.input_mode,
            "pose_topic": self.config.pose.topic,
            "db_service": self.config.interface.db_load_service,
        }

    def _acceptance_guard(self) -> Optional[str]:
        if self.motion is None:
            return "로봇 하드웨어가 초기화되지 않았습니다"
        if self.motion.holding_object:
            return "현재 그리퍼가 물체를 들고 있어 새 작업을 받을 수 없습니다"
        return None

    def execute_task(self, task: RobotTask) -> None:
        """DB lookup -> fresh Any6D pose -> pick -> terminal response."""
        db_result = None
        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.DB_LOOKUP,
            message="DB에서 물체의 마지막 위치를 조회합니다",
        )

        try:
            if self.motion is None:
                raise RuntimeError("로봇 하드웨어가 초기화되지 않았습니다")
            if self.motion.holding_object:
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.BLOCKED_HOLDING_OBJECT,
                    message=(
                        "이전 물체를 들고 있습니다. 내려놓기 동작 또는 "
                        "그리퍼 해제 후 다시 요청해야 합니다"
                    ),
                )
                return

            db_result = self.db.lookup(task)
            db_payload = db_result.to_payload()
            if db_result.location_known and db_result.item is not None:
                location = str(db_result.item.get("location", ""))
                self.get_logger().info(
                    f"Task {task.task_id}: DB location='{location}'"
                )
                wait_message = (
                    f"DB 위치 '{location}' 확인; 새로운 Any6D 자세를 기다립니다"
                )
            else:
                self.get_logger().warning(
                    f"Task {task.task_id}: no DB location; "
                    "waiting for Any6D at the current view"
                )
                wait_message = (
                    "DB에 저장된 위치가 없어 현재 시야에서 Any6D 자세를 기다립니다"
                )

            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.WAITING_POSE,
                message=wait_message,
                extra={"db": db_payload},
            )

            self.pose_provider.reset_for_task()
            target = self.pose_provider.wait_for_target(
                self.config.pose.wait_timeout_sec
            )
            if target is None:
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.NOT_FOUND,
                    message=(
                        f"{self.config.pose.wait_timeout_sec:.1f}초 동안 "
                        "유효한 Any6D 자세를 받지 못했습니다"
                    ),
                    extra={"db": db_payload},
                )
                return

            success = self.motion.pick_and_return_home(target)
            if not success:
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.DRY_RUN,
                    message="드라이런 모드이므로 로봇 동작을 실행하지 않았습니다",
                    extra={"db": db_payload},
                )
                return

            self.state.publish_event(
                task,
                status="completed",
                success=True,
                outcome=TaskOutcome.PICK_COMPLETED,
                message="물체를 든 상태로 홈 복귀를 완료했습니다",
                extra={
                    "db": db_payload,
                    "pose_sequence": target.source_sequence,
                },
            )

        except (
            DBLookupError,
            PoseValidationError,
            GripperError,
            RuntimeError,
        ) as error:
            self.get_logger().error(f"Task {task.task_id} failed: {error}")
            extra = {"db": db_result.to_payload()} if db_result else None
            self.state.publish_event(
                task,
                status="completed",
                success=False,
                outcome=TaskOutcome.FAILED,
                message=str(error),
                extra=extra,
            )
        except Exception as error:
            self.get_logger().error(
                f"Task {task.task_id} unexpected failure: {error}"
            )
            self.state.publish_event(
                task,
                status="completed",
                success=False,
                outcome=TaskOutcome.FAILED,
                message=f"예상하지 못한 오류: {error}",
            )
        finally:
            self.state.finish_task(task)
            self.get_logger().info("Waiting for the next state-node task")

    def run(self) -> None:
        interface = self.config.interface
        self.get_logger().info(
            f"Waiting for state-node tasks on {interface.state_task_topic}"
        )
        if self.config.pose.input_mode == "any6d":
            self.get_logger().info(
                f"Any6D pose topic: {self.config.pose.topic}"
            )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            task = self.state.take_next_task()
            if task is not None:
                self.execute_task(task)

    def shutdown_hardware(self) -> None:
        if self.motion is not None:
            self.motion.shutdown()
