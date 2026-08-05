"""Top-level ROS 2 control node and task orchestrator."""

from __future__ import annotations

import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from .config import AppConfig, DEFAULT_CONFIG
from .db_client import DBClient
from .detection_client import DetectionClient
from .models import (
    DBLookupError,
    GripperError,
    PoseValidationError,
    RobotTask,
    TargetPose,
    TaskOutcome,
)
from .motion_executor import MotionExecutor
from .pose_provider import PoseProvider, create_pose_provider
from .state_interface import StateInterface


class RobotControlNode(Node):
    def __init__(self, config: AppConfig = DEFAULT_CONFIG) -> None:
        """
        [초기화 함수]
        ROS 2 노드를 생성하고 로봇 제어에 필요한 기본 설정들을 초기화합니다.
        - 모션(Motion) 제어기는 일단 None으로 두고 나중에 API를 바인딩할 때 생성합니다.
        - Any6D 포즈(자세) 제공자, DB 통신 클라이언트, 그리고 상태(State) 관리 인터페이스를 설정합니다.
        """
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
        self.detector = DetectionClient(self, config.search)
        self.state = StateInterface(
            self,
            config.interface,
            status_supplier=self._status_payload,
            acceptance_guard=self._acceptance_guard,
        )

    def bind_dsr_api(self, dsr_api) -> None:
        """
        [로봇 API 바인딩 함수]
        두산 로봇(DSR) 초기화(DR_init)가 완료된 후 호출됩니다.
        전달받은 로봇 하드웨어 API를 사용하여 실제 로봇 동작을 담당하는 모션 제어기(MotionExecutor)를 생성합니다.
        """
        self.motion = MotionExecutor(
            self,
            dsr_api,
            self.config.robot,
            self.config.motion,
            self.config.gripper,
            self.config.search,
        )

    def initialize_hardware(self) -> None:
        """
        [하드웨어 초기화 함수]
        생성된 모션 제어기를 통해 로봇 하드웨어를 설정(configure)하고 초기화(initialize)합니다.
        초기화가 끝나면 로봇 상태를 '준비 완료(Ready)'로 변경하고 관련 토픽 정보를 로그로 남깁니다.
        """
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
        """
        [상태 정보 제공 헬퍼 함수]
        현재 로봇의 상태 정보(모션 활성화 여부, 동작 중인지 여부, 물체를 쥐고 있는지 여부 등)를
        딕셔너리 형태로 묶어서 반환합니다. 외부에서 로봇 상태를 요청할 때 사용됩니다.
        """
        return {
            "motion_enabled": self.config.robot.enable_motion,
            "robot_busy": self.motion.busy if self.motion else False,
            "holding_object": (
                self.motion.holding_object if self.motion else False
            ),
            "target_input_mode": self.config.pose.input_mode,
            "pose_topic": self.config.pose.topic,
            "db_service": self.config.interface.db_load_service,
            "detection_request_topic": (
                self.config.search.detection_request_topic
            ),
        }

    def _acceptance_guard(self) -> Optional[str]:
        """
        [작업 수락 검사 헬퍼 함수]
        새로운 작업을 수락해도 되는 상태인지 검사합니다.
        작업을 받을 수 없는 경우(하드웨어 미연결, 이미 물체를 들고 있는 경우)에는 거절 사유(문자열)를 반환하고,
        작업을 받을 수 있다면 None을 반환합니다.
        """
        if self.motion is None:
            return "로봇 하드웨어가 초기화되지 않았습니다"
        if self.motion.holding_object:
            return "현재 그리퍼가 물체를 들고 있어 새 작업을 받을 수 없습니다"
        return None

    def execute_task(self, task: RobotTask) -> None:
        """
        [작업 실행 메인 로직 함수]
        단일 로봇 작업(Task)을 전체 흐름에 따라 실행합니다.
        1. DB 조회: 찾고자 하는 물체의 마지막 위치를 DB에서 불러옵니다.
        2. 포즈(Any6D) 대기: 카메라나 비전 시스템으로부터 타겟 물체의 정확한 위치(Pose)를 기다립니다.
        3. 로봇 동작: 전달받은 포즈를 향해 로봇을 움직여 물체를 집고(Pick), 다시 홈 위치로 복귀(Return home)합니다.
        4. 상태 업데이트: 작업 도중 또는 완료 시 성공/실패 여부를 상태 인터페이스를 통해 지속적으로 발행(Publish)합니다.
        """
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

            # DB 조회 단계
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
                not_found_message = (
                    f"{self.config.pose.wait_timeout_sec:.1f}초 동안 "
                    "유효한 Any6D 자세를 받지 못했습니다"
                )
            else:
                self.get_logger().warning(
                    f"Task {task.task_id}: no DB location; starting search"
                )
                target = self._search_unknown_object(task, db_payload)
                not_found_message = (
                    "모든 탐색 구역에서 물체를 찾지 못했거나 "
                    "유효한 Any6D 자세를 받지 못했습니다"
                )
            if target is None:  # 시간 내에 비전 정보를 받지 못한 경우
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.NOT_FOUND,
                    message=not_found_message,
                    extra={"db": db_payload},
                )
                return

            # 실제 로봇 구동(Pick) 및 홈 복귀 단계
            success = self.motion.pick_and_return_home(target)
            if not success:  # 드라이런(가상 테스트) 모드 등 실제 동작을 안한 경우
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.DRY_RUN,
                    message="드라이런 모드이므로 로봇 동작을 실행하지 않았습니다",
                    extra={"db": db_payload},
                )
                return

            # 성공적으로 작업을 마쳤을 경우
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
            # 예상된 에러 처리 (DB 조회 실패, 잘못된 자세 정보, 그리퍼 에러 등)
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
            # 예상치 못한 시스템 에러 처리
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
            # 성공/실패 여부와 상관없이 작업을 종료 상태로 처리
            self.state.finish_task(task)
            self.get_logger().info("Waiting for the next state-node task")

    def _search_unknown_object(
        self,
        task: RobotTask,
        db_payload: dict[str, Any],
    ) -> Optional[TargetPose]:
        """Search four viewpoints and return the first detected pose."""
        if self.motion is None:
            raise RuntimeError("로봇 하드웨어가 초기화되지 않았습니다")

        for zone in range(1, 5):
            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.SEARCHING,
                message=f"탐색 {zone}구역으로 이동합니다",
                extra={"db": db_payload, "search_zone": zone},
            )
            self.motion.move_to_search_zone(zone)
            self.pose_provider.reset_for_task()
            detected = self.detector.request_detection(
                task,
                zone,
                self.config.search.detection_timeout_sec,
            )
            if not detected:
                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.ZONE_NOT_FOUND,
                    message=f"탐색 {zone}구역에서 물체를 찾지 못했습니다",
                    extra={"db": db_payload, "search_zone": zone},
                )
            else:
                target = self.pose_provider.wait_for_target(
                    self.config.search.pose_timeout_sec
                )
                if target is not None:
                    return target
                self.get_logger().warning(
                    f"Zone {zone}: detector reported found but no valid pose "
                    "arrived"
                )

            self._observe_landmark(task, zone, db_payload)

        self.motion.move_home()
        return None

    def _observe_landmark(
        self,
        task: RobotTask,
        zone: int,
        db_payload: dict[str, Any],
    ) -> None:
        """Look for configured landmarks after the requested target is absent."""
        candidates = self.config.search.landmark_targets
        candidate_names = [name for name, _ in candidates]
        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.LANDMARK_SEARCHING,
            message="녹색 상자 또는 회색 수납장을 탐지합니다",
            extra={
                "db": db_payload,
                "search_zone": zone,
                "landmark_candidates": candidate_names,
            },
        )
        found = self.detector.request_detection(
            task,
            zone,
            self.config.search.detection_timeout_sec,
            request_kind="landmark",
            candidates=candidates,
        )
        if not found:
            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.LANDMARK_NOT_FOUND,
                message="랜드마크를 찾지 못해 다음 탐색구역으로 이동합니다",
                extra={"db": db_payload, "search_zone": zone},
            )
            return

        dwell_sec = self.config.search.landmark_dwell_sec
        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.LANDMARK_FOUND,
            message=f"랜드마크를 찾아 {dwell_sec:.1f}초 대기합니다",
            extra={"db": db_payload, "search_zone": zone},
        )
        time.sleep(dwell_sec)

    def run(self) -> None:
        """
        [메인 실행 루프 함수]
        노드가 살아있는 동안 지속적으로 반복 실행되는 루프입니다.
        1. rclpy.spin_once를 통해 ROS 2 이벤트 및 콜백을 처리합니다.
        2. 상태 인터페이스로부터 대기 중인 다음 작업(Task)이 있는지 확인하고 가져옵니다.
        3. 새 작업이 있다면 execute_task 함수를 호출해 실행합니다.
        """
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
        """
        [하드웨어 종료 함수]
        노드가 종료되거나 시스템이 멈출 때 호출되며,
        로봇 하드웨어 및 모션 제어기를 안전하게 종료(Shutdown)합니다.
        """
        if self.motion is not None:
            self.motion.shutdown()
