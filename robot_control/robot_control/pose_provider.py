"""Manual and Any6D target-pose providers."""

from __future__ import annotations

import os
import threading
import time
from typing import Optional, Protocol

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .config import PoseConfig
from .models import PoseValidationError, TargetPose
from .pose_utils import (
    drl_posx_to_matrix,
    matrix_to_drl_posx,
    pose_stamped_to_matrix,
    rotation_distance_deg,
    validate_homogeneous_matrix,
)


class PoseProvider(Protocol):
    def reset_for_task(self) -> None:
        """Discard poses produced before the current task."""

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        """Wait for one validated target pose."""


class ManualPoseProvider:
    """Read one Doosan TCP pose from standard input for each state task."""

    def __init__(self, node: Node, config: PoseConfig) -> None:
        self.node = node
        self.config = config
        self.sequence = 0

    def reset_for_task(self) -> None:
        return

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        del timeout_sec  # Terminal input intentionally has no automatic timeout.
        while rclpy.ok():
            try:
                raw = input(
                    "\n목표 TCP 자세를 입력하세요 "
                    "[X Y Z A B C] (mm, degree / q: 종료): "
                ).strip()
            except EOFError:
                return None

            if raw.lower() in {"q", "quit", "exit"}:
                return None

            fields = raw.replace(",", " ").split()
            if len(fields) != 6:
                self.node.get_logger().error(
                    "정확히 6개 값을 입력해야 합니다: X Y Z A B C"
                )
                continue

            try:
                pose = [float(value) for value in fields]
                matrix = drl_posx_to_matrix(pose)
            except (ValueError, PoseValidationError) as error:
                self.node.get_logger().error(f"잘못된 목표 자세: {error}")
                continue

            if matrix[2, 3] < self.config.min_depth_mm:
                self.node.get_logger().error(
                    f"목표 Z={matrix[2, 3]:.2f}mm가 "
                    f"최소값 {self.config.min_depth_mm:.2f}mm보다 낮습니다"
                )
                continue

            self.sequence += 1
            return TargetPose(matrix, pose, self.sequence)
        return None


class Any6DPoseProvider:
    """Subscribe to Any6D and expose fresh, stable, base-frame grasp poses."""

    def __init__(
        self,
        node: Node,
        config: PoseConfig,
        *,
        enable_motion: bool,
    ) -> None:
        self.node = node
        self.config = config
        self._lock = threading.Lock()
        self._latest_matrix: Optional[np.ndarray] = None
        self._latest_stamp_ns = 0
        self._latest_sequence = 0
        self._consumed_sequence = 0
        self._stable_samples = 0
        self._warned_zero_stamp = False
        self._object_to_grasp = self._load_object_to_grasp(enable_motion)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = node.create_subscription(
            PoseStamped,
            config.topic,
            self._pose_callback,
            qos,
        )

    def _load_object_to_grasp(self, enable_motion: bool) -> np.ndarray:
        if self.config.pose_is_tcp_grasp:
            return np.eye(4, dtype=float)

        path = self.config.object_to_grasp_npy
        if not path:
            if enable_motion:
                raise RuntimeError(
                    "Robot motion blocked: object_to_grasp_npy is required "
                    "when pose_is_tcp_grasp is False"
                )
            self.node.get_logger().warning(
                "object_to_grasp_npy is empty; using identity in dry-run mode"
            )
            return np.eye(4, dtype=float)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        return validate_homogeneous_matrix(
            np.load(path),
            "T_object_grasp",
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        expected = self.config.expected_base_frame
        if expected and message.header.frame_id != expected:
            self.node.get_logger().error(
                f"Rejected pose frame '{message.header.frame_id}'; "
                f"expected '{expected}'"
            )
            return

        try:
            matrix = pose_stamped_to_matrix(message)
        except PoseValidationError as error:
            self.node.get_logger().error(f"Rejected Any6D pose: {error}")
            return

        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns == 0:
            stamp_ns = self.node.get_clock().now().nanoseconds
            if not self._warned_zero_stamp:
                self.node.get_logger().warning(
                    "Any6D header.stamp is zero; using reception time"
                )
                self._warned_zero_stamp = True

        with self._lock:
            if self._latest_matrix is None:
                self._stable_samples = 1
            else:
                position_delta = np.linalg.norm(
                    matrix[:3, 3] - self._latest_matrix[:3, 3]
                )
                angle_delta = rotation_distance_deg(
                    self._latest_matrix[:3, :3],
                    matrix[:3, :3],
                )
                if (
                    position_delta
                    <= self.config.max_stable_position_delta_mm
                    and angle_delta
                    <= self.config.max_stable_rotation_delta_deg
                ):
                    self._stable_samples += 1
                else:
                    self._stable_samples = 1

            self._latest_matrix = matrix
            self._latest_stamp_ns = stamp_ns
            self._latest_sequence += 1

    def reset_for_task(self) -> None:
        with self._lock:
            self._consumed_sequence = self._latest_sequence
            self._stable_samples = 0

    def _get_fresh_target(self) -> Optional[TargetPose]:
        with self._lock:
            if self._latest_matrix is None:
                return None
            if self._latest_sequence <= self._consumed_sequence:
                return None
            if self._stable_samples < self.config.required_stable_samples:
                return None

            base_object = self._latest_matrix.copy()
            stamp_ns = self._latest_stamp_ns
            sequence = self._latest_sequence
            self._consumed_sequence = sequence

        age_sec = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age_sec < -0.1 or age_sec > self.config.max_age_sec:
            raise PoseValidationError(
                f"Any6D pose is stale or time-invalid: age={age_sec:.3f}s"
            )
        if base_object[2, 3] < self.config.min_depth_mm:
            raise PoseValidationError(
                f"Object Z={base_object[2, 3]:.2f}mm is below "
                f"{self.config.min_depth_mm:.2f}mm"
            )

        base_grasp = validate_homogeneous_matrix(
            base_object @ self._object_to_grasp,
            "T_base_grasp",
        )
        if base_grasp[2, 3] < self.config.min_depth_mm:
            raise PoseValidationError(
                f"Grasp Z={base_grasp[2, 3]:.2f}mm is below "
                f"{self.config.min_depth_mm:.2f}mm"
            )
        return TargetPose(
            base_grasp,
            matrix_to_drl_posx(base_grasp),
            sequence,
        )

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, max(0.0, remaining)),
            )
            try:
                target = self._get_fresh_target()
            except PoseValidationError as error:
                self.node.get_logger().warning(
                    f"Pose blocked while waiting: {error}"
                )
                continue
            if target is not None:
                return target
        return None


def create_pose_provider(
    node: Node,
    config: PoseConfig,
    *,
    enable_motion: bool,
) -> PoseProvider:
    if config.input_mode == "manual":
        return ManualPoseProvider(node, config)
    if config.input_mode == "any6d":
        return Any6DPoseProvider(
            node,
            config,
            enable_motion=enable_motion,
        )
    raise ValueError("Pose input_mode must be 'manual' or 'any6d'")
