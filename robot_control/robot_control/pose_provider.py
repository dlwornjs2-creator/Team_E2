"""Manual input and Any6D service-response pose validation."""

from __future__ import annotations

import os
from typing import Optional, Protocol

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .config import PoseConfig
from .camera_transform import CameraToBaseTransformer
from .models import PoseValidationError, TargetPose
from .pose_utils import (
    drl_posx_to_matrix,
    matrix_to_drl_posx,
    pose_stamped_to_matrix,
    validate_homogeneous_matrix,
)


class PoseProvider(Protocol):
    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        """Get a manual target when manual mode is enabled."""

    def target_from_pose(
        self,
        message: PoseStamped,
        sequence: int,
        base_tcp_posx: Optional[list[float]] = None,
    ) -> TargetPose:
        """Validate a detector-service pose and create a robot target."""


class ManualPoseProvider:
    """Read one Doosan TCP pose from standard input for each state task."""

    def __init__(self, node: Node, config: PoseConfig) -> None:
        self.node = node
        self.config = config
        self.sequence = 0

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        del timeout_sec
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

    def target_from_pose(
        self,
        message: PoseStamped,
        sequence: int,
        base_tcp_posx: Optional[list[float]] = None,
    ) -> TargetPose:
        del message, sequence, base_tcp_posx
        raise RuntimeError("Any6D pose is unavailable in manual input mode")


class Any6DPoseProvider:
    """Validate a pose returned directly by the Any6D detection service."""

    def __init__(
        self,
        node: Node,
        config: PoseConfig,
        *,
        enable_motion: bool,
    ) -> None:
        self.node = node
        self.config = config
        self._camera_to_base = CameraToBaseTransformer(
            config.tcp_to_camera,
            config.accepted_camera_frames,
            config.camera_position_scale_to_mm,
        )
        self._object_to_grasp = self._load_object_to_grasp(enable_motion)

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
        return validate_homogeneous_matrix(np.load(path), "T_object_grasp")

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        del timeout_sec
        raise RuntimeError("Any6D targets must come from /any6d/detect")

    def target_from_pose(
        self,
        message: PoseStamped,
        sequence: int,
        base_tcp_posx: Optional[list[float]] = None,
    ) -> TargetPose:
        if base_tcp_posx is None:
            raise PoseValidationError(
                "Current base-frame TCP pose is required for camera conversion"
            )
        camera_object = pose_stamped_to_matrix(message)
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns:
            age_sec = (
                self.node.get_clock().now().nanoseconds - stamp_ns
            ) / 1e9
            if age_sec < -0.1 or age_sec > self.config.max_age_sec:
                raise PoseValidationError(
                    f"Any6D pose is stale or time-invalid: age={age_sec:.3f}s"
                )
        base_object = self._camera_to_base.transform(
            camera_object,
            base_tcp_posx,
            message.header.frame_id,
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
