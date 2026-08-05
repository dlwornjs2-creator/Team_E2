"""Eye-in-hand camera pose conversion for Any6D detections."""

from __future__ import annotations

import numpy as np

from .models import PoseValidationError
from .pose_utils import drl_posx_to_matrix, validate_homogeneous_matrix


class CameraToBaseTransformer:
    """Convert T_camera_object into T_base_object using the current TCP pose."""

    def __init__(
        self,
        tcp_to_camera: tuple[tuple[float, ...], ...],
        accepted_camera_frames: tuple[str, ...],
        camera_position_scale_to_mm: float,
    ) -> None:
        self.tcp_to_camera = validate_homogeneous_matrix(
            np.asarray(tcp_to_camera, dtype=float),
            "T_tcp_camera",
        )
        self.accepted_camera_frames = accepted_camera_frames
        if (
            not np.isfinite(camera_position_scale_to_mm)
            or camera_position_scale_to_mm <= 0
        ):
            raise ValueError("camera_position_scale_to_mm must be positive")
        self.camera_position_scale_to_mm = camera_position_scale_to_mm

    def transform(
        self,
        camera_object: np.ndarray,
        base_tcp_posx: list[float],
        frame_id: str,
    ) -> np.ndarray:
        if frame_id not in self.accepted_camera_frames:
            expected = ", ".join(self.accepted_camera_frames)
            raise PoseValidationError(
                f"Rejected camera frame '{frame_id}'; expected one of: {expected}"
            )
        base_tcp = drl_posx_to_matrix(base_tcp_posx)
        camera_object = validate_homogeneous_matrix(
            camera_object,
            "T_camera_object",
        )
        camera_object[:3, 3] *= self.camera_position_scale_to_mm
        return validate_homogeneous_matrix(
            base_tcp @ self.tcp_to_camera @ camera_object,
            "T_base_object",
        )
