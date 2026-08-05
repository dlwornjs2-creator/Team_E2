"""Pure pose-validation and conversion helpers."""

from __future__ import annotations

import warnings

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from .models import PoseValidationError


def validate_homogeneous_matrix(matrix: np.ndarray, name: str) -> np.ndarray:
    """Validate a 4x4 rigid transform and return a defensive float copy."""
    transform = np.asarray(matrix, dtype=float)
    if transform.shape != (4, 4):
        raise PoseValidationError(
            f"{name} must have shape (4, 4), got {transform.shape}"
        )
    if not np.all(np.isfinite(transform)):
        raise PoseValidationError(f"{name} contains NaN or infinity")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise PoseValidationError(f"{name} has an invalid homogeneous last row")

    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise PoseValidationError(f"{name} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3):
        raise PoseValidationError(f"{name} rotation determinant is not +1")
    return transform.copy()


def matrix_to_drl_posx(matrix: np.ndarray) -> list[float]:
    """Convert a base-frame matrix to Doosan [X, Y, Z, A, B, C]."""
    transform = validate_homogeneous_matrix(matrix, "target matrix")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        abc = Rotation.from_matrix(transform[:3, :3]).as_euler(
            "ZYZ",
            degrees=True,
        )
    return [
        float(transform[0, 3]),
        float(transform[1, 3]),
        float(transform[2, 3]),
        float(abc[0]),
        float(abc[1]),
        float(abc[2]),
    ]


def drl_posx_to_matrix(pose: list[float]) -> np.ndarray:
    """Convert Doosan [X, Y, Z, A, B, C] to a homogeneous matrix."""
    if len(pose) != 6 or not np.all(np.isfinite(pose)):
        raise PoseValidationError("Doosan pose must contain six finite values")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_euler(
        "ZYZ",
        pose[3:6],
        degrees=True,
    ).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def pose_stamped_to_matrix(message: PoseStamped) -> np.ndarray:
    """Reconstruct the frame-to-object transform from PoseStamped."""
    position = message.pose.position
    orientation = message.pose.orientation
    quaternion = np.array(
        [orientation.x, orientation.y, orientation.z, orientation.w],
        dtype=float,
    )
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-8:
        raise PoseValidationError("Any6D quaternion is invalid")
    quaternion /= norm

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = [position.x, position.y, position.z]
    return validate_homogeneous_matrix(transform, "T_frame_object")


def rotation_distance_deg(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular distance between rotation matrices."""
    relative = Rotation.from_matrix(first).inv() * Rotation.from_matrix(second)
    return float(np.degrees(relative.magnitude()))
