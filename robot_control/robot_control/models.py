"""Shared data models and domain errors."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped


class TaskOutcome(str, Enum):
    QUEUED = "queued"
    DB_LOOKUP = "db_lookup"
    WAITING_POSE = "waiting_pose"
    SEARCHING = "searching"
    ZONE_NOT_FOUND = "zone_not_found"
    LANDMARK_SEARCHING = "landmark_searching"
    LANDMARK_FOUND = "landmark_found"
    LANDMARK_NOT_FOUND = "landmark_not_found"
    PICK_COMPLETED = "pick_completed"
    NOT_FOUND = "not_found"
    DRY_RUN = "dry_run"
    REJECTED = "rejected"
    BLOCKED_HOLDING_OBJECT = "blocked_holding_object"
    FAILED = "failed"


class PoseValidationError(RuntimeError):
    """A received or calculated robot pose is invalid or unsafe."""


class GripperError(RuntimeError):
    """The gripper timed out or reported a safety condition."""


class DBLookupError(RuntimeError):
    """The DB service is missing or returned an invalid response."""


@dataclass(frozen=True)
class TargetPose:
    matrix: np.ndarray
    posx: list[float]
    source_sequence: int
    grasp_candidates: tuple[tuple[str, np.ndarray], ...] = ()


@dataclass(frozen=True)
class DetectionResult:
    detected: bool
    pose: Optional[PoseStamped]
    detected_name: str = ""
    detected_class_label: str = ""


@dataclass(frozen=True)
class RobotTask:
    task_id: str
    name: str
    class_label: str
    requested_by: str
    command: str = "pick"


@dataclass(frozen=True)
class DBLookupResult:
    location_known: bool
    query: dict[str, str]
    item: Optional[dict[str, Any]]

    def to_payload(self) -> dict[str, Any]:
        return {
            "location_known": self.location_known,
            "query": self.query,
            "item": self.item,
        }
