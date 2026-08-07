"""Tests for Tool-Z approach and retreat generation."""

from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from robot_control.motion_executor import MotionExecutor
from robot_control.models import TargetPose


def test_tool_z_approach_and_lift_use_same_safe_side():
    executor = object.__new__(MotionExecutor)
    executor.motion_config = SimpleNamespace(
        approach_mode="tool_z",
        tool_insertion_axis=(0.0, 0.0, 1.0),
        approach_distance_mm=50.0,
        lift_distance_mm=70.0,
    )
    target = np.eye(4, dtype=float)
    target[:3, :3] = Rotation.from_euler(
        "y", 90.0, degrees=True
    ).as_matrix()
    target[:3, 3] = [400.0, 20.0, 100.0]
    tool_z_base = target[:3, 2]

    approach = executor._make_approach_matrix(target)
    lift = executor._make_lift_matrix(target)

    assert np.allclose(
        approach[:3, 3],
        target[:3, 3] - tool_z_base * 50.0,
    )
    assert np.allclose(
        lift[:3, 3],
        target[:3, 3] - tool_z_base * 70.0,
    )


def test_base_z_lift_behavior_is_preserved():
    executor = object.__new__(MotionExecutor)
    executor.motion_config = SimpleNamespace(
        approach_mode="base_z",
        lift_distance_mm=50.0,
    )
    target = np.eye(4, dtype=float)
    target[:3, 3] = [400.0, 20.0, 100.0]

    lift = executor._make_lift_matrix(target)

    assert np.allclose(lift[:3, 3], [400.0, 20.0, 150.0])


def test_joint_cost_selects_candidate_with_less_joint_travel():
    class Logger:
        def info(self, message):
            del message

        def warning(self, message):
            del message

    class Node:
        def get_logger(self):
            return Logger()

    class Dsr:
        DR_BASE = 0

        def __init__(self):
            self.calls = 0

        def get_current_posj(self):
            return [0.0] * 6

        def get_current_solution_space(self):
            return 2

        def ikin(self, posx, solution_space, ref=0):
            del posx, solution_space, ref
            self.calls += 1
            return [90.0] * 6 if self.calls == 1 else [10.0] * 6

    first = np.eye(4, dtype=float)
    second = np.eye(4, dtype=float)
    second[0, 3] = 1.0
    executor = object.__new__(MotionExecutor)
    executor.dsr = Dsr()
    executor.node = Node()
    executor.robot_config = SimpleNamespace(enable_motion=True)
    target = TargetPose(
        first,
        [0.0] * 6,
        1,
        (("first", first), ("second", second)),
    )

    selected = executor._select_lowest_joint_cost_target(target)

    assert np.allclose(selected.matrix, second)
