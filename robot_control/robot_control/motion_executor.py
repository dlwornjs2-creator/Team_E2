"""Doosan motion and OnRobot RG2 execution layer."""

from __future__ import annotations

import time

import numpy as np
from rclpy.node import Node

from .config import GripperConfig, MotionConfig, RobotConfig
from .models import GripperError, TargetPose
from .onrobot import RG
from .pose_utils import matrix_to_drl_posx, validate_homogeneous_matrix


class MotionExecutor:
    def __init__(
        self,
        node: Node,
        dsr_api,
        robot_config: RobotConfig,
        motion_config: MotionConfig,
        gripper_config: GripperConfig,
    ) -> None:
        self.node = node
        self.dsr = dsr_api
        self.robot_config = robot_config
        self.motion_config = motion_config
        self.gripper_config = gripper_config
        self.busy = False
        self.holding_object = False

        if motion_config.approach_mode not in {"base_z", "tool_z"}:
            raise ValueError("approach_mode must be 'base_z' or 'tool_z'")
        axis = np.asarray(motion_config.tool_insertion_axis, dtype=float)
        if not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-8:
            raise ValueError("tool_insertion_axis must be a finite non-zero vector")

        self.gripper = RG(
            gripper_config.name,
            gripper_config.toolchanger_ip,
            gripper_config.toolchanger_port,
        )

    def configure(self) -> None:
        if not self.robot_config.enable_motion:
            return
        if self.robot_config.tool_name:
            self.dsr.set_tool(self.robot_config.tool_name)
        if self.robot_config.tcp_name:
            self.dsr.set_tcp(self.robot_config.tcp_name)

    def move_home(self) -> None:
        self.dsr.movej(
            list(self.robot_config.home_joint),
            vel=self.robot_config.joint_vel,
            acc=self.robot_config.joint_acc,
        )
        self.dsr.mwait()

    def initialize(self) -> None:
        if not self.robot_config.enable_motion:
            self.node.get_logger().warning(
                "Dry-run mode: robot and gripper commands are disabled"
            )
            self.holding_object = False
            return

        self.move_home()
        self.gripper.open_gripper(
            force_val=self.gripper_config.force_tenth_newton
        )
        self._wait_for_gripper(require_grip=False)
        self.holding_object = False

    def _wait_for_gripper(self, *, require_grip: bool) -> None:
        deadline = time.monotonic() + self.gripper_config.timeout_sec
        while time.monotonic() < deadline:
            status = self.gripper.get_status()
            if any(status[2:7]):
                raise GripperError(f"RG2 safety status active: {status}")
            if not status[0]:
                if require_grip and not status[1]:
                    raise GripperError(
                        "RG2 motion finished but grip-detected bit is low"
                    )
                return
            time.sleep(0.1)
        raise GripperError("Timed out while waiting for RG2 motion")

    def _make_approach_matrix(self, target: np.ndarray) -> np.ndarray:
        approach = target.copy()
        if self.motion_config.approach_mode == "base_z":
            approach[2, 3] += self.motion_config.approach_distance_mm
        else:
            axis = np.asarray(
                self.motion_config.tool_insertion_axis,
                dtype=float,
            )
            axis /= np.linalg.norm(axis)
            insertion_direction_base = target[:3, :3] @ axis
            approach[:3, 3] -= (
                insertion_direction_base
                * self.motion_config.approach_distance_mm
            )
        return validate_homogeneous_matrix(approach, "approach matrix")

    def _make_lift_matrix(self, target: np.ndarray) -> np.ndarray:
        lift = target.copy()
        lift[2, 3] += self.motion_config.lift_distance_mm
        return validate_homogeneous_matrix(lift, "lift matrix")

    def pick_and_return_home(self, target: TargetPose) -> bool:
        """Approach, grasp, lift, and return home while holding the object."""
        approach_pos = matrix_to_drl_posx(
            self._make_approach_matrix(target.matrix)
        )
        lift_pos = matrix_to_drl_posx(self._make_lift_matrix(target.matrix))

        self.node.get_logger().info(f"target posx: {target.posx}")
        self.node.get_logger().info(f"approach posx: {approach_pos}")
        self.node.get_logger().info(f"lift posx: {lift_pos}")

        if not self.robot_config.enable_motion:
            self.node.get_logger().warning(
                "Dry-run complete; no hardware motion sent"
            )
            return False

        self.busy = True
        try:
            dr_base = getattr(self.dsr, "DR_BASE", 0)
            dr_abs = getattr(self.dsr, "DR_MV_MOD_ABS", 0)

            self.dsr.movel(
                approach_pos,
                vel=self.motion_config.approach_vel,
                acc=self.motion_config.approach_acc,
                ref=dr_base,
                mod=dr_abs,
            )
            self.dsr.mwait()

            self.dsr.movel(
                target.posx,
                vel=self.motion_config.grasp_vel,
                acc=self.motion_config.grasp_acc,
                ref=dr_base,
                mod=dr_abs,
            )
            self.dsr.mwait()

            self.gripper.close_gripper(
                force_val=self.gripper_config.force_tenth_newton
            )
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True

            self.dsr.movel(
                lift_pos,
                vel=self.motion_config.lift_vel,
                acc=self.motion_config.lift_acc,
                ref=dr_base,
                mod=dr_abs,
            )
            self.dsr.mwait()
            self.move_home()
            self.node.get_logger().info(
                "Pick completed; robot returned home while holding object"
            )
            return True
        finally:
            self.busy = False

    def shutdown(self) -> None:
        try:
            self.gripper.close_connection()
        except Exception as error:  # Best-effort cleanup only.
            self.node.get_logger().warning(f"RG2 disconnect failed: {error}")
