"""Doosan motion and OnRobot RG2 execution layer."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node

from .config import GripperConfig, MotionConfig, RobotConfig, SearchConfig
from .models import GripperError, PoseValidationError, TargetPose
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
        search_config: SearchConfig,
    ) -> None:
        self.node = node
        self.dsr = dsr_api
        self.robot_config = robot_config
        self.motion_config = motion_config
        self.gripper_config = gripper_config
        self.search_config = search_config
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
        if not self.robot_config.enable_motion:
            self.node.get_logger().info("Dry-run: search zone 1 (home)")
            return
        self.dsr.movej(
            list(self.robot_config.home_joint),
            vel=self.robot_config.joint_vel,
            acc=self.robot_config.joint_acc,
        )
        self.dsr.mwait()

    def current_tcp_posx(self) -> list[float]:
        """Return the current TCP pose in the robot base frame."""
        if not self.robot_config.enable_motion:
            raise RuntimeError("Current TCP pose is unavailable in dry-run mode")
        dr_base = getattr(self.dsr, "DR_BASE", 0)
        result = self.dsr.get_current_posx(ref=dr_base)
        if (
            isinstance(result, (tuple, list))
            and len(result) == 2
            and hasattr(result[0], "__len__")
        ):
            pose = result[0]
        else:
            pose = result
        if pose is None or len(pose) != 6:
            raise RuntimeError(f"Invalid get_current_posx result: {result}")
        return [float(value) for value in pose]

    def move_to_search_zone(self, zone: int) -> None:
        """Move to one of the four configured search viewpoints."""
        if zone not in {1, 2, 3, 4}:
            raise ValueError(f"Unsupported search zone: {zone}")
        if not self.robot_config.enable_motion:
            self.node.get_logger().info(f"Dry-run: move to search zone {zone}")
            return

        self.busy = True
        try:
            if zone in {1, 2}:
                self.move_home()
                if zone == 2:
                    self.dsr.movel(
                        [
                            self.search_config.zone2_base_x_mm,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        vel=self.search_config.linear_vel,
                        acc=self.search_config.linear_acc,
                        ref=getattr(self.dsr, "DR_BASE", 0),
                        mod=getattr(self.dsr, "DR_MV_MOD_REL", 1),
                    )
                    self.dsr.mwait()
            else:
                self.dsr.movej(
                    list(self.search_config.zone3_joint),
                    vel=self.robot_config.joint_vel,
                    acc=self.robot_config.joint_acc,
                )
                self.dsr.mwait()
                if zone == 4:
                    self.dsr.movel(
                        [
                            self.search_config.zone4_base_x_mm,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                        vel=self.search_config.linear_vel,
                        acc=self.search_config.linear_acc,
                        ref=getattr(self.dsr, "DR_BASE", 0),
                        mod=getattr(self.dsr, "DR_MV_MOD_REL", 1),
                    )
                    self.dsr.mwait()
        finally:
            self.busy = False

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
        if self.motion_config.approach_mode == "base_z":
            lift[2, 3] += self.motion_config.lift_distance_mm
        else:
            axis = np.asarray(
                self.motion_config.tool_insertion_axis,
                dtype=float,
            )
            axis /= np.linalg.norm(axis)
            insertion_direction_base = target[:3, :3] @ axis
            lift[:3, 3] -= (
                insertion_direction_base
                * self.motion_config.lift_distance_mm
            )
        return validate_homogeneous_matrix(lift, "lift matrix")

    def _select_lowest_joint_cost_target(self, target: TargetPose) -> TargetPose:
        """Select the reachable grasp candidate with minimum joint travel."""
        if not target.grasp_candidates or not self.robot_config.enable_motion:
            return target
        current = np.asarray(self.dsr.get_current_posj(), dtype=float)
        solution_space = int(self.dsr.get_current_solution_space())
        if current.shape != (6,) or not np.all(np.isfinite(current)):
            raise PoseValidationError("Invalid current joint position")
        if solution_space < 0 or solution_space > 7:
            raise PoseValidationError(
                f"Invalid current solution space: {solution_space}"
            )

        evaluated = []
        dr_base = getattr(self.dsr, "DR_BASE", 0)
        for name, matrix in target.grasp_candidates:
            posx = matrix_to_drl_posx(matrix)
            try:
                joints = np.asarray(
                    self.dsr.ikin(posx, solution_space, ref=dr_base),
                    dtype=float,
                )
            except Exception as error:
                self.node.get_logger().warning(
                    f"IK rejected grasp candidate {name}: {error}"
                )
                continue
            if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                self.node.get_logger().warning(
                    f"IK returned invalid joints for {name}: {joints}"
                )
                continue
            delta = (joints - current + 180.0) % 360.0 - 180.0
            cost = float(np.linalg.norm(delta))
            evaluated.append((cost, name, matrix, posx))

        if not evaluated:
            raise PoseValidationError("No reachable grasp candidate from IK")
        cost, name, matrix, posx = min(evaluated, key=lambda item: item[0])
        self.node.get_logger().info(
            f"Selected grasp={name}, joint_cost={cost:.2f} deg"
        )
        return TargetPose(matrix, posx, target.source_sequence)

    def pick_and_return_home(self, target: TargetPose) -> bool:
        """Approach, grasp, lift, and return home while holding the object."""
        target = self._select_lowest_joint_cost_target(target)
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
                "Pick completed; robot returned home while holding object. Starting force release monitoring..."
            )
            self.wait_for_force_release()
            return True
        finally:
            self.busy = False

    def wait_for_force_release(self) -> bool:
        """Monitor external torque at JHOME_POS while holding object until external force releases gripper."""
        if not self.robot_config.enable_motion:
            self.node.get_logger().info("Dry-run: force release monitoring skipped")
            self.holding_object = False
            return True

        self.node.get_logger().info(
            f"Compliance ON, waiting for external force (> {self.motion_config.force_threshold_nm} Nm)..."
        )
        task_compliance_ctrl = getattr(self.dsr, "task_compliance_ctrl", None)
        release_compliance_ctrl = getattr(self.dsr, "release_compliance_ctrl", None)
        get_external_torque = getattr(self.dsr, "get_external_torque", None)

        if task_compliance_ctrl is not None:
            task_compliance_ctrl(stx=list(self.motion_config.compliance_stiffness))

        released = False
        try:
            while rclpy.ok() and self.holding_object and not released:
                if get_external_torque is not None:
                    ext = get_external_torque()
                    if ext:
                        peak = max(abs(v) for v in ext)
                        if peak > self.motion_config.force_threshold_nm:
                            self.node.get_logger().info(
                                f"External force detected: {peak:.2f} Nm -> Opening gripper and returning to ready state"
                            )
                            self.gripper.open_gripper(
                                force_val=self.gripper_config.force_tenth_newton
                            )
                            self._wait_for_gripper(require_grip=False)
                            self.holding_object = False
                            released = True
                            break
                time.sleep(0.05)
        finally:
            if release_compliance_ctrl is not None:
                release_compliance_ctrl()

        if released:
            self.move_home()
            self.node.get_logger().info("Force release completed; robot in ready state.")
        return released

    def open_green_box(self, landmark: TargetPose) -> bool:
        """Pick the green-box lid handle, place the lid aside, and return above it."""
        landmark = self._select_lowest_joint_cost_target(landmark)
        grasp = landmark.matrix.copy()
        grasp[2, 3] += self.search_config.green_box_grasp_z_offset_mm
        grasp = validate_homogeneous_matrix(grasp, "green box lid grasp")

        approach = grasp.copy()
        approach[2, 3] += self.motion_config.approach_distance_mm
        lifted = grasp.copy()
        lifted[2, 3] += self.search_config.green_box_lift_z_mm
        shifted = lifted.copy()
        shifted[1, 3] += self.search_config.green_box_place_y_mm
        lowered = shifted.copy()
        lowered[2, 3] -= self.search_config.green_box_place_down_z_mm

        approach_pos = matrix_to_drl_posx(approach)
        grasp_pos = matrix_to_drl_posx(grasp)
        lifted_pos = matrix_to_drl_posx(lifted)
        shifted_pos = matrix_to_drl_posx(shifted)
        lowered_pos = matrix_to_drl_posx(lowered)
        self.node.get_logger().info(f"green box handle posx: {grasp_pos}")
        self.node.get_logger().info(f"green box lid lifted posx: {lifted_pos}")
        self.node.get_logger().info(f"green box lid place posx: {lowered_pos}")

        if not self.robot_config.enable_motion:
            self.node.get_logger().warning("Dry-run: green box opening skipped")
            return False

        self.busy = True
        try:
            dr_base = getattr(self.dsr, "DR_BASE", 0)
            dr_abs = getattr(self.dsr, "DR_MV_MOD_ABS", 0)

            def move_linear(pos, vel, acc):
                self.dsr.movel(pos, vel=vel, acc=acc, ref=dr_base, mod=dr_abs)
                self.dsr.mwait()

            move_linear(
                approach_pos,
                self.motion_config.approach_vel,
                self.motion_config.approach_acc,
            )
            move_linear(
                grasp_pos,
                self.motion_config.grasp_vel,
                self.motion_config.grasp_acc,
            )
            self.gripper.close_gripper(
                force_val=self.gripper_config.force_tenth_newton
            )
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True

            move_linear(
                lifted_pos,
                self.motion_config.lift_vel,
                self.motion_config.lift_acc,
            )
            move_linear(
                shifted_pos,
                self.motion_config.lift_vel,
                self.motion_config.lift_acc,
            )
            move_linear(
                lowered_pos,
                self.motion_config.grasp_vel,
                self.motion_config.grasp_acc,
            )
            self.gripper.open_gripper(
                force_val=self.gripper_config.force_tenth_newton
            )
            self._wait_for_gripper(require_grip=False)
            self.holding_object = False

            # Retrace the placement path to the pose where the lid was lifted.
            move_linear(
                shifted_pos,
                self.motion_config.lift_vel,
                self.motion_config.lift_acc,
            )
            move_linear(
                lifted_pos,
                self.motion_config.lift_vel,
                self.motion_config.lift_acc,
            )
            return True
        finally:
            self.busy = False

    def open_gray_box(self, landmark: TargetPose) -> bool:
        """Grasp the gray-box handle, pull it in Base +Y, and release it."""
        landmark = self._select_lowest_joint_cost_target(landmark)
        grasp = landmark.matrix.copy()
        grasp[1, 3] += self.search_config.gray_box_handle_y_offset_mm
        grasp = validate_homogeneous_matrix(grasp, "gray box handle grasp")
        approach = grasp.copy()
        approach[2, 3] += self.motion_config.approach_distance_mm
        opened = grasp.copy()
        opened[1, 3] += self.search_config.gray_box_open_y_mm

        approach_pos = matrix_to_drl_posx(approach)
        grasp_pos = matrix_to_drl_posx(grasp)
        opened_pos = matrix_to_drl_posx(opened)
        self.node.get_logger().info(f"gray box handle posx: {grasp_pos}")
        self.node.get_logger().info(f"gray box opened posx: {opened_pos}")

        if not self.robot_config.enable_motion:
            self.node.get_logger().warning("Dry-run: gray box opening skipped")
            return False

        self.busy = True
        try:
            dr_base = getattr(self.dsr, "DR_BASE", 0)
            dr_abs = getattr(self.dsr, "DR_MV_MOD_ABS", 0)

            def move_linear(pos, vel, acc):
                self.dsr.movel(pos, vel=vel, acc=acc, ref=dr_base, mod=dr_abs)
                self.dsr.mwait()

            move_linear(
                approach_pos,
                self.motion_config.approach_vel,
                self.motion_config.approach_acc,
            )
            move_linear(
                grasp_pos,
                self.motion_config.grasp_vel,
                self.motion_config.grasp_acc,
            )
            self.gripper.close_gripper(
                force_val=self.gripper_config.force_tenth_newton
            )
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True
            move_linear(
                opened_pos,
                self.motion_config.lift_vel,
                self.motion_config.lift_acc,
            )
            self.gripper.open_gripper(
                force_val=self.gripper_config.force_tenth_newton
            )
            self._wait_for_gripper(require_grip=False)
            self.holding_object = False
            return True
        finally:
            self.busy = False

    def shutdown(self) -> None:
        try:
            self.gripper.close_connection()
        except Exception as error:  # Best-effort cleanup only.
            self.node.get_logger().warning(f"RG2 disconnect failed: {error}")
