"""Static configuration for the modular Any6D robot controller.

The values intentionally match the previous single-file controller.  They can
later be replaced or overridden with ROS 2 parameters without changing the
other modules.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotConfig:
    robot_id: str = "dsr01"
    robot_model: str = "m0609"
    tool_name: str = "Tool Weight_1"
    tcp_name: str = "GripperDA_v1"
    home_joint: tuple[float, ...] = (
        0.0,
        0.0,
        90.0,
        0.0,
        90.0,
        180.0,
    )
    joint_vel: float = 30.0
    joint_acc: float = 30.0
    enable_motion: bool = True


@dataclass(frozen=True)
class GripperConfig:
    name: str = "rg2"
    toolchanger_ip: str = "192.168.1.1"
    toolchanger_port: int = 502
    force_tenth_newton: int = 200
    timeout_sec: float = 8.0


@dataclass(frozen=True)
class MotionConfig:
    approach_vel: float = 30.0
    approach_acc: float = 60.0
    grasp_vel: float = 15.0
    grasp_acc: float = 30.0
    lift_vel: float = 20.0
    lift_acc: float = 40.0
    approach_distance_mm: float = 50.0
    lift_distance_mm: float = 50.0
    approach_mode: str = "base_z"
    tool_insertion_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class SearchConfig:
    zone2_base_x_mm: float = 250.0
    zone3_joint: tuple[float, ...] = (
        6.0,
        55.0,
        43.0,
        -91.0,
        96.0,
        186.0,
    )
    zone4_base_x_mm: float = -290.0
    linear_vel: float = 30.0
    linear_acc: float = 60.0
    detection_request_topic: str = "/any6d/detection_request"
    detection_result_topic: str = "/any6d/detection_result"
    detection_timeout_sec: float = 10.0
    pose_timeout_sec: float = 5.0
    landmark_targets: tuple[tuple[str, str], ...] = (
        ("녹색 상자", "green_box"),
        ("회색 수납장", "gray_cabinet"),
    )
    landmark_dwell_sec: float = 3.0


@dataclass(frozen=True)
class PoseConfig:
    input_mode: str = "any6d"
    topic: str = "/any6d/object_pose_base"
    expected_base_frame: str = "base"
    min_depth_mm: float = 2.0
    max_age_sec: float = 0.5
    required_stable_samples: int = 3
    max_stable_position_delta_mm: float = 5.0
    max_stable_rotation_delta_deg: float = 5.0
    pose_is_tcp_grasp: bool = True
    object_to_grasp_npy: str = ""
    wait_timeout_sec: float = 30.0


@dataclass(frozen=True)
class InterfaceConfig:
    control_init_service: str = "/control/init"
    state_task_topic: str = "/state/robot_request"
    control_result_topic: str = "/control/robot_result"
    db_load_service: str = "/db/load"
    db_service_wait_timeout_sec: float = 2.0
    db_response_timeout_sec: float = 5.0
    max_pending_tasks: int = 10


@dataclass(frozen=True)
class AppConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    search: SearchConfig = field(default_factory=SearchConfig)
    pose: PoseConfig = field(default_factory=PoseConfig)
    interface: InterfaceConfig = field(default_factory=InterfaceConfig)


DEFAULT_CONFIG = AppConfig()
