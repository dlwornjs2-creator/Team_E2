#!/usr/bin/env python3
"""Convert the latest Any6D camera-frame pose to the Doosan base frame.

Input : /tmp/any6d_d435i/pose_camera.npy  (Any6D pose, metres)
Output: ~/Any6D/pose_base.npy              (base pose, millimetres)

This uses the same transform order as the user's tested robot_move.py:
    T_base_object = T_base_gripper @ T_gripper_camera @ T_camera_object
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation

import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


def doosan_posx_to_matrix(posx) -> np.ndarray:
    """Convert Doosan [x,y,z,rx,ry,rz] (mm, degrees) to T_base_gripper.

    The ZYZ convention deliberately matches the existing robot_move.py.
    """
    x, y, z, rx, ry, rz = [float(value) for value in posx]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def pose_camera_to_base(T_camera_object_m: np.ndarray, T_gripper_camera_mm: np.ndarray, T_base_gripper_mm: np.ndarray) -> np.ndarray:
    """Return T_base_object in millimetres, including its 3-D rotation."""
    if T_camera_object_m.shape != (4, 4) or T_gripper_camera_mm.shape != (4, 4):
        raise ValueError("Both pose matrices must be 4 x 4.")
    T_camera_object_mm = np.asarray(T_camera_object_m, dtype=np.float64).copy()
    T_camera_object_mm[:3, 3] *= 1000.0  # Any6D uses metres; Doosan uses mm.
    return T_base_gripper_mm @ T_gripper_camera_mm @ T_camera_object_mm


def main() -> int:
    parser = argparse.ArgumentParser(description="Any6D camera pose -> Doosan base pose")
    parser.add_argument("--pose-camera", default="/tmp/any6d_d435i/pose_camera.npy")
    parser.add_argument("--gripper-camera", required=True,
                        help="Path to T_gripper2camera.npy from the same wrist-mounted camera calibration")
    parser.add_argument("--output", default="~/Any6D/pose_base.npy")
    parser.add_argument("--topic", default="/robot_base_coordinate",
                        help="Base-frame pose topic (geometry_msgs/PoseStamped)")
    parser.add_argument("--frame-id", default="base", help="PoseStamped header.frame_id")
    parser.add_argument("--publish-count", type=int, default=10,
                        help="Publish the same latest pose this many times before exiting")
    parser.add_argument("--publish-hz", type=float, default=10.0)
    args = parser.parse_args()

    pose_camera_path = Path(args.pose_camera).expanduser()
    gripper_camera_path = Path(args.gripper_camera).expanduser()
    output_path = Path(args.output).expanduser()
    if not pose_camera_path.is_file():
        raise SystemExit(f"Any6D pose file not found: {pose_camera_path}")
    if not gripper_camera_path.is_file():
        raise SystemExit(f"Hand-Eye matrix not found: {gripper_camera_path}")
    if args.publish_count < 1 or args.publish_hz <= 0:
        raise SystemExit("--publish-count must be >= 1 and --publish-hz must be > 0")

    T_camera_object = np.load(pose_camera_path)
    T_gripper_camera = np.load(gripper_camera_path)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    rclpy.init()
    node = rclpy.create_node("any6d_pose_to_base", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import get_current_posx
        robot_posx = get_current_posx()[0]
        T_base_gripper = doosan_posx_to_matrix(robot_posx)
        T_base_object = pose_camera_to_base(T_camera_object, T_gripper_camera, T_base_gripper)

        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(output_path, T_base_object)
        np.savetxt(output_path.with_suffix(".txt"), T_base_object, fmt="%.9f")

        angles_zyz = Rotation.from_matrix(T_base_object[:3, :3]).as_euler("ZYZ", degrees=True)
        # ROS geometry messages use metres.  The saved matrix remains in mm so
        # it can be used directly with the Doosan API, which also uses mm.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(PoseStamped, args.topic, qos)
        quaternion_xyzw = Rotation.from_matrix(T_base_object[:3, :3]).as_quat()
        message = PoseStamped()
        message.header.frame_id = args.frame_id
        message.pose.position.x = float(T_base_object[0, 3] / 1000.0)
        message.pose.position.y = float(T_base_object[1, 3] / 1000.0)
        message.pose.position.z = float(T_base_object[2, 3] / 1000.0)
        message.pose.orientation.x = float(quaternion_xyzw[0])
        message.pose.orientation.y = float(quaternion_xyzw[1])
        message.pose.orientation.z = float(quaternion_xyzw[2])
        message.pose.orientation.w = float(quaternion_xyzw[3])

        interval = 1.0 / args.publish_hz
        for _ in range(args.publish_count):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(interval)

        print("========== Any6D object pose in robot base ==========")
        print(f"Position (mm): X={T_base_object[0, 3]:.2f}, Y={T_base_object[1, 3]:.2f}, Z={T_base_object[2, 3]:.2f}")
        print(f"Rotation ZYZ (deg): RZ1={angles_zyz[0]:.2f}, RY={angles_zyz[1]:.2f}, RZ2={angles_zyz[2]:.2f}")
        print("T_base_object:")
        print(T_base_object)
        print(f"Saved: {output_path}")
        print(f"Published {args.publish_count} times: {args.topic} (geometry_msgs/PoseStamped, metres)")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        sys.exit(130)
