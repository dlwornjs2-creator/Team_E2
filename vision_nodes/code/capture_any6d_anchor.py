#!/usr/bin/env python3
"""Save one RealSense RGB-D anchor frame for Any6D registration.

Run after starting the RealSense ROS2 driver.  A preview window opens:
  SPACE : save the current colour image, aligned depth image, and intrinsics
  Q / ESC : quit without saving

The saved directory contains:
  anchor_rgb.png    colour image (standard BGR PNG; OpenCV-readable)
  anchor_depth.png  aligned 16-bit depth image (normally millimetres)
  anchor_K.npy      3x3 camera intrinsic matrix
  metadata.json     capture details and ROS topic names
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class Any6DAnchorCapture(Node):
    def __init__(self, color_topic: str, depth_topic: str, info_topic: str):
        super().__init__("any6d_anchor_capture")
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.K = None
        self.color_stamp_ns = None
        self.depth_stamp_ns = None

        self.create_subscription(Image, color_topic, self.color_callback, 10)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, info_topic, self.info_callback, 10)

    @staticmethod
    def stamp_ns(message: Image) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def color_callback(self, message: Image) -> None:
        try:
            self.color = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            self.color_stamp_ns = self.stamp_ns(message)
        except Exception as exc:
            self.get_logger().error(f"컬러 변환 실패: {exc}")

    def depth_callback(self, message: Image) -> None:
        try:
            # passthrough preserves the original 16UC1 depth values.
            self.depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            self.depth_stamp_ns = self.stamp_ns(message)
        except Exception as exc:
            self.get_logger().error(f"Depth 변환 실패: {exc}")

    def info_callback(self, message: CameraInfo) -> None:
        self.K = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def ready(self) -> bool:
        return self.color is not None and self.depth is not None and self.K is not None


def save_anchor(node: Any6DAnchorCapture, output_dir: Path, topics: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_dir / "anchor_rgb.png"), node.color):
        raise RuntimeError("anchor_rgb.png 저장 실패")
    if not cv2.imwrite(str(output_dir / "anchor_depth.png"), node.depth):
        raise RuntimeError("anchor_depth.png 저장 실패")
    np.save(output_dir / "anchor_K.npy", node.K)

    sync_ms = abs(node.color_stamp_ns - node.depth_stamp_ns) / 1_000_000
    metadata = {
        "captured_unix_time": time.time(),
        "color_shape": list(node.color.shape),
        "depth_shape": list(node.depth.shape),
        "depth_dtype": str(node.depth.dtype),
        "color_depth_timestamp_gap_ms": round(sync_ms, 3),
        "topics": topics,
        "K": node.K.tolist(),
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\n저장 완료: {output_dir}")
    print("다음 단계: 이 RGB-D 한 세트에서 물체 mask를 만들고 Any6D register_any6d()를 실행합니다.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture a RealSense RGB-D anchor frame for Any6D")
    parser.add_argument("--output", default="~/Any6D/anchors/object_001")
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--info-topic", default="/camera/camera/aligned_depth_to_color/camera_info")
    args = parser.parse_args()

    output_dir = Path(args.output).expanduser().resolve()
    topics = {"color": args.color_topic, "aligned_depth": args.depth_topic, "camera_info": args.info_topic}

    rclpy.init()
    node = Any6DAnchorCapture(args.color_topic, args.depth_topic, args.info_topic)
    window_name = "Any6D anchor capture  |  SPACE: save  Q/ESC: quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    print("토픽을 기다리는 중입니다. 미리보기에서 물체가 잘 보이는 대각선 방향을 잡고 SPACE를 누르세요.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.ready():
                preview = node.color.copy()
                text = "SPACE: capture  |  Q / ESC: quit"
                cv2.putText(preview, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(window_name, preview)
            else:
                waiting = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(waiting, "Waiting for color + aligned depth + camera_info", (20, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                cv2.imshow(window_name, waiting)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" ") and node.ready():
                save_anchor(node, output_dir, topics)
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
