#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grounding DINO로 고정 객체 클래스를 검색하는 ROS 2 서비스 노드.

기능
- `interfaces/srv/DbSave` 타입의 `/detect_all_objects` 요청을 받음
- request.request JSON 문자열을 검출 트리거/요청 식별 정보로 사용
- `/detect_all_objects` 요청 시 최신 컬러 프레임에서 전체 고정 클래스 검색
- confidence 기준 미만 결과 제거
- 클래스별 최대 1개만 유지
- 같은 물체에 겹쳐 검출된 클래스 중 confidence가 가장 높은 결과만 유지
- `/set_picked_object`로 전달받은 단일 클래스는 이후 검색 결과에서 제외
- 검출 결과를 OpenCV 창에 일정 시간 표시

고정 클래스
- yellow_can
- green_box
- gray_box
- white_bear
- aircon_remote
- green_frog
- otter_in_can
"""

import argparse
import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python 패키지를 불러오지 못했습니다. "
        "ROS 2 Humble 환경에서 실행하세요.\n"
        f"원래 오류: {exc}"
    )

try:
    from interfaces.srv import DbSave
    from vision_nodes.srv import SetPickedObject
except ImportError as exc:
    raise SystemExit(
        "interfaces.srv.DbSave 또는 vision_nodes.srv.SetPickedObject를 불러오지 못했습니다.\n"
        "서비스 패키지명이 다르면 import 경로를 실제 패키지명으로 바꾸세요.\n"
        f"원래 오류: {exc}"
    )

try:
    from groundingdino.util.inference import Model
except ImportError as exc:
    raise SystemExit(
        "GroundingDINO를 불러오지 못했습니다. "
        "GroundingDINO 환경을 활성화하세요.\n"
        f"원래 오류: {exc}"
    )


CLASS_DEFINITIONS = (
    ("yellow_can", "yellow cylindrical can"),
    ("green_box", "green rectangular box"),
    ("gray_box", "gray rectangular storage box"),
    ("white_bear", "white bear plush toy"),
    ("aircon_remote", "gray air conditioner remote control"),
    ("green_frog", "green frog plush toy"),
    ("otter_in_can", "otter plush toy in a wooden barrel"),
)

CLASS_NAMES = tuple(name for name, _ in CLASS_DEFINITIONS)
CLASS_PROMPTS = tuple(prompt for _, prompt in CLASS_DEFINITIONS)
CLASS_NAME_SET = frozenset(CLASS_NAMES)

COLORS = (
    (0, 220, 255),
    (0, 200, 0),
    (160, 160, 160),
    (255, 180, 0),
    (220, 80, 220),
    (40, 220, 120),
    (255, 120, 60),
)

RESET_NAMES = {"", "none", "clear", "reset"}


@dataclass(frozen=True)
class Candidate:
    box: np.ndarray
    confidence: float
    class_id: int
    class_name: str


def box_area(box: np.ndarray) -> float:
    width = max(0.0, float(box[2] - box[0]))
    height = max(0.0, float(box[3] - box[1]))
    return width * height


def intersection_area(a: np.ndarray, b: np.ndarray) -> float:
    width = max(0.0, min(float(a[2]), float(b[2])) - max(float(a[0]), float(b[0])))
    height = max(0.0, min(float(a[3]), float(b[3])) - max(float(a[1]), float(b[1])))
    return width * height


def is_same_object(
    a: np.ndarray,
    b: np.ndarray,
    iou_threshold: float,
    ios_threshold: float,
) -> bool:
    intersection = intersection_area(a, b)
    if intersection <= 0.0:
        return False

    area_a = box_area(a)
    area_b = box_area(b)
    union = area_a + area_b - intersection
    smaller = min(area_a, area_b)

    iou = intersection / union if union > 0.0 else 0.0
    ios = intersection / smaller if smaller > 0.0 else 0.0
    return iou >= iou_threshold or ios >= ios_threshold


def select_unique_candidates(
    candidates: Sequence[Candidate],
    iou_threshold: float,
    ios_threshold: float,
) -> list[Candidate]:
    selected: list[Candidate] = []
    selected_class_ids: set[int] = set()

    for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True):
        if candidate.class_id in selected_class_ids:
            continue

        if any(
            is_same_object(
                candidate.box,
                accepted.box,
                iou_threshold,
                ios_threshold,
            )
            for accepted in selected
        ):
            continue

        selected.append(candidate)
        selected_class_ids.add(candidate.class_id)

    return selected


def draw_results(
    image: np.ndarray,
    results: Sequence[Candidate],
    picked_class: Optional[str],
) -> np.ndarray:
    output = image.copy()
    height, width = output.shape[:2]

    for item in results:
        x1, y1, x2, y2 = np.round(item.box).astype(int)
        x1 = int(np.clip(x1, 0, width - 1))
        x2 = int(np.clip(x2, 0, width - 1))
        y1 = int(np.clip(y1, 0, height - 1))
        y2 = int(np.clip(y2, 0, height - 1))

        color = COLORS[item.class_id % len(COLORS)]
        label = f"{item.class_name} {item.confidence:.3f}"
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 3)

        (text_width, text_height), baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            2,
        )
        label_top = max(0, y1 - text_height - baseline - 8)
        label_bottom = min(height - 1, label_top + text_height + baseline + 8)
        label_right = min(width - 1, x1 + text_width + 10)

        cv2.rectangle(
            output,
            (x1, label_top),
            (label_right, label_bottom),
            color,
            -1,
        )
        cv2.putText(
            output,
            label,
            (x1 + 5, label_bottom - baseline - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

    status = f"Detected: {len(results)}"
    if picked_class:
        status += f" | excluded picked: {picked_class}"

    cv2.putText(
        output,
        status,
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return output


class DinoAllObjectsService(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("dino_all_object_node")
        self.args = args
        self.bridge = CvBridge()

        self.latest_frame: Optional[np.ndarray] = None
        self.display_frame: Optional[np.ndarray] = None
        self.display_until = 0.0
        self.picked_class: Optional[str] = None

        self.get_logger().info("Grounding DINO 모델을 불러오는 중...")
        self.model = Model(
            model_config_path=str(Path(args.config).expanduser()),
            model_checkpoint_path=str(Path(args.weights).expanduser()),
        )
        self.get_logger().info("Grounding DINO 모델 로드 완료")

        self.image_sub = self.create_subscription(
            Image,
            args.color_topic,
            self.image_callback,
            10,
        )
        self.detect_service = self.create_service(
            DbSave,
            args.detect_service,
            self.detect_callback,
        )
        self.picked_service = self.create_service(
            SetPickedObject,
            args.picked_service,
            self.picked_callback,
        )
        self.window_timer = self.create_timer(0.03, self.update_window)

        self.get_logger().info(f"검출 서비스: {args.detect_service}")
        self.get_logger().info(f"픽업 클래스 서비스: {args.picked_service}")
        self.get_logger().info(f"카메라 토픽: {args.color_topic}")
        self.get_logger().info(f"confidence 기준: {args.conf_threshold:.2f}")

    def image_callback(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            ).copy()
        except Exception as exc:
            self.get_logger().error(f"카메라 이미지 변환 실패: {exc}")

    def get_latest_frame(self) -> Optional[np.ndarray]:
        return None if self.latest_frame is None else self.latest_frame.copy()

    def run_detection(self, image: np.ndarray) -> list[Candidate]:
        detections = self.model.predict_with_classes(
            image=image,
            classes=CLASS_PROMPTS,
            box_threshold=self.args.conf_threshold,
            text_threshold=self.args.text_threshold,
        )

        candidates: list[Candidate] = []
        for box, confidence, class_id in zip(
            np.asarray(detections.xyxy),
            np.asarray(detections.confidence),
            np.asarray(detections.class_id),
        ):
            if class_id is None:
                continue

            class_id = int(class_id)
            confidence = float(confidence)
            if not 0 <= class_id < len(CLASS_NAMES):
                continue
            if confidence < self.args.conf_threshold:
                continue

            class_name = CLASS_NAMES[class_id]
            if class_name == self.picked_class:
                continue

            candidates.append(
                Candidate(
                    box=np.asarray(box, dtype=np.float32),
                    confidence=confidence,
                    class_id=class_id,
                    class_name=class_name,
                )
            )

        return select_unique_candidates(
            candidates,
            self.args.same_object_iou,
            self.args.same_object_ios,
        )

    @staticmethod
    def parse_detect_request(raw_request: str) -> dict:
        """DbSave.request의 JSON 문자열을 검증합니다.

        필수 키는 없으며 `{}`도 허용합니다. `request_id`와 `source`가 있으면
        검출 응답 JSON에 그대로 포함합니다.
        """
        raw_request = str(raw_request).strip()
        if not raw_request:
            raise ValueError("request JSON 문자열이 비어 있습니다. 예: '{}'")

        try:
            payload = json.loads(raw_request)
        except json.JSONDecodeError as exc:
            raise ValueError(f"request가 올바른 JSON이 아닙니다: {exc}") from exc

        if not isinstance(payload, dict):
            raise ValueError("request JSON은 객체 형태여야 합니다. 예: '{}'")
        return payload

    def make_detect_response_json(
        self,
        request_payload: dict,
        detected_names: Sequence[str],
    ) -> str:
        """기존 class ID 목록을 DbSave.response용 JSON 문자열로 만듭니다."""
        payload = {
            "source": "dino_all_object_node",
            "request_id": str(request_payload.get("request_id", "")),
            "request_source": str(request_payload.get("source", "")),
            "model_names": list(detected_names),
            "count": len(detected_names),
            "excluded_model_name": self.picked_class or "",
        }
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    def detect_callback(self, request, response):
        response.success = False
        response.response = ""
        response.message = ""

        try:
            request_payload = self.parse_detect_request(request.request)
        except ValueError as exc:
            response.response = json.dumps(
                {"model_names": [], "count": 0},
                ensure_ascii=False,
                separators=(",", ":"),
            )
            response.message = str(exc)
            return response

        frame = self.get_latest_frame()
        if frame is None:
            response.response = self.make_detect_response_json(request_payload, [])
            response.message = "아직 카메라 프레임을 받지 못했습니다."
            return response

        try:
            selected = self.run_detection(frame)
        except Exception as exc:
            self.get_logger().exception(f"Grounding DINO 분석 실패: {exc}")
            response.response = self.make_detect_response_json(request_payload, [])
            response.message = f"DINO 분석 실패: {exc}"
            return response

        result_image = draw_results(frame, selected, self.picked_class)
        self.display_frame = result_image
        self.display_until = time.monotonic() + self.args.result_display_sec

        # SetPickedObject로 받은 클래스는 run_detection()에서 이미 제외됩니다.
        detected_names = [item.class_name for item in selected]
        response.success = bool(detected_names)
        response.response = self.make_detect_response_json(
            request_payload,
            detected_names,
        )

        if detected_names:
            response.message = f"{len(detected_names)}개 객체 검출"
        else:
            excluded = f" 픽업 제외={self.picked_class}" if self.picked_class else ""
            response.message = (
                f"confidence {self.args.conf_threshold:.2f} 이상 검출 없음"
                f"{excluded}"
            )

        self.get_logger().info(
            f"{response.message}; response={response.response}"
        )
        self.save_result_image(result_image)
        return response

    def picked_callback(self, request, response):
        class_name = request.model_name.strip()

        if class_name.lower() in RESET_NAMES:
            previous = self.picked_class
            self.picked_class = None
            response.success = True
            response.message = f"픽업 제외 클래스를 초기화했습니다. 이전 값={previous}"
        elif class_name not in CLASS_NAME_SET:
            response.success = False
            response.message = (
                f"알 수 없는 클래스입니다: {class_name}. "
                f"허용값={','.join(CLASS_NAMES)}"
            )
            self.get_logger().warning(response.message)
            return response
        else:
            self.picked_class = class_name
            response.success = True
            response.message = (
                f"픽업 완료 클래스 {class_name}을 이후 검출 결과에서 제외합니다."
            )

        self.get_logger().info(response.message)
        return response

    def save_result_image(self, image: np.ndarray) -> None:
        if not self.args.output:
            return

        output_path = Path(self.args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), image):
            self.get_logger().warning(f"결과 이미지 저장 실패: {output_path}")

    def update_window(self) -> None:
        frame = self.get_latest_frame()
        if frame is None:
            return

        if self.display_frame is not None and time.monotonic() < self.display_until:
            shown = self.display_frame
        else:
            self.display_frame = None
            shown = frame
            status = "Waiting for detect service..."
            if self.picked_class:
                status += f" | picked={self.picked_class}"

            cv2.putText(
                shown,
                status,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow(self.args.window_name, shown)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            self.get_logger().info("Q/ESC 입력으로 종료합니다.")
            if rclpy.ok():
                rclpy.shutdown()

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Grounding DINO 전체 객체 검출 ROS 2 서비스"
    )
    dino_home = Path(
        os.getenv("GROUNDINGDINO_HOME", "~/GroundingDINO")
    ).expanduser()
    parser.add_argument(
        "--config",
        default=str(
            dino_home
            / "groundingdino/config/GroundingDINO_SwinT_OGC.py"
        ),
        help="GroundingDINO config 경로",
    )
    parser.add_argument(
        "--weights",
        default=str(
            dino_home
            / "weights/groundingdino_swint_ogc.pth"
        ),
        help="GroundingDINO checkpoint 경로",
    )
    parser.add_argument(
        "--color-topic",
        default="/camera/camera/color/image_raw",
        help="RealSense 컬러 이미지 토픽",
    )
    parser.add_argument(
        "--detect-service",
        default="/detect_all_objects",
        help="전체 클래스 검색 서비스명",
    )
    parser.add_argument(
        "--picked-service",
        default="/set_picked_object",
        help="Any6D 노드에서 class_label을 전달받는 서비스명",
    )
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=0.30,
        help="검출 및 표시 최소 confidence (기본: 0.30)",
    )
    parser.add_argument(
        "--text-threshold",
        type=float,
        default=0.10,
        help="Grounding DINO text threshold (기본: 0.10)",
    )
    parser.add_argument("--same-object-iou", type=float, default=0.45)
    parser.add_argument("--same-object-ios", type=float, default=0.70)
    parser.add_argument(
        "--result-display-sec",
        type=float,
        default=3.0,
        help="검출 결과 화면 유지 시간 (기본: 3초)",
    )
    parser.add_argument(
        "--window-name",
        default="Grounding DINO All Objects",
    )
    parser.add_argument(
        "--output",
        default="",
        help="결과 이미지 저장 경로. 빈 값이면 저장하지 않음",
    )
    return parser.parse_args()


def main() -> None:
    rclpy.init()
    node = DinoAllObjectsService(parse_args())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
