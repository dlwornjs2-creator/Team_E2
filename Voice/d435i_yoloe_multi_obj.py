#!/usr/bin/env python3
"""GPT + YOLOE-26 candidate detection + Any6D pose tracking for a D435i.

Terminal Korean input -> GPT English YOLOE prompt + conservative profile routing
-> YOLOE candidate mask -> class-specific Any6D OBJ-mesh pose.  Frames are read from d435i_frame_bridge.py, not from a
USB webcam.  On a tracking failure or r key, YOLOE detects the target again.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Must be set before importing packages that import PyTorch.
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import cv2
import numpy as np
import scipy.io.wavfile as wav
import sounddevice as sd
import trimesh
from dotenv import load_dotenv
from openai import OpenAI
from ultralytics import YOLOE

import rclpy
from interfaces.srv import TargetSearch


@dataclass(frozen=True)
class ObjectProfile:
    profile_id: str
    display_name: str
    mesh_path: Path
    object_height_mm: float


@dataclass
class PromptUpdate:
    korean_query: str
    classes: list[str]
    primary_class: str
    profile_id: str | None
    explanation: str


@dataclass
class Detection:
    xyxy: np.ndarray
    confidence: float
    class_id: int
    mask: np.ndarray | None


class GPTPromptConverter:
    """Create YOLOE prompts and choose one of the registered OBJ profiles.

    Selection is conservative: an explicitly conflicting colour must never be
    forced into a registered profile merely because the shape noun matches.
    """

    def __init__(self, model: str, api_key: str) -> None:
        self.client = OpenAI(api_key=api_key)
        self.model = model

    @staticmethod
    def _normalise(text: str) -> str:
        return " ".join(text.lower().replace("_", " ").replace("-", " ").split())

    def _rule_based_profile(self, query: str) -> tuple[str | None, str]:
        q = self._normalise(query)
        has_can = any(x in q for x in ("캔", "can", "tin"))
        has_box = any(x in q for x in ("상자", "박스", "box", "carton"))
        has_yellow = any(x in q for x in ("노란", "노랑", "yellow"))
        has_green = any(x in q for x in ("초록", "녹색", "green"))
        other_colour = any(x in q for x in (
            "빨간", "빨강", "red", "파란", "파랑", "blue", "검정", "검은", "black",
            "흰색", "하얀", "white", "주황", "orange", "보라", "purple", "분홍", "pink",
        ))

        # Explicit colour conflicts are rejected rather than guessed.
        if has_can:
            if has_green or other_colour:
                return None, "캔 형태는 맞지만 등록된 노란색 캔과 색상이 다릅니다."
            # User requested generic 'can' to default to the registered yellow can.
            return "yellow_can", "일반 캔 입력은 등록된 노란색 캔으로 연결했습니다."

        if has_box:
            if has_yellow or other_colour:
                return None, "상자 형태는 맞지만 등록된 초록색 상자와 색상이 다릅니다."
            # Generic box defaults to the only registered box profile.
            return "green_box", "일반 상자 입력은 등록된 초록색 상자로 연결했습니다."

        return None, "형태 키워드만으로 등록 OBJ를 확정하지 못했습니다."

    def convert(self, korean_query: str) -> PromptUpdate:
        rule_profile, rule_reason = self._rule_based_profile(korean_query)
        prompt = f"""
사용자가 RealSense 영상에서 찾고 싶은 물체를 입력했다.

사용자 입력:
{korean_query}

해야 할 일:
1. YOLOE-26에 사용할 짧은 영어 클래스 명사구를 만든다.
2. 아래 등록된 OBJ 중 입력과 같은 물체일 때만 profile_id를 선택한다.

등록 OBJ:
- yellow_can: 노란색 원통형 캔
- green_box: 초록색 직육면체 상자

매우 중요한 판정 규칙:
- 색상이 명시되면 반드시 지킨다.
- "캔" 또는 "can"처럼 색상이 없는 일반 입력은 yellow_can으로 볼 수 있다.
- "초록색 캔", "green can"은 yellow_can이 아니다. profile_id는 null이다.
- "상자" 또는 "box"처럼 색상이 없는 일반 입력은 green_box로 볼 수 있다.
- "노란색 상자", "yellow box"는 green_box가 아니다. profile_id는 null이다.
- 형태나 색상 중 하나가 명백히 충돌하면 절대 비슷하다고 선택하지 않는다.
- 등록되지 않은 물체면 profile_id는 null이다.
- classes는 1~5개, 짧은 영어 명사구만 사용한다.
- primary_class에는 색상과 형태가 입력에 있으면 모두 포함한다.

반드시 JSON만 출력:
{{
  "primary_class": "string",
  "classes": ["string"],
  "profile_id": "yellow_can 또는 green_box 또는 null",
  "explanation": "한국어 한 문장"
}}
""".strip()
        response = self.client.responses.create(model=self.model, input=prompt, temperature=0)
        raw = response.output_text.strip()
        start, end = raw.find("{"), raw.rfind("}")
        if start < 0 or end < start:
            raise ValueError(f"GPT 응답에서 JSON을 찾지 못했습니다: {raw}")
        data = json.loads(raw[start : end + 1])

        classes: list[str] = []
        for value in data.get("classes", []):
            value = self._normalise(str(value))
            if value and value not in classes:
                classes.append(value)
        primary = self._normalise(str(data.get("primary_class", "")))
        if primary and primary not in classes:
            classes.insert(0, primary)
        if not classes:
            raise ValueError("GPT가 유효한 YOLOE 클래스 이름을 만들지 못했습니다.")

        gpt_profile = data.get("profile_id")
        if gpt_profile not in {"yellow_can", "green_box"}:
            gpt_profile = None

        # Deterministic colour/shape rules take precedence over GPT.
        profile_id = rule_profile
        if rule_profile is None:
            q = self._normalise(korean_query)
            mentions_known_shape = any(x in q for x in ("캔", "can", "tin", "상자", "박스", "box", "carton"))
            mentions_conflicting_colour = any(x in q for x in (
                "초록", "녹색", "green", "노란", "노랑", "yellow", "빨간", "빨강", "red",
                "파란", "파랑", "blue", "검정", "검은", "black", "흰색", "하얀", "white",
                "주황", "orange", "보라", "purple", "분홍", "pink",
            ))
            # For unrelated/indirect wording GPT may help; explicit shape+colour
            # conflicts remain unsupported.
            if not (mentions_known_shape and mentions_conflicting_colour):
                profile_id = gpt_profile

        explanation = str(data.get("explanation", "")).strip()
        combined = f"{rule_reason} {explanation}".strip()
        return PromptUpdate(korean_query, classes[:5], primary or classes[0], profile_id, combined)


class InputThread(threading.Thread):
    def __init__(self, converter: GPTPromptConverter, updates: queue.Queue[PromptUpdate], stop: threading.Event) -> None:
        super().__init__(daemon=True)
        self.converter, self.updates, self.stop = converter, updates, stop

    def _record_and_transcribe(self) -> str | None:
        fs = 16000
        duration = 5
        print(f"\n[STT] {duration}초 동안 마이크로 말씀해 주세요...")
        try:
            recording = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
            sd.wait()
            print("[STT] 녹음 완료. 음성을 텍스트로 변환 중...")
            
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
                tmp_path = tmp_file.name
            
            wav.write(tmp_path, fs, recording)
            
            with open(tmp_path, "rb") as audio_file:
                transcript = self.converter.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file
                )
            os.remove(tmp_path)
            
            text = transcript.text.strip()
            print(f"[STT 결과] {text}")
            return text
        except Exception as exc:
            print(f"[STT 오류] {exc}")
            return None

    def run(self) -> None:
        print("\n찾을 물체를 한국어로 입력하세요. 예: 검은색 직사각형 상자")
        print("명령: /q 종료, /v 음성입력, /help 도움말\n")
        while not self.stop.is_set():
            try:
                text = input("찾을 물체> ").strip()
            except (EOFError, KeyboardInterrupt):
                self.stop.set()
                return
            if not text:
                continue
            if text.lower() in {"/q", "/quit", "/exit"}:
                self.stop.set()
                return
            if text.lower() in {"/v", "/voice"}:
                transcribed = self._record_and_transcribe()
                if not transcribed:
                    continue
                try:
                    override = input("엔터 키를 누르면 위 텍스트를 전송합니다. (수정하려면 새 텍스트 입력, 취소는 /c)> ").strip()
                except (EOFError, KeyboardInterrupt):
                    self.stop.set()
                    return
                if override.lower() in {"/c", "c"}:
                    print("전송이 취소되었습니다.")
                    continue
                elif override:
                    text = override
                else:
                    text = transcribed
            elif text.lower() == "/help":
                print("한국어로 대상 물체를 입력합니다. /v=음성입력, r=YOLOE로 다시 찾기, q=종료")
                continue
            
            print("[GPT] YOLOE 검색 문구와 OBJ 프로필을 판정하는 중...")
            try:
                self.updates.put(self.converter.convert(text))
            except Exception as exc:
                print(f"[GPT 오류] {exc}")


def read_latest(frame_path: Path):
    try:
        with np.load(frame_path) as data:
            return data["color_bgr"], data["depth"], data["K"], int(data["frame_id"])
    except (OSError, KeyError, ValueError):
        return None


def detection_mask(det: Detection, depth_m: np.ndarray) -> np.ndarray:
    """Use YOLOE segmentation when present; otherwise use its tight rectangle."""
    h, w = depth_m.shape
    if det.mask is not None and det.mask.shape == (h, w) and det.mask.sum() >= 40:
        mask = det.mask.copy()
    else:
        x1, y1, x2, y2 = np.rint(det.xyxy).astype(int)
        x1, x2 = np.clip([x1, x2], 0, w)
        y1, y2 = np.clip([y1, y2], 0, h)
        mask = np.zeros((h, w), dtype=bool)
        mask[y1:y2, x1:x2] = True
    # Invalid depth must never be treated as the object.
    return mask & np.isfinite(depth_m) & (depth_m > 0.10) & (depth_m < 2.0)


def extract_detections(result: Any, image_shape: tuple[int, int]) -> list[Detection]:
    if result.boxes is None or len(result.boxes) == 0:
        return []
    boxes = result.boxes.xyxy.detach().cpu().numpy()
    confs = result.boxes.conf.detach().cpu().numpy()
    classes = result.boxes.cls.detach().cpu().numpy().astype(int)
    masks = None if result.masks is None else result.masks.data.detach().cpu().numpy()
    h, w = image_shape
    detections = []
    for index, (box, conf, class_id) in enumerate(zip(boxes, confs, classes)):
        mask = None
        if masks is not None and index < len(masks):
            mask = cv2.resize(masks[index], (w, h), interpolation=cv2.INTER_NEAREST) > 0.5
        detections.append(Detection(box.astype(np.float32), float(conf), int(class_id), mask))
    return detections


def draw_candidates(frame: np.ndarray, detections: list[Detection], names: Any, selected: int | None) -> None:
    for index, det in enumerate(detections):
        x1, y1, x2, y2 = np.rint(det.xyxy).astype(int)
        color = (0, 255, 255) if index == selected else (120, 120, 120)
        name = names.get(det.class_id, str(det.class_id)) if isinstanced(names, dict) else str(det.class_id)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{name} {det.confidence:.2f}", (x1, max(20, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def load_mesh_in_meters(mesh_path: Path, object_height_mm: float) -> trimesh.Trimesh:
    """Load a normalized OBJ, add a fallback texture when needed, and scale it."""
    loaded = trimesh.load(mesh_path, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh) or loaded.is_empty:
        raise ValueError(f"유효한 삼각형 메쉬를 읽지 못했습니다: {mesh_path}")
    mesh = loaded.copy()

    # FoundationPose (used internally by Any6D) assumes that every mesh has a
    # texture image and calls ``mesh.visual.material.image.convert('RGB')``.
    # Blender-exported CAD/plain-colour OBJ files often have no image texture,
    # which otherwise raises: AttributeError: 'NoneType' object has no attribute
    # 'convert'.  Give such meshes a neutral 2x2 texture so they can still be
    # used; the geometry/depth is unchanged.
    material_image = getattr(getattr(mesh.visual, "material", None), "image", None)
    if material_image is None:
        neutral_texture = np.full((2, 2, 3), 180, dtype=np.uint8)
        uv = np.zeros((len(mesh.vertices), 2), dtype=np.float64)
        mesh.visual = trimesh.visual.texture.TextureVisuals(uv=uv, image=neutral_texture)
        print("[OBJ] 텍스처가 없는 OBJ입니다. Any6D용 중성 텍스처를 자동 적용합니다.")

    z_extent = float(mesh.extents[2])
    if z_extent <= 0:
        raise ValueError("OBJ의 Z 방향 크기가 0입니다. 메쉬 방향을 확인하세요.")
    if object_height_mm <= 0:
        raise ValueError("--object-height-mm에는 실제 물체의 높이(mm)를 0보다 크게 넣어야 합니다.")

    # The supplied raw mesh spans about 2.0 units in Z, not metres.
    # Any6D receives depth in metres, therefore the mesh must use metres too.
    scale = (object_height_mm / 1000.0) / z_extent
    mesh.apply_scale(scale)
    return mesh


def main() -> int:
    parser = argparse.ArgumentParser(description="D435i: GPT + YOLOE candidate detection + Any6D pose")
    parser.add_argument("--any6d-root", default="~/Any6D")
    parser.add_argument("--yellow-can-mesh", default="~/Any6D/yellow_can.obj",
                        help="노란색 캔 OBJ 경로")
    parser.add_argument("--yellow-can-height-mm", type=float, required=True,
                        help="노란색 캔의 실제 Z방향 높이(mm)")
    parser.add_argument("--green-box-mesh", default="~/Any6D/green_box.obj",
                        help="초록색 상자 OBJ 경로")
    parser.add_argument("--green-box-height-mm", type=float, required=True,
                        help="초록색 상자의 실제 Z방향 높이(mm)")
    parser.add_argument("--frame-dir", default="/tmp/any6d_d435i")
    parser.add_argument("--env-file", default=".env", help="OPENAI_API_KEY가 있는 .env 경로")
    parser.add_argument("--weights", default="yoloe-26s-seg.pt")
    parser.add_argument("--yolo-device", default="cpu", help="기본 cpu. 여유가 있으면 0으로 변경")
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--max-width", type=int, default=640)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--register-iterations", type=int, default=3)
    parser.add_argument("--track-iterations", type=int, default=2)
    parser.add_argument("--use-aliases", action="store_true", help="GPT 동의어 모두를 YOLOE 클래스에 적용")
    args = parser.parse_args()

    root = Path(args.any6d_root).expanduser().resolve()
    profiles = {
        "yellow_can": ObjectProfile(
            "yellow_can", "노란색 캔",
            Path(args.yellow_can_mesh).expanduser().resolve(),
            args.yellow_can_height_mm,
        ),
        "green_box": ObjectProfile(
            "green_box", "초록색 상자",
            Path(args.green_box_mesh).expanduser().resolve(),
            args.green_box_height_mm,
        ),
    }
    frame_path = Path(args.frame_dir).expanduser() / "latest_rgbd.npz"
    env_file = Path(args.env_file).expanduser()
    if not env_file.is_absolute():
        env_file = root / env_file
    if not root.is_dir():
        raise SystemExit("--any6d-root 경로를 확인하세요.")
    missing = [f"{p.display_name}: {p.mesh_path}" for p in profiles.values() if not p.mesh_path.is_file()]
    if missing:
        raise SystemExit("등록 OBJ 파일을 찾지 못했습니다:\n- " + "\n- ".join(missing))
    load_dotenv(env_file)
    api_key = os.getenv("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise SystemExit(f"OPENAI_API_KEY를 읽지 못했습니다: {env_file}")

    os.chdir(root)
    sys.path.insert(0, str(root))
    from estimater import Any6D
    from d435i_box_pose_worker import draw_pose, draw_pose_axes, resize_rgbd

    debug_dir = root / "live_obj_debug"
    debug_dir.mkdir(parents=True, exist_ok=True)

    mesh = None
    estimator = None
    active_profile_id: str | None = None

    # ROS 2 노드 및 클라이언트 초기화
    rclpy.init()
    ros_node = rclpy.create_node('voice_node')
    search_cli = ros_node.create_client(TargetSearch, 'sm/request_search')

    def activate_profile(profile_id: str):
        nonlocal mesh, estimator, active_profile_id
        profile = profiles[profile_id]
        loaded_mesh = load_mesh_in_meters(profile.mesh_path, profile.object_height_mm)
        profile_debug_dir = debug_dir / profile_id
        profile_debug_dir.mkdir(parents=True, exist_ok=True)
        loaded_estimator = Any6D(
            symmetry_tfs=None,
            mesh=loaded_mesh,
            debug_dir=str(profile_debug_dir),
            debug=0,
        )
        loaded_estimator.make_rotation_grid(min_n_views=12, inplane_step=90)
        mesh = loaded_mesh
        estimator = loaded_estimator
        active_profile_id = profile_id
        print(
            f"[OBJ 선택] {profile.display_name} -> {profile.mesh_path.name} | "
            f"크기={mesh.extents[0] * 1000:.1f} x {mesh.extents[1] * 1000:.1f} x "
            f"{mesh.extents[2] * 1000:.1f} mm"
        )
        print(f"[Any6D] lightweight mode: {len(estimator.rot_grid)} initial rotations")

    print(f"[YOLOE] loading: {args.weights} (device={args.yolo_device})")
    model = YOLOE(args.weights)
    model.set_classes(["object"])
    updates: queue.Queue[PromptUpdate] = queue.Queue()
    stop = threading.Event()
    converter = GPTPromptConverter(os.getenv("OPENAI_MODEL", "gpt-4.1-mini"), api_key)
    InputThread(converter, updates, stop).start()

    pose = None
    have_target = False
    target_name = "(type a target in terminal)"
    active_classes = ["object"]
    last_frame_id = -1
    last_init_attempt = 0.0
    title = "GPT + YOLOE + Any6D D435i"
    try:
        while not stop.is_set():
            latest = None
            while True:
                try:
                    latest = updates.get_nowait()
                except queue.Empty:
                    break
            if latest is not None:
                active_classes = latest.classes if args.use_aliases else [latest.primary_class]
                model.set_classes(active_classes)
                target_name = latest.primary_class
                pose = None
                last_init_attempt = 0.0
                print(f"[GPT] target={target_name}; YOLOE classes={active_classes}")
                if latest.explanation:
                    print(f"[GPT] {latest.explanation}")

                if latest.profile_id is None:
                    have_target = False
                    estimator = None
                    mesh = None
                    active_profile_id = None
                    print(
                        f"[OBJ 미등록] '{latest.korean_query}'에 대응하는 OBJ가 아직 추가되지 않았습니다."
                    )
                    print("[안내] YOLOE 탐지는 가능하지만 포즈 추정은 할 수 없습니다.")
                else:
                    try:
                        if latest.profile_id != active_profile_id or estimator is None:
                            activate_profile(latest.profile_id)
                        have_target = True
                        
                        if search_cli.service_is_ready():
                            req = TargetSearch.Request()
                            req.target_name = target_name
                            req.class_label = latest.profile_id
                            search_cli.call_async(req)
                            print(f"[상태머신 요청] '{target_name}' 탐색을 상태머신에 지시했습니다.")
                        else:
                            print("[상태머신 오류] sm/request_search 서비스가 켜져있지 않습니다.")
                            
                    except Exception as exc:
                        have_target = False
                        estimator = None
                        mesh = None
                        active_profile_id = None
                        print(f"[OBJ 오류] 프로필을 불러오지 못했습니다: {exc}")
                        print("[안내] YOLOE 탐지는 가능하지만 포즈 추정은 할 수 없습니다.")

            item = read_latest(frame_path)
            if item is None or item[3] == last_frame_id:
                time.sleep(0.01)
                continue
            color_bgr, depth_raw, K, frame_id = item
            last_frame_id = frame_id
            depth_m = depth_raw.astype(np.float32)
            if np.issubdtype(depth_raw.dtype, np.integer):
                depth_m *= args.depth_scale
            color_bgr, depth_m, K = resize_rgbd(color_bgr, depth_m, K, args.max_width)
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
            view = color_bgr.copy()

            if pose is None and target_name != "(type a target in terminal)":
                prediction = model.predict(source=color_bgr, conf=args.conf, imgsz=args.imgsz,
                                           device=args.yolo_device, verbose=False, agnostic_nms=True)[0]
                detections = extract_detections(prediction, depth_m.shape)
                selected = max(range(len(detections)), key=lambda i: detections[i].confidence) if detections else None
                draw_candidates(view, detections, prediction.names, selected)
                if selected is not None and not have_target:
                    cv2.putText(view, "YOLOE detected, but OBJ is not registered", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 165, 255), 2)
                elif selected is not None and time.monotonic() - last_init_attempt >= 1.0:
                    cv2.putText(view, "YOLOE candidate found: initializing Any6D...", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
                    cv2.imshow(title, view)
                    cv2.waitKey(1)
                    mask = detection_mask(detections[selected], depth_m)
                    if int(mask.sum()) >= 100:
                        last_init_attempt = time.monotonic()
                        print(f"[YOLOE] candidate {target_name}: {detections[selected].confidence:.2f}; Any6D initializing...")
                        try:
                            pose = estimator.register(K=K, rgb=color_rgb, depth=depth_m, ob_mask=mask,
                                                      iteration=args.register_iterations, name="d435i_yoloe")
                            np.save(Path(args.frame_dir).expanduser() / "pose_camera.npy", pose)
                            with open(Path(args.frame_dir).expanduser() / "target_name.txt", "w", encoding="utf-8") as f:
                                f.write(str(active_profile_id or target_name))
                                
                            print("[Any6D] initial pose found. Tracking started.")
                        except Exception as exc:
                            pose = None
                            print(f"[Any6D] initial pose failed: {exc}")
                    else:
                        print("[YOLOE] 후보 영역에 유효한 깊이 픽셀이 부족합니다.")
            elif pose is not None and estimator is not None and mesh is not None:
                try:
                    pose = estimator.track_one_any6d(rgb=color_rgb, depth=depth_m, K=K,
                                                      iteration=args.track_iterations)
                    np.save(Path(args.frame_dir).expanduser() / "pose_camera.npy", pose)
                    with open(Path(args.frame_dir).expanduser() / "target_name.txt", "w", encoding="utf-8") as f:
                        f.write(str(active_profile_id or target_name))
                    view = draw_pose(view, K, pose, mesh.bounds)
                    view = draw_pose_axes(view, K, pose)
                except Exception as exc:
                    pose = None
                    print(f"[Any6D] tracking lost: {exc}. Re-detecting with YOLOE.")

            if pose is not None:
                mode = "TRACKING"
            elif have_target:
                mode = f"YOLOE DETECTING / OBJ={active_profile_id}"
            else:
                mode = "YOLOE ONLY / OBJ NOT REGISTERED"
            status = f"Target: {target_name} | {mode} | r: redetect  q: quit"
            cv2.putText(view, status, (10, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)
            cv2.imshow(title, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                pose = None
                last_init_attempt = 0.0
                print("[Any6D] reset requested. Re-detecting target with YOLOE.")
            
            # Allow ROS 2 to process events (non-blocking)
            rclpy.spin_once(ros_node, timeout_sec=0.0)
    finally:
        stop.set()
        cv2.destroyAllWindows()
        ros_node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
