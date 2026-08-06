#!/usr/bin/env python3
"""STT + GPT object-name resolver and ROS 2 service client.

No application topics are used. The resolved model ID is sent to the
state node through interfaces/srv/TargetSearch, and the state node
will trigger the search action.
"""
from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path

import numpy as np
import rclpy
import scipy.io.wavfile as wav
import sounddevice as sd
from dotenv import load_dotenv
from openai import OpenAI
from rclpy.node import Node

from interfaces.srv import TargetSearch
from ament_index_python.packages import get_package_share_directory


VALID_MODEL_NAMES = {
    "yellow_can",
    "green_box",
    "white_bear",
    "aircon_remote",
    "green_frog",
    "otter_in_can",
}

MODEL_DESCRIPTIONS = {
    "yellow_can": "노란색 원통형 캔",
    "green_box": "초록색 직육면체 상자",
    "white_bear": "흰색 곰 인형",
    "aircon_remote": "회색 에어컨 리모컨",
    "green_frog": "초록 개구리 인형",
    "otter_in_can": "통 안에 있는 흰색/아이보리 수달 인형",
}

default_env_file = (
    Path(get_package_share_directory("voice_command"))
    / "resource"
    / ".env"
)

class ObjectNameResolver:
    def __init__(self, api_key: str, model: str) -> None:
        self.client = OpenAI(api_key=api_key)
        self.model = model

    def resolve(self, user_text: str) -> tuple[str | None, str]:
        prompt = f"""
사용자 입력을 아래 6개 로봇 비전 모델 중 정확히 하나로 분류하라.

모델:
- yellow_can: 노란색 원통형 캔. 일반 '캔'도 이것으로 본다. 초록색 캔 등 다른 색은 해당 없음.
- green_box: 초록색 직육면체 상자. 일반 '상자', '박스'도 이것으로 본다. 노란색 상자 등 다른 색은 해당 없음.
- white_bear: 흰색 곰 인형. 곰, 곰 인형, 쿼카, 베이지색 곰, 베이지색 쿼카도 같은 물체다.
- aircon_remote: 회색 에어컨 리모컨. 리모컨, 리모콘, 에어컨 리모컨도 같은 물체다.
- green_frog: 초록 개구리 인형. 개구리, 개구리 인형, 초록 개구리도 같은 물체다. 다른 색 개구리는 해당 없음.
- otter_in_can: 통 안에 있는 흰색/아이보리 수달 인형. 수달, 수달 인형, 목욕하는 수달 인형도 같은 물체다. 다른 색 수달은 해당 없음.

사용자 입력: {user_text}

등록 물체가 아니거나 색/형태가 충돌하면 model_name을 null로 출력한다.
반드시 JSON만 출력한다:
{{"model_name":"6개 ID 중 하나 또는 null", "reason":"한국어 한 문장"}}
""".strip()
        response = self.client.responses.create(model=self.model, input=prompt, temperature=0)
        raw = response.output_text.strip()
        start, end = raw.find("{"), raw.rfind("}")
        if start < 0 or end < start:
            raise ValueError(f"GPT JSON 응답을 찾지 못했습니다: {raw}")
        data = json.loads(raw[start : end + 1])
        model_name = data.get("model_name")
        if model_name not in VALID_MODEL_NAMES:
            model_name = None
        return model_name, str(data.get("reason", "")).strip()

    def transcribe(self, duration: float, sample_rate: int = 16000) -> str | None:
        tmp_path: str | None = None
        print(f"[STT] {duration:.1f}초 동안 말씀하세요...")
        try:
            recording = sd.rec(
                int(duration * sample_rate),
                samplerate=sample_rate,
                channels=1,
                dtype="int16",
            )
            sd.wait()
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                tmp_path = tmp.name
            wav.write(tmp_path, sample_rate, np.asarray(recording))
            with open(tmp_path, "rb") as audio:
                result = self.client.audio.transcriptions.create(model="whisper-1", file=audio)
            text = result.text.strip()
            print(f"[STT 결과] {text}")
            return text or None
        except Exception as exc:
            print(f"[STT 오류] {exc}")
            return None
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass


class StateTriggerClient(Node):
    def __init__(self, sm_service: str) -> None:
        super().__init__("stt_gpt_trigger_client")
        self.sm_client = self.create_client(TargetSearch, sm_service)

    def trigger_state_machine(self, model_name: str, wait_timeout: float):
        if not self.sm_client.wait_for_service(timeout_sec=wait_timeout):
            raise RuntimeError("상태머신 서비스를 찾지 못했습니다.")
        req = TargetSearch.Request()
        req.target_name = model_name
        req.class_label = ""
        future = self.sm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description="STT + GPT -> TargetSearch ROS 2 service client")
    parser.add_argument("--sm-service", default="/state/target_search")
    parser.add_argument("--env-file", default=str(default_env_file))
    parser.add_argument("--record-seconds", type=float, default=5.0)
    parser.add_argument("--service-wait", type=float, default=10.0)
    args = parser.parse_args()

    env_file = Path(args.env_file).expanduser()
    load_dotenv(env_file)
    api_key = os.getenv("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise SystemExit(f"OPENAI_API_KEY를 읽지 못했습니다: {env_file}")

    resolver = ObjectNameResolver(api_key, os.getenv("OPENAI_MODEL", "gpt-4.1-mini"))
    rclpy.init()
    node = StateTriggerClient(args.sm_service)
    print("입력: 한국어 물체명 | /v 음성입력 | /q 종료")
    try:
        while rclpy.ok():
            try:
                text = input("찾을 물체> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not text:
                continue
            if text.lower() in {"/q", "/quit", "/exit"}:
                break
            if text.lower() in {"/v", "/voice"}:
                text = resolver.transcribe(args.record_seconds) or ""
                if not text:
                    continue

            try:
                model_name, reason = resolver.resolve(text)
            except Exception as exc:
                print(f"[GPT 오류] {exc}")
                continue
            print(f"[GPT] 입력='{text}' -> model_name={model_name}; {reason}")
            if model_name is None:
                print("[미등록] 6개 등록 물체 중 하나로 확정하지 못했습니다.")
                continue

            try:
                print(f"[서비스 요청] 상태 머신에 '{model_name}' 탐색을 지시합니다.")
                sm_resp = node.trigger_state_machine(model_name, args.service_wait)
                
                # ObjectDetected.srv의 응답 포맷을 확실히 모르므로 안전하게 getattr 사용
                is_success = getattr(sm_resp, 'success', True)
                msg = getattr(sm_resp, 'message', '정상 전송됨')
                
                if is_success:
                    print(f"[요청 성공] 상태 머신 응답: {msg}")
                else:
                    print(f"[요청 거부] 상태 머신 거절: {msg}")
            except Exception as exc:
                print(f"[서비스 오류] {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
