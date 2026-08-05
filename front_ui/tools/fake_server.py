"""개발용 가짜 /state 서버.

back_ui가 실제로 내보낼 것과 완전히 같은 HTTP 인터페이스(주소·경로·JSON 스키마)를
흉내낸다. 스키마는 docs/hidden_object_search_ui_spec_v4.md 6장을 그대로 따른다.

back_ui가 준비되면 이 파일 대신 `ros2 run back_ui node`를 띄우기만 하면 되고,
front_ui 쪽(state_client.py, views/*)은 손대지 않는다. state_client는 이게
fake_server인지 진짜 back_ui인지 구분할 방법이 없다.

실행:
    conda activate front_ui
    cd front_ui
    python tools/fake_server.py
"""

import json
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import config as cfg  # noqa: E402
import labels as L  # noqa: E402

START = time.time()

STAGE_CODES = L.STAGE_ORDER  # idle ... done, 명세 6장 순서 그대로
STAGE_DURATION = 4.0  # 각 단계에 머무는 시간(초). 데모 속도 조절용.
DRAWER_PERIOD = 10.0  # 서랍 open_ratio 가 0->1->0 왕복하는 주기(초)


def _stage_progress(elapsed: float) -> tuple[str, float]:
    """경과 시간을 STAGES 순환으로 환산한다.

    (현재 stage 코드, 그 stage 안에서 흐른 시간) 을 돌려준다.
    """
    cycle = STAGE_DURATION * len(STAGE_CODES)
    pos = elapsed % cycle
    idx = int(pos // STAGE_DURATION)
    return STAGE_CODES[idx], pos - idx * STAGE_DURATION


def _drawer_open_ratio(elapsed: float) -> float:
    """0 -> 1 -> 0 삼각파.

    시작과 끝만 바뀌면 서랍이 순간이동하듯 보인다는 명세 6장 요구사항 때문에,
    가짜 서버에서도 중간값이 계속 갱신되게 한다.
    """
    pos = (elapsed % DRAWER_PERIOD) / DRAWER_PERIOD
    return round(1.0 - abs(1.0 - 2 * pos), 3)


def build_snapshot() -> dict:
    elapsed = time.time() - START
    stage, stage_elapsed = _stage_progress(elapsed)
    open_ratio = _drawer_open_ratio(elapsed)
    opening = _drawer_open_ratio(elapsed + 0.05) > open_ratio
    if open_ratio <= 0.02:
        zone_state = "closed"
    elif open_ratio >= 0.98:
        zone_state = "open"
    else:
        zone_state = "opening" if opening else "closing"

    return {
        "ts": time.time(),
        "frame_id": int(elapsed * 10),
        "system": {
            "state": "RUN",
            "nodes": {
                "image": True, "main": True, "db": True, "voice": True, "state": True,
            },
            "robot_connected": True,
            "camera_connected": True,
            "gripper_state": "open",
            "object_count_total": 12,
            "object_count_confirmed": 9,
            "object_count_unknown": 3,
        },
        "task": {
            "task_id": "task_demo_001",
            "voice_command": "빨간 컵 좀 찾아줘",
            "target_id": "cup_red_01",
            "target_name": "빨간 컵",
            "status": "RUNNING",
            "stage": stage,
            "action": "Drawer_A_2 내부를 관측하고 있습니다.",
            "action_reason": "초기 관측에서 대상이 확인되지 않았습니다.",
            "elapsed_sec": round(elapsed, 1),
            "current_zone": "drawer_a_2",
            "detections": [
                {"label": "빨간 컵 후보", "confidence": 0.91},
            ],
        },
        "objects": [
            {
                "id": "cup_red_01", "name": "빨간 컵", "category": "cup",
                "pos": [0.42, 0.18, 0.31], "zone": "drawer_a_2",
                "status": "confirmed", "confidence": 0.92,
                "last_seen": "2026-08-04T12:00:00",
            },
            {
                "id": "box_blue_01", "name": "파란 상자", "category": "box",
                "pos": None, "zone": None,
                "status": "unknown", "confidence": None,
                "last_seen": "2026-08-04T11:40:00",
            },
        ],
        "zones": [
            {
                "id": "drawer_a_2", "name": "Drawer A-2", "type": "drawer",
                "pos": [0.40, 0.10, 0.20], "size": [0.30, 0.40, 0.12],
                "open_axis": [1.0, 0.0, 0.0],
                "state": zone_state, "open_ratio": open_ratio,
                "search_state": "observing",
            },
        ],
        "recent_tasks": [
            {
                "task_id": "task_20260804_001", "target_name": "빨간 컵",
                "result": "SUCCESS", "ended_at": "2026-08-04T12:03:38", "duration_sec": 38,
            },
            {
                "task_id": "task_20260804_000", "target_name": "파란 상자",
                "result": "FAILED", "ended_at": "2026-08-04T11:40:52", "duration_sec": 52,
            },
        ],
    }


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, payload: dict, status: int = 200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == cfg.STATE_PATH:
            self._send_json(build_snapshot())
        elif path == cfg.HEALTH_PATH:
            self._send_json({"ok": True})
        elif path == cfg.FRAME_PATH:
            # TODO: 작업(monitor) 화면에서 카메라 패널을 붙일 때 실제 JPEG로 구현한다.
            self._send_json({"error": "not_implemented"}, status=501)
        else:
            self._send_json({"error": "not_found"}, status=404)

    def log_message(self, fmt, *args):
        pass  # 폴링마다 콘솔에 찍히면 개발 중 로그가 묻힌다. 필요하면 지운다.


def main():
    parsed = urlparse(cfg.BASE_URL)
    addr = (parsed.hostname, parsed.port)
    httpd = ThreadingHTTPServer(addr, Handler)
    print(f"fake_server: http://{addr[0]}:{addr[1]}{cfg.STATE_PATH}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
