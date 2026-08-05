"""front_ui가 폴링하는 HTTP 서버.

계약은 저장소 루트 `front_ui/README.md`("back_ui가 줘야 하는 것")를 그대로
따른다 — 주소·경로·응답 스키마가 거기 적힌 것과 정확히 같아야 front_ui
코드를 한 줄도 안 고치고 `tools/fake_server.py` 대신 이걸 붙일 수 있다.

front_ui 쪽 코드(config.py 등)는 안 가져온다 — 패키지가 다르고 conda(front_ui)
/ 시스템 파이썬(ROS) 환경도 서로 다르다(CLAUDE.md 절대 규칙 1번). 주소
상수만 여기 그대로 다시 적어서 값만 맞춘다.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HOST = "127.0.0.1"
PORT = 8765

STATE_PATH = "/state"
HEALTH_PATH = "/health"
FRAME_PATH = "/frame.jpg"


def _make_handler(state_store, frame_store):
    """StateStore/FrameStore를 클로저로 들고 있는 Handler 클래스를 만든다.

    BaseHTTPRequestHandler는 요청마다 새로 인스턴스화되는 구조라 생성자로
    의존성을 넘길 방법이 없다 — 그래서 클래스를 그때그때 만들어서 넘긴다.
    """

    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, payload: dict, status: int = 200):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _send_jpeg(self, jpeg_bytes: bytes):
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg_bytes)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(jpeg_bytes)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == STATE_PATH:
                self._send_json(state_store.get_snapshot())
            elif path == HEALTH_PATH:
                self._send_json({"ok": True})
            elif path == FRAME_PATH:
                _frame_id, jpeg_bytes = frame_store.get_latest()
                if jpeg_bytes is None:
                    # 아직 이미지 토픽 구독이 없다 — fake_server.py와 같은 스텁 응답.
                    self._send_json({"error": "not_implemented"}, status=501)
                else:
                    self._send_jpeg(jpeg_bytes)
            else:
                self._send_json({"error": "not_found"}, status=404)

        def log_message(self, fmt, *args):
            pass  # ROS 로그(get_logger)와 섞이면 콘솔이 지저분해진다.

    return Handler


def start_server(state_store, frame_store, host: str = HOST, port: int = PORT) -> ThreadingHTTPServer:
    """백그라운드 스레드에서 서버를 띄우고 httpd 인스턴스를 돌려준다.

    ROS 노드는 rclpy.spin()이 메인 스레드를 차지해야(콜백을 받으려면) 해서,
    HTTP 서버는 별도 스레드에서 돌아야 한다. 반환값은 node.py가 종료할 때
    `httpd.shutdown()`을 부르는 데 쓴다.
    """
    handler_cls = _make_handler(state_store, frame_store)
    httpd = ThreadingHTTPServer((host, port), handler_cls)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd
