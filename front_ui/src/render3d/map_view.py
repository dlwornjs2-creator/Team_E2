"""3D 지도 Flet 컨트롤. 마우스로 돌리고 확대한다.

명세: docs/3d_map_spec.md 8장.

지금은 구현 순서 1~3(투영 성능 측정 -> 그리드/축 -> 마우스 회전·줌) 단계다.
zones/objects/robot을 실제로 그리는 건 4~6단계에서 이 파일의 `_redraw()`에
draw_box_wire·draw_marker 호출을 추가하는 식으로 이어붙인다. `build_map()`
(snapshot을 받는 최종 형태)도 그때 생긴다 — 지금은 더미 점으로 좌표계와
Canvas 성능만 확인한다.
"""

import time

import flet as ft
import flet.canvas as cv
import numpy as np

import theme as t
from render3d.projection import Camera
from render3d.scene import load_scene_config
from render3d.shapes import draw_axes, draw_grid, draw_points, draw_robot_marker

_DEFAULT_YAW = -0.6
_DEFAULT_PITCH = 0.5
_DEFAULT_SCALE = 300.0
_SCALE_MIN, _SCALE_MAX = 50.0, 1200.0
_PITCH_LIMIT = 1.4  # 라디안. 이 이상 돌면 화면이 뒤집힌다 (명세 8장)

_DRAG_SENSITIVITY = 0.01  # 드래그 픽셀 -> 라디안
_SCROLL_SENSITIVITY = 0.0015  # 휠 델타 -> 배율 변화


class MapView:
    """`.control`을 패널 content에 넣는다. 드래그=회전, 휠=확대, 더블클릭=리셋."""

    def __init__(self, width: int = 480, height: int = 320, dummy_points: int = 0):
        self._width = width
        self._height = height
        self._cam = Camera(
            yaw=_DEFAULT_YAW,
            pitch=_DEFAULT_PITCH,
            scale=_DEFAULT_SCALE,
            center=(width / 2, height / 2),
        )
        self._yaw0 = self._cam.yaw
        self._pitch0 = self._cam.pitch

        # 구현 순서 1~3 확인용 더미 점. zones/objects가 실제로 붙으면(4~6단계) 없앤다.
        if dummy_points:
            rng = np.random.default_rng(0)
            self._dummy_points = (rng.random((dummy_points, 3)) - 0.5) * 1.6
        else:
            self._dummy_points = None

        # 바닥판 120x64cm, 로봇 위치는 왼쪽 변에서 28cm(y) / 뒤쪽 변에서 33cm(x).
        # base_link(로봇=원점) 기준으로 계산한 값이 assets/scene_config.json에 있다.
        scene = load_scene_config()
        self._floor_cfg = scene.get("floor", {"size": [2.0, 2.0], "grid_step": 0.2})
        self._boxes = scene.get("boxes", [])  # TODO(4단계): draw_box_wire로 그린다

        self._frame_label = ft.Text(
            "", size=t.SIZE_LABEL, color=t.TEXT_FAINT, font_family=t.MONO
        )
        self._canvas = cv.Canvas(width=width, height=height, shapes=[])
        self._redraw()

        self.control = ft.GestureDetector(
            content=ft.Stack(
                width=width,
                height=height,
                controls=[
                    self._canvas,
                    ft.Container(top=4, left=6, content=self._frame_label),
                ],
            ),
            drag_interval=16,  # ~60fps. "조작 중 지연이 느껴지면 안 된다"(명세 8장)
            mouse_cursor=ft.MouseCursor.MOVE,
            on_pan_start=self._on_pan_start,
            on_pan_update=self._on_pan_update,
            on_scroll=self._on_scroll,
            on_double_tap=self._on_double_tap,
        )

    def _redraw(self):
        """shapes를 다시 계산해 Canvas에 반영하고, 걸린 시간을 라벨에 찍는다.

        측정 대상은 "numpy 투영 + 도형 리스트 구성"까지다 — 명세 12장이 말하는
        진짜 병목(Canvas에 도형 수천 개를 넘기는 비용)은 이 함수가 끝난 뒤
        클라이언트가 실제로 그리는 단계라 여기선 못 잰다. 드래그해보면서
        눈으로 끊기는지 확인해야 하는 이유다.
        """
        t0 = time.perf_counter()
        shapes: list = []
        draw_grid(shapes, self._floor_cfg, self._cam)
        draw_axes(shapes, self._cam)
        if self._dummy_points is not None:
            draw_points(shapes, self._dummy_points, self._cam, t.STATUS["searching"], radius=1.5)
        draw_robot_marker(shapes, self._cam)  # 큰 빨간 점. 링크 포인트클라우드 붙기 전 임시 표현
        elapsed_ms = (time.perf_counter() - t0) * 1000

        self._canvas.shapes = shapes
        self._frame_label.value = f"{elapsed_ms:.2f} ms · 도형 {len(shapes)}개"

    def _on_pan_start(self, e: ft.DragStartEvent):
        self._yaw0 = self._cam.yaw
        self._pitch0 = self._cam.pitch

    def _on_pan_update(self, e: ft.DragUpdateEvent):
        delta = e.global_delta
        if delta is None:
            return
        self._cam.yaw = self._yaw0 + delta.x * _DRAG_SENSITIVITY
        pitch = self._pitch0 - delta.y * _DRAG_SENSITIVITY
        self._cam.pitch = max(-_PITCH_LIMIT, min(_PITCH_LIMIT, pitch))
        self._redraw()
        self.control.update()

    def _on_scroll(self, e: ft.ScrollEvent):
        factor = 1.0 - e.scroll_delta.y * _SCROLL_SENSITIVITY
        new_scale = self._cam.scale * factor
        self._cam.scale = max(_SCALE_MIN, min(_SCALE_MAX, new_scale))
        self._redraw()
        self.control.update()

    def _on_double_tap(self, e):
        self._cam.yaw = _DEFAULT_YAW
        self._cam.pitch = _DEFAULT_PITCH
        self._cam.scale = _DEFAULT_SCALE
        self._redraw()
        self.control.update()
