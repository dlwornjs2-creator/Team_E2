"""홈 화면.

┌──────────────────┬──────────────────┐
│ 현재 작업 상태     │ 로봇 시스템 상태   │
├──────────────────┼──────────────────┤
│ 찾고 있는 3d 모델  │ 물체 위치 3D 맵    │
├──────────────────┴──────────────────┤
│ 최근 실행 경과                        │
└─────────────────────────────────────┘

HomeView는 화면 구조를 한 번만 만들고, 새 스냅샷이 올 때마다 update()가 각
패널의 안쪽 content만 다시 만들어 끼워 넣는다(패널 박스 자체는 그대로).

"찾고 있는 3d 모델"(ObjectViewer)과 "물체 위치 3D 맵"(MapView) 두 패널만 예외
— 둘 다 드래그로 카메라/회전 상태를 들고 있어야 해서 update()가 절대 다시
안 만든다. 폴링마다 body 전체를 새로 만들던 예전 방식은 그 서브트리까지
매번 새로 만들어서, 드래그하는 도중에 다른 패널이 갱신될 때마다 뚝뚝 끊기는
원인이었다.
"""

from pathlib import Path

import flet as ft

import theme as t
import labels as L
from components.panel import panel, kv
from components import status as st
from components.object_viewer import ObjectViewer
from render3d.map_view import MapView

# "찾고 있는 3d 모델" 패널 표시 우선순위: 드래그 회전 뷰어(.obj) > 사전 렌더
# 이미지(.png, tools/render_object.py) > 텍스트 카드. 2026-08-05 결정으로
# 인터랙티브 뷰어를 예외로 추가했다 — docs/hidden_object_search_ui_spec_v4.md 상단 참고.
ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets"
MODELS_DIR = ASSETS_DIR / "models"
RENDERS_DIR = ASSETS_DIR / "renders"

# ObjectViewer는 드래그 회전 상태(yaw/pitch)를 들고 있어야 해서 절대 매번
# 새로 만들면 안 된다. object_id별로 캐싱해서 재사용한다. HomeView 인스턴스
# 자체가 화면 전환 중에도 살아있긴 하지만(아래 main.py), 혹시 다시 만들어져도
# 이 캐시가 있으면 회전 상태가 남는다.
_viewer_cache: dict[str, ObjectViewer] = {}


def _get_viewer(object_id: str, obj_path: Path) -> ObjectViewer:
    viewer = _viewer_cache.get(object_id)
    if viewer is None:
        viewer = ObjectViewer(obj_path)
        _viewer_cache[object_id] = viewer
    return viewer


def _kv_control(key: str, control: ft.Control):
    """kv() 와 같은 모양이되 값 자리에 컨트롤을 넣는다."""
    return ft.Row(
        alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
        controls=[
            ft.Text(key, size=t.SIZE_BODY, color=t.TEXT_DIM),
            control,
        ],
    )


def _format_elapsed(seconds) -> str:
    """초 단위 숫자를 MM:SS 로. 값이 없으면 명세대로 --:--."""
    if seconds is None:
        return "--:--"
    seconds = int(seconds)
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def _stage_progress_ring(stage: str | None, status: str | None) -> ft.Control:
    """STAGE_ORDER 상 현재 단계 위치를 원형 진행률로 보여준다.

    스키마에 progress(%) 필드가 따로 없어서, task.stage가 STAGES 순서 중
    몇 번째인지로 계산한다 (idle=0% ~ done=100%).
    """
    order = L.STAGE_ORDER
    if stage in order and len(order) > 1:
        value = order.index(stage) / (len(order) - 1)
    else:
        value = 0.0

    if stage == "failed" or status in ("FAILED", "ERROR"):
        color = t.STATUS["error"]
    elif stage == "done" or status == "SUCCESS":
        color = t.STATUS["confirmed"]
    else:
        color = t.ACCENT

    size = 64
    return ft.Stack(
        width=size,
        height=size,
        controls=[
            ft.ProgressRing(
                value=value,
                width=size,
                height=size,
                stroke_width=6,
                color=color,
                bgcolor=t.SURFACE_ALT,
            ),
            ft.Container(
                width=size,
                height=size,
                alignment=ft.Alignment.CENTER,
                content=ft.Text(
                    f"{round(value * 100)}%",
                    size=t.SIZE_LABEL,
                    color=t.TEXT,
                    weight=ft.FontWeight.W_600,
                ),
            ),
        ],
    )


def _find_object(objects: list, target_id) -> dict | None:
    for obj in objects or []:
        if obj.get("id") == target_id:
            return obj
    return None


def _target_model_content(task: dict, objects: list) -> ft.Control:
    """대상 물체의 드래그 뷰어 / 사전 렌더 이미지, 없으면 텍스트 카드로 대체."""
    target_id = task.get("target_id")
    target_name = task.get("target_name")

    if not target_id:
        return ft.Container(
            alignment=ft.Alignment.CENTER,
            content=ft.Text("탐색 중인 물체 없음", size=t.SIZE_BODY, color=t.TEXT_FAINT),
        )

    model_path = MODELS_DIR / f"{target_id}.obj"
    if model_path.exists():
        viewer = _get_viewer(target_id, model_path)
        # Container.alignment=CENTER로 크기 고정된 자식을 가운데 놓는다.
        return ft.Container(
            expand=True,
            alignment=ft.Alignment.CENTER,
            content=ft.Column(
                tight=True,
                horizontal_alignment=ft.CrossAxisAlignment.CENTER,
                spacing=6,
                controls=[
                    viewer.control,
                    ft.Text("드래그해서 돌려보기", size=t.SIZE_LABEL, color=t.TEXT_FAINT),
                ],
            ),
        )

    render_path = RENDERS_DIR / f"{target_id}.png"
    if render_path.exists():
        return ft.Container(
            alignment=ft.Alignment.CENTER,
            content=ft.Image(
                src=f"renders/{target_id}.png",
                fit=ft.BoxFit.CONTAIN,
                height=180,
            ),
        )

    obj = _find_object(objects, target_id)
    category = obj.get("category") if obj else None
    return ft.Column(
        alignment=ft.MainAxisAlignment.CENTER,
        horizontal_alignment=ft.CrossAxisAlignment.CENTER,
        expand=True,
        controls=[
            ft.Text(target_name or target_id, size=t.SIZE_BIG, color=t.TEXT),
            ft.Text(category or L.EMPTY, size=t.SIZE_BODY, color=t.TEXT_DIM),
        ],
    )


class HomeView:
    """`.control`은 한 번만 만들고, `.update(snapshot)`으로 값만 갱신한다."""

    def __init__(self):
        self._current_task_body = ft.Column(spacing=8, controls=[])
        self._system_state_body = ft.Column(spacing=10, controls=[])
        self._target_model_body = ft.Container(alignment=ft.Alignment.CENTER)
        self._target_id_shown = object()  # target_id와 절대 안 같을 초기값

        current_task_panel = panel(
            "현재 작업 상태", accent=t.STATUS["searching"], content=self._current_task_body
        )
        system_state_panel = panel(
            "로봇 시스템 상태", accent=t.TEXT_FAINT, content=self._system_state_body
        )
        target_model_panel = panel("찾고 있는 3d 모델", content=self._target_model_body)

        # TODO(3d_map_spec 구현 순서 4~6): 바닥판·로봇 마커까지 됐고, 책상/벽/
        # zones/objects는 아직이다. dummy_points 성능 테스트는 끝났으니 껐다
        # (필요하면 다시 켤 수 있게 인자만 남겨둔다).
        # 카메라 회전 상태를 유지해야 해서 update()에서는 절대 다시 안 만든다
        # (ObjectViewer와 같은 이유 — home_view.py 상단 주석 참고).
        self._object_map_view = MapView(width=520, height=280, dummy_points=0)
        object_map_panel = panel("물체 위치 3D 맵", content=self._object_map_view.control)
        recent_panel = panel("최근 실행 경과")

        self.control = ft.Column(
            spacing=t.GAP,
            controls=[
                ft.Row([current_task_panel, system_state_panel], spacing=t.GAP, expand=3),
                ft.Row([target_model_panel, object_map_panel], spacing=t.GAP, expand=3),
                ft.Row([recent_panel], spacing=t.GAP, expand=2),
            ],
        )
        self.update(None)

    def update(self, snapshot: dict | None):
        system = (snapshot or {}).get("system", {})
        task = (snapshot or {}).get("task", {})
        objects = (snapshot or {}).get("objects", [])

        self._current_task_body.controls = [
            ft.Row(
                spacing=14,
                vertical_alignment=ft.CrossAxisAlignment.CENTER,
                controls=[
                    _stage_progress_ring(task.get("stage"), task.get("status")),
                    ft.Column(
                        spacing=6,
                        expand=True,
                        controls=[
                            ft.Text(
                                task.get("target_name") or "탐색 중인 물체 없음",
                                size=t.SIZE_BIG,
                                color=t.TEXT if task else t.TEXT_FAINT,
                            ),
                            _kv_control("작업 상태", st.task_status(task.get("status"))),
                        ],
                    ),
                ],
            ),
            ft.Divider(height=1, color=t.BORDER),
            kv("현재 단계", L.get(L.STAGE, task.get("stage"))),
            kv("경과 시간", _format_elapsed(task.get("elapsed_sec")), mono=True),
            kv("음성 명령", task.get("voice_command") or L.EMPTY),
        ]

        self._system_state_body.controls = [
            _kv_control("시스템 상태", st.system_state(system.get("state"))),
            ft.Divider(height=1, color=t.BORDER),
            ft.Text("노드", size=t.SIZE_LABEL, color=t.TEXT_DIM),
            st.node_list(system.get("nodes")),
            ft.Divider(height=1, color=t.BORDER),
            _kv_control("로봇 연결", st.online(system.get("robot_connected"))),
            _kv_control("카메라 연결", st.online(system.get("camera_connected"))),
            kv("그리퍼", L.get(L.GRIPPER, system.get("gripper_state"))),
            ft.Divider(height=1, color=t.BORDER),
            kv("등록 물체", str(system.get("object_count_total", 0)), mono=True),
            kv(
                "확인된 물체",
                str(system.get("object_count_confirmed", 0)),
                t.STATUS["confirmed"],
                mono=True,
            ),
            kv(
                "위치 불명",
                str(system.get("object_count_unknown", 0)),
                t.STATUS["unknown"],
                mono=True,
            ),
        ]

        # target_id가 안 바뀌었으면 이 안(드래그 뷰어 포함)은 절대 다시 안 만든다.
        target_id = task.get("target_id")
        if target_id != self._target_id_shown:
            self._target_id_shown = target_id
            self._target_model_body.content = _target_model_content(task, objects)
