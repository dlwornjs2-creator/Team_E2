"""홈 화면.

┌──────────────────┬──────────────────┐
│ 현재 작업 상태     │ 로봇 시스템 상태   │
├──────────────────┼──────────────────┤
│ 찾고 있는 3d 모델  │ 물체 위치 3D 맵    │
├──────────────────┴──────────────────┤
│ 최근 실행 경과                        │
└─────────────────────────────────────┘
"""

import flet as ft

import theme as t
from components.panel import panel, kv


def build_home() -> ft.Control:
    current_task = panel(
        "현재 작업 상태",
        accent=t.STATUS["searching"],
        content=ft.Column(
            spacing=8,
            controls=[
                ft.Text("탐색 중인 물체 없음", size=t.SIZE_BIG, color=t.TEXT),
                ft.Divider(height=1, color=t.BORDER),
                kv("작업 상태", "-"),
                kv("현재 단계", "-"),
                kv("경과 시간", "--:--", mono=True),
                kv("음성 명령", "-"),
            ],
        ),
    )

    system_state = panel(
        "로봇 시스템 상태",
        accent=t.SYSTEM_STATE["IDLE"],
        content=ft.Column(
            spacing=8,
            controls=[
                kv("시스템 상태", "IDLE", t.SYSTEM_STATE["IDLE"]),
                kv("로봇 연결", "-"),
                kv("카메라 연결", "-"),
                kv("그리퍼", "-"),
                ft.Divider(height=1, color=t.BORDER),
                kv("등록 물체", "0", mono=True),
                kv("확인된 물체", "0", t.STATUS["confirmed"], mono=True),
                kv("위치 불명", "0", t.STATUS["unknown"], mono=True),
            ],
        ),
    )

    target_model = panel("찾고 있는 3d 모델")
    object_map = panel("물체 위치 3D 맵")
    recent = panel("최근 실행 경과")

    return ft.Column(
        spacing=t.GAP,
        controls=[
            ft.Row([current_task, system_state], spacing=t.GAP, expand=3),
            ft.Row([target_model, object_map], spacing=t.GAP, expand=3),
            ft.Row([recent], spacing=t.GAP, expand=2),
        ],
    )