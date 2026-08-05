"""작업 화면 (탐색 모니터).

┌──────────────────┬──────────────────┐
│ 현재 카메라 화면   │ 현재 판단과 행동   │
├──────────────────┼──────────────────┤
│ 3D 지도           │ 현재 실행 단계     │
├──────────────────┴──────────────────┤
│ 작업 정보                             │
└─────────────────────────────────────┘
"""

import flet as ft

import theme as t
from components.panel import panel, kv

# 명세 5장 stage 코드값 순서
STAGES = [
    ("idle", "대기"),
    ("initial_observe", "초기 관측"),
    ("select_zone", "탐색 위치 선정"),
    ("open_container", "서랍/문 열기"),
    ("internal_reobserve", "내부 재관측"),
    ("verify_candidate", "후보 확인"),
    ("approach_grasp", "파지 접근"),
    ("verify_grasp", "파지 검증"),
    ("transport", "지정 위치로 이송"),
    ("place", "물체 내려놓기"),
    ("return_home", "홈 복귀"),
    ("done", "완료"),
]


def stage_row(label: str, state: str):
    """state: done | current | todo"""
    if state == "current":
        dot, color, weight = t.ACCENT, t.TEXT, ft.FontWeight.W_600
    elif state == "done":
        dot, color, weight = t.STATUS["confirmed"], t.TEXT_DIM, ft.FontWeight.NORMAL
    else:
        dot, color, weight = t.BORDER, t.TEXT_FAINT, ft.FontWeight.NORMAL

    return ft.Row(
        spacing=8,
        controls=[
            ft.Container(width=6, height=6, bgcolor=dot, border_radius=3),
            ft.Text(label, size=t.SIZE_BODY, color=color, weight=weight),
        ],
    )


def build_monitor(snapshot: dict | None = None) -> ft.Control:
    # TODO: 이 화면은 아직 snapshot을 쓰지 않는다. "다음 작업 순서" 3번(화면 연결)에서 채운다.
    camera = panel(
        "현재 카메라 화면",
        content=ft.Column(
            spacing=8,
            controls=[
                ft.Container(
                    expand=True,
                    bgcolor="#0A0D12",
                    border_radius=4,
                    alignment=ft.Alignment.CENTER,
                    content=ft.Text(
                        "카메라 연결 없음",
                        size=t.SIZE_BODY,
                        color=t.TEXT_FAINT,
                    ),
                ),
                ft.Text("검출 결과 없음", size=t.SIZE_BODY, color=t.TEXT_DIM),
            ],
        ),
    )

    action = panel(
        "현재 판단과 행동",
        accent=t.ACCENT,
        content=ft.Column(
            spacing=14,
            controls=[
                ft.Column(
                    spacing=4,
                    controls=[
                        ft.Text("현재 행동", size=t.SIZE_LABEL, color=t.TEXT_DIM),
                        ft.Text("-", size=t.SIZE_VALUE, color=t.TEXT),
                    ],
                ),
                ft.Divider(height=1, color=t.BORDER),
                ft.Column(
                    spacing=4,
                    controls=[
                        ft.Text("행동 이유", size=t.SIZE_LABEL, color=t.TEXT_DIM),
                        ft.Text("-", size=t.SIZE_BODY, color=t.TEXT_DIM),
                    ],
                ),
            ],
        ),
    )

    map3d = panel("3D 지도")

    stages = panel(
        "현재 실행 단계",
        content=ft.Column(
            spacing=7,
            scroll=ft.ScrollMode.AUTO,
            controls=[stage_row(label, "todo") for _, label in STAGES],
        ),
    )

    info = panel(
        "작업 정보",
        content=ft.Row(
            spacing=t.GAP * 3,
            controls=[
                ft.Column(
                    spacing=6,
                    expand=1,
                    controls=[
                        kv("작업 ID", "-", mono=True),
                        kv("대상 물체", "-"),
                        kv("음성 명령", "-"),
                    ],
                ),
                ft.Column(
                    spacing=6,
                    expand=1,
                    controls=[
                        kv("작업 상태", "-"),
                        kv("경과 시간", "--:--", mono=True),
                        kv("현재 구역", "-", mono=True),
                    ],
                ),
                ft.Container(expand=2, content=ft.Text("탐색 영역 상태 표", size=t.SIZE_BODY, color=t.TEXT_FAINT)),
            ],
        ),
    )

    return ft.Column(
        spacing=t.GAP,
        controls=[
            ft.Row([camera, action], spacing=t.GAP, expand=3),
            ft.Row([map3d, stages], spacing=t.GAP, expand=3),
            ft.Row([info], spacing=t.GAP, expand=2),
        ],
    )