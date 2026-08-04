"""front_ui 진입점.

좌측 네비게이션 + 상단 바 + 화면 전환만 담당한다.
데이터 폴링은 client/state_client.py 가 생기면 여기에 연결한다.
"""

import flet as ft

import theme as t
from views.home_view import build_home
from views.monitor_view import build_monitor
from views.log_view import build_log

PAGES = [
    ("홈", ft.Icons.HOME_OUTLINED, ft.Icons.HOME, build_home),
    ("작업", ft.Icons.VISIBILITY_OUTLINED, ft.Icons.VISIBILITY, build_monitor),
    ("로그", ft.Icons.LIST_ALT_OUTLINED, ft.Icons.LIST_ALT, build_log),
]


def main(page: ft.Page):
    page.title = "숨은 물체 탐색 모니터"
    page.bgcolor = t.BG
    page.padding = 0
    page.spacing = 0
    page.theme_mode = ft.ThemeMode.DARK
    page.window.width = 1440
    page.window.height = 900
    page.window.min_width = 1100
    page.window.min_height = 700

    title = ft.Text(PAGES[0][0], size=t.SIZE_BODY, color=t.TEXT_DIM)

    # 연결 상태 배지. state_client 가 붙으면 여기 색과 글자를 바꾼다.
    conn_dot = ft.Container(width=7, height=7, border_radius=4, bgcolor=t.STATUS["error"])
    conn_text = ft.Text("연결 끊김", size=t.SIZE_LABEL, color=t.TEXT_DIM)

    topbar = ft.Container(
        height=44,
        padding=ft.Padding.symmetric(horizontal=16),
        bgcolor=t.RAIL,
        border=ft.Border.only(bottom=ft.BorderSide(1, t.BORDER)),
        content=ft.Row(
            alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
            controls=[
                title,
                ft.Row([conn_dot, conn_text], spacing=6),
            ],
        ),
    )

    body = ft.Container(
        expand=True,
        padding=t.GAP,
        content=build_home(),
    )

    def on_nav_change(e):
        idx = e.control.selected_index
        label, _, _, builder = PAGES[idx]
        title.value = label
        body.content = builder()

    rail = ft.NavigationRail(
        selected_index=0,
        bgcolor=t.RAIL,
        min_width=76,
        label_type=ft.NavigationRailLabelType.ALL,
        indicator_color=t.SURFACE_ALT,
        on_change=on_nav_change,
        destinations=[
            ft.NavigationRailDestination(icon=icon, selected_icon=sel, label=label)
            for label, icon, sel, _ in PAGES
        ],
    )

    page.controls.append(
        ft.Row(
            spacing=0,
            expand=True,
            controls=[
                rail,
                ft.Container(width=1, bgcolor=t.BORDER),
                ft.Column(spacing=0, expand=True, controls=[topbar, body]),
            ],
        )
    )


ft.run(main)