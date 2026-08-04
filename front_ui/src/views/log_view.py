"""로그 화면.

┌──────────┬──────────┐
│ 작업 기록 │ 물건 위치 │        ← 탭
├──────────┴──────────┴──────┐
│                            │
│  선택한 탭 내용               │
│                            │
└────────────────────────────┘
"""

import flet as ft

import theme as t
from components.panel import panel

TABS = ["작업 기록", "물건 위치"]


def build_log() -> ft.Control:
    body = ft.Container(expand=True)

    def render(index: int):
        body.content = panel(TABS[index])

    def make_tab(index: int, label: str):
        def on_click(e):
            for i, btn in enumerate(buttons):
                selected = i == index
                btn.bgcolor = t.ACCENT if selected else t.SURFACE
                btn.content.color = t.BG if selected else t.TEXT_DIM
            render(index)
            e.page.update()

        return ft.Container(
            width=150,
            height=34,
            border_radius=17,
            alignment=ft.Alignment.CENTER,
            bgcolor=t.ACCENT if index == 0 else t.SURFACE,
            border=ft.Border.all(1, t.BORDER),
            on_click=on_click,
            content=ft.Text(
                label,
                size=t.SIZE_BODY,
                weight=ft.FontWeight.W_600,
                color=t.BG if index == 0 else t.TEXT_DIM,
            ),
        )

    buttons = [make_tab(i, label) for i, label in enumerate(TABS)]
    render(0)

    return ft.Column(
        spacing=t.GAP,
        controls=[
            ft.Row(buttons, spacing=t.GAP),
            body,
        ],
    )