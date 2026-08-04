import flet as ft


def main(page: ft.Page):
    page.title = "숨은 물체 탐색 모니터"
    page.padding = 0

    body = ft.Container(content=ft.Text("여기에 화면", size=20), expand=True)

    def on_nav_change(e):
        idx = e.control.selected_index
        body.content = ft.Text(["홈", "작업", "로그"][idx], size=20)

    rail = ft.NavigationRail(
        selected_index=0,
        label_type=ft.NavigationRailLabelType.ALL,
        destinations=[
            ft.NavigationRailDestination(icon=ft.Icons.HOME, label="홈"),
            ft.NavigationRailDestination(icon=ft.Icons.VISIBILITY, label="작업"),
            ft.NavigationRailDestination(icon=ft.Icons.LIST_ALT, label="로그"),
        ],
        on_change=on_nav_change,
    )

    page.controls.append(
        ft.Row([rail, ft.VerticalDivider(width=1), body], expand=True)
    )


ft.run(main)