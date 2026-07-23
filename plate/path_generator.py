"""접시 세척 경로 생성.

접시 평면(사용자 좌표계 XY) 위의 동심 반원 Waypoint 를 계산한다.
로봇 의존성이 없는 순수 계산 모듈이므로 로봇 없이 단독 검증 가능하다.

와이퍼 방식: 각 반지름에서 지정 횟수만큼 왕복한 뒤 반지름을 줄여
안쪽으로 들어간다.
"""

import math


def semicircle_radii(plate_radius, brush_width, overlap, edge_margin):
    """세척할 동심 반원들의 반지름 목록. 바깥쪽부터 안쪽 순서.

    spacing = brush_width * (1 - overlap)
    솔 자체에 폭이 있으므로 가장 안쪽 반원이 중심을 정확히 지나지
    않아도 솔 접촉 영역이 중앙을 덮는다.
    """
    spacing = brush_width * (1.0 - overlap)
    if spacing <= 0:
        raise ValueError("spacing <= 0 : overlap 이 1 이상입니다")

    r_outer = plate_radius - edge_margin
    if r_outer <= 0:
        raise ValueError("edge_margin 이 접시 반지름보다 큽니다")

    radii = []
    r = r_outer
    r_min = brush_width / 2.0
    while r > r_min:
        radii.append(round(r, 3))
        r -= spacing

    if not radii:
        radii = [round(r_outer, 3)]
    return radii


def semicircle_points(radius, angle_step, reverse=False,
                      start_angle=0.0, sweep=180.0):
    """반지름 radius 인 반원 위의 점들. -> [(x, y), ...]"""
    steps = max(1, int(round(sweep / angle_step)))
    pts = []
    for i in range(steps + 1):
        deg = start_angle + (sweep * i / steps)
        rad = math.radians(deg)
        pts.append((radius * math.cos(rad), radius * math.sin(rad)))

    if reverse:
        pts.reverse()
    return pts


def generate_semicircle_path(plate_radius, brush_width, overlap,
                             edge_margin, angle_step, passes=2,
                             start_angle=0.0, sweep=180.0):
    """와이퍼식 동심 반원 세척 경로.

    각 반지름에서 passes 회 편도 스트로크(왕복)를 수행한 뒤 안쪽
    반지름으로 이동한다. 반지름이 바뀔 때는 직전 스트로크가 끝난
    쪽에서 시작하도록 방향을 맞춰 불필요한 복귀 이동을 없앤다.

      passes=1 : 왼→오
      passes=2 : 왼→오→왼      (1왕복)
      passes=4 : 2왕복

    반환: [[(x, y), ...], ...]  반지름 단위로 묶인 Waypoint 리스트
    """
    radii = semicircle_radii(plate_radius, brush_width, overlap, edge_margin)

    path = []
    ends_at_start = True   # 직전 반지름 스트로크가 시작쪽에서 끝났는지

    for r in radii:
        first_reverse = not ends_at_start   # 직전이 끝난 위치에서 이어지도록

        stroke = []
        for i in range(passes):
            reverse = first_reverse if i % 2 == 0 else not first_reverse
            pts = semicircle_points(r, angle_step, reverse=reverse,
                                    start_angle=start_angle, sweep=sweep)
            if stroke:
                pts = pts[1:]      # 앞 스트로크 끝점과 중복 제거
            stroke.extend(pts)

        path.append(stroke)

        if first_reverse:
            ends_at_start = (passes % 2 == 1)
        else:
            ends_at_start = (passes % 2 == 0)

    return path