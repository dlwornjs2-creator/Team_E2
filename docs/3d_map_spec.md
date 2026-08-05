# 3D 맵 구현 명세

`front_ui`의 3D 지도 부분만 다룬다. 전체 명세는 `hidden_object_search_ui_spec_v4.md` 참고.

이 문서 하나만 읽고 구현할 수 있어야 한다.

---

## 1. 무엇을 만드는가

로봇 작업 공간을 위에서 비스듬히 내려다본 3D 뷰. 두 화면에서 쓴다.

| 화면 | 목적 | 갱신 |
|---|---|---|
| 홈 | DB에 등록된 전체 물체 위치 조망 | 500 ms |
| 작업 | 현재 탐색 중인 구역 강조 | 100 ms |

**같은 렌더 함수를 옵션만 다르게 호출한다.** 구현을 두 벌 만들지 않는다.

### 그리는 대상

| 대상 | 표현 | 데이터 출처 |
|---|---|---|
| 바닥 | 그리드 선 | `scene_config.json` (고정) |
| 책상 / 벽 | 와이어프레임 상자 | `scene_config.json` (고정) |
| 선반 / 서랍장 / 문 | 와이어프레임 상자 | `/state` 의 `zones` |
| 물체 | 점 + 이름 라벨 | `/state` 의 `objects` |
| 로봇 팔 | 포인트 클라우드 | `/state` 의 `robot.links` + 사전 샘플링 점 |

---

## 2. 절대 제약

### 런타임 3D 라이브러리를 쓰지 않는다

`pyrender`, `open3d`, `trimesh`, `matplotlib`, OpenGL, WebView 모두 런타임에 쓰지 않는다.
헤드리스 환경 의존성과 발표 PC 실행 실패 위험을 없애기 위한 결정이다.

**런타임 의존성은 `flet`, `numpy`, `requests` 세 개뿐이다.**

`trimesh`는 개발 PC에서 점을 뽑는 오프라인 스크립트에서만 쓴다.
그 스크립트는 배포에 포함되지 않는다.

### RViz를 임베드하지 않는다

Flet은 Flutter 캔버스, RViz는 Qt+OpenGL 별도 프로세스라 임베드가 불가능하다.
화면 캡처를 스트리밍하는 방법은 상호작용이 사라지므로 채택하지 않는다.

### Flet 1.x API

인터넷 예제는 대부분 0.x다. 소문자 모듈이 대문자 클래스로 바뀌었다.

| 0.x | 1.x |
|---|---|
| `ft.padding.all()` | `ft.Padding.all()` |
| `ft.border.all()` | `ft.Border.all()` |
| `ft.alignment.center` | `ft.Alignment.CENTER` |
| `ft.colors.RED` | `ft.Colors.RED` |

Canvas는 `import flet.canvas as cv` 로 쓴다.
`page.update()`는 이벤트 핸들러 밖(폴링 스레드 등)에서만 직접 부른다.

---

## 3. 좌표계

- 기준: **`base_link`**. 고정형 팔이므로 `base_link` = world = 원점 (0,0,0)
- 축: **x 전방, y 좌측, z 상방** (REP-103, 오른손, z-up)
- 단위: **미터**
- `/state`로 들어오는 좌표는 이미 변환이 끝난 값이다. front_ui는 TF를 다루지 않는다

로봇이 원점이므로 "로봇 위치"를 따로 받지 않는다. 원점에 좌표축 마커만 그린다.

---

## 4. 파일 구조

```
front_ui/
├── src/
│   ├── render3d/
│   │   ├── __init__.py
│   │   ├── projection.py      투영 계산. 표현이 바뀌어도 변하지 않는 부분
│   │   ├── shapes.py          상자·그리드·점 그리기
│   │   ├── map_view.py        Flet 컨트롤. 마우스 조작 포함
│   │   └── scene.py           scene_config.json 로딩
│   └── assets/
│       ├── scene_config.json  고정 배경 (바닥, 책상, 벽)
│       └── points/            사전 샘플링된 링크 점
│           ├── link1.json
│           └── ...
└── tools/
    └── mesh_to_points.py      개발 PC 전용. URDF 메시 → 점 JSON
```

---

## 5. projection.py

가장 먼저 만든다. 나머지가 전부 이걸 쓴다.

### 요구사항

```python
class Camera:
    """시점. yaw/pitch/scale 을 바꾸면 화면이 회전·확대된다."""
    yaw: float      # 라디안. 기본 -0.6
    pitch: float    # 라디안. 기본 0.5 (위에서 내려다봄)
    scale: float    # 픽셀 / 미터. 기본 300
    center: tuple   # 화면 중심 픽셀 좌표

    def matrix(self) -> np.ndarray:  # (3,3)
        ...

def project(points_3d: np.ndarray, cam: Camera) -> np.ndarray:
    """(N,3) base_link 좌표 -> (N,3) [x_px, y_px, depth]

    직교투영을 쓴다. 원근투영은 쓰지 않는다 (작업 공간이 1~2m 라 차이가 없고,
    직교가 좌표 읽기에 더 유리하다).

    depth 는 정렬용이며 값이 클수록 뒤쪽이다.
    """
```

**반드시 벡터화할 것.** 점 하나씩 반복문으로 돌리면 링크 점 수천 개에서 느려진다.
`points @ R.T` 형태의 행렬곱 한 번으로 처리한다.

y축은 화면에서 아래로 증가하므로 z를 뒤집어야 위가 위로 간다.

---

## 6. shapes.py

### draw_grid

바닥 그리드. `scene_config.json`의 `floor` 사용.
선 색은 `theme.BORDER`, 원점을 지나는 두 선만 조금 밝게.

### draw_box_wire

직육면체를 와이어프레임으로. 꼭짓점 8개를 투영하고 모서리 12개를 선으로 잇는다.

**와이어프레임을 쓰는 이유:** 채운 면으로 그리면 서랍 안의 물체와 로봇 팔이
가려진다. 또 painter's algorithm 특성상 팔이 상자를 관통할 때 앞뒤가 틀리게 나온다.
와이어프레임은 두 문제가 동시에 사라진다.

```python
def draw_box_wire(shapes, center, size, cam, color, rpy=None, width=1.0):
    ...
```

### draw_points

포인트 클라우드. 로봇 팔에 쓴다.

```python
def draw_points(shapes, points_3d, cam, color, radius=1.2):
    """depth 내림차순 정렬 후 뒤에서부터 찍는다."""
    proj = project(points_3d, cam)
    order = np.argsort(-proj[:, 2])
    for x, y, _ in proj[order]:
        shapes.append(cv.Circle(float(x), float(y), radius, paint=...))
```

### draw_marker

물체 하나. 점 + 이름 라벨.
`depth`에 따라 반지름을 3~6px 사이로 조절해 원근감을 준다.
색은 `theme.STATUS[obj["status"]]`.

### draw_axes

원점에 x(빨강) / y(초록) / z(파랑) 축을 각 10cm 길이로. 로봇 위치 표시 역할.

---

## 7. 서랍·문 열림 표현

프로젝트 핵심 메시지를 화면에서 보여 주는 부분이다. 반드시 구현한다.

```python
# 서랍: 열림 축 방향으로 이동
offset = np.array(zone["open_axis"]) * zone["open_ratio"] * zone["size"][axis_idx]
center = np.array(zone["pos"]) + offset

# 문: 힌지 기준 회전
angle = zone["open_ratio"] * math.pi / 2
```

`open_ratio`는 0.0~1.0 연속값이며 여는 동안 계속 갱신된다.
따라서 **매 프레임 위치를 다시 계산해야 한다.** 캐시하지 말 것.

현재 탐색 중인 구역(`task.current_zone`)은 테두리를 `theme.ACCENT`로 강조한다.

---

## 8. map_view.py

Flet 컨트롤로 감싼다.

```python
def build_map(snapshot: dict, cam: Camera, show_robot: bool = True) -> ft.Control:
    """cv.Canvas 를 담은 컨트롤을 반환한다."""
```

### 그리는 순서 (뒤 → 앞)

1. 바닥 그리드
2. 고정 배경 상자 (책상, 벽)
3. `zones` 상자
4. 로봇 팔 포인트 클라우드
5. `objects` 마커
6. 원점 좌표축
7. 라벨 텍스트

**라벨을 마지막에 그린다.** 중간에 그리면 점에 가려진다.

### 마우스 조작

| 동작 | 결과 |
|---|---|
| 드래그 | `cam.yaw`, `cam.pitch` 변경 |
| 휠 | `cam.scale` 변경 (50~1200 범위 제한) |
| 더블클릭 | 기본 시점 복귀 |

`pitch`는 -1.4 ~ 1.4 라디안으로 제한한다. 넘어가면 화면이 뒤집힌다.

`ft.GestureDetector`로 감싸고, 조작 중에는 폴링 데이터를 기다리지 말고
즉시 다시 그린다. 지연이 느껴지면 안 된다.

---

## 9. scene_config.json

고정 배경. 로봇이 몰라도 되고 UI에서만 쓰므로 `/state`로 받지 않는다.
줄자로 대략 재서 넣는다. ±5cm 오차는 문제없다.

```json
{
  "floor": { "size": [2.0, 2.0], "grid_step": 0.2 },
  "boxes": [
    { "name": "desk", "pos": [0.5, 0.0, 0.36], "size": [1.2, 0.6, 0.72] },
    { "name": "wall", "pos": [1.0, 0.0, 0.75], "size": [0.05, 2.0, 1.5] }
  ]
}
```

`pos`는 상자 **중심** 좌표다. 바닥에 놓인 책상이면 z = 높이/2 가 된다.

---

## 10. 로봇 팔 포인트 클라우드

### 오프라인 샘플링 (개발 PC에서 한 번만)

```python
# tools/mesh_to_points.py
import trimesh, numpy as np, json, pathlib

LINK_MESHES = {
    "link1": "/path/to/link1.stl",
    # URDF 의 <visual><geometry><mesh filename="..."> 에서 찾는다
}
N_POINTS = 300

out = pathlib.Path("src/assets/points")
out.mkdir(parents=True, exist_ok=True)

for name, path in LINK_MESHES.items():
    mesh = trimesh.load(path, force="mesh")
    pts, _ = trimesh.sample.sample_surface(mesh, N_POINTS)
    json.dump(np.round(pts, 4).tolist(), open(out / f"{name}.json", "w"))
    print(f"{name}: {len(pts)} points")
```

주의사항:

- 메시 단위가 mm 인 경우가 있다. 1000으로 나눠 m 로 맞춘다
- 메시 원점이 링크 원점과 다르면 URDF 의 `<origin>` 오프셋을 적용한다
- `sample_surface`는 표면적 비례로 균등하게 뽑는다

### 런타임 적용

```python
pts = LINK_POINTS[name]                     # (N,3) 링크 로컬 좌표
world = pts @ R_from_rpy(link["rpy"]).T + np.array(link["pos"])
draw_points(shapes, world, cam, color)
```

`link["pos"]`, `link["rpy"]`는 `/state`의 `robot.links`에서 온다.
`back_ui`가 TF를 조회해 base_link 기준으로 변환한 값이다.

### 점 개수 배분

밀도와 밝기로 위계를 만든다. 전부 똑같이 그리면 뭐가 뭔지 알 수 없다.

| 대상 | 점 개수 | 밝기 |
|---|---|---|
| 로봇 팔 | 링크당 200~400 | 밝게 |
| 배경 상자 | 와이어프레임 | 어둡게 |
| 물체 | 큰 점 1개 | 상태 색, 가장 밝게 |

---

## 11. 스냅샷 스키마 (이 문서 관련 부분만)

```json
{
  "task": { "current_zone": "drawer_a_2" },

  "objects": [
    { "id": "cup_red_01", "name": "빨간 컵",
      "pos": [0.42, 0.18, 0.31], "status": "confirmed" }
  ],

  "zones": [
    { "id": "drawer_a_2", "name": "Drawer A-2", "type": "drawer",
      "pos": [0.40, -0.15, 0.20], "size": [0.30, 0.40, 0.12],
      "open_axis": [1.0, 0.0, 0.0], "open_ratio": 0.8,
      "search_state": "observing" }
  ],

  "robot": {
    "links": [
      { "name": "link1", "pos": [0, 0, 0.15], "rpy": [0, 0, 0.52] }
    ]
  }
}
```

`objects[].status` 는 `theme.STATUS` 의 키다:
`unknown` / `searching` / `confirmed` / `held` / `warning` / `error`

`robot`이 없거나 비어 있으면 팔을 그리지 않는다. 오류가 아니다.

---

## 12. 성능 목표

- 총 점 2000~3000개에서 작업 화면 10 Hz 유지
- numpy 연산은 프레임당 1 ms 미만이어야 한다 (벡터화 필수)
- 실제 병목은 Flet Canvas 에 도형 수천 개를 넘기는 쪽이다

**Canvas 성능은 초반에 반드시 측정한다.** 점 3000개를 더미로 뿌리고
프레임 시간을 재본 뒤 나머지를 구현할 것. 안 나오면 대응책이 있다.

- 점 개수를 절반으로 줄인다 (형태는 유지된다)
- 홈 화면은 2 Hz 로 낮춘다
- 배경은 시점이 바뀔 때만 다시 그리고 팔만 매 프레임 갱신한다

---

## 13. 구현 순서

각 단계가 끝나면 실행해서 눈으로 확인한 뒤 다음으로 간다.

1. **`projection.py` + 성능 측정**
   더미 점 3000개를 랜덤 생성해 화면에 뿌리고 프레임 시간을 잰다.
   여기서 성능이 안 나오면 이후 설계가 달라지므로 가장 먼저 확인한다.

2. **`draw_grid` + `draw_axes`**
   바닥 그리드와 원점 축만. 좌표계가 맞는지 여기서 확인한다.
   x가 앞, y가 왼쪽, z가 위로 가는지 반드시 눈으로 볼 것.

3. **마우스 회전·줌**
   드래그하면 그리드가 따라 도는지 확인. 이게 되면 이후 작업이 훨씬 수월하다.

4. **`draw_box_wire` + `scene_config.json`**
   책상과 벽이 뜬다.

5. **`zones` 렌더링 + 서랍 열림**
   `open_ratio`를 0→1로 흔들어 서랍이 스르륵 나오는지 확인한다.

6. **`objects` 마커 + 라벨**

7. **`mesh_to_points.py` + 로봇 팔**
   마지막에 붙인다. 앞 단계가 다 되어 있어야 확인이 쉽다.

---

## 14. 하지 말 것

- 요청하지 않은 기능 추가 (물체 클릭 상세, 애니메이션 보간, 시점 프리셋 등)
- 런타임에 STL/DAE 파싱
- 삼각형 단위 렌더링 (RViz 재구현이 된다)
- 채운 면으로 상자 그리기 (겹침 문제 발생)
- 배경을 포인트 클라우드로 (물체와 팔이 묻힌다)
- 원근투영
- 색 하드코딩 (`theme.py` 상수를 쓴다)

각 단계마다 실행 가능한 상태로 멈추고 확인받은 뒤 다음으로 간다.

---

## 15. 화면별 적용 (front_ui 결정사항)

- **홈**: `build_map(snapshot, cam, show_robot=False)` — 로봇 팔 제외, 전체 물체·구역 조망
- **작업**: `build_map(snapshot, cam, show_robot=True)` — 로봇 팔 포함, `task.current_zone` 강조

로봇 팔(`mesh_to_points.py`, 10장)은 구현 순서 7번(맨 마지막)이고 홈 화면에는
애초에 안 쓰이므로, 홈 화면 작업 범위에서는 뒤로 미룬다.
