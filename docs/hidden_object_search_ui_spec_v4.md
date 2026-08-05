# 구현 명세 v4: 숨은 물체 탐색 프로젝트 UI

> v3 대비 변경 사항
> - **Flet 1.x API 주의사항** 섹션 신설 (0.x 예제와 다른 부분)
> - `src/theme.py`, `src/config.py`, `src/components/panel.py` 추가 반영
> - 화면 레이아웃 구현 완료 상태 반영, 진행 현황 표 추가
> - 화면별 담당 파일 명시

> **2026-08-05 정정** — 8장·11장의 "런타임 OBJ 로딩 금지", "자유 회전 인터랙티브
> 3D 뷰어 금지"는 **홈 화면 "찾고 있는 3d 모델" 패널에 한해 예외**로 바뀌었다.
> 사용자가 직접 승인한 결정이다. pyrender/trimesh/open3d 같은 외부 3D 라이브러리는
> 여전히 안 쓴다 — `components/object_viewer.py`가 numpy만으로 OBJ 면 중심점을
> 투영·셰이딩해서 점으로 스플랫하고 BMP로 인코딩해 그린다. 8장의 "3D 지도"(zone
> 박스·물체 점 지도)에 대한 러너타임 렌더러 금지는 그대로 유효하다 — 이건 이
> 프로젝트 발표 안정성 때문에 정한 것이라 별개로 지킨다.

---

## 1. 프로젝트 목적

로봇은 음성 명령을 받아, 가려져 있는 물체를 찾아 파지하고 지정 위치로 옮긴다.

탐색 과정에는 단순 물체 검출뿐 아니라 아래 행동이 포함된다.

1. 초기 카메라 관측
2. 대상 물체가 보이지 않으면 탐색 위치 또는 공간 선택
3. 서랍 또는 문 열기
4. 내부 재관측
5. 물체 후보 확인
6. 파지 접근 및 파지 검증
7. 지정 위치로 이송 및 배치
8. 홈 자세 복귀
9. 성공 / 실패 / 취소 기록

UI의 목적은 로봇을 조작하는 것이 아니다. UI는 현재 탐색 과정, 카메라 영상, DB에 저장된 물체 위치, 과거 작업 기록을 사람이 보기 쉽게 보여 주는 **관측 전용 대시보드**다.

프로젝트의 핵심 메시지는 다음과 같다.

> 로봇이 단순히 화면 속 물체를 검출하는 것이 아니라, 대상이 보이지 않을 때 서랍·문을 열고 재관측하며 탐색 행동을 반복한다.

이 메시지를 화면에서 보여 주는 핵심 장치는 **3D 지도의 서랍·문 열림 표현**과 **현재 판단과 행동 패널**이다.

---

## 2. 시스템 구성

```text
[이미지 노드] [메인 노드] [db] [음성 노드] [state]
                     │
                     │  ROS 2 (토픽 / interfaces srv)
              ┌──────┴──────┐
              │   back_ui    │   구독 + 상태 병합 + JPEG 인코딩 + HTTP 서버
              └──────┬──────┘
                     │
                     │  HTTP (JSON + JPEG) @ 127.0.0.1:8765
              ┌──────┴──────┐
              │  front_ui    │   Flet 앱. ROS 의존성 없음
              └─────────────┘
```

### back_ui (ROS 패키지)

- 기존 노드들의 토픽·서비스를 구독한다.
- 받은 정보를 **화면 갱신 단위 하나의 상태 스냅샷**으로 병합한다.
- 최신 카메라 프레임을 JPEG로 인코딩해 메모리에 보관한다.
- localhost HTTP 서버를 띄워 스냅샷과 프레임을 제공한다.
- 로봇에 어떤 명령도 발행하지 않는다. **구독 전용이다.**

### front_ui (Flet 앱)

- `rclpy`를 import 하지 않는다.
- HTTP로 상태를 폴링해 화면을 그린다.
- ROS 설치 없이 단독 실행·패키징이 가능해야 한다.
- 의존성은 `flet`, `numpy`, `requests` 셋뿐이다. 늘리지 않는다.

---

## 3. 저장소 구조

```text
~/cobot_ws/src/Team_E2/
├── README.md
├── .gitignore
├── back_ui/                    ROS 패키지 (UI 브리지)
│   ├── back_ui/
│   │   ├── __init__.py
│   │   ├── node.py             rclpy 노드, 토픽 구독
│   │   ├── state_store.py      스냅샷 병합, 락 관리
│   │   ├── frame_store.py      최신 JPEG 보관
│   │   └── http_server.py      /state, /frame.jpg, /health
│   ├── package.xml
│   ├── setup.py / setup.cfg
│   ├── resource/back_ui
│   └── test/
├── front_ui/                   Flet 앱 (ROS 무관)
│   ├── COLCON_IGNORE           colcon 빌드 제외 (빈 파일, 커밋 필수)
│   ├── README.md
│   ├── environment.yaml
│   ├── pyproject.toml
│   ├── src/
│   │   ├── main.py             진입점, NavigationRail, 상단 바
│   │   ├── theme.py            색·간격·글자 크기 토큰
│   │   ├── config.py           BASE_URL, 폴링 주기
│   │   ├── assets/
│   │   │   ├── icon.png
│   │   │   └── renders/        선택: 사전 렌더 물체 이미지
│   │   ├── client/
│   │   │   ├── state_client.py HTTP 폴링, 연결 상태 판정
│   │   │   └── schema.py       스냅샷 → 데이터클래스 변환
│   │   ├── views/
│   │   │   ├── home_view.py
│   │   │   ├── monitor_view.py
│   │   │   └── log_view.py
│   │   ├── components/
│   │   │   └── panel.py        panel(), placeholder(), kv()
│   │   └── render3d/
│   │       ├── projection.py   project(), 회전행렬 상수
│   │       └── map_painter.py  draw_zone(), draw_object()
│   ├── tools/
│   │   └── fake_server.py      개발용 가짜 /state 서버
│   └── tests/
├── db/                         DB 노드 (기존)
├── state/                      상태확인 노드 (기존)
└── interfaces/                 srv 정의 (기존)
    └── srv/
        ├── DbLoad.srv
        ├── DbSave.srv
        └── NodeInit.srv
```

`front_ui`는 `cobot_ws/src/` 아래에 있으므로 `COLCON_IGNORE`가 반드시 있어야 한다. 빈 파일이면 되고 **커밋해야 한다.**

---

## 4. 개발 환경

### 환경 분리 원칙

| 대상 | 환경 | 규칙 |
|---|---|---|
| `front_ui` | conda env `front_ui` | ROS를 source 하지 않는다 |
| `back_ui` | 시스템 파이썬 + ROS | conda를 deactivate 하고 작업한다 |

conda가 활성화된 상태로 `colcon build`나 `ros2 run`을 하면 `undefined symbol`, `GLIBCXX not found` 오류가 난다.

```bash
conda config --set auto_activate_base false
```

`~/.bashrc`에 ROS `source`가 박혀 있으면 conda 환경에도 `PYTHONPATH`가 새어 든다.

```bash
conda activate front_ui
echo $PYTHONPATH                # /opt/ros/... 가 찍히면 안 됨
python -c "import rclpy"        # ModuleNotFoundError 나오는 게 정상
```

`rclpy` import가 성공하면 환경이 오염된 것이다. 그 상태로 개발하면 패키징 단계에서 터진다.

### front_ui 설치·실행

```bash
cd ~/cobot_ws/src/Team_E2/front_ui
conda env create -f environment.yaml
conda activate front_ui
flet run
```

`environment.yaml`:

```yaml
name: front_ui
channels:
  - conda-forge
dependencies:
  - python=3.11
  - pip
  - pip:
      - flet[all]
      - numpy
      - requests
```

### back_ui 빌드·실행

```bash
conda deactivate
source /opt/ros/humble/setup.bash
cd ~/cobot_ws
colcon build --packages-select back_ui --symlink-install
source install/setup.bash
ros2 run back_ui node
```

`--symlink-install`을 쓰면 파이썬 파일 수정 시 재빌드가 필요 없다.

### 터미널 역할 고정

```bash
# 터미널 A — ROS 전용 (conda 없음)
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash

# 터미널 B — Flet 전용 (ROS 없음)
conda activate front_ui
cd ~/cobot_ws/src/Team_E2/front_ui
```

### 개발 중 실행

```bash
# B-1
python tools/fake_server.py

# B-2
flet run
```

`http://127.0.0.1:8765/state`를 브라우저로 열어 JSON이 보이면 정상이다.

---

## 5. Flet 1.x API 주의사항

**인터넷 예제와 대부분의 튜토리얼은 Flet 0.x 기준이다.** 그대로 붙여넣으면 `module 'flet.controls.xxx' has no attribute 'yyy'` 오류가 난다. 소문자 모듈 함수가 대문자 클래스의 classmethod로 바뀌었다.

| 0.x (검색 결과에 많음) | 1.x (현재) |
|---|---|
| `ft.app(target=main)` | `ft.run(main)` |
| `ft.padding.all(10)` | `ft.Padding.all(10)` |
| `ft.padding.symmetric(0, 10)` | `ft.Padding.symmetric(vertical=0, horizontal=10)` |
| `ft.padding.only(left=10)` | `ft.Padding.only(left=10)` |
| `ft.margin.all(10)` | `ft.Margin.all(10)` |
| `ft.border.all(1, c)` | `ft.Border.all(1, c)` |
| `ft.border.only(bottom=...)` | `ft.Border.only(bottom=...)` |
| `ft.border_radius.all(8)` | `ft.BorderRadius.all(8)` |
| `ft.alignment.center` | `ft.Alignment.CENTER` |
| `ft.colors.RED` | `ft.Colors.RED` |
| `ft.icons.HOME` | `ft.Icons.HOME` |
| `page.window_width = 1440` | `page.window.width = 1440` |

추가로 알아둘 것:

- **`Padding.symmetric` / `Margin.symmetric`은 키워드 인자만 받는다.** `ft.Padding.symmetric(0, 10)`은 오류다.
- **자동 업데이트**: 이벤트 핸들러와 `main()`이 끝나면 Flet이 `page.update()`를 자동 호출한다. 대부분 직접 부를 필요가 없다.
- **단, 폴링 스레드처럼 이벤트 핸들러 밖에서 화면을 바꿀 때는 `page.update()`를 직접 불러야 한다.** `state_client`가 붙으면 이 경우가 대부분이 된다.
- `FilePicker`는 서비스가 되어 `page.services`에 추가해야 동작한다. 이 프로젝트에서는 쓰지 않는다.

새 코드를 붙여넣을 때는 **소문자 → 대문자 규칙을 먼저 적용**하고 실행하면 대체로 맞는다.

---

## 6. 통신 프로토콜

### 엔드포인트

| 메서드 | 경로 | 응답 | 설명 |
|---|---|---|---|
| GET | `/state` | `application/json` | 전체 상태 스냅샷 |
| GET | `/frame.jpg?id={frame_id}` | `image/jpeg` | 최신 카메라 프레임 |
| GET | `/health` | `application/json` | back_ui 생존 확인 |

기본 주소는 `http://127.0.0.1:8765`. `src/config.py`의 `BASE_URL`에서 변경한다.

HTTP 서버는 표준 라이브러리 `http.server`로 구현한다. 추가 의존성이 없고 localhost 폴링 수준에서 성능 문제도 없다.

### 폴링 정책

`src/config.py` 상수로 관리한다.

| 화면 | 상수 | 주기 |
|---|---|---|
| 작업 | `POLL_MONITOR` | 0.1초 |
| 홈 | `POLL_HOME` | 0.5초 |
| 로그 | `POLL_LOG` | 2.0초 |

- 카메라 프레임은 `/state`의 `frame_id`가 이전 값과 달라졌을 때만 새로 요청한다.
- 이미지 URL에 `frame_id`를 쿼리로 붙여, Flet 이미지 캐시로 화면이 갱신되지 않는 문제를 회피한다.

```python
ft.Image(src=f"{BASE_URL}/frame.jpg?id={frame_id}")
```

### 연결 상태 판정

- HTTP 요청 실패·타임아웃 → 상단 바 배지가 `연결 끊김`
- 요청은 성공했으나 `ts`가 `STALE_AFTER`(3초) 이상 갱신되지 않음 → `데이터 정지`
- 둘 중 하나라도 발생하면 마지막 스냅샷을 유지하되 **회색 처리해 최신값이 아님을 알린다.** 정상인 척 보여 주지 않는다.

배지는 `src/main.py`의 `conn_dot` / `conn_text` 컨트롤이다. `state_client`가 이 둘의 색과 문구를 갱신한다.

### 상태 스냅샷 스키마

```json
{
  "ts": 1754300000.123,
  "frame_id": 1482,

  "system": {
    "state": "RUN",
    "nodes": {
      "image": true, "main": true, "db": true, "voice": true, "state": true
    },
    "robot_connected": true,
    "camera_connected": true,
    "gripper_state": "open",
    "object_count_total": 12,
    "object_count_confirmed": 9,
    "object_count_unknown": 3
  },

  "task": {
    "task_id": "task_20260804_001",
    "voice_command": "빨간 컵 좀 찾아줘",
    "target_id": "cup_red_01",
    "target_name": "빨간 컵",
    "status": "RUNNING",
    "stage": "internal_reobserve",
    "action": "Drawer_A_2 내부를 관측하고 있습니다.",
    "action_reason": "초기 관측에서 대상이 확인되지 않았습니다.",
    "elapsed_sec": 28,
    "current_zone": "drawer_a_2",
    "detections": [
      { "label": "빨간 컵 후보", "confidence": 0.91 }
    ]
  },

  "objects": [
    {
      "id": "cup_red_01",
      "name": "빨간 컵",
      "category": "cup",
      "pos": [0.42, 0.18, 0.31],
      "zone": "drawer_a_2",
      "status": "confirmed",
      "confidence": 0.92,
      "last_seen": "2026-08-04T12:00:00"
    }
  ],

  "zones": [
    {
      "id": "drawer_a_2",
      "name": "Drawer A-2",
      "type": "drawer",
      "pos": [0.40, 0.10, 0.20],
      "size": [0.30, 0.40, 0.12],
      "open_axis": [1.0, 0.0, 0.0],
      "state": "open",
      "open_ratio": 0.8,
      "search_state": "observing"
    }
  ],

  "recent_tasks": [
    {
      "task_id": "task_20260804_000",
      "target_name": "파란 상자",
      "result": "FAILED",
      "ended_at": "2026-08-04T11:40:52",
      "duration_sec": 52
    }
  ]
}
```

### 열거값 정의

**`system.state`** — `state` 노드 기준. `src/theme.py`의 `SYSTEM_STATE`에 색이 정의되어 있다.

| 값 | 의미 | 표시 |
|---|---|---|
| `LOAD` | 노드 초기화 중 | 초기화 중 |
| `IDLE` | 대기 | 대기 |
| `RUN` | 작업 수행 중 | 작업 중 |

`ERROR`는 `state` 노드에 없으므로 별도 상태로 두지 않는다. 오류는 `system.nodes` 중 `false`가 있거나 `robot_connected` / `camera_connected`가 `false`인 것으로 표현한다.

`system.nodes`는 `interfaces/srv/NodeInit.srv` 기반 초기화 확인 결과를 반영한다.

**`task.status`**: `RUNNING` / `SUCCESS` / `FAILED` / `CANCELED`

**`task.stage`** — 코드값으로 주고 한글 라벨은 front_ui가 매핑한다. 매핑표는 `src/views/monitor_view.py`의 `STAGES` 리스트다.

| 코드값 | 표시 라벨 |
|---|---|
| `idle` | 대기 |
| `initial_observe` | 초기 관측 |
| `select_zone` | 탐색 위치 선정 |
| `open_container` | 서랍/문 열기 |
| `internal_reobserve` | 내부 재관측 |
| `verify_candidate` | 후보 확인 |
| `approach_grasp` | 파지 접근 |
| `verify_grasp` | 파지 검증 |
| `transport` | 지정 위치로 이송 |
| `place` | 물체 내려놓기 |
| `return_home` | 홈 복귀 |
| `done` | 완료 |
| `failed` | 실패 |

**`objects[].status`** — `src/theme.py`의 `STATUS` 딕셔너리 키와 1:1 대응한다.

| 값 | 의미 | 색상 |
|---|---|---|
| `unknown` | 미확인 / 위치 불명 | 회색 `#6B7688` |
| `searching` | 탐색 중 | 파랑 `#4A90D9` |
| `confirmed` | 확인됨 / 발견 | 초록 `#3FB27F` |
| `held` | 손에 있음 | 보라 `#9B7FD4` |
| `warning` | 경고 | 주황 `#E0A458` |
| `error` | 오류 / 실패 | 빨강 `#D9564A` |

**`zones[].type`**: `shelf` / `drawer` / `door` / `divider`
**`zones[].state`**: `closed` / `opening` / `open` / `closing`
**`zones[].search_state`**: `untouched` / `observing` / `done` / `found` / `failed`

### 서랍·문 열림 표현 요구

`open_ratio`는 0.0~1.0이며 **여는 동작 중에도 중간값이 계속 갱신되어야 한다.** 시작과 끝에만 값이 바뀌면 화면에서 서랍이 순간이동하듯 보인다. 프로젝트 핵심 메시지를 살리는 부분이므로 메인 노드 측에 반드시 요청한다.

- 서랍: `pos + open_axis * open_ratio * (해당 축 size)` 만큼 이동
- 문: 힌지축 기준 `open_ratio * 90도` 회전

---

## 7. 좌표계

- 기준 프레임: **`base_link` (로봇 고정 베이스)**
- 로봇은 **고정형 팔**이므로 `base_link`는 이동하지 않는다. `base_link` = world로 간주한다.
- 축: **x 전방, y 좌측, z 상방** (ROS REP-103, 오른손 좌표계, z-up)
- 단위: **미터(m)**
- 회전은 도(degree), 순서 XYZ 고정

**`/state`로 나가는 모든 좌표는 이미 `base_link` 기준으로 변환이 끝난 값이어야 한다.** 카메라 프레임 좌표를 그대로 보내지 않는다. TF 변환은 이미지 노드 또는 메인 노드가 담당하며, **`back_ui`와 `front_ui`는 TF를 다루지 않는다.**

---

## 8. 3D 지도 구현 방식

### 런타임 렌더러를 쓰지 않는다

pyrender, OSMesa, EGL, 런타임 OBJ 로딩을 사용하지 않는다. 헤드리스 환경 의존성 문제와 발표 PC에서의 실행 실패 위험을 없애기 위함이다.

대신 **numpy로 직접 3D 투영을 계산하고 Flet Canvas에 벡터로 그린다.**

### 그리는 대상

| 대상 | 표현 |
|---|---|
| 선반, 서랍장, 칸막이, 문 | 직육면체 |
| 물체 | 점 + 이름 라벨 |
| 로봇 베이스 | 원점 마커 + 좌표축 |

### 구현 방법

1. 고정된 사선 위쪽 시점의 회전행렬 `R`(3×3)을 상수로 둔다.
2. 직교투영으로 3D 좌표를 2D 화면 좌표로 변환한다.
3. 직육면체는 꼭짓점 8개를 투영하고, 면 6개를 중심 depth 내림차순 정렬해 뒤에서부터 그린다 (painter's algorithm).
4. 물체는 투영된 2D 좌표에 원과 라벨을 그린다. depth에 따라 원 크기를 약간 조절해 원근감을 준다.

```python
# src/render3d/projection.py
def project(p_3d, R, scale, origin_2d):
    """base_link 기준 3D 좌표 -> (x2d, y2d, depth)"""
    v = R @ np.asarray(p_3d, dtype=float)
    x2d = origin_2d[0] + v[0] * scale
    y2d = origin_2d[1] - v[2] * scale
    return x2d, y2d, v[1]
```

### 함수 분리 원칙

```text
projection.project()        투영 계산. 표현 방식이 바뀌어도 변하지 않는다.
map_painter.draw_zone()     직육면체 그리기
map_painter.draw_object()   ← 교체 지점
```

`draw_object`만 바꾸면 점 → 사전 렌더 이미지로 전환된다. 런타임 3D 렌더링은 어느 쪽이든 필요 없다.

### 선택 구현: 물체 이미지 표현

일정에 여유가 있을 때만 진행한다. **필수 아님.**

- 개발 PC에서 물체별 OBJ를 오프라인으로 미리 렌더해 `src/assets/renders/{object_id}.png`로 저장
- 배경 투명 PNG, 128×128 정도
- 런타임에는 이미지를 표시만 하므로 3D 라이브러리가 배포판에 포함되지 않는다
- 사전 렌더 도구는 무엇을 써도 무방하다 (Blender CLI, trimesh + matplotlib 등)
- `renders/*.png`는 빌드 산출물이므로 `.gitignore` 대상이다

### 두 화면의 지도 차이

| | 홈 3D 맵 | 작업 화면 3D 지도 |
|---|---|---|
| 목적 | DB 등록 전체 물체 조망 | 현재 탐색 구역 강조 |
| 표시 | 모든 물체 + 모든 구역 | 구역 중심, 대상 구역 강조 |
| 서랍·문 열림 | 반영 | 반영 (중요) |
| 갱신 | 500 ms | 100 ms |

두 지도는 **동일한 렌더 함수를 옵션만 다르게 호출**해 만든다. 별도 구현을 두지 않는다.

---

## 9. 화면 구성

좌측 NavigationRail 메뉴는 세 개다. `src/main.py`의 `PAGES` 리스트로 관리한다.

```text
홈       build_home()      views/home_view.py
작업     build_monitor()   views/monitor_view.py
로그     build_log()       views/log_view.py
```

로봇을 조작하는 버튼은 만들지 않는다. 화면 이동, 표 행 선택, 상세 열기 같은 **정보 조회 상호작용만** 허용한다.

### 공통 패널 컴포넌트

모든 박스는 `src/components/panel.py`의 함수로 만든다. 테두리·여백·제목 위치를 한 곳에서 관리하기 위함이다.

```python
panel(title, content=None, accent=None)   # 제목 달린 박스. content 없으면 안내 문구
placeholder(name)                          # 미구현 안내
kv(key, value, value_color=None, mono=False)  # 라벨 + 값 한 줄
```

색·간격·글자 크기는 전부 `src/theme.py` 상수를 쓴다. 하드코딩한 색 문자열을 화면 코드에 쓰지 않는다.

패널 사이 간격은 `theme.GAP`, 안쪽 여백은 `theme.PAD`다. 레이아웃이 답답하거나 헐거우면 이 두 값만 조정한다.

---

### 화면 1. 홈 — `src/views/home_view.py`

```text
┌──────────────────┬──────────────────┐
│ 현재 작업 상태     │ 로봇 시스템 상태   │   expand=3
├──────────────────┼──────────────────┤
│ 찾고 있는 3d 모델  │ 물체 위치 3D 맵    │   expand=3
├──────────────────┴──────────────────┤
│ 최근 실행 경과                        │   expand=2
└─────────────────────────────────────┘
```

높이 비율은 `ft.Row(..., expand=N)`으로 준다. 창 크기가 바뀌어도 비율이 유지된다.

**현재 작업 상태**
- 음성 명령 원문 (`task.voice_command`)
- 대상 물체 이름 (크게)
- 작업 상태, 현재 단계, 경과 시간
- 작업이 없으면 "탐색 중인 물체 없음"

**로봇 시스템 상태**
- 시스템 상태: `LOAD` / `IDLE` / `RUN`
- 노드별 생존 표시: 이미지 / 메인 / db / 음성 / state
- 로봇 연결, 카메라 연결, 그리퍼 상태
- 등록 물체 수 / 확인된 물체 수 / 위치 불명 물체 수

**찾고 있는 3d 모델**
- 현재 대상 물체의 사전 렌더 이미지 또는 이름 + 카테고리 카드
- 대상이 없으면 "탐색 중인 물체 없음"
- 사전 렌더 이미지가 없으면 텍스트 카드로 대체

**물체 위치 3D 맵**
- 8장 방식. 물체는 점 + 라벨
- 물체 상태에 따른 색상 적용
- 서랍·문 열림 상태 반영

**최근 실행 경과**
- `recent_tasks` 최근 5건, 한 줄씩
- 시각, 대상 물체, 결과, 소요 시간

---

### 화면 2. 작업 — `src/views/monitor_view.py`

```text
┌──────────────────┬──────────────────┐
│ 현재 카메라 화면   │ 현재 판단과 행동   │   expand=3
├──────────────────┼──────────────────┤
│ 3D 지도           │ 현재 실행 단계     │   expand=3
├──────────────────┴──────────────────┤
│ 작업 정보                             │   expand=2
└─────────────────────────────────────┘
```

**현재 카메라 화면**
- `/frame.jpg?id={frame_id}` 표시
- 검출 박스·마스크·라벨은 **이미지 노드가 합성한 프레임을 그대로 받는다.** front_ui는 인식 모델을 실행하지 않는다.
- 하단에 `task.detections`를 텍스트로 표시

```text
빨간 컵 후보   confidence 0.91
```

- 대상을 찾지 못했을 때도 화면을 비우지 않는다.

```text
현재 시야에서 대상 물체를 찾지 못했습니다.
```

- 카메라 미연결 시 안내 Placeholder

**현재 판단과 행동**

행동과 이유를 분리해 표시한다. 이 패널이 프로젝트 핵심 메시지를 전달하는 자리다.

```text
현재 행동
서랍 A-2를 열고 있습니다.

행동 이유
대상 물체가 현재 카메라 시야에서 관측되지 않았습니다.
서랍 A-2 내부를 재관측하기 위해 열기 동작을 수행합니다.
```

**3D 지도**
- 8장 방식. 현재 대상 구역 강조
- 서랍·문이 열리는 동안 `open_ratio`에 따라 움직여야 한다

**현재 실행 단계**

`STAGES` 리스트 순서대로 나열한다. `stage_row(label, state)`로 한 줄씩 그리며 `state`는 셋 중 하나다.

| state | 점 색 | 글자 |
|---|---|---|
| `current` | `theme.ACCENT` 주황 | 밝게, 굵게 |
| `done` | 초록 | 흐리게 |
| `todo` | 테두리색 | 가장 흐리게 |

**작업 정보**
- 작업 ID, 음성 명령 원문, 대상 물체
- 작업 상태, 경과 시간, 현재 구역
- 탐색 영역 상태 표

| 영역 | 상태 | 열림 | 비고 |
|---|---|---|---|
| Drawer_A_1 | 탐색 완료 | 닫힘 | 대상 없음 |
| Drawer_A_2 | 관측 중 | 열림 80% | - |
| Shelf_B_1 | 후보 발견 | - | 빨간 컵 후보 |
| Door_C | 미탐색 | 닫힘 | - |

---

### 화면 3. 로그 — `src/views/log_view.py`

상단에 탭 두 개를 둔다. `ft.Tabs`가 아니라 직접 만든 pill 버튼 두 개로 구현했다. 와이어프레임 형태에 맞추고 API 변경 위험을 피하기 위함이다.

```text
[ 작업 기록 ]  [ 물건 위치 ]
```

선택된 탭은 `theme.ACCENT` 배경, 나머지는 `theme.SURFACE`다.

#### 탭 1. 작업 기록

| 실행 시각 | 대상 물체 | 결과 | 소요 시간 | 탐색 구역 수 | 파지 결과 |
|---|---|---|---:|---:|---|
| 2026-08-04 12:03 | 빨간 컵 | 성공 | 38초 | 3 | 성공 |
| 2026-08-04 11:40 | 파란 상자 | 실패 | 52초 | 4 | 미수행 |

결과 표시: 성공 / 실패 / 취소 / 오류 (코드값 `SUCCESS` / `FAILED` / `CANCELED` / `ERROR`)

**작업 상세** — 행 선택 시 패널 또는 다이얼로그

- 작업 ID, 음성 명령 원문, 대상 물체
- 시작 / 종료 시간, 총 수행 시간
- 최종 결과, 발견 위치, 파지 성공 여부
- 실패 원인, 탐색한 구역 수

**이벤트 타임라인**

```text
00:00  작업 시작
00:03  초기 관측 수행
00:08  대상 미관측
00:12  Drawer_A_2 열기
00:18  내부 재관측
00:22  빨간 컵 후보 발견
00:28  후보 검증 완료
00:33  파지 수행
00:36  지정 위치로 이송
00:38  작업 성공
```

**주요 이미지** — 작업별 최대 3장

- 초기 관측 이미지
- 대상 후보를 처음 발견한 이미지
- 파지 직전 또는 최종 결과 이미지

실패 작업이면 마지막 이미지와 실패 원인을 함께 보여 준다.

실패 원인 예시: 대상 물체 미발견 / 후보 검증 실패 / 파지 실패 / 카메라 연결 오류 / 로봇 상태 오류 / 서랍 또는 문 조작 실패

#### 탭 2. 물건 위치

`db` 노드에 등록된 물체 위치 정보를 표로 정리한다. 현재 작업에서 방금 검출한 좌표만 보는 화면이 아니다.

| 물체 이름 | 위치 라벨 | 좌표 (m, base_link) | 상태 | 마지막 확인 시각 | 신뢰도 |
|---|---|---|---|---|---:|
| 빨간 컵 | Drawer_A_2 | 0.42, 0.18, 0.31 | 확인됨 | 2026-08-04 12:00 | 0.92 |
| 파란 상자 | Shelf_B_1 | - | 위치 불명 | 2026-08-04 11:40 | - |

- 신뢰도가 없으면 `-` (JSON에서는 `null`)
- 좌표가 `base_link` 기준 미터임을 표 제목에 명시
- 좌표·시각·ID는 `theme.MONO` 고정폭 글꼴을 쓴다. 자리수가 흔들리면 읽기 어렵다

기본 화면에는 원시 ROS 토픽, 관절각, 디버그 콘솔 전체를 표시하지 않는다.

---

## 10. 화면 톤

어두운 배경을 쓴다. 카메라 영상과 3D 지도가 화면의 절반을 차지하는데 밝은 배경이면 영상이 뜨는 자리마다 눈이 튀고, 발표장 프로젝터에서도 어두운 쪽이 잘 보인다.

강조색은 주황 하나만 쓰고 **"지금 진행 중인 것"에만** 붙인다. 현재 단계, 선택된 탭, 진행 중인 작업이 그 대상이다. 강조가 여러 곳에 흩어지면 어디를 봐야 할지 알 수 없게 된다.

상태 색 6가지는 물체·구역 상태 표시에만 쓴다. 장식으로 쓰지 않는다.

---

## 11. 구현하지 않을 것

- UI에서 탐색 시작, 취소, 재시도, 홈 복귀 같은 로봇 제어
- 수동 로봇 이동, 관절 제어, 그리퍼 조작
- 비상정지
- 음성 인식 및 음성 명령 처리 (원문 텍스트를 받아 표시만 한다)
- 카메라에서 직접 물체 검출 또는 마스크 생성
- DB 쓰기, DB 파일 직접 접근 (`db` 노드가 소유한다)
- SLAM, 카메라 캘리브레이션, 3D 위치 추정, TF 변환
- 런타임 OBJ 로딩 및 3D 렌더링
- 자유 회전·줌이 가능한 인터랙티브 3D 뷰어
- Three.js, JavaScript, WebView

`front_ui`는 `back_ui`가 제공한 스냅샷을 받아 보여 주는 역할만 맡는다.

---

## 12. 초기 실행 조건

실제 로봇·카메라·ROS·DB가 없어도 `tools/fake_server.py` + `flet run` 만으로 실행되어야 한다.

가짜 서버 초기 데이터에 아래를 포함한다.

- 빨간 컵: Drawer_A_2 내부, `confirmed`
- 파란 상자: Shelf_B_1, `unknown`
- Drawer_A_2: `open_ratio`가 0.0 → 1.0으로 서서히 변하는 시나리오 포함
- 현재 작업: 빨간 컵 탐색 중, 단계 `internal_reobserve`
- 음성 명령 원문: "빨간 컵 좀 찾아줘"
- 카메라: 샘플 이미지 반복 재생 또는 Placeholder
- 최근 실행 경과: 성공 1건, 실패 1건 이상

**서랍이 열리는 시나리오를 반드시 넣는다.** 발표에서 보여 줄 장면이므로 개발 내내 확인되어야 한다.

---

## 13. 진행 현황

| 파일 | 내용 | 상태 |
|---|---|---|
| `src/theme.py` | 색·간격·글자 토큰 | 완료 |
| `src/config.py` | 주소·폴링 주기 | 완료 |
| `src/components/panel.py` | `panel()`, `placeholder()`, `kv()` | 완료 |
| `src/main.py` | NavigationRail, 상단 바, 화면 전환 | 완료 |
| `src/views/home_view.py` | 4분할 + 하단 레이아웃 | 레이아웃 완료, 데이터 미연결 |
| `src/views/monitor_view.py` | 4분할 + 하단 레이아웃, 단계 목록 | 레이아웃 완료, 데이터 미연결 |
| `src/views/log_view.py` | 탭 2개 + 본문 | 레이아웃 완료, 내용 미구현 |
| `tools/fake_server.py` | 가짜 `/state` 서버 | 미착수 |
| `src/client/schema.py` | JSON → 데이터클래스 | 미착수 |
| `src/client/state_client.py` | 폴링, 연결 판정 | 미착수 |
| `src/render3d/projection.py` | 투영 함수 | 미착수 |
| `src/render3d/map_painter.py` | 박스·점 그리기 | 미착수 |
| `back_ui/*` | ROS 노드 전체 | 미착수 |

### 다음 작업 순서

1. `tools/fake_server.py` — 스냅샷 스키마를 그대로 흉내내는 서버
2. `src/client/schema.py` + `state_client.py` — 폴링, 연결 판정, 데이터클래스 변환
3. 화면에 실제 값 연결 (작업 → 홈 → 로그 순)
4. `src/render3d/` — 3D 지도
5. `back_ui` — 실제 토픽 구독 및 `/state` 제공
6. 실연동 검증 (완료 기준 10~13)

**1~2번을 먼저 끝내는 것이 중요하다.** 화면부터 채우면 나중에 데이터 구조가 안 맞아 전부 뜯어고치게 된다. 스키마에 빠진 필드는 코드가 없을 때 고치는 것이 가장 싸다.

---

## 14. 완료 기준

### MVP (필수)

1. `flet run`으로 앱이 실행되고 `홈 / 작업 / 로그`로 이동할 수 있다.
2. 홈에서 시스템 상태, 물체 3D 맵, 최근 실행 경과를 볼 수 있다.
3. 작업 화면에서 카메라 프레임, 현재 판단과 행동, 실행 단계, 3D 지도, 탐색 영역 상태를 볼 수 있다.
4. 로그 화면에서 작업 목록, 상세 타임라인, 주요 이미지, 물건 위치 표를 볼 수 있다.
5. 3D 지도에 서랍·문 열림이 `open_ratio`에 따라 연속적으로 반영된다.
6. UI에 로봇 제어 버튼이 없다.
7. `fake_server.py`만으로 모든 화면이 정상 동작한다.
8. 데이터·이미지가 누락되어도 앱이 종료되지 않고 Placeholder를 표시한다.
9. 서버 연결이 끊기면 `연결 끊김` 배지가 뜨고 값이 회색 처리된다.

### 실연동 (발표 전 필수)

10. `back_ui`가 실제 ROS 토픽을 구독해 `/state`를 제공한다.
11. 실제 카메라 프레임 1장 이상이 작업 화면에 표시된다.
12. `db` 노드에 등록된 실제 물체 1건 이상이 홈 3D 맵과 물건 위치 표에 표시된다.
13. 실제 서랍 열기 동작이 3D 지도에 반영되는 것을 확인한다.

**10~13번은 mock으로 대체할 수 없다.** MVP 9개를 전부 통과해도 실연동을 한 번도 안 해본 상태가 될 수 있으므로 별도 기준으로 분리한다.

---

## 15. 협의 필요 항목

다른 노드 담당자와 합의가 필요한 사항이다.

| 항목 | 요청 대상 | 내용 |
|---|---|---|
| `open_ratio` 연속 발행 | 메인 노드 | 서랍·문 여는 중 중간값을 계속 발행 |
| `action_reason` 문자열 | 메인 노드 | 행동 이유를 사람이 읽을 문장으로 제공 |
| `stage` 코드값 | 메인 노드 | 6장 열거값 표에 맞춰 발행 |
| 좌표 변환 | 이미지 / 메인 노드 | `base_link` 기준으로 변환 후 발행 |
| 음성 명령 원문 | 음성 노드 | 인식된 텍스트 원문을 발행 |
| 검출 결과 합성 프레임 | 이미지 노드 | 박스·라벨이 그려진 이미지 제공 |
| 작업 이력 조회 | `db` 노드 | 완료 작업 목록·상세 조회 인터페이스. 현재 `DbLoad.srv`가 물체 위치만 다루는 구조라면 이력용 테이블·조회가 추가로 필요하다 |
| 노드 초기화 상태 | `state` 노드 | `NodeInit.srv` 결과를 `back_ui`가 받을 수 있는 형태로 제공 |
