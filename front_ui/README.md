# front_ui

숨은 물체 탐색 코봇의 관측 전용 대시보드. Flet 앱, ROS를 전혀 모른다
(`rclpy` import 금지 — 자세한 규칙은 저장소 루트 `CLAUDE.md` 참고).

## 실행

```bash
conda env create -f environment.yaml   # 최초 1회
conda activate front_ui
cd front_ui
flet run
```

개발 중에는 `back_ui` 대신 `python tools/fake_server.py`를 띄워서 확인한다.
front_ui 코드는 반대편이 `fake_server`인지 진짜 `back_ui`인지 구분하지 않는다
— 아래 계약(HTTP 주소·경로·JSON 스키마)만 똑같이 맞추면 그대로 붙는다.

---

## back_ui가 줘야 하는 것

`back_ui`를 작성할 때 이 표를 계약(contract)으로 삼는다. **실제로 지금
front_ui 코드가 읽는 필드만** 적었다 — 명세 문서(`docs/hidden_object_search_ui_spec_v4.md`,
`docs/3d_map_spec.md`)에는 있지만 아직 화면에서 안 쓰는 필드는 "미사용"으로
따로 표시했다. 정확한 참고 구현은 `tools/fake_server.py` — 스키마가 헷갈리면
그 파일을 그대로 베끼면 된다.

### 1. HTTP 서버

`front_ui/src/config.py`에 주소가 있다. 기본값:

| 항목 | 값 |
|---|---|
| BASE_URL | `http://127.0.0.1:8765` |
| 요청 타임아웃 | 1.0초 (`REQUEST_TIMEOUT`) |
| 폴링 주기 | 0.5초 (`POLL_HOME`) — 지금은 화면 상관없이 이 값 하나만 씀 |
| 응답 헤더 | `Content-Type: application/json; charset=utf-8`, `Access-Control-Allow-Origin: *` (브라우저에서 뜰 때 CORS 막히면 안 됨) |

세 경로를 응답해야 한다:

| 경로 | 필수 여부 | 내용 |
|---|---|---|
| `GET /state` | **필수** | 아래 2번 스키마. front_ui가 0.5초마다 계속 부른다 |
| `GET /health` | 필수 | `{"ok": true}` 아무거나 200으로만 오면 됨. 연결 확인용은 아니고(그건 `/state`의 `ts`로 판단) 생존 확인용 |
| `GET /frame.jpg?id={frame_id}` | **아직 미사용** | "작업" 화면 카메라 패널이 아직 자리만 잡아둔 상태라 지금은 안 불림. 나중에 붙을 때를 대비해 주소만 예약돼 있다 |

`GET /state`가 실패하거나(연결 끊김) 응답의 `ts`가 3초(`STALE_AFTER`) 이상
과거면 front_ui는 상단 배지를 "연결 끊김"/"지연"으로 표시한다 — **`ts`는
반드시 매 응답마다 현재 unix timestamp(초)로 갱신해서 보내야 한다.**

### 2. `GET /state` 응답 스키마

한 덩어리 JSON 스냅샷이다(토픽별로 따로 안 준다 — 화면 갱신 타이밍이
어긋나는 것을 막기 위함, CLAUDE.md 참고). 최상위 키:

```json
{
  "ts": 1733300000.0,
  "frame_id": 123,
  "system": { ... },
  "task": { ... },
  "objects": [ ... ],
  "zones": [ ... ],
  "robot": { "links": [ ... ] },
  "recent_tasks": [ ... ]
}
```

| 최상위 키 | 필수 | 비고 |
|---|---|---|
| `ts` | 필수 | 위 참고 |
| `frame_id` | 미사용 | `/frame.jpg`가 붙을 때 이미지 캐시 무효화용(id 바뀔 때만 재요청). 지금 넣어놔도 손해 없음 |
| `system` | 필수 | 로봇 시스템 상태 패널 |
| `task` | 필수 | 현재 작업 상태 패널 + 3D 맵의 현재 구역 강조 |
| `objects` | 필수(빈 배열 허용) | 3D 맵 마커, "찾고 있는 3d 모델" 패널 |
| `zones` | 필수(빈 배열 허용) | 3D 맵 서랍/문 박스. 지금 `fake_server`는 빈 배열만 보냄 — 실측 위치 없이 임의 좌표 넣는 걸 금지했기 때문(2026-08-05 결정) |
| `robot.links` | 필수(빈 배열/생략 허용) | 3D 맵 로봇 팔 점군 위치. 없거나 비어 있으면 그냥 안 그림(에러 아님) |
| `recent_tasks` | 필수(빈 배열 허용) | "최근 실행 경과" 패널, 최근 5건만 씀(더 보내도 앞 5개만 씀) |

#### `system`

| 필드 | 타입 | 허용값 / 예시 | 비고 |
|---|---|---|---|
| `state` | string | `LOAD` / `IDLE` / `RUN` | `labels.SYSTEM_STATE` |
| `nodes` | object | `{"image": true, "main": true, "db": true, "voice": true, "state": true}` | **키 5개 고정**(`labels.NODE` 참고), 값은 bool |
| `robot_connected` | bool | | |
| `camera_connected` | bool | | |
| `gripper_state` | string | `open` / `closed` / `holding` | `labels.GRIPPER` |
| `object_count_total` | int | | |
| `object_count_confirmed` | int | | |
| `object_count_unknown` | int | | |

#### `task`

| 필드 | 타입 | 허용값 / 예시 | 비고 |
|---|---|---|---|
| `task_id` | string | | |
| `voice_command` | string \| null | | |
| `target_id` | string \| null | | objects[].id와 매칭. `assets/models/{target_id}.obj` 있으면 그 3D 모델을 드래그 뷰어로 보여줌 |
| `target_name` | string \| null | | |
| `status` | string | `RUNNING`/`SUCCESS`/`FAILED`/`CANCELED`/`ERROR` | `labels.TASK_STATUS` |
| `stage` | string | `labels.STAGE_ORDER` 12개 코드 중 하나(`idle`~`done`) + `failed` | 진행률 원형이 이 순서상 위치로 %를 계산함 |
| `elapsed_sec` | number \| null | | null이면 "--:--"로 표시 |
| `current_zone` | string \| null | | `zones[].id`와 매칭 — 3D 맵에서 이 구역만 강조색 |
| `action` | string | | **미사용** (작업 화면 "현재 판단과 행동" 패널이 아직 정적 placeholder) |
| `action_reason` | string | | **미사용**(위와 같음) |
| `detections` | array | | **미사용** |

#### `objects[]`

| 필드 | 타입 | 허용값 / 예시 | 비고 |
|---|---|---|---|
| `id` | string | | |
| `name` | string | | |
| `category` | string | | 사전 렌더/뷰어 다 없을 때 텍스트 카드에 씀 |
| `pos` | `[x,y,z]` \| null | 미터, base_link 기준 | **null이면 3D 맵에 안 그림** — "위치 불명"은 좌표가 아니라 이 필드가 null인 것으로 표현 |
| `status` | string | `unknown`/`searching`/`confirmed`/`held`/`warning`/`error` | `theme.STATUS` 색 키와 정확히 같아야 함 |
| `zone` | string \| null | | **미사용** |
| `confidence` | number \| null | | **미사용** |
| `last_seen` | string(ISO) | | **미사용** |

#### `zones[]`

| 필드 | 타입 | 허용값 / 예시 | 비고 |
|---|---|---|---|
| `id` | string | | `task.current_zone`과 매칭 |
| `name` | string | | **미사용**(표시 안 함, id만 씀) |
| `type` | string | `drawer` / `door` (그 외는 서랍처럼 취급) | `door`는 회전(힌지 근사), 그 외는 직선 이동 |
| `pos` | `[x,y,z]` | 미터, 박스 중심, `open_ratio=0`(닫힘) 기준 | |
| `size` | `[x,y,z]` | 미터 | |
| `open_axis` | `[x,y,z]` | 단위벡터, 보통 `[1,0,0]`류 | `type=drawer`일 때만 씀 |
| `open_ratio` | number | 0.0~1.0 | **여는 동안 계속 갱신해서 보내야 함**(캐시 안 하고 매 폴링 다시 그림) |
| `search_state` | string | `untouched`/`observing`/`done`/`found`/`failed` | `current_zone`이 아닐 때 이 값으로 색 결정 |

#### `robot.links[]`

| 필드 | 타입 | 허용값 / 예시 | 비고 |
|---|---|---|---|
| `name` | string | `base_link`/`link_1`~`link_6` (M0609 기준) | `front_ui/src/assets/robot/<name>.npy`와 이름이 정확히 일치해야 함(`tools/mesh_to_points.py` 참고) |
| `pos` | `[x,y,z]` | 미터, base_link(=world) 기준 | TF/FK 계산 다 끝난 값으로. front_ui는 TF를 안 다룸 |
| `rpy` | `[roll,pitch,yaw]` | 라디안 | `render3d/shapes.py`의 `rpy_matrix()`: `R = Rz(yaw)·Ry(pitch)·Rx(roll)` 순서 |

7개 링크 다 안 보내도 됨 — 보낸 것만 그린다. `render3d/robot_points.py`가
링크별 점군에 위 pos/rpy를 적용한 뒤, base_link 정면축 보정(90도 고정,
관절 각도와 무관)까지 자체적으로 적용한다 — back_ui는 이 보정을 신경 안
써도 된다.

#### `recent_tasks[]`

| 필드 | 타입 | 허용값 / 예시 | 비고 |
|---|---|---|---|
| `task_id` | string | | **미사용**(표시 안 함) |
| `target_name` | string | | |
| `result` | string | `SUCCESS`/`FAILED`/`CANCELED`/`ERROR` (task.status와 같은 코드 공간) | |
| `ended_at` | string(ISO 8601) | `2026-08-04T12:03:38` | 화면엔 시:분만 잘라서 보여줌 |
| `duration_sec` | number | | |

### 3. back_ui가 몰라도 되는 것

- `front_ui/src/assets/scene_config.json` (바닥판·선반·수납장·박스 크기/위치):
  front_ui가 직접 들고 있는 고정 배경이다. 로봇도 모르고 `/state`로도 안
  보낸다 — 줄자로 잰 값이라 back_ui가 계산할 필요 없음.
- `front_ui/src/assets/robot/*.npy` (로봇 팔 mesh 점군): `tools/mesh_to_points.py`가
  doosan-robot2 mesh에서 미리 뽑아둔 결과물. back_ui는 링크 이름 + pos/rpy만
  주면 된다.
- `front_ui/src/assets/models/*.obj` (물체 3D 모델): 있으면 드래그 뷰어로
  보여주고, 없으면 자동으로 텍스트 카드로 대체한다(안 죽음).

---

## 저장소 구조·진행 상황

`PROGRESS.md` 참고.
