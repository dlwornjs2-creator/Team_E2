# front_ui 진행 상황

이 문서는 "지금 뭐가 됐고 뭐가 안 됐는지, 왜 이렇게 만들었는지"를 기록한다.
설계 목표는 `docs/hidden_object_search_ui_spec_v4.md`와
`docs/3d_map_spec.md`(저장소 루트) 참고. 이 문서는 진행 로그 + 다음 사람이
바로 이어받을 수 있게 하는 메모다.

마지막 갱신: 2026-08-05

---

## 1. 지금 실행하면 뭐가 되는가

```bash
# 터미널 1
conda activate front_ui
cd front_ui
python tools/fake_server.py

# 터미널 2
conda activate front_ui
cd front_ui
flet run
```

- 홈 화면: 실시간으로 갱신됨 (0.5초 폴링, 클릭 안 해도 됨)
  - **현재 작업 상태**: 원형 진행률(단계 순서 기준 %), 작업 상태 배지, 현재 단계, 경과 시간, 음성 명령
  - **로봇 시스템 상태**: 시스템 상태, 노드 생존, 로봇/카메라 연결, 그리퍼, 물체 수
  - **찾고 있는 3d 모델**: 드래그로 돌려볼 수 있는 3D 뷰어 (지금은 데모용 개구리 인형 mesh를 `cup_red_01` 자리에 씀)
  - **물체 위치 3D 맵**: 드래그 회전 / 휠 줌 / 더블클릭 리셋 가능한 3D 지도. 지금은 바닥판 + 로봇 위치(빨간 점)만 있음
  - **최근 실행 경과**: 아직 미구현 (placeholder)
- 작업 화면, 로그 화면: 레이아웃만 있고 데이터 미연결 (다음 순서 참고)
- 상단 연결 배지: `fake_server`를 껐다 켜면 정상적으로 "연결 끊김" ↔ "연결됨" 전환됨

---

## 2. 아키텍처 결정 사항 (왜 이렇게 만들었는지)

### 2.1 back_ui는 아직 없다 — `fake_server.py`가 완전히 대체한다

`front_ui`는 `state_client.py`가 HTTP로 `/state`를 받아오는 것 말고는 아무것도
모른다. `fake_server.py`가 `back_ui`와 **완전히 같은 인터페이스**(주소·경로·
JSON 스키마)로 흉내내므로, `back_ui`가 생기면 `python tools/fake_server.py`
대신 `ros2 run back_ui node`를 띄우기만 하면 되고 front_ui 코드는 한 줄도
안 고친다.

### 2.2 폴링 스레드에서 `page.update()`를 직접 부르면 안 된다

Flet 소스 확인 결과 `page.update()`는 스레드 안전하지 않다 — 이벤트 루프
소유 스레드가 아닌 곳(우리가 만든 폴링 스레드)에서 부르면 큐에만 쌓이고
클라이언트로 안 나간다. 마우스 클릭 같은 진짜 클라이언트 이벤트가 와야
그때 밀린 게 한꺼번에 반영되는 것처럼 보였다 (실제로 겪은 버그).

**해결**: `page.run_task()`(`asyncio.run_coroutine_threadsafe`로 이벤트
루프에 안전하게 작업을 넘기는 공식 API)를 씀. `main.py`의 `on_status` 참고.

### 2.3 폴링마다 화면 전체를 다시 만들지 않는다 (컴포넌트 단위 갱신)

처음엔 스냅샷이 올 때마다 `build_home()`을 다시 호출해 화면 전체를 새로
만들었다. 문제: 드래그 중인 3D 뷰어(`ObjectViewer`, `MapView`)까지 매번
새로 만들어져서, 드래그하는 도중 다른 패널이 갱신될 때마다 뷰어가 끊겼다.

**해결**: `HomeView` 같은 화면 클래스는 `.control`(구조)을 한 번만 만들고,
`.update(snapshot)`은 각 패널의 안쪽 content만 갈아끼운다. `ObjectViewer`/
`MapView`가 들어있는 패널은 **관련 데이터(target_id 등)가 실제로 안 바뀌면
그 서브트리를 절대 건드리지 않는다.** `main.py`도 화면 인스턴스를 최초
방문 때 한 번만 만들고 캐싱해서 재사용한다.

### 2.4 Canvas에 도형을 개별로 수천 개 넘기면 클라이언트가 멈춘다

3D 지도에 더미 점 3000개를 `cv.Circle` 3000개(개별 도형 객체)로 그렸더니
드래그는 물론 다른 버튼 클릭까지 반응이 없어졌다. 서버 쪽 객체 생성 비용도
있지만, 진짜 문제는 클라이언트(Flutter)가 도형 수천 개를 하나하나 따로
그려야 하는 비용이었다.

**해결**: `cv.Points`(좌표 여러 개를 한 도형에 묶어서 한 번에 넘기는 API)로
교체. 도형 개수 3025개 → 26개, 서버 쪽 처리 시간 18ms → 2.4ms. 로봇 팔
포인트클라우드(구현 순서 7번)도 처음부터 이 방식으로 만들어야 한다 —
`render3d/shapes.py`의 `draw_points()` 참고.

### 2.5 "찾고 있는 3d 모델" 패널은 런타임 OBJ 로딩을 예외로 허용받았다

원래 명세(`hidden_object_search_ui_spec_v4.md` 8·11장)는 런타임 OBJ 로딩과
인터랙티브 3D 뷰어를 금지했다. 사용자가 이 패널 하나에 한해 예외를
승인했다 (`docs/hidden_object_search_ui_spec_v4.md` 상단 "2026-08-05 정정"
참고). 단, `pyrender`/`trimesh`/`open3d` 같은 외부 3D 라이브러리는 여전히
안 쓴다 — `components/object_viewer.py`가 numpy만으로 면 중심점을 투영·
셰이딩해서 점으로 스플랫하고 BMP로 인코딩해 `ft.Image`에 raw bytes로
먹인다. front_ui 의존성(flet/numpy/requests)은 안 늘었다.

3D 지도(홈의 "물체 위치 3D 맵", 작업 화면 예정) 쪽은 원래 명세대로
런타임 렌더러 금지가 그대로 유효하다 — `render3d/`는 numpy 투영 +
`flet.canvas` 벡터 드로잉만 쓴다.

---

## 3. 파일 구조 현황

```
front_ui/
├── PROGRESS.md              이 문서
├── src/
│   ├── main.py               완료 — 화면 전환, state_client 연결, 스레드 안전 갱신
│   ├── theme.py               완료 — ROBOT 색상 추가됨
│   ├── config.py              완료
│   ├── labels.py               완료
│   ├── client/
│   │   ├── state_client.py     완료 — 폴링, 연결 판정
│   │   └── schema.py            미착수(비어있음) — 지금 뷰 코드가 dict를 직접
│   │                             쓰도록 짜여 있어서 당장은 필요 없음. 스키마가
│   │                             복잡해지면 그때 만든다
│   ├── components/
│   │   ├── panel.py             완료
│   │   ├── status.py            완료
│   │   └── object_viewer.py      완료 — 드래그 회전 3D 물체 뷰어
│   ├── views/
│   │   ├── home_view.py          진행 중 — 4개 패널 실데이터 연결(현재 작업/
│   │   │                          시스템 상태/3d 모델/3D 맵), "최근 실행 경과"만 남음
│   │   ├── monitor_view.py        레이아웃만, 데이터 미연결. STAGES 목록이
│   │   │                          labels.py와 중복 정의돼 있음 (정리 필요)
│   │   └── log_view.py            레이아웃만, 데이터 미연결
│   ├── render3d/
│   │   ├── projection.py         완료 — Camera, project(). numpy 검증: 3000점
│   │   │                          0.05ms/프레임
│   │   ├── shapes.py              진행 중 — draw_grid/draw_axes/draw_points/
│   │   │                          draw_robot_marker 있음. draw_box_wire/
│   │   │                          draw_marker(물체 라벨)는 아직
│   │   ├── map_view.py            진행 중 — 마우스 회전·줌·리셋 되는 MapView.
│   │   │                          zones/objects 실데이터 연결 전, build_map()
│   │   │                          (snapshot 받는 최종 형태)도 아직
│   │   └── scene.py               완료 — scene_config.json 로더
│   └── assets/
│       ├── scene_config.json     바닥판만 있음 (책상/벽 boxes는 빈 배열)
│       ├── models/cup_red_01.obj  드래그 뷰어용 데모 mesh (개구리)
│       └── renders/cup_red_01.png 정적 렌더 폴백 (뷰어 우선순위상 지금 안 쓰임)
└── tools/
    ├── fake_server.py          완료 — 명세 스키마 그대로, stage/open_ratio가
    │                            시간에 따라 실제로 움직임. /frame.jpg는 501 스텁
    ├── render_object.py         완료 — OBJ → PNG 오프라인 사전렌더 (지금은
    │                            안 씀, OBJ 없는 물체용 폴백 경로로 남겨둠)
    └── mesh_to_points.py        미착수 — 로봇 팔 포인트클라우드용 (구현 순서 7번)

back_ui/                       미착수 (ROS 노드 전체)
```

---

## 4. 확인이 필요한 가정 (틀렸으면 알려주면 바로 고침)

| 가정 | 위치 | 틀렸을 때 영향 |
|---|---|---|
| `ft.Image(src="renders/xxx.png")`가 `src/assets/` 기준 상대경로로 해석됨 | `home_view.py` | 렌더 PNG 폴백 경로가 안 뜰 수 있음 (지금은 OBJ 뷰어가 우선이라 실제로는 안 씀) |
| scene_config 바닥판 방향: 64cm=전방(x), 120cm=좌우(y), 로봇이 왼쪽 변에서 28cm/뒤쪽 변에서 33cm | `assets/scene_config.json` | 3D 지도에서 바닥판·로봇 위치가 실제와 다르게 보임. `pos`/`size` 부호만 바꾸면 됨 |
| Flet 데스크톱/웹 모드 모두에서 `cv.Points`, `GestureDetector.on_scroll/on_double_tap` 동작 | `render3d/map_view.py` | 사용자가 직접 `flet run`으로 확인 중 |

---

## 5. 알려진 정리거리 (급하지 않음)

- `monitor_view.py`의 `STAGES` 리스트가 `labels.py`의 `STAGE`/`STAGE_ORDER`와
  중복 정의돼 있다. `labels.STAGE_ORDER`를 쓰도록 정리하면 된다.
- `environment.yaml`, `.gitignore` 둘 다 원래 파일 내용 자리에 "이 내용을
  만들 때 쓴 터미널 명령"이 그대로 들어가 있다 (`cat > x << 'EOF' ...`).
  동작에는 지장 없지만(둘 다 우연히 유효한 문법) 다음에 손볼 때 정리하면 좋음.

---

## 6. 다음 순서

`docs/3d_map_spec.md` 13장 기준, 지금 3~4단계 사이:

1. ~~`projection.py` + 성능 측정~~ 완료
2. ~~`draw_grid` + `draw_axes`~~ 완료 (+ 로봇 마커도 추가함, 명세엔 없던 것)
3. ~~마우스 회전·줌~~ 완료
4. **`draw_box_wire` + `scene_config.json`의 책상/벽** ← 다음
5. `zones` 렌더링 + 서랍 열림 (`open_ratio` 연속 반영)
6. `objects` 마커 + 라벨
7. `mesh_to_points.py` + 로봇 팔 (draw_points로, Circle 아니고 Points로 처음부터)

그 다음: 작업(monitor) 화면에 실데이터 연결, 로그 화면 구현, `back_ui` 실제
ROS 노드 구현.
