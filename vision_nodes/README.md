# vision_nodes

Grounding DINO 기반 전체 객체 검출과 Grounding DINO + Any6D 기반 6D pose 추정을 하나의 ROS 2 패키지로 묶은 패키지입니다.

이 패키지는 다음 환경과 외부 프로젝트를 기준으로 구성했습니다.

- ROS 2 Humble
- RealSense D435i
- Grounding DINO
- Any6D
- 기존 `interfaces` 패키지

## 1. 패키지 구성

```text
vision_nodes/
├── CMakeLists.txt
├── package.xml
├── README.md
├── srv/
│   └── SetPickedObject.srv
├── nodes/
│   ├── dino_all_object_node.py
│   └── dino_any6d_node.py
└── code/
    ├── estimater.py
    ├── Utils.py
    ├── sam2_instantmesh.py
    ├── capture_any6d_anchor.py
    └── any6d_pose_to_base.py
```

이 패키지에서 새로 생성하는 서비스 인터페이스는 `SetPickedObject.srv` 하나입니다.

다음 서비스 인터페이스는 기존 `interfaces` 패키지에서 가져와 사용합니다.

```text
interfaces/srv/DetectObject.srv
interfaces/srv/DbSave.srv
```

## 2. 전체 통신 구조

```text
제어/UI 노드
   │
   │ /find_object_pose
   │ DetectObject.request 안의 JSON으로 class_label 전달
   ▼
dino_any6d_node
   │
   ├─ /set_picked_object 호출
   │  받은 class_label을 model_name으로 전달
   │
   ├─ Grounding DINO로 대상 검출
   └─ Any6D로 카메라 좌표계 6D pose 계산
             │
             ▼
dino_all_object_node
   ├─ /set_picked_object에서 제외 클래스 저장
   └─ /detect_all_objects 요청 시 전체 검출 결과에서 제외 클래스 제거
```

서비스 소유 관계는 다음과 같습니다.

| 서비스 이름 | 서비스 타입 | 서버 | 클라이언트 |
|---|---|---|---|
| `/find_object_pose` | `interfaces/srv/DetectObject` | `dino_any6d_node` | 제어/UI 노드 |
| `/set_picked_object` | `vision_nodes/srv/SetPickedObject` | `dino_all_object_node` | `dino_any6d_node` |
| `/detect_all_objects` | `interfaces/srv/DbSave` | `dino_all_object_node` | 제어/UI 노드 |

`/set_picked_object` 서버는 `dino_all_object_node` 하나만 생성합니다. `dino_any6d_node`는 이 서비스를 호출하는 클라이언트이므로 서비스 이름 충돌이 없습니다.

---

# 3. `dino_any6d_node`

실행:

```bash
ros2 run vision_nodes dino_any6d_node
```

ROS 노드 이름:

```text
/dsr01/dino_any6d_find_object_server
```

## 3.1 역할

1. RealSense의 컬러, 정렬된 depth, CameraInfo를 구독합니다.
2. `/find_object_pose`로 찾을 클래스 ID를 받습니다.
3. 받은 클래스 ID를 `/set_picked_object`로 `dino_all_object_node`에 전달합니다.
4. Grounding DINO로 요청 객체를 찾습니다.
5. 요청 객체가 없으면 `green_box`, `gray_box` 순서의 fallback 검출을 수행합니다.
6. 선택된 객체의 OBJ mesh를 불러옵니다.
7. Any6D로 카메라 좌표계 기준 6D pose를 계산합니다.
8. 결과를 `/find_object_pose`의 응답 JSON 문자열로 반환합니다.

## 3.2 구독 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | Grounding DINO와 Any6D 컬러 영상 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 컬러 영상에 정렬된 depth |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | 카메라 내부 파라미터 K |

## 3.3 `/find_object_pose` 서비스 서버

```text
서비스 이름: /find_object_pose
서비스 타입: interfaces/srv/DetectObject
역할: 제어/UI 노드에서 요청한 객체를 검출하고 카메라 좌표계 6D pose 반환
```

노드 코드에서 사용하는 서비스 필드는 다음과 같습니다.

```srv
string request
---
bool success
string response
string message
```

### 받는 값: `request.request`

`request`는 JSON 문자열이어야 합니다.

필수 값:

- `request_id`: 요청 식별자
- `class_label`: 고정 클래스 ID

`class_label` 대신 `object_name` 또는 `name` 필드도 코드에서 읽을 수 있지만, 기본 사용은 `class_label`을 권장합니다.

요청 예시:

```json
{
  "request_id": "pose_001",
  "class_label": "green_frog"
}
```

ROS 2 호출 예시:

```bash
ros2 service call /find_object_pose \
  interfaces/srv/DetectObject \
  '{request: "{\"request_id\":\"pose_001\",\"class_label\":\"green_frog\"}"}'
```

허용 클래스 ID:

```text
yellow_can
green_box
gray_box
white_bear
aircon_remote
green_frog
otter_in_can
```

### 내부 처리

`class_label=green_frog`를 받으면 다음 순서로 처리합니다.

```text
1. JSON과 class_label 유효성 검사
2. /set_picked_object에 model_name=green_frog 전달
3. 전달 성공 응답 확인
4. Grounding DINO로 green_frog 검출
5. 미검출 시 green_box, gray_box fallback 검색
6. 선택된 객체에 Any6D 실행
7. camera_color_optical_frame 기준 pose 반환
```

현재 코드는 `/set_picked_object` 전달을 Any6D 검출보다 먼저 수행합니다. 따라서 Any6D 추정이 이후 실패하더라도 `dino_all_object_node`에는 해당 클래스가 제외 대상으로 저장될 수 있습니다.

`/set_picked_object` 서버를 찾지 못하거나 응답이 실패하면 `/find_object_pose` 처리도 실패로 반환하며, Grounding DINO/Any6D 처리를 계속하지 않습니다.

### 보내는 값: 응답

#### 성공 시

- `success`: `true`
- `response`: pose 정보가 포함된 JSON 문자열
- `message`: 검출 클래스, confidence, fallback 여부 설명

`response` JSON 예시:

```json
{
  "request_id": "pose_001",
  "detected": true,
  "detected_name": "초록 개구리 인형",
  "detected_class_label": "green_frog",
  "pose": {
    "frame_id": "camera_color_optical_frame",
    "stamp": {
      "sec": 0,
      "nanosec": 0
    },
    "position": {
      "x": 0.12,
      "y": 0.03,
      "z": 0.55
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  }
}
```

- position 단위: metre
- orientation: quaternion `x, y, z, w`
- frame: `camera_color_optical_frame`

#### 요청 객체와 fallback 박스가 모두 없을 때

통신 처리는 정상 완료되므로 서비스의 `success`는 `true`이고, 실제 검출 여부는 JSON 안의 `detected=false`로 구분합니다.

```json
{
  "request_id": "pose_001",
  "detected": false,
  "detected_name": "",
  "detected_class_label": "",
  "pose": null
}
```

#### 오류 시

아래 경우에는 `success=false`가 반환됩니다.

- JSON 형식 오류
- `request_id` 누락
- class ID 누락 또는 미지원 class ID
- 다른 pose 요청이 이미 처리 중
- `/set_picked_object` 서버를 찾지 못함
- `/set_picked_object` 응답 timeout 또는 실패
- Grounding DINO/Any6D 실행 오류

## 3.4 `/set_picked_object` 서비스 클라이언트

```text
서비스 이름: /set_picked_object
서비스 타입: vision_nodes/srv/SetPickedObject
서버 노드: dino_all_object_node
```

Any6D 노드는 `/find_object_pose`에서 받은 `class_label`을 그대로 `model_name`으로 전달합니다.

전달 예시:

```text
/find_object_pose 요청 class_label: green_frog
                  ↓
/set_picked_object 요청 model_name: green_frog
```

서비스 서버 발견 기본 대기 시간:

```text
3.0초
```

서비스 응답 기본 대기 시간:

```text
3.0초
```

실행 옵션:

```bash
ros2 run vision_nodes dino_any6d_node -- \
  --picked-service-wait-sec 5.0 \
  --picked-response-timeout-sec 5.0
```

---

# 4. `dino_all_object_node`

실행:

```bash
ros2 run vision_nodes dino_all_object_node
```

ROS 노드 이름:

```text
/dino_all_objects_service
```

## 4.1 역할

1. RealSense 컬러 영상을 구독합니다.
2. `/set_picked_object`로 Any6D 노드가 전달한 클래스 ID를 저장합니다.
3. `/detect_all_objects` 요청이 오면 최신 카메라 프레임을 Grounding DINO로 분석합니다.
4. confidence 기준 미만 결과를 제거합니다.
5. 클래스별 최대 1개 결과만 유지합니다.
6. 겹치는 박스가 같은 물체로 판단되면 confidence가 높은 결과만 유지합니다.
7. `picked_class`와 같은 클래스는 검출되더라도 결과와 화면 표시에서 제외합니다.
8. 남은 클래스 ID 목록을 `DbSave.response`의 JSON 문자열로 반환합니다.

## 4.2 구독 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 전체 객체 Grounding DINO 검출 |

## 4.3 `/set_picked_object` 서비스 서버

서비스 정의:

```srv
# 픽업 완료된 고정 클래스명
string model_name
---
bool success
string message
```

```text
서비스 이름: /set_picked_object
서비스 타입: vision_nodes/srv/SetPickedObject
요청 주체: dino_any6d_node
```

### 받는 값

```text
request.model_name
```

예시:

```yaml
model_name: green_frog
```

### 처리

유효한 클래스이면 다음 값으로 저장합니다.

```python
self.picked_class = "green_frog"
```

그 이후 `/detect_all_objects`에서 `green_frog`가 실제로 검출되어도 결과에 포함하지 않습니다.

예:

```text
카메라 실제 검출:
yellow_can, green_frog, gray_box

picked_class:
green_frog

최종 반환:
yellow_can, gray_box
```

한 번에 저장되는 제외 클래스는 하나입니다. 새 class ID가 들어오면 기존 값을 새 값으로 교체합니다.

초기화 요청으로 사용할 수 있는 값:

```text
""
none
clear
reset
```

### 보내는 값

성공 예시:

```yaml
success: true
message: "픽업 완료 클래스 green_frog을 이후 검출 결과에서 제외합니다."
```

지원하지 않는 class ID 예시:

```yaml
success: false
message: "알 수 없는 클래스입니다: cup. 허용값=..."
```

직접 테스트:

```bash
ros2 service call /set_picked_object \
  vision_nodes/srv/SetPickedObject \
  "{model_name: 'green_frog'}"
```

초기화:

```bash
ros2 service call /set_picked_object \
  vision_nodes/srv/SetPickedObject \
  "{model_name: 'reset'}"
```

## 4.4 `/detect_all_objects` 서비스 서버

```text
서비스 이름: /detect_all_objects
서비스 타입: interfaces/srv/DbSave
역할: 요청을 트리거로 최신 카메라 프레임에서 전체 고정 클래스를 검출
```

사용하는 서비스 정의:

```srv
# 데이터 저장 — 작업 1회분을 통째로 반영한다
string request
---
bool success
string response
string message
```

이 노드에서는 `DbSave` 타입을 사용하지만 실제 DB insert/update는 수행하지 않습니다. `request` 문자열을 전체 객체 검출 트리거와 요청 식별용으로 사용하고, 검출된 클래스 ID를 `response` JSON 문자열로 반환합니다.

### 받는 값: `request.request`

`request`는 비어 있지 않은 JSON 객체 문자열이어야 합니다.

최소 요청:

```json
{}
```

권장 요청:

```json
{
  "source": "ui",
  "request_id": "detect_001"
}
```

현재 코드에서 사용하는 요청 필드:

| 필드 | 필수 여부 | 용도 |
|---|---|---|
| `source` | 선택 | 호출한 노드 또는 UI 이름. 응답의 `request_source`로 복사 |
| `request_id` | 선택 | 요청 식별자. 응답에 그대로 복사 |

그 외 JSON 필드가 있어도 검출 처리에는 사용하지 않습니다.

호출 예시:

```bash
ros2 service call /detect_all_objects \
  interfaces/srv/DbSave \
  '{request: "{\"source\":\"ui\",\"request_id\":\"detect_001\"}"}'
```

최소 호출:

```bash
ros2 service call /detect_all_objects \
  interfaces/srv/DbSave \
  '{request: "{}"}'
```

### 검출 대상 클래스 ID

```text
yellow_can
green_box
gray_box
white_bear
aircon_remote
green_frog
otter_in_can
```

### 처리 순서

```text
1. request.request JSON 파싱
2. 최신 컬러 프레임 확인
3. Grounding DINO로 전체 고정 클래스 검출
4. confidence 기준 미만 제거
5. picked_class 제거
6. 클래스별 중복 및 겹치는 박스 정리
7. response.response에 JSON 문자열 작성
```

`picked_class` 제외는 응답을 만든 뒤가 아니라 검출 후보를 수집하는 단계에서 수행됩니다. 따라서 제외 클래스는 `model_names`뿐 아니라 결과 화면의 bounding box에도 표시되지 않습니다.

### 보내는 값: `response.success`

현재 의미는 다음과 같습니다.

| 상황 | `success` |
|---|---|
| 제외 후 남은 객체가 1개 이상 | `true` |
| 제외 후 남은 객체가 없음 | `false` |
| JSON 오류, 카메라 없음, DINO 오류 | `false` |

### 보내는 값: `response.response`

`response`는 JSON 문자열입니다.

포함 필드:

| 필드 | 의미 |
|---|---|
| `source` | 응답 생성 노드. 항상 `dino_all_object_node` |
| `request_id` | 요청 JSON에서 받은 `request_id` |
| `request_source` | 요청 JSON에서 받은 `source` |
| `model_names` | 최종 검출된 고정 클래스 ID 배열 |
| `count` | `model_names` 개수 |
| `excluded_model_name` | `/set_picked_object`로 받은 제외 클래스. 없으면 빈 문자열 |

검출 성공 예시:

```yaml
success: true
response: '{"source":"dino_all_object_node","request_id":"detect_001","request_source":"ui","model_names":["yellow_can","gray_box"],"count":2,"excluded_model_name":"green_frog"}'
message: "2개 객체 검출"
```

`response` JSON을 펼치면 다음과 같습니다.

```json
{
  "source": "dino_all_object_node",
  "request_id": "detect_001",
  "request_source": "ui",
  "model_names": [
    "yellow_can",
    "gray_box"
  ],
  "count": 2,
  "excluded_model_name": "green_frog"
}
```

아무것도 검출되지 않은 예시:

```yaml
success: false
response: '{"source":"dino_all_object_node","request_id":"detect_001","request_source":"ui","model_names":[],"count":0,"excluded_model_name":"green_frog"}'
message: "confidence 0.30 이상 검출 없음 픽업 제외=green_frog"
```

JSON 요청 오류 예시:

```yaml
success: false
response: '{"model_names":[],"count":0}'
message: "request가 올바른 JSON이 아닙니다: ..."
```

카메라 프레임을 받지 못한 경우:

```yaml
success: false
response: '{"source":"dino_all_object_node",...,"model_names":[],"count":0,...}'
message: "아직 카메라 프레임을 받지 못했습니다."
```

---

# 5. 서비스 간 실제 동작 예시

## 5.1 Any6D 요청

제어 노드가 다음 요청을 전송합니다.

```json
{
  "request_id": "pose_001",
  "class_label": "green_frog"
}
```

## 5.2 Any6D 노드가 제외 클래스 전달

`dino_any6d_node`가 자동으로 다음 서비스를 호출합니다.

```yaml
service: /set_picked_object
model_name: green_frog
```

## 5.3 전체 객체 검출 요청

제어/UI 노드가 다음 요청을 보냅니다.

```json
{
  "source": "ui",
  "request_id": "detect_001"
}
```

카메라에서 다음 클래스가 검출되었다고 가정합니다.

```text
yellow_can
green_frog
gray_box
```

`green_frog`는 이미 제외 클래스로 저장되어 있으므로 최종 응답은 다음과 같습니다.

```json
{
  "source": "dino_all_object_node",
  "request_id": "detect_001",
  "request_source": "ui",
  "model_names": ["yellow_can", "gray_box"],
  "count": 2,
  "excluded_model_name": "green_frog"
}
```

---

# 6. Grounding DINO 기본 경로

두 노드는 다음 기본값을 사용합니다.

```text
GroundingDINO root:
~/GroundingDINO

Config:
~/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py

Weights:
~/GroundingDINO/weights/groundingdino_swint_ogc.pth
```

다른 위치를 사용할 경우 환경변수를 설정할 수 있습니다.

```bash
export GROUNDINGDINO_HOME=/path/to/GroundingDINO
```

Grounding DINO는 실행 환경에 editable install하는 것을 권장합니다.

```bash
conda activate Any6D
cd ~/GroundingDINO
pip install -e .
```

설치 확인:

```bash
python3 -c "import groundingdino; print(groundingdino.__file__)"
```

---

# 7. Any6D 외부 폴더와 `code/` 사용

ROS 노드 코드는 이 패키지에 있지만, Any6D 본체와 모델 파일은 기존 위치를 유지합니다.

```text
~/Any6D
~/GroundingDINO
```

`code/` 폴더는 수정한 Any6D 핵심 파일과 보조 코드를 보관하기 위한 백업 및 배포용 폴더입니다.

## 7.1 수정 파일 적용

먼저 원본을 백업합니다.

```bash
cp ~/Any6D/estimater.py ~/Any6D/estimater.py.bak
cp ~/Any6D/foundationpose/Utils.py ~/Any6D/foundationpose/Utils.py.bak
cp ~/Any6D/sam2_instantmesh.py ~/Any6D/sam2_instantmesh.py.bak
```

그다음 수정본을 적용합니다.

```bash
cp code/estimater.py ~/Any6D/estimater.py
cp code/Utils.py ~/Any6D/foundationpose/Utils.py
cp code/sam2_instantmesh.py ~/Any6D/sam2_instantmesh.py
```

## 7.2 `code/estimater.py`

Any6D 원본을 프로젝트 환경에 맞게 수정한 파일입니다.

주요 수정 내용:

- OBJ에 PNG 텍스처가 없어도 즉시 실패하지 않도록 처리
- UV 텍스처가 있으면 PNG/MTL 텍스처 사용
- 텍스처가 없고 vertex color가 있으면 vertex color 사용
- 텍스처와 vertex color가 모두 없으면 중성색 기본 텍스처 생성
- `mesh.visual.material.image`가 `None`인 경우 검사
- Open3D/trimesh vertex color를 렌더링 데이터로 전달
- `reset_object()`에서 mesh 중심 계산 및 중심 기준 정점 이동
- 객체 변경 시 `model_center`, mesh, texture, mesh tensor 재설정

## 7.3 `code/Utils.py`

Any6D 내부 FoundationPose 유틸리티 수정본입니다.

주요 수정 내용:

- Python 3.10에서 직접 빌드한 확장 모듈을 사용하도록 `import mycpp` 적용
- `mycpp` import 실패 시 오류 원인 출력
- 텍스처 이미지와 vertex color를 모두 처리하도록 mesh tensor 생성 로직 보완
- 원본 NVIDIA 저작권 헤더 유지

핵심 import 변경:

```python
# 기존 형태
import mycpp.build.mycpp as mycpp

# 수정 형태
import mycpp
```

## 7.4 `code/sam2_instantmesh.py`

Any6D의 SAM2 + InstantMesh 처리 수정본입니다.

주요 용도와 수정 목적:

- SAM2 모델 및 config 경로 처리
- `sam2.1_hiera_l.yaml` 사용
- `sam2.1_hiera_large.pt` 사용
- 객체 마스크를 이용한 배경 제거와 InstantMesh 입력 이미지 생성
- Zero123++ 다중 뷰 이미지 생성
- InstantMesh를 이용한 vertex-color OBJ 생성
- 설치된 SAM2/InstantMesh 구조에 맞춘 import 및 경로 처리

## 7.5 `code/capture_any6d_anchor.py`

RealSense에서 Any6D용 RGB-D anchor를 저장하는 편의 코드입니다.

저장 파일:

```text
anchor_rgb.png
anchor_depth.png
anchor_K.npy
metadata.json
```

실행 예:

```bash
python3 code/capture_any6d_anchor.py \
  --output ~/Any6D/anchors/object_001
```

미리보기에서 `SPACE`를 누르면 저장하고 `Q` 또는 `ESC`로 종료합니다.

## 7.6 `code/any6d_pose_to_base.py`

Any6D의 카메라 좌표계 pose를 Doosan 로봇 base 좌표계로 변환하는 보조 코드입니다.

변환 순서:

```text
T_base_object = T_base_gripper @ T_gripper_camera @ T_camera_object
```

실행 예:

```bash
python3 code/any6d_pose_to_base.py \
  --pose-camera ~/Any6D/pose_camera.npy \
  --gripper-camera ~/Any6D/T_gripper2camera.npy
```

---

# 8. 빌드

패키지를 워크스페이스에 복사합니다.

```bash
cp -r vision_nodes ~/cobot_ws/src/
cd ~/cobot_ws
```

기존 `interfaces` 패키지에 다음 서비스가 생성되어 있어야 합니다.

```text
interfaces/srv/DetectObject.srv
interfaces/srv/DbSave.srv
```

빌드:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select interfaces vision_nodes
source install/setup.bash
```

서비스 인터페이스 확인:

```bash
ros2 interface show vision_nodes/srv/SetPickedObject
ros2 interface show interfaces/srv/DetectObject
ros2 interface show interfaces/srv/DbSave
```

---

# 9. 실행 순서

RealSense를 먼저 실행합니다.

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

`dino_all_object_node`를 먼저 실행하면 Any6D 노드가 `/set_picked_object` 서버를 바로 찾을 수 있습니다.

터미널 1:

```bash
conda activate Any6D
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
ros2 run vision_nodes dino_all_object_node
```

터미널 2:

```bash
conda activate Any6D
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
ros2 run vision_nodes dino_any6d_node
```

서비스 확인:

```bash
ros2 service list
ros2 service type /find_object_pose
ros2 service type /detect_all_objects
ros2 service type /set_picked_object
```

예상 타입:

```text
/find_object_pose     interfaces/srv/DetectObject
/detect_all_objects   interfaces/srv/DbSave
/set_picked_object    vision_nodes/srv/SetPickedObject
```

노드 확인:

```bash
ros2 node list
```

예상 노드:

```text
/dino_all_objects_service
/dsr01/dino_any6d_find_object_server
```

---

# 10. 빠른 테스트

## 10.1 `/set_picked_object` 직접 테스트

```bash
ros2 service call /set_picked_object \
  vision_nodes/srv/SetPickedObject \
  "{model_name: 'green_frog'}"
```

## 10.2 `/detect_all_objects` 테스트

```bash
ros2 service call /detect_all_objects \
  interfaces/srv/DbSave \
  '{request: "{\"source\":\"ui\",\"request_id\":\"detect_001\"}"}'
```

카메라에서 `green_frog`가 검출되어도 `response.response` JSON의 `model_names`에는 포함되지 않아야 합니다.

## 10.3 `/find_object_pose` 테스트

```bash
ros2 service call /find_object_pose \
  interfaces/srv/DetectObject \
  '{request: "{\"request_id\":\"pose_001\",\"class_label\":\"green_frog\"}"}'
```

이 요청을 실행하면 `dino_any6d_node`가 `/set_picked_object`도 자동으로 호출합니다.

---

# 11. Reference and attribution

본 프로젝트는 **Any6D**와 **Grounding DINO**의 연구 및 공개 구현을 활용하고 인용했습니다. `code/`에 포함된 수정 파일은 원본 프로젝트를 현재 ROS 2, RealSense, OBJ/vertex-color 처리 환경에 맞게 변경한 버전이며, 원본 프로젝트의 라이선스 및 파일별 저작권 고지를 따라야 합니다.

## 11.1 Any6D

- Taeyeop Lee, Bowen Wen, Minjun Kang, Gyuree Kang, In So Kweon, Kuk-Jin Yoon, **“Any6D: Model-free 6D Pose Estimation of Novel Objects,” CVPR 2025.**
- GitHub: https://github.com/taeyeopl/Any6D
- Paper: https://openaccess.thecvf.com/content/CVPR2025/html/Lee_Any6D_Model-free_6D_Pose_Estimation_of_Novel_Objects_CVPR_2025_paper.html

```bibtex
@inproceedings{lee2025any6d,
  title={Any6D: Model-free 6D Pose Estimation of Novel Objects},
  author={Lee, Taeyeop and Wen, Bowen and Kang, Minjun and Kang, Gyuree and Kweon, In So and Yoon, Kuk-Jin},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  year={2025}
}
```

## 11.2 Grounding DINO

- Shilong Liu, Zhaoyang Zeng, Tianhe Ren, Feng Li, Hao Zhang, Jie Yang, Qing Jiang, Chunyuan Li, Jianwei Yang, Hang Su, Jun Zhu, Lei Zhang, **“Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection,” ECCV 2024.**
- GitHub: https://github.com/IDEA-Research/GroundingDINO
- Paper: https://arxiv.org/abs/2303.05499

```bibtex
@inproceedings{liu2024groundingdino,
  title={Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection},
  author={Liu, Shilong and Zeng, Zhaoyang and Ren, Tianhe and Li, Feng and Zhang, Hao and Yang, Jie and Jiang, Qing and Li, Chunyuan and Yang, Jianwei and Su, Hang and Zhu, Jun and Zhang, Lei},
  booktitle={European Conference on Computer Vision},
  year={2024}
}
```

## 11.3 수정 코드 고지

- `code/estimater.py`와 `code/sam2_instantmesh.py`는 Any6D 공개 구현을 기반으로 수정했습니다.
- `code/Utils.py`는 Any6D가 사용하는 NVIDIA FoundationPose 계열 유틸리티를 기반으로 하며 파일 상단의 NVIDIA 저작권 고지를 유지했습니다.
- `nodes/dino_any6d_node.py`와 `nodes/dino_all_object_node.py`는 Grounding DINO 공개 구현의 inference API를 사용합니다.
- 이 패키지의 `Apache-2.0` 표기는 새로 작성한 ROS 2 패키지 구성 및 연결 코드에 대한 표기이며, 포함된 업스트림 파일의 원래 라이선스를 대체하지 않습니다.
