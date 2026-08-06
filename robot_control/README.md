# robot_control

Doosan M0609, OnRobot RG2, Any6D를 연결하여 물체를 집는 ROS 2 제어
패키지입니다. 상태 노드의 작업 요청을 받으면 DB에서 물체 정보를 조회하고,
새로운 Any6D 자세를 기다린 뒤 다음 순서로 동작합니다.

```text
JHOME_POS 이동 → RG2 열기 → 접근점 이동 → grasp 자세 이동
→ RG2 닫기 → Base Z 방향 상승 → JHOME_POS 복귀
```

> **주의:** 기본 설정은 실제 모션이 활성화된 상태입니다. 제어 노드를 실행하는
> 즉시 로봇이 `JHOME_POS`로 이동하고 RG2가 열립니다. 실제 셀에서 실행하기 전에
> 작업 공간을 비우고 비상정지 버튼을 준비하십시오.

## 지원 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Doosan M0609 제어기: `192.168.1.100`
- OnRobot RG2 Tool Changer: `192.168.1.1:502`
- PC 유선 IP 예시: `192.168.1.104/24`
- Doosan ROS 2 드라이버:
  [`ROKEY-SPARK/doosan-robot2_2026`](https://github.com/ROKEY-SPARK/doosan-robot2_2026)
- Team_E2 `state` 브랜치의 `interfaces`, `db`, `state` 패키지

Python 패키지:

```bash
python3 -m pip install numpy scipy "pymodbus==2.5.3"
```

`onrobot.py`가 `pymodbus.client.sync` API를 사용하므로 pymodbus 3.x가 아닌
2.5.x 버전이 필요합니다.

## 패키지 배치

새 작업공간을 사용하는 예시는 다음과 같습니다.

```bash
mkdir -p ~/cobot_ws/src
cd ~/cobot_ws/src

git clone --branch state \
  https://github.com/dlwornjs2-creator/Team_E2.git
git clone https://github.com/ROKEY-SPARK/doosan-robot2_2026.git
```

`robot_control` 디렉터리는 Team_E2 저장소 루트에 다음과 같이 배치합니다.

```text
~/cobot_ws/src/Team_E2/
├── db/
├── interfaces/
├── robot_control/
└── state/
```

## 빌드

```bash
cd ~/cobot_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

필수 인터페이스가 생성됐는지 확인합니다.

```bash
ros2 interface show interfaces/srv/DbLoad
ros2 interface show interfaces/srv/NodeInit
ros2 pkg executables robot_control
```

모듈형 제어 노드 실행 파일은 다음과 같이 표시되어야 합니다.

```text
robot_control robot_control_any6d_module
```

## 실제 로봇 네트워크

PC와 로봇을 유선 LAN으로 연결하고 PC 인터페이스를 `192.168.1.0/24`
대역으로 설정합니다. 로봇 전용 인터페이스에는 기본 게이트웨이를 설정하지 않는
것을 권장합니다.

```bash
ip -brief address
ping -c 3 192.168.1.100
ping -c 3 192.168.1.1
```

두 장치가 모두 응답해야 실제 모드를 실행할 수 있습니다.

## 실행 전 설정

[`robot_control/config.py`](robot_control/config.py)의 값을 실제 셀에 맞게
확인하십시오.

주요 기본값:

```python
robot_id = "dsr01"
robot_model = "m0609"
tool_name = "Tool Weight_1"
tcp_name = "GripperDA_v1"
home_joint = (0.0, 0.0, 90.0, 0.0, 90.0, 180.0)
enable_motion = True

input_mode = "any6d"
detection_service = "/find_object_pose"
pose_is_tcp_grasp = True
```

먼저 로봇을 움직이지 않고 확인하려면 다음 값을 `False`로 변경하십시오.

```python
enable_motion = False
```

`pose_is_tcp_grasp=True`이면 Any6D 자세를 최종 TCP grasp 자세로 직접
사용합니다. Any6D가 물체 중심 자세를 발행한다면 이를 `False`로 변경하고
`object_to_grasp_npy`에 `T_object_grasp` 4×4 행렬 파일을 지정해야 합니다.

## DB 미등록 물체 탐색

DB 조회 결과에 유효한 `location`이 없으면 다음 네 구역을 순서대로 탐색합니다.

1. 홈 조인트 `(0, 0, 90, 0, 90, 180)`
2. 1구역에서 Base X 방향 `+250 mm`
3. 조인트 `(6, 55, 43, -91, 96, 186)`
4. 3구역에서 Base X 방향 `-290 mm`

각 구역에 도착한 뒤 제어 노드는 현재 TCP 자세를 먼저 탐지노드에 전달하고,
TCP 등록이 성공하면 Any6D 탐지 서비스를 호출합니다.

```text
/update_robot_tcp_pose (interfaces/srv/UpdateTcpPose)
```

TCP 전달 요청 필드:

```text
tcp_pose: [X, Y, Z, A, B, C]
```

`X/Y/Z` 단위는 mm, `A/B/C` 단위는 degree이며 Doosan ZYZ 자세입니다.
탐지노드는 `success=true`로 응답해야 합니다. 서비스가 없거나 실패 또는 2초
타임아웃이 발생하면 제어 노드는 그 요청의 `/find_object_pose`를 호출하지 않습니다.

```text
/find_object_pose (interfaces/srv/DetectObject)
```

서비스의 `request` 문자열에 들어가는 목표물 요청 JSON 예시:

```json
{
  "request_id": "pick-001:zone-2",
  "request_type": "target",
  "task_id": "pick-001",
  "search_zone": 2,
  "object_name": "yellow_can",
  "name": "yellow_can",
  "class_label": "yellow_can"
}
```

탐지 성공 시 서비스 응답의 `success=true`와 함께 `response` 문자열에 다음
JSON을 반환합니다. pose가 같은 응답에 포함되므로 별도 pose 토픽은 없습니다.

```json
{
  "request_id": "pick-001:zone-2",
  "detected": true,
  "detected_name": "yellow_can",
  "detected_class_label": "yellow_can",
  "pose": {
    "frame_id": "camera_color_optical_frame",
    "stamp": {"sec": 0, "nanosec": 0},
    "position": {"x": 0.367289, "y": 0.008193, "z": 0.035476},
    "orientation": {"x": -0.9999971, "y": -0.0024120, "z": -0.0000005, "w": 0.0000681}
  }
}
```

탐지 실패 예시:

```json
{"request_id":"pick-001:zone-2","detected":false}
```

제어 노드는 서비스 응답을 기본 20초 기다립니다. 응답 또는 유효 자세가 없으면
다음 구역으로 이동합니다. 네 구역 모두 실패하면 홈으로 돌아갑니다.

요청한 물체를 해당 구역에서 찾지 못했거나 유효한 pose를 받지 못하면, 다음
구역으로 이동하기 전에 랜드마크 탐지를 한 번 더 요청합니다.

```json
{
  "request_id": "pick-001:zone-2:landmark",
  "request_type": "landmark",
  "task_id": "pick-001",
  "search_zone": 2,
  "candidate_targets": [
    {"object_name":"green_box","name":"green_box","class_label":"green_box"},
    {"object_name":"gray_box","name":"gray_box","class_label":"gray_box"}
  ]
}
```

두 랜드마크 중 하나를 찾은 경우 서비스 응답 JSON에 같은 `request_id`와
`detected=true`를 반환합니다. 랜드마크 응답에는 pose가 필요하지 않습니다.
제어 노드는 발견 응답 후 현재 구역에서 3초 대기한 다음 다음 탐색구역으로
이동합니다. 랜드마크도 찾지 못하면 별도 대기 없이 다음 구역으로 이동합니다.

관련 값은 `SearchConfig`에서 변경할 수 있습니다. Any6D 실행 노드가 아직
구현되지 않은 상태에서는 각 구역의 탐지 요청이 타임아웃으로 처리됩니다.

탐지 노드가 반환하는 `pose`는 카메라 좌표계의 `T_camera_object`입니다.
제어 노드는 탐지 순간의 현재 TCP 자세를 로봇에서 읽고 다음 순서로 Base 좌표로
변환한 후 이동합니다.

```text
T_base_object = T_base_tcp × T_tcp_camera × T_camera_object
```

Any6D의 `T_camera_object` 위치는 m 단위로 받고 변환 과정에서 1000을 곱해
mm로 변경합니다. `T_tcp_camera`는 `/Downloads/any6d_pose_to_base.py`와 함께
검증된 `/Downloads/T_gripper2camera.npy`의 eye-in-hand 캘리브레이션 값이며
`PoseConfig.tcp_to_camera`에 내장되어 있습니다. 허용 카메라 frame은 기본적으로 `camera`, `camera_link`,
`camera_color_optical_frame`입니다. 실제 카메라 캘리브레이션 또는 frame 이름이
다르면 실제모드 실행 전에 반드시 이 설정을 수정해야 합니다.

탐지 노드에 전달하는 `object_name`은 다음 OBJ 식별자 중 하나이며 문자열을
변환하지 않고 그대로 전달합니다: `yellow_can`, `green_box`, `gray_box`,
`white_bear`, `aircon_remote`, `green_frog`, `otter_in_can`.

## 실제 모드 실행

각 명령은 별도 터미널에서 실행합니다. 모든 터미널에서 작업공간을 source해야
합니다.

### 1. Doosan 실제 모드 드라이버

```bash
cd ~/cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=192.168.1.100 \
  model:=m0609 \
  name:=dsr01 \
  gui:=false
```

로그에서 다음 항목을 확인합니다.

```text
Connected to DRCF
ROBOT_STATE : STATE_STANDBY
Configured and activated dsr_controller2
```

### 2. DB 노드

```bash
cd ~/cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run db db
```

제어 노드는 `/db/load` 서비스를 사용합니다. DB 노드가 없으면 작업 요청이
실패합니다.

### 3. 모듈형 robot_control

먼저 Any6D conda 환경에서 검출 서버를 실행합니다. 이 서버는 카메라 좌표계의
자세만 반환하며 로봇을 직접 움직이지 않습니다.

```bash
cd ~/Any6D
conda activate <Any6D 환경 이름>
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
python dino_any6d_with_control.py --service /find_object_pose
```

다른 터미널에서 서비스가 준비되었는지 확인합니다.

```bash
ros2 service type /find_object_pose
```

출력은 `interfaces/srv/DetectObject`여야 합니다.

```bash
cd ~/cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_control robot_control_any6d_module
```

`enable_motion=True`이면 이 시점에 실제로 다음 초기화가 실행됩니다.

1. Tool/TCP 설정
2. `JHOME_POS` 이동
3. RG2 열기
4. `/control/init` 준비 완료
5. 작업 요청 대기

준비 상태 확인:

```bash
ros2 service call /control/init interfaces/srv/NodeInit \
  "{request: '{\"node\":\"operator\"}'}"
```

응답의 `success`, `ready`가 모두 `true`여야 합니다.

### 4. 상태 노드 선택 실행

현재 `state` 브랜치의 상태 노드는 초기화 확인 기능을 제공합니다.

```bash
ros2 run state state_node --ros-args \
  -p targets:="['db', 'control']"
```

상태 노드는 `/control/task` 서비스를 호출하고 `/state/robot_result` 서비스를
제공해야 합니다. 통합 전에는 아래처럼 작업 서비스를 직접 호출할 수 있습니다.

## 작업 요청 형식

### 상태 노드 액션 (기본 인터페이스)

상태 노드는 `/control/search` 액션으로 탐색 및 pick 작업을 요청합니다.

```text
/control/search (interfaces/action/Search)
```

터미널 호출 예시:

```bash
ros2 action send_goal /control/search interfaces/action/Search \
  "{target_name: aircon_remote, class_label: aircon_remote}" \
  --feedback
```

Goal을 받으면 기존 작업 큐와 동일하게 DB 조회, 탐색 구역 이동, Any6D 검출,
pick 및 홈 복귀를 수행합니다. 진행 중에는 `step`과 `progress` feedback을
전송합니다. 성공 Result의 `location`은 DB 등록 위치 또는
`search_zone_1`~`search_zone_4`이며, 찾지 못하면 빈 문자열입니다.

실행 중인 실제 로봇 모션은 액션 cancel로 안전하게 정지시킬 수 없으므로 cancel
요청을 거절합니다. 아직 큐에서 시작하지 않은 Goal만 취소할 수 있습니다.

### 호환 서비스

서비스:

```text
/control/task (interfaces/srv/ControlTask)
```

JSON 예시:

```json
{
  "task_id": "pick-001",
  "name": "빨간 컵",
  "class_label": "cup",
  "command": "pick",
  "requested_by": "state_node"
}
```

호출 예시:

```bash
ros2 service call /control/task interfaces/srv/ControlTask \
  "{request: '{\"task_id\":\"test-task\",\"name\":\"aircon_remote\",\"class_label\":\"aircon_remote\",\"command\":\"pick\",\"requested_by\":\"operator\"}'}"
```

이 요청이 실제 pick 흐름의 시작점입니다. 제어 노드는 작업을 접수한 뒤 내부적으로
`/find_object_pose`를 호출합니다. `/find_object_pose`를 터미널에서 직접 호출하는
것은 카메라·Any6D 단독 진단용이며 pick을 시작하지 않습니다.

작업 요청 후 제어 노드는 다음 순서로 처리합니다.

1. `name`으로 `/db/load` 조회
2. 결과가 없으면 `class_label`로 재조회
3. 작업 요청 이후의 새로운 Any6D 자세 대기
4. 유효한 자세를 받으면 pick 실행

## DB 계약

요청 서비스:

```text
/db/load (interfaces/srv/DbLoad)
```

요청 예시:

```json
{"name": "빨간 컵"}
```

또는:

```json
{"class_label": "cup"}
```

정상 응답의 `response` 문자열은 다음 형태의 JSON이어야 합니다.

```json
{
  "count": 1,
  "items": [
    {
      "name": "빨간 컵",
      "class_label": "cup",
      "location": "table_a",
      "last_seen": "2026-08-04T16:30:00"
    }
  ]
}
```

현재 `location`은 로그와 결과 메시지에 포함되며, 로봇을 해당 장소로 이동시키는
기능은 포함하지 않습니다.

## Any6D 서비스 자세 계약

서비스:

```text
/find_object_pose (interfaces/srv/DetectObject)
```

요구 조건:

- `pose.frame_id`는 기본적으로 `camera_color_optical_frame`
- 탐지노드 입력 위치 단위는 **m**
- 입력 방향은 카메라 좌표계 기준 quaternion `(x, y, z, w)`
- 제어 노드가 현재 TCP 자세와 eye-in-hand 보정 행렬을 사용해 Base 좌표로 변환
- timestamp는 현재 시각 기준 0.5초 이내
- 기본 최소 Base Z는 2 mm

Any6D 서비스 응답의 위치는 m 단위이며 제어 노드가 Base 변환 과정에서 mm로
변환합니다. `stamp`가 0이면
수신 시각의 pose로 취급합니다.

## 결과 확인

제어 노드는 진행 및 완료 결과를 다음 상태 노드 서비스로 요청합니다.

```text
/state/robot_result (interfaces/srv/RobotResult)
```

상태 노드는 `request`의 결과 JSON을 저장한 뒤 `success=true`로 응답해야 합니다.

주요 outcome:

| outcome | 의미 |
|---|---|
| `queued` | 작업 요청 접수 |
| `db_lookup` | DB 조회 중 |
| `waiting_pose` | 새 Any6D 자세 대기 |
| `pick_completed` | 파지·상승·홈 복귀 완료 |
| `not_found` | 제한 시간 내 유효 자세 없음 |
| `blocked_holding_object` | 이미 물체를 들고 있음 |
| `failed` | DB, 자세, 그리퍼 또는 로봇 오류 |

완료 후에는 물체를 든 상태로 홈에 복귀하며 자동 place 동작은 없습니다. RG2를
열거나 place 동작을 수행하기 전에는 다음 작업이 거부됩니다.

## 종료

다음 순서로 `Ctrl+C`를 입력합니다.

1. `robot_control_any6d_module`
2. DB 및 상태 노드
3. Doosan bringup

드라이버를 종료해도 RG2는 현재 개폐 상태를 유지합니다. 물체를 들고 있다면
그리퍼 상태를 현장에서 확인하십시오.
