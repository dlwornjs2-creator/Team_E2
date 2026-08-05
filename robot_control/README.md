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
topic = "/any6d/object_pose_base"
expected_base_frame = "base"
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

각 구역에 도착한 뒤 제어 노드는 미래의 Any6D 실행 노드에 다음 토픽으로
탐지를 요청합니다.

```text
/any6d/detection_request (std_msgs/msg/String)
```

요청 JSON 예시:

```json
{
  "request_id": "pick-001:zone-2",
  "task_id": "pick-001",
  "search_zone": 2,
  "name": "cup",
  "class_label": "cup"
}
```

Any6D 실행 노드는 같은 `request_id`를 사용해 다음 토픽에 결과를 발행해야
합니다.

```text
/any6d/detection_result (std_msgs/msg/String)
```

탐지 성공 예시:

```json
{"request_id":"pick-001:zone-2","detected":true}
```

탐지 실패 예시:

```json
{"request_id":"pick-001:zone-2","detected":false}
```

`detected=true`인 경우 Any6D 노드는 기존
`/any6d/object_pose_base` 토픽에도 해당 물체의 `PoseStamped`를 발행해야
합니다. 제어 노드는 탐지 응답을 기본 10초, 자세를 추가로 5초 기다립니다.
응답 또는 유효 자세가 없으면 다음 구역으로 이동합니다. 네 구역 모두 실패하면
홈으로 돌아가 `not_found` 결과를 발행한 뒤 다음 요청을 기다립니다.

요청한 물체를 해당 구역에서 찾지 못했거나 유효한 pose를 받지 못하면, 다음
구역으로 이동하기 전에 랜드마크 탐지를 한 번 더 요청합니다.

```json
{
  "request_id": "pick-001:zone-2:landmark",
  "request_type": "landmark",
  "task_id": "pick-001",
  "search_zone": 2,
  "candidate_targets": [
    {"name":"녹색 상자","class_label":"green_box"},
    {"name":"회색 수납장","class_label":"gray_cabinet"}
  ]
}
```

두 랜드마크 중 하나를 찾은 경우 탐지노드는 같은 `request_id`로
`detected=true`를 반환합니다. 랜드마크 pose는 pick에 사용하지 않습니다.
제어 노드는 발견 응답 후 현재 구역에서 3초 대기한 다음 다음 탐색구역으로
이동합니다. 랜드마크도 찾지 못하면 별도 대기 없이 다음 구역으로 이동합니다.

관련 값은 `SearchConfig`에서 변경할 수 있습니다. Any6D 실행 노드가 아직
구현되지 않은 상태에서는 각 구역의 탐지 요청이 타임아웃으로 처리됩니다.

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

현재 상태 노드 구현에는 `/state/robot_request` 작업 생성 기능이 없으므로 통합
전까지는 아래와 같이 테스트 요청을 직접 발행할 수 있습니다.

## 작업 요청 형식

토픽:

```text
/state/robot_request (std_msgs/msg/String)
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

발행 예시:

```bash
ros2 topic pub --once /state/robot_request std_msgs/msg/String \
  "{data: '{\"task_id\":\"pick-001\",\"name\":\"빨간 컵\",\"class_label\":\"cup\",\"command\":\"pick\",\"requested_by\":\"state_node\"}'}"
```

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

## Any6D 자세 계약

토픽:

```text
/any6d/object_pose_base (geometry_msgs/msg/PoseStamped)
```

요구 조건:

- `header.frame_id`는 `base`
- 위치 단위는 **mm**
- 방향은 로봇 Base 좌표계 기준 quaternion `(x, y, z, w)`
- timestamp는 현재 시각 기준 0.5초 이내
- 연속 3개 샘플이 안정적이어야 함
- 샘플 간 위치 변화는 5 mm 이하
- 샘플 간 회전 변화는 5° 이하
- 기본 최소 Base Z는 2 mm

ROS 표준 위치 단위는 m이지만 이 노드의 계약은 mm입니다. Any6D 발행 노드가
m를 사용하면 반드시 발행 전에 mm로 변환하십시오.

테스트 자세를 반복 발행하는 예시:

```bash
ros2 topic pub -r 10 /any6d/object_pose_base \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: 367.289, y: 8.193, z: 35.476}, orientation: {x: -0.9999971, y: -0.0024120, z: -0.0000005, w: 0.0000681}}}"
```

작업 요청보다 먼저 들어온 자세는 버려집니다. 작업 요청이 DB 조회를 통과한 뒤
30초 안에 안정적인 새 자세가 들어와야 합니다.

## 결과 확인

제어 결과 토픽:

```bash
ros2 topic echo /control/robot_result
```

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
