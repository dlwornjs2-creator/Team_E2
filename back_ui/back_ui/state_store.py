"""현재 상태 스냅샷을 들고 있는 스레드 세이프 저장소.

HTTP 서버 스레드(읽기)와 ROS 토픽 콜백(쓰기)이 같은 데이터를 동시에 건드릴
수 있어서 락으로 보호한다.

`robot.links`는 `/joint_states` 구독(node.py)이 실제 값으로 채운다 —
back_ui가 처음으로 붙인 진짜 토픽이다(2026-08-05). 나머지(`system`/
`objects`/`zones`/`recent_tasks`)는 아직 구독할 실제 토픽이 없어서(팀원
노드들이 아직 코드 자체가 없음 — image/voice/main) front_ui가 기대하는
스키마(저장소 루트 front_ui/README.md)만 맞춘 정적값이다. `task`만 시간에
따라 움직이는 더미값인데, 폴링이 끊기지 않고 계속 값을 보내는지 눈으로
확인하기 위한 용도다.
"""

import threading
import time

# front_ui/src/labels.py의 STAGE_ORDER와 정확히 같은 순서. back_ui가
# front_ui 코드를 가져오면 안 되니(환경이 다름, CLAUDE.md) 값만 그대로
# 옮겨 적는다 — 여기 순서가 바뀌면 front_ui 쪽도 같이 맞춰야 한다.
_STAGE_ORDER = [
    "idle",
    "initial_observe",
    "select_zone",
    "open_container",
    "internal_reobserve",
    "verify_candidate",
    "approach_grasp",
    "verify_grasp",
    "transport",
    "place",
    "return_home",
    "done",
]
_STAGE_DURATION = 4.0  # 각 단계에 머무는 시간(초). 데모 속도 조절용.
_ROBOT_STALE_AFTER = 2.0  # 이 시간 동안 /joint_states가 안 오면 연결 끊긴 걸로 본다


def _dummy_task(elapsed: float) -> dict:
    """TODO(다음 단계): 실제 토픽 콜백이 채우도록 교체한다.

    지금은 시간이 지나면 단계가 계속 순환하고 elapsed_sec도 계속 늘어난다
    — front_ui 진행률 원형이 움직이는 걸로 "폴링이 살아있고 값이 계속
    갱신되는지"를 눈으로 확인할 수 있다(정적 스냅샷만으로는 매번 같은
    값이라 새로 온 건지 캐시된 건지 구분이 안 됐다).
    """
    cycle = _STAGE_DURATION * len(_STAGE_ORDER)
    pos = elapsed % cycle
    stage = _STAGE_ORDER[int(pos // _STAGE_DURATION)]
    return {
        "task_id": "back_ui_dummy_task",
        "voice_command": "(더미) 빨간 컵 좀 찾아줘",
        "target_id": None,
        "target_name": "(더미 데이터 — 아직 토픽 구독 전)",
        "status": "RUNNING",
        "stage": stage,
        "elapsed_sec": round(elapsed, 1),
        "current_zone": None,
        "action": "",
        "action_reason": "",
        "detections": [],
    }


def _base_snapshot() -> dict:
    """`task`를 뺀 나머지는 아직 전부 "모른다"에 해당하는 중립값이다.
    front_ui는 이미 null/빈 배열을 안내 문구로 처리하도록 만들어져 있어서
    화면이 비거나 죽지 않는다."""
    return {
        "frame_id": 0,
        "system": {
            "state": "IDLE",
            "nodes": {
                "image": False,
                "main": False,
                "db": False,
                "voice": False,
                "state": False,
            },
            "robot_connected": False,  # get_snapshot()이 매번 실측값으로 덮어씀
            "camera_connected": False,
            "gripper_state": "open",
            "object_count_total": 0,
            "object_count_confirmed": 0,
            "object_count_unknown": 0,
        },
        "objects": [],
        "zones": [],
        "recent_tasks": [],
    }


class StateStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._start = time.time()
        self._snapshot = _base_snapshot()
        self._robot_links: list[dict] = []  # /joint_states가 아직 안 왔으면 빈 배열
        self._last_joint_states_time: float | None = None  # robot_connected 판단용

    def update_robot_links(self, links: list[dict]):
        """`/joint_states` 콜백(node.py)이 매 메시지마다 부른다.

        메시지가 왔다는 사실 자체가 "로봇 연결됨"의 증거라 여기서 같이
        기록한다 — system.robot_connected를 계속 False로 고정해뒀던 게
        실제로 연결돼 있는데도 "연결 안 됨"으로 보이는 버그였다(2026-08-05).
        """
        with self._lock:
            self._robot_links = links
            self._last_joint_states_time = time.time()

    def get_snapshot(self) -> dict:
        """HTTP 핸들러가 요청마다 부른다.

        `ts`는 저장해두지 않고 매번 새로 찍는다 — front_ui가 이 값으로
        "연결 끊김"을 판단해서(3초 넘게 안 바뀌면 지연으로 봄, README 참고)
        요청이 실제로 살아서 처리됐다는 증거가 돼야 하기 때문이다.
        """
        with self._lock:
            snap = dict(self._snapshot)
            snap["system"] = dict(snap["system"])  # 얕은 복사라 안쪽도 따로 떠야 함
            robot_links = self._robot_links
            last_joint_states_time = self._last_joint_states_time

        now = time.time()
        snap["ts"] = now
        snap["task"] = _dummy_task(now - self._start)
        snap["robot"] = {"links": robot_links}
        snap["system"]["robot_connected"] = (
            last_joint_states_time is not None
            and (now - last_joint_states_time) < _ROBOT_STALE_AFTER
        )
        return snap
