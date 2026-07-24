"""음식물 유무 판정 및 배출(털기) 동작.

팀원이 작성한 그릇용 코드를 접시 파트에 맞게 모듈화한 것.
동작 순서와 좌표/속도는 원본 그대로 유지한다.

원본과 다른 점:
  - rclpy 초기화/노드 생성은 하지 않는다(PlateController 가 이미 노드를 가짐)
  - set_tool/set_tcp 는 controller.setup() 이 담당하므로 여기서 하지 않는다
  - 마지막 5단계는 원본 좌표 대신 세척 위치 1 접근점(WASH1_APPROACH_J)으로
    이동한다
"""

import time

from DSR_ROBOT2 import get_tool_force, DR_BASE
from DR_common2 import posj, posx

# 모션은 정지 가드를 거친 버전을 쓴다 — 정지 시 털기/이동 루프도 즉시 끊긴다
from .motion_guard import movej, movel

from .waypoints import WASH1_APPROACH_J


# ================= 판정 파라미터 (원본 값) =================
# 음식물이 있으면 Fz 가 음수 방향으로 더 크게 측정된다.
FOOD_FZ_THRESHOLD = -2.62      # [N] 이 값 이하이면 음식물 있음
SAMPLES = 50
SAMPLE_INTERVAL = 0.05

# ================= 속도 (원본 값) =================
MOVE_VEL, MOVE_ACC = 80.0, 60.0        # 1단계 배출 위치 근처로 이동
ROTATE_VEL, ROTATE_ACC = 60.0, 45.0    # 2단계 배출 자세로 회전
SHAKE_COUNT = 3                        # 3단계 털기 횟수
SHAKE_J_VEL, SHAKE_J_ACC = 550.0, 650.0
RETURN_VEL, RETURN_ACC = 70.0, 50.0    # 4단계 원래 자세 복귀

# ================= 좌표 (원본 값) =================
CARRY_POSE = posx(412.896, -179.774, 156.351, 98.104, -111.108, 4.111)
SHAKE_CENTER_POSE = posx(441.982, -200.638, 208.385, 93.258, -106.471, -83.031)
SHAKE_UP_J = posj(16.253, 16.677, 101.101, -69.403, 93.417, -142.738)
SHAKE_DOWN_J = posj(16.253, 20.465, 104.930, -69.145, 90.723, -135.623)
RETURN_POSE = posx(427.901, -172.895, 172.360, 88.571, -116.869, -1.792)


def average_force(ref=DR_BASE, samples=SAMPLES):
    """툴 힘을 여러 번 측정해 평균값을 반환한다."""
    total = [0.0] * 6

    for _ in range(samples):
        force = get_tool_force(ref)
        for i in range(6):
            total[i] += float(force[i])
        time.sleep(SAMPLE_INTERVAL)

    return [value / samples for value in total]


def check_food(log=None):
    """음식물 유무를 판정한다. -> (food_exists, fz)"""
    if log:
        log.info("음식물 무게 측정을 시작합니다.")
    time.sleep(2.0)

    force = average_force(DR_BASE, SAMPLES)
    fz = force[2]
    food_exists = fz <= FOOD_FZ_THRESHOLD

    if log:
        log.info(f"현재 Fz: {fz:.3f} N / 임계값: {FOOD_FZ_THRESHOLD:.3f} N")
        log.info("판정: 음식물이 올려져 있음" if food_exists
                 else "판정: 음식물이 없음")

    return food_exists, fz


def dispose_food(log=None):
    """음식물 배출(털기) 동작. 1~4단계."""
    if log:
        log.info("음식물 배출 동작을 시작합니다.")

    # 1단계: 그릇 자세를 유지하며 배출 위치 근처로 이동
    if log:
        log.info("1단계: 배출 위치 근처로 이동")
    movel(CARRY_POSE, vel=MOVE_VEL, acc=MOVE_ACC)

    # 2단계: 음식물 배출 자세로 회전
    if log:
        log.info("2단계: 배출 자세로 회전")
    movel(SHAKE_CENTER_POSE, vel=ROTATE_VEL, acc=ROTATE_ACC)

    # 3단계: movej 로 빠른 위아래 털기
    if log:
        log.info("3단계: 빠른 위아래 털기 시작")
    for i in range(SHAKE_COUNT):
        if log:
            log.info(f"  털기 {i + 1}/{SHAKE_COUNT}")
        movej(SHAKE_UP_J, vel=SHAKE_J_VEL, acc=SHAKE_J_ACC)
        movej(SHAKE_DOWN_J, vel=SHAKE_J_VEL, acc=SHAKE_J_ACC)

    # 4단계: 원래 자세로 복귀
    if log:
        log.info("4단계: 원래 자세로 복귀")
    movel(RETURN_POSE, vel=RETURN_VEL, acc=RETURN_ACC)

    if log:
        log.info("음식물 배출 완료")

# ================= 5단계: 다음 작업 위치 =================
# 원본의 '다음 작업 위치' 좌표 대신 세척 위치 1 접근점으로 이동한다.
NEXT_VEL, NEXT_ACC = 80.0, 60.0


def move_to_next_position(log=None):
    """5단계: 다음 작업 위치(= 세척 위치 1 접근점)로 이동한다."""
    if log:
        log.info("5단계: 세척 위치 1 접근점으로 이동합니다.")
    movej(WASH1_APPROACH_J, vel=NEXT_VEL, acc=NEXT_ACC)
    if log:
        log.info("다음 작업 위치 이동 완료")


def run_food_check(log=None):
    """음식물 확인 -> (있으면) 배출 -> 세척 위치 1 접근점으로 이동.

    반환값: 음식물이 있었는지 여부
    """
    food_exists, _fz = check_food(log)

    if food_exists:
        dispose_food(log)
    elif log:
        log.info("음식물 배출 동작을 생략합니다.")

    move_to_next_position(log)
    return food_exists