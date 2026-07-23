#!/usr/bin/env python3

# 식기 세척 모듈 전역 파라미터 및 실측 좌표 데이터
# ==============================================================================

# 로봇 및 ROS 2 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "rokey_scrub_bowl"

# 그리퍼 DO(Digital Output) 핀 설정 (ON=1, OFF=0)
PIN_GRASP = 3
PIN_RELEASE = 2
GRIPPER_WAIT_TIME = 3.0

# 속도 및 가속도 파라미터
VEL_FAST, ACC_FAST = 100.0, 150.0
VEL_SLOW, ACC_SLOW = 50.0, 100.0

# 순응 제어 강성(Stiffness) 파라미터 (1/2 낮춤 -> 순응도 2배 증대 반영)
STIFFNESS_OUTER = [250, 250, 75, 100, 100, 100]  # PART A: 외부/하단 세척용
STIFFNESS_INNER = [150, 150, 50, 50, 50, 50]     # PART B: 내부 바닥/내벽 세척용
STIFFNESS_PLACE = [500, 500, 100, 200, 200, 200] # Sequence 3: 거치대 하강용

# 로봇 좌표 데이터 (DSR_ROBOT2 초기화 전 안전을 위해 순수 List/Dict 형태로 보관)
COORD_DATA = {
    "HOME": {"type": "posj", "val": [0, 0, 90, 0, 90, 0]},
    "PICK_BOWL": {"type": "posx", "val": [382.319, 7.696, 53.540, 7.159, -156.805, 8.008]},
    "PICK_UP": {"type": "posx", "val": [378.928, 0.177, 223.254, 12.306, -155.026, 20.238]},
    "SPONGE": {"type": "posx", "val": [530.988, 34.401, 229.682, 166.904, -149.576, 16.073]},
    "WAY": {"type": "posx", "val": [622.800, -51.316, 322.614, 10.299, 149.650, -112.982]},
    "RACK": {"type": "posj", "val": [-20.314, 57.958, 43.346, 82.163, 86.473, -125.400]}
}