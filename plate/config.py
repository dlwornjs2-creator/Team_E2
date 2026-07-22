"""접시 파트 실험값 및 설정. 실장비 테스트 후 조정."""

# ================= 실행 모드 =================
# 가상(시뮬레이션) 환경에서는 툴/TCP 가 컨트롤러에 등록돼 있지 않으므로
# 확인을 건너뛴다. 실물 구동 시 반드시 False 로 되돌릴 것.
VIRTUAL_MODE = False

# ================= 로봇 연결 =================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight"
TCP_NAME = "GripperDA_v1"

# ================= 그리퍼 (DO 방식) =================
GRIP_OPEN_CH = 2       # DO2 = 열기
GRIP_CLOSE_CH = 3      # DO3 = 닫기
GRIP_WAIT = 1.5        # [s] 그리퍼 동작 대기

# ================= 모션 속도 =================
VEL_J, ACC_J = 40.0, 40.0            # 관절 이동
VEL_X, ACC_X = 100.0, 100.0          # 직선 이동
VEL_J_SLOW, ACC_J_SLOW = 20.0, 20.0
VEL_X_SLOW, ACC_X_SLOW = 30.0, 30.0  # 파지/배치 정밀 구간

# ================= 접시 파지 / 배치 =================
PICK_APPROACH_HEIGHT = 60.0    # [mm] PICK 위쪽 접근 높이
PICK_RETREAT_HEIGHT = 60.0     # [mm] 파지 후 수직 상승 높이
PLACE_APPROACH_HEIGHT = 60.0   # [mm] PLACE 위쪽 접근 높이
PLACE_RETREAT_HEIGHT = 60.0    # [mm] 놓은 뒤 수직 이탈 높이

# ================= 사용자 좌표계 =================
# 펜던트에서 등록한 접시 평면 좌표계 ID
# TODO: 펜던트에서 확인한 실제 번호로 교체
DISH1_COORD = 101      # dish1 = 접시 앞면
DISH2_COORD = 102      # dish2 = 접시 뒷면

# ================= 세척 진입 =================
# 솔 높이 230mm + 접시가 그리퍼 아래로 내려온 만큼의 여유
WASH_SAFE_HEIGHT = 350.0    # [mm, base] 이동 시 확보할 안전 높이
WASH_HOVER_HEIGHT = 80.0    # [mm] 세척 시작점 위에서 하강 시작할 높이
WASH_DESCEND_VEL = 30.0     # [mm/s] 하강 속도

# ================= 세척 경로 =================
PLATE_RADIUS = 82.5             # [mm] 접시 반지름 (지름 165mm)
GRIP_MARGIN = 50.0              # [mm] 그리퍼가 물고 있는 rim 폭
BRUSH_EFFECTIVE_WIDTH = 40.0    # [mm] 솔 유효 세척 폭
PATH_OVERLAP = 0.3
EDGE_MARGIN = 5.0
ANGLE_STEP = 15.0
WASH_PASSES = 2

# ================= 세척 동작 =================
WASH_VEL, WASH_ACC = 50.0, 50.0   # 세척 경로 이동 속도
WASH_APPROACH_VEL = 30.0          # 세척 면 진입/이탈 속도
WASH_PLANE_Z = 0.0                # [mm] 좌표계 원점 기준 세척 평면 높이
WASH_SAFE_Z = 50.0                # [mm] 진입/이탈 시 띄울 높이

# ================= 무게 판정 (미확정) =================
EMPTY_PLATE_WEIGHT = 0.0
WEIGHT_THRESHOLD = 0.0

