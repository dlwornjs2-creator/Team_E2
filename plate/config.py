"""접시 파트 실험값 및 설정. 실장비 테스트 후 조정."""

# ---------------- 로봇 연결 ----------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight"
TCP_NAME = "GripperDA_v1"

VIRTUAL_MODE = True

# ---------------- 그리퍼 (DO 방식) ----------------
GRIP_OPEN_CH = 2       # DO2 = 열기
GRIP_CLOSE_CH = 3      # DO3 = 닫기
GRIP_WAIT = 1.5        # [s] 그리퍼 동작 대기 (실측 후 조정)

# ---------------- 모션 속도 ----------------
VEL_J, ACC_J = 40.0, 40.0        # 관절 이동 (안전하게 낮게 시작)
VEL_X, ACC_X = 100.0, 100.0      # 직선 이동

VEL_J_SLOW, ACC_J_SLOW = 20.0, 20.0
VEL_X_SLOW, ACC_X_SLOW = 30.0, 30.0   # 파지 전후 정밀 구간

# ---------------- 접시 파지 ----------------
PICK_APPROACH_HEIGHT = 80.0   # [mm] PICK 위쪽 접근 높이
PICK_RETREAT_HEIGHT = 100.0   # [mm] 파지 후 들어올릴 높이

# ---------------- 접시 규격 (경로 생성용, 추후 사용) ----------------
PLATE_RADIUS = 120.0
BRUSH_EFFECTIVE_WIDTH = 40.0
PATH_OVERLAP = 0.3
EDGE_MARGIN = 5.0

# ---------------- 세척 (추후 사용) ----------------
PLATE_WASH_ANGLE = 45.0
TARGET_FORCE = 3.0

# ---------------- 무게 판정 (추후 사용) ----------------
EMPTY_PLATE_WEIGHT = 0.0
WEIGHT_THRESHOLD = 0.0

