"""접시 처리 컨트롤러. ROS Node가 아닌 일반 Python 클래스."""

from DSR_ROBOT2 import (
    set_tool, set_tcp, get_tool, get_tcp,
    movej, movel,
    set_velj, set_accj, set_velx, set_accx,
    DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
)
from DR_common2 import posx

from . import config as cfg
from .gripper import Gripper
from .waypoints import HOME, PLATE_PICK, PLATE_PLACE


class PlateController:
    """접시 파지 → 무게확인 → 세척 → 배치 담당.

    import 만으로는 로봇이 움직이지 않으며, 각 메서드를 호출할 때만 동작한다.
    """

    def __init__(self, node):
        self.node = node
        self.gripper = Gripper(node)
        self._log = node.get_logger()

    # ---------------- 초기화 ----------------
    def setup(self):
        """툴/TCP 적용 및 확인, 기본 속도 설정. 동작 전 1회 호출."""
        set_tool(cfg.TOOL_NAME)
        set_tcp(cfg.TCP_NAME)

        act_tool, act_tcp = get_tool(), get_tcp()
        self._log.info(f"active tool={act_tool!r}, tcp={act_tcp!r}")

        if act_tool != cfg.TOOL_NAME or act_tcp != cfg.TCP_NAME:
            if cfg.VIRTUAL_MODE:
                self._log.warn(
                    "Tool/TCP 미적용 상태지만 VIRTUAL_MODE 이므로 계속 진행합니다. "
                    "실물 구동 시에는 반드시 확인이 필요합니다."
                )
            else:
                self._log.error(
                    f"Tool/TCP 적용 실패 (요청 {cfg.TOOL_NAME!r}/{cfg.TCP_NAME!r}, "
                    f"현재 {act_tool!r}/{act_tcp!r}). 등록명과 로봇 연결 상태를 "
                    "확인하세요."
                )
                return False

        set_velj(cfg.VEL_J)
        set_accj(cfg.ACC_J)
        set_velx(cfg.VEL_X, cfg.ACC_X)
        set_accx(cfg.ACC_X, cfg.ACC_X)
        return True

    # ---------------- 기본 이동 ----------------
    def move_home(self):
        self._log.info("Move to HOME")
        movej(HOME, vel=cfg.VEL_J, acc=cfg.ACC_J)

    # ---------------- Step 1: 접시 파지 ----------------
    def pick_plate(self):
        """HOME → APPROACH → PICK → 파지 → 수직 RETREAT"""
        self._log.info("Pick plate: start")

        # 그리퍼를 미리 열어둔다
        self.gripper.open()

        # 접시 위쪽 접근 지점으로 이동 (PICK 자세 그대로, Z만 위)
        approach = self._offset_z(PLATE_PICK, cfg.PICK_APPROACH_HEIGHT)
        movel(approach, vel=cfg.VEL_X, acc=cfg.ACC_X, mod=DR_MV_MOD_ABS)

        # 파지 위치로 저속 하강 (자세 변화 없이 직선)
        movel(PLATE_PICK, vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
              mod=DR_MV_MOD_ABS)

        # 파지
        self.gripper.close()
        self._log.info("Pick plate: grasped")

        # 수직 상승: base 기준 Z+ 로만 이동
        #   - ref=DR_BASE  : 로봇 자세와 무관하게 지면 수직 방향
        #   - 회전 성분 0  : 파지 시 자세(접시 기울기)를 그대로 유지
        self.lift_vertical(cfg.PICK_RETREAT_HEIGHT)

        self._log.info("Pick plate: done")

    # ---------------- 수직 이동 ----------------
    def lift_vertical(self, dz, slow=True):
        """현재 자세를 유지한 채 base Z 방향으로만 dz[mm] 이동.

        회전 성분을 0으로 두므로 접시 기울기가 변하지 않고,
        ref=DR_BASE 이므로 그리퍼가 기울어져 있어도 지면 수직으로 올라간다.
        양수면 상승, 음수면 하강.
        """
        vel = cfg.VEL_X_SLOW if slow else cfg.VEL_X
        acc = cfg.ACC_X_SLOW if slow else cfg.ACC_X

        movel(posx(0.0, 0.0, dz, 0.0, 0.0, 0.0),
              vel=vel, acc=acc,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

    # ---------------- 유틸 ----------------
    @staticmethod
    def _offset_z(pos, dz):
        """주어진 posx에서 Z만 dz 만큼 이동한 새 posx 반환."""
        return posx(pos[0], pos[1], pos[2] + dz, pos[3], pos[4], pos[5])