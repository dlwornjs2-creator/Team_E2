"""접시 처리 컨트롤러. ROS Node 가 아닌 일반 Python 클래스.

import 만으로는 로봇이 움직이지 않으며, 각 메서드를 호출할 때만 동작한다.

세척은 펜던트에 등록한 사용자 좌표계(dish1=101, dish2=102) 기준으로
수행한다. 좌표계 원점은 접시 중심이 수세미에 닿는 지점이고, +Z 가
수세미 방향(접시 법선), XY 평면이 접시 면이다. 따라서 XY 로만 움직이면
접시 면 위에서만 이동한다.
"""

import math

from DSR_ROBOT2 import (
    set_tool, set_tcp, get_tool, get_tcp, get_current_posx,
    movej, movel, movec,
    set_velj, set_accj, set_velx, set_accx,
    set_ref_coord,
    task_compliance_ctrl, set_stiffnessx, set_desired_force,
    release_force, release_compliance_ctrl,
    DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_FC_MOD_ABS,
)
from DR_common2 import posx

from . import config as cfg
from .gripper import Gripper
from .waypoints import (
    HOME, PLATE_PICK, PLATE_PLACE, PLATE_PLACE_J,
    PLATE_PLACE_APPROACH, PLATE_PLACE_APPROACH_J,
    DISH1_VIA_J, DISH1_WASH_START_J,
    DISH2_VIA_J, DISH2_WASH_START_J,
)


class PlateController:
    """접시 파지 -> 무게확인 -> 세척 -> 배치 담당."""

    def __init__(self, node):
        self.node = node
        self.gripper = Gripper(node)
        self._log = node.get_logger()

    # ================= 초기화 =================
    def setup(self):
        """툴/TCP 적용 및 확인, 기본 속도 설정. 동작 전 1회 호출."""
        set_tool(cfg.TOOL_NAME)
        set_tcp(cfg.TCP_NAME)

        act_tool, act_tcp = get_tool(), get_tcp()
        self._log.info(f"active tool={act_tool!r}, tcp={act_tcp!r}")

        if act_tool != cfg.TOOL_NAME or act_tcp != cfg.TCP_NAME:
            if cfg.VIRTUAL_MODE:
                self._log.warn(
                    "Tool/TCP 미적용 상태지만 VIRTUAL_MODE 이므로 진행합니다."
                )
            else:
                self._log.error(
                    f"Tool/TCP 적용 실패 (요청 {cfg.TOOL_NAME!r}/{cfg.TCP_NAME!r}, "
                    f"현재 {act_tool!r}/{act_tcp!r}). 펜던트에서 활성화했는지, "
                    "로봇 연결이 정상인지 확인하세요."
                )
                return False

        set_velj(cfg.VEL_J)
        set_accj(cfg.ACC_J)
        set_velx(cfg.VEL_X, cfg.ACC_X)
        set_accx(cfg.ACC_X, cfg.ACC_X)
        return True

    # ================= 기본 이동 =================
    def move_home(self):
        self._log.info("Move to HOME")
        movej(HOME, vel=cfg.VEL_J, acc=cfg.ACC_J)

    def lift_vertical(self, dz, vel=None, acc=None):
        """현재 자세를 유지한 채 base Z 방향으로만 dz[mm] 이동.

        회전 성분 0  -> 파지 자세 유지
        ref=DR_BASE  -> 그리퍼가 기울어져 있어도 지면 수직 방향
        양수 상승, 음수 하강.

        속도는 호출부가 명시한다(파지/배치가 서로 다른 값을 쓰므로).
        """
        vel = cfg.VEL_X_SLOW if vel is None else vel
        acc = cfg.ACC_X_SLOW if acc is None else acc

        movel(posx(0.0, 0.0, dz, 0.0, 0.0, 0.0),
              vel=vel, acc=acc, ref=DR_BASE, mod=DR_MV_MOD_REL)

    # ================= Step 1: 접시 파지 =================
    def pick_plate(self):
        """APPROACH -> PICK -> 파지 -> 수직 RETREAT

        이 배포판의 movej 는 posx 를 받지 않으므로(Invalid type : pos)
        posx 로 이동할 때는 movel 을 쓴다.
        """
        self._log.info("Pick plate: start")

        self.gripper.open()

        # 파지점 위쪽으로 접근
        approach = self._offset_z(PLATE_PICK, cfg.PICK_APPROACH_HEIGHT)
        movel(approach, vel=cfg.VEL_X, acc=cfg.ACC_X, mod=DR_MV_MOD_ABS)

        # 파지점으로 저속 하강
        movel(PLATE_PICK, vel=cfg.PICK_APPROACH_VEL, acc=cfg.ACC_X_SLOW,
              mod=DR_MV_MOD_ABS)

        self.gripper.close()
        self._log.info("Pick plate: grasped")

        # 접시를 들고 수직 상승 (PICK 전용 높이/속도)
        self.lift_vertical(cfg.PICK_RETREAT_HEIGHT, vel=cfg.PICK_RETREAT_VEL)

        cur, sol = get_current_posx()
        self._log.info(f"Pick plate: done, pos={[round(v, 1) for v in cur]}, "
                       f"sol={sol}")

    # ================= Step 2: 무게 확인 =================
    def check_plate_weight(self):
        """접시 무게 측정.

        TODO: 측정 방식 미확정 (로봇 Force/Torque, 외부 Load Cell,
              외부 모듈에서 결과만 수신). 방식이 정해져도 이 함수만
              교체하면 되도록 분리해 둔다.
        """
        self._log.info("Check weight: (not implemented, assume normal)")
        return 0.0

    def is_abnormal_weight(self, weight):
        """TODO: EMPTY_PLATE_WEIGHT / WEIGHT_THRESHOLD 확정 후 구현"""
        return False

    def request_content_disposal(self):
        """내용물 제거 요청.

        TODO: 다른 팀원 담당 기능과 연동. 현재는 음식물이 이미
              제거된 것으로 가정하고 통과한다.
        """
        self._log.info("Content disposal: skipped (assumed already empty)")

    # ================= Step 3: 세척 =================
    def move_to_wash_start(self, via_j, start_j, label=""):
        """경유점을 거쳐 세척 시작점으로 이동.

        둘 다 관절값(posj)으로 이동해 solution space 불일치를 막는다.
        """
        self._log.info(f"Move to wash start: {label}")

        self._log.info("  -> via point")
        movej(via_j, vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info("  -> wash start")
        movej(start_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

    def leave_wash_area(self, via_j, label=""):
        """세척 면에서 경유점으로 이탈."""
        self._log.info(f"Leave wash area: {label}")
        movej(via_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

    def _force_on(self, coord):
        """유저 좌표계 Z(수세미 방향)로 일정 힘을 유지하도록 순응 제어 시작.

        힘제어 함수(task_compliance_ctrl / set_stiffnessx /
        set_desired_force)는 ref 인자를 받지 않고 전역 기준 좌표계를
        따르므로, set_ref_coord 로 먼저 좌표계를 지정한다.
        힘 부호는 양수 (+Z 가 수세미 방향).
        """
        ret = set_ref_coord(coord)
        self._log.info(f"set_ref_coord({coord}) -> {ret}")

        task_compliance_ctrl()
        set_stiffnessx(cfg.WASH_STIFFNESS, time=0.0)
        set_desired_force([0.0, 0.0, cfg.TARGET_FORCE, 0.0, 0.0, 0.0],
                          [0, 0, 1, 0, 0, 0],
                          time=0.0, mod=DR_FC_MOD_ABS)
        self._log.info(f"Force control ON ({cfg.TARGET_FORCE}N, coord={coord})")

    def _force_off(self):
        """힘/순응 제어 해제 후 기준 좌표계를 base 로 복원."""
        try:
            release_force(time=0.0)
            release_compliance_ctrl()
            self._log.info("Force control OFF")
        except Exception as e:
            self._log.warn(f"Force release failed: {e}")
        finally:
            set_ref_coord(DR_BASE)

    def wash_arcs(self, coord, radii=None, passes=None,
                  start_angle=0.0, z_offset=0.0):
        """현재 위치를 중심으로 하는 동심 반원들을 movec 으로 왕복 세척.

        현재 위치(= 좌표계 원점, 접시 중심)를 중심으로 각 반지름마다
        start_angle -> +90 -> +180 반원을 passes 회 왕복한 뒤 다음
        반지름으로 넘어간다.

        세 점의 자세를 현재 자세로 동일하게 지정하므로 파지 자세가
        유지되고, Z 를 고정하므로 좌표계 XY 평면(접시 면)에서만 움직인다.
        """
        radii = cfg.WASH_RADII if radii is None else radii
        passes = cfg.WASH_PASSES if passes is None else passes

        center, _ = get_current_posx(ref=coord)
        cx, cy, cz = center[0], center[1], center[2] + z_offset
        rx, ry, rz = center[3], center[4], center[5]

        a0 = math.radians(start_angle)
        a90 = math.radians(start_angle + 90.0)
        a180 = math.radians(start_angle + 180.0)

        self._log.info(f"Wash arcs (coord={coord}): radii={radii}, "
                       f"passes={passes}, start_angle={start_angle}")

        if cfg.USE_FORCE_CONTROL:
            self._force_on(coord)

        try:
            for r in radii:
                self._log.info(f"  radius {r}mm")

                p0 = posx(cx + r * math.cos(a0), cy + r * math.sin(a0),
                          cz, rx, ry, rz)
                mid = posx(cx + r * math.cos(a90), cy + r * math.sin(a90),
                           cz, rx, ry, rz)
                p180 = posx(cx + r * math.cos(a180), cy + r * math.sin(a180),
                            cz, rx, ry, rz)

                # 호 시작점으로 이동
                movel(p0, vel=cfg.WASH_VEL, acc=cfg.WASH_ACC,
                      ref=coord, mod=DR_MV_MOD_ABS)

                # 반원 왕복 (이전 호의 끝점이 다음 호의 시작점)
                for i in range(passes):
                    end = p180 if i % 2 == 0 else p0
                    movec(mid, end, vel=cfg.WASH_VEL, acc=cfg.WASH_ACC,
                          ref=coord, mod=DR_MV_MOD_ABS)
                    self._log.info(f"    pass {i + 1}/{passes}")
        finally:
            if cfg.USE_FORCE_CONTROL:
                self._force_off()

        # 중심으로 복귀
        movel(center, vel=cfg.WASH_VEL, acc=cfg.WASH_ACC,
              ref=coord, mod=DR_MV_MOD_ABS)
        self._log.info("Wash arcs done")

    def wash_face(self, coord, via_j, start_j, start_angle, label=""):
        """경유점 경유 -> 세척 시작점 -> 동심 반원 세척 -> 경유점 이탈"""
        self._log.info(f"--- Wash face: {label} (coord={coord}) ---")

        self.move_to_wash_start(via_j, start_j, label)
        self.wash_arcs(coord, start_angle=start_angle)
        self.leave_wash_area(via_j, label)

        self._log.info(f"--- Wash face: {label} done ---")

    def wash_plate(self):
        """양면 세척: dish1(앞면) -> dish2(뒷면)

        주의: dish1 계열은 sol 2, dish2 는 sol 7 이라 두 면 사이를
              이동할 때 J1 이 크게 회전한다. 경로 확인 필요.
        """
        self._log.info("===== Wash plate: start =====")

        self.wash_face(cfg.DISH1_COORD, DISH1_VIA_J, DISH1_WASH_START_J,
                       cfg.DISH1_START_ANGLE, "dish1 (front)")
        self.wash_face(cfg.DISH2_COORD, DISH2_VIA_J, DISH2_WASH_START_J,
                       cfg.DISH2_START_ANGLE, "dish2 (back)")

        self._log.info("===== Wash plate: done =====")

    # ================= Step 4: 접시 배치 =================
    def place_plate(self, via_j=None):
        """경유점 -> 접근 자세 -> 놓는 지점 -> release -> 접근 자세 복귀

        위아래 이동을 movel 로 하면 특이점에 걸리므로, 두 지점을 모두
        관절값(posj)으로 티칭해 movej 로만 이동한다.
        """
        self._log.info("Place plate: start")

        # 1) 경유점으로 이동
        via_j = DISH2_VIA_J if via_j is None else via_j
        self._log.info("  [1] move to via point")
        movej(via_j, vel=cfg.VEL_J, acc=cfg.ACC_J)

        # 2) 배치 접근 자세로 이동
        self._log.info("  [2] place approach pose")
        movej(PLATE_PLACE_APPROACH_J, vel=cfg.VEL_J, acc=cfg.ACC_J)

        # 3) 놓는 지점으로 하강 (movej — 특이점 회피)
        self._log.info("  [3] descend to place point")
        movej(PLATE_PLACE_J, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

        # 4) 놓기
        self.gripper.open()
        self._log.info("  [4] released")

        # 5) 접근 자세로 복귀 (movej)
        self._log.info("  [5] back to approach pose")
        movej(PLATE_PLACE_APPROACH_J, vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info("Place plate: done")

    # ================= 전체 시나리오 =================
    def run_plate_task(self):
        """Pick -> Weight -> (Disposal) -> Wash -> Place -> Home"""
        self._log.info("########## Plate task: START ##########")

        self.move_home()
        self.pick_plate()

        weight = self.check_plate_weight()
        if self.is_abnormal_weight(weight):
            self.request_content_disposal()

        self.wash_plate()
        self.place_plate()
        self.move_home()

        self._log.info("########## Plate task: COMPLETE ##########")

    # ================= 테스트 유틸 =================
    def shake_in_coord(self, coord, amp=20.0, times=4, axis="x"):
        """좌표계 축 방향 왕복. 좌표계 방향 확인용."""
        self._log.info(f"Shake in coord {coord}: {axis}axis +-{amp}mm x{times}")

        start, _ = get_current_posx(ref=coord)
        self._log.info(f"  start (coord {coord}) = "
                       f"{[round(v, 1) for v in start]}")

        dx = amp if axis == "x" else 0.0
        dy = amp if axis == "y" else 0.0
        dz = amp if axis == "z" else 0.0

        for i in range(times):
            sign = 1.0 if i % 2 == 0 else -1.0
            movel(posx(dx * sign * 2, dy * sign * 2, dz * sign * 2,
                       0.0, 0.0, 0.0),
                  vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
                  ref=coord, mod=DR_MV_MOD_REL)
            self._log.info(f"  step {i + 1}/{times}")

        movel(start, vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
              ref=coord, mod=DR_MV_MOD_ABS)
        self._log.info("Shake done, returned to start")

    # ================= 유틸 =================
    @staticmethod
    def _offset_z(pos, dz):
        """주어진 posx 에서 base Z 만 dz 만큼 이동한 새 posx 반환."""
        return posx(pos[0], pos[1], pos[2] + dz, pos[3], pos[4], pos[5])