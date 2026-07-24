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
    set_ref_coord, wait,
    amovel, check_motion, mwait, get_tool_force,
    task_compliance_ctrl, set_stiffnessx, set_desired_force,
    release_force, release_compliance_ctrl, check_force_condition,
    DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z,
    DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_QSTOP,
)
from DR_common2 import posx
from dsr_msgs2.srv import MoveStop

from . import config as cfg
from .gripper import Gripper
from . import food_check
from .waypoints import (
    HOME, PLATE_PICK, PLATE_PLACE, PLATE_PLACE_J,
    PLATE_PLACE_APPROACH, PLATE_PLACE_APPROACH_J,
    PLACE_VIA, PLACE_VIA_J,
    WASH1_APPROACH_J, WASH1_START_J,
    WASH2_APPROACH_J, WASH2_START_J,
    ROTATE_RELEASE_SAFE_J, ROTATE_RELEASE_J,
    ROTATE_GRAB_SAFE_J, ROTATE_GRAB_J,
)


class PlateController:
    """접시 파지 -> 무게확인 -> 세척 -> 배치 담당."""

    def __init__(self, node):
        self.node = node
        self.gripper = Gripper(node)
        self._log = node.get_logger()

        # 이 배포판에는 stop() 함수가 없어 motion/move_stop 서비스를
        # 직접 호출한다(기어 프로젝트에서 검증된 방식).
        self._stop_cli = node.create_client(MoveStop, "motion/move_stop")

    def stop_motion(self, st_mode=DR_QSTOP):
        """진행 중인 모션을 정지시킨다."""
        req = MoveStop.Request()
        req.stop_mode = st_mode
        self._stop_cli.call_async(req)

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

    # ================= Step 2: 음식물 확인 / 배출 =================
    def check_plate_weight(self):
        """툴 Fz 를 평균 내어 음식물 유무를 판정한다. -> (food_exists, fz)"""
        return food_check.check_food(self._log)

    def is_abnormal_weight(self, result):
        """check_plate_weight 결과에서 음식물 유무만 꺼낸다."""
        food_exists, _fz = result
        return food_exists

    def request_content_disposal(self):
        """음식물 배출(털기) 동작. 팀원 코드 기반."""
        food_check.dispose_food(self._log)

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

    # 툴 좌표계 축 인덱스 -> DR_AXIS_* 상수
    _AXIS_CONST = (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z)

    def _force_on(self, force_sign, magnitude=None):
        """순응 제어 시작 + 힘 지령 (툴 좌표계 기준).

        magnitude 를 주지 않으면 접근용 강한 힘(APPROACH_FORCE)을 건다.
        접촉 후에는 _set_force 로 약한 힘(PRESS_FORCE)으로 낮춘다.

        힘제어 함수(task_compliance_ctrl / set_stiffnessx /
        set_desired_force)는 ref 인자를 받지 않고 전역 기준 좌표계를
        따르므로 set_ref_coord(DR_TOOL) 로 먼저 지정한다.

        실측상 툴 X축이 접시 법선이며, 세척 위치에 따라 접시가 반대로
        향하므로 force_sign 으로 방향을 지정한다(위치1 -1, 위치2 +1).
        반원 경로는 유저 좌표계(dish1) 기준 그대로다.
        """
        magnitude = cfg.APPROACH_FORCE if magnitude is None else magnitude
        ret = set_ref_coord(DR_TOOL)

        fd = [0.0] * 6
        dr = [0] * 6
        fd[cfg.FORCE_AXIS] = force_sign * magnitude
        dr[cfg.FORCE_AXIS] = 1

        r1 = task_compliance_ctrl()
        r2 = set_stiffnessx(cfg.WASH_STIFFNESS, time=0.0)
        r3 = set_desired_force(fd, dr, time=0.0, mod=DR_FC_MOD_ABS)

        self._log.info(f"ref_coord(TOOL)={ret}, compliance={r1}, "
                       f"stiffness={r2}, force={r3} "
                       f"(force {fd[cfg.FORCE_AXIS]}N on "
                       f"tool axis {cfg.FORCE_AXIS})")

    def _wait_contact(self):
        """접촉이 감지될 때까지 기다린다. 힘 지령은 그대로 유지한다.

        release_force() 를 부르지 않으므로 순응 제어와 힘 지령이
        끊기지 않고 이어진다.
        """
        deadline = int(cfg.CONTACT_TIMEOUT / cfg.CONTACT_POLL)
        for i in range(deadline):
            if cfg.FORCE_LOG and i % cfg.FORCE_LOG_EVERY == 0:
                self.log_tool_force("    [contact] ")
            if check_force_condition(self._AXIS_CONST[cfg.FORCE_AXIS],
                                     min=cfg.CONTACT_FORCE,
                                     ref=DR_TOOL) == 0:
                self._log.info("  contact detected")
                return True
            wait(cfg.CONTACT_POLL)

        self._log.warn(f"  no contact within {cfg.CONTACT_TIMEOUT}s")
        return False

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

    def log_tool_force(self, prefix=""):
        """툴 좌표계 기준 현재 힘/토크를 로그로 남긴다. -> 법선 축 힘

        check_force_condition 은 조건 통과 여부만 알려주므로,
        임계값을 실측으로 정하려면 실제 값을 봐야 한다.
        """
        f = get_tool_force(DR_TOOL)
        normal = f[cfg.FORCE_AXIS]
        self._log.info(
            f"{prefix}F=[{f[0]:6.2f} {f[1]:6.2f} {f[2]:6.2f}] "
            f"T=[{f[3]:6.2f} {f[4]:6.2f} {f[5]:6.2f}]  "
            f"normal(axis{cfg.FORCE_AXIS})={normal:6.2f}N")
        return normal

    def _set_force(self, force_sign, magnitude):
        """툴 좌표계 법선 축으로 지정한 크기의 힘을 건다."""
        fd = [0.0] * 6
        dr = [0] * 6
        fd[cfg.FORCE_AXIS] = force_sign * magnitude
        dr[cfg.FORCE_AXIS] = 1
        return set_desired_force(fd, dr, time=0.0, mod=DR_FC_MOD_ABS)

    def wash_arcs(self, coord, radii=None, passes=None,
                  start_angle=0.0, z_offset=0.0, force_sign=None):
        """동심 반원 세척 (뒤집기 전 볼록면용).

        현재 위치(= 유저 좌표계 원점, 접시 중심)를 중심으로 각 반지름마다
        start_angle -> +90 -> +180 반원을 movec 으로 passes 회 왕복한다.

        이동은 유저 좌표계(coord), 힘/순응은 툴 좌표계 기준이다.
        세 점의 자세를 현재 자세로 동일하게 지정하므로 파지 자세가
        유지되고, Z 를 고정하므로 접시 면에서만 움직인다.
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
            sign = cfg.WASH1_FORCE_SIGN if force_sign is None else force_sign
            self._force_on(sign)            # 접근용 강한 힘
            self._wait_contact()            # 접촉 확인
            release_force(time=0.0)         # 힘만 해제 (순응은 유지)

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

    def wash_sweeps(self, coord, force_sign, y_offsets=None,
                    back_j=None, z_dists=None):
        """직선 쓸기 세척 (뒤집은 뒤 오목면용).

        이동/힘 모두 **툴 좌표계** 기준이다(원호 세척만 유저 좌표계).

        접근할 때만 힘을 걸어 접촉시키고, 접촉되면 힘 지령을 해제한다.
        접시가 곡면이라 쓸다 보면 자연히 눌리므로 힘을 계속 줄 필요가
        없고, 계속 주면 관절 부하가 커져 안전모드에 걸린다.
        순응 제어는 유지해 곡면을 따라가게 한다.

        한 오프셋에서 SWEEP_Z_DIST 만큼 나갔다가, 수세미 반대 방향으로
        살짝 빼서 접촉을 끊고 세척 시작 자세로 복귀한다. 이를 Y 오프셋
        마다 반복해 면을 채운다.

        coord 인자는 호출부 호환을 위해 받지만 사용하지 않는다.
        """
        y_offsets = cfg.SWEEP_Y_OFFSETS if y_offsets is None else y_offsets
        back_dx = -force_sign * cfg.SWEEP_BACKOFF

        # 오프셋별 쓸기 거리. 길이가 안 맞으면 기본값으로 채운다.
        if z_dists is None:
            z_dists = cfg.SWEEP_Z_DISTS
        if len(z_dists) != len(y_offsets):
            z_dists = [cfg.SWEEP_Z_DIST] * len(y_offsets)

        self._log.info(f"Wash sweeps (TOOL frame): y_offsets={y_offsets}, "
                       f"dists={z_dists}, sign={cfg.SWEEP_Z_SIGN}")

        def move_tool(dy=0.0, dz=0.0, dx=0.0, fast=False):
            """툴 좌표계 상대 이동(동기).

            fast=True 는 접촉이 없거나 약한 구간(오프셋 이동, 후퇴)에
            쓰는 빠른 속도다. 실제 쓸어내는 구간은 기본 속도를 쓴다.
            """
            vel = cfg.SWEEP_MOVE_VEL if fast else cfg.SWEEP_VEL
            acc = cfg.SWEEP_MOVE_ACC if fast else cfg.SWEEP_ACC
            movel(posx(dx, dy, dz, 0.0, 0.0, 0.0),
                  vel=vel, acc=acc,
                  ref=DR_TOOL, mod=DR_MV_MOD_REL)

        try:
            for i, dy in enumerate(y_offsets):
                sweep_dz = cfg.SWEEP_Z_SIGN * z_dists[i]
                self._log.info(f"  sweep {i + 1}/{len(y_offsets)} "
                               f"(tool y={dy}, dist={z_dists[i]}mm)")

                # 세척 시작 자세로 복귀 (매 스트로크 같은 지점에서 출발)
                if back_j is not None and i > 0:
                    movej(back_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

                # 접근용 힘으로 접촉시킨 뒤 힘만 해제 (순응은 유지)
                if cfg.USE_FORCE_CONTROL:
                    self._force_on(force_sign)
                    self._wait_contact()
                    release_force(time=0.0)
                    self._log.info("  force released (compliance only)")

                # Y 오프셋으로 이동 (접촉 유지된 채 옆으로만 이동)
                move_tool(dy=dy, fast=True)

                # 바깥으로 쓸기
                if cfg.FORCE_LOG:
                    self.log_tool_force("    [out]  ")
                move_tool(dz=sweep_dz)
                if cfg.FORCE_LOG:
                    self.log_tool_force("    [end]  ")

                # 접시 반대 방향으로 빼서 접촉을 끊는다
                move_tool(dx=back_dx, fast=True)

                # 힘/순응 해제 후 다음 스트로크 준비
                if cfg.USE_FORCE_CONTROL:
                    self._force_off()
        finally:
            if cfg.USE_FORCE_CONTROL:
                self._force_off()

        self._log.info("Wash sweeps done")

    def wash_face(self, approach_j, start_j, start_angle,
                  force_sign, coord=None, label="", skip_approach=False,
                  sweep=False):
        """접근점 경유 -> 세척 시작점 -> 동심 반원 세척 -> 접근점 이탈

        좌표계는 dish1(101) 하나만 사용한다. 세척 위치가 달라도 좌표계
        원점(접시 중심 = 수세미 접촉점)은 같으므로 반원 경로는 동일하다.
        """
        coord = cfg.DISH1_COORD if coord is None else coord
        self._log.info(f"--- Wash: {label} (coord={coord}) ---")

        if skip_approach:
            self._log.info("  -> approach (skipped, already in position)")
        else:
            self._log.info("  -> approach")
            movej(approach_j, vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info("  -> wash start")
        movej(start_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

        if sweep:
            self.wash_sweeps(coord, force_sign, back_j=start_j)
        else:
            self.wash_arcs(coord, start_angle=start_angle,
                           force_sign=force_sign)

        self._log.info("  -> leave")
        movej(approach_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

        self._log.info(f"--- Wash: {label} done ---")

    def wash_plate(self, skip_first_approach=False):
        """세척 위치 1(앞면) -> 세척 위치 2(뒷면) 순으로 세척한다.

        skip_first_approach=True 면 위치 1 의 접근 이동을 건너뛴다.
        (food_check 의 5단계가 이미 접근점으로 옮겨놓은 경우)

        세척 방식은 면에 따라 다르다.
          위치 1 (앞면, 볼록) : 동심 반원 (wash_arcs)
          위치 2 (뒷면, 오목) : 직선 쓸기 (wash_sweeps)
        오목면에서 원호를 그리면 곡면이 수세미를 밀어내 큰 힘이 걸리므로
        뒷면만 직선으로 쓸어낸다.
        """
        self._log.info("===== Wash plate: start =====")

        self.wash_face(WASH1_APPROACH_J, WASH1_START_J,
                       cfg.WASH1_START_ANGLE, cfg.WASH1_FORCE_SIGN,
                       label="wash pos 1 (front, arc)",
                       skip_approach=skip_first_approach,
                       sweep=False)
        self.wash_face(WASH2_APPROACH_J, WASH2_START_J,
                       cfg.WASH2_START_ANGLE, cfg.WASH2_FORCE_SIGN,
                       label="wash pos 2 (back, sweep)",
                       sweep=True)

        self._log.info("===== Wash plate: done =====")

    # ================= 접시 회전 (재파지) =================
    def rotate_plate_once(self):
        """접시 파지 위치를 rim 을 따라 한 칸(약 60도) 옮긴다.

        릴리즈 안전 -> 릴리즈 구역 -> open -> 릴리즈 안전
          -> 그랩 안전 -> 그랩 구역 -> close -> 그랩 안전
        """
        vel, acc = cfg.ROTATE_VEL_J, cfg.ROTATE_ACC_J

        # 접시 내려놓기
        self._log.info("  release side")
        movej(ROTATE_RELEASE_SAFE_J, vel=vel, acc=acc)
        movej(ROTATE_RELEASE_J, vel=vel, acc=acc)
        self.gripper.open()
        movej(ROTATE_RELEASE_SAFE_J, vel=vel, acc=acc)

        # 옮긴 위치에서 다시 잡기
        self._log.info("  grab side")
        movej(ROTATE_GRAB_SAFE_J, vel=vel, acc=acc)
        movej(ROTATE_GRAB_J, vel=vel, acc=acc)
        self.gripper.close()
        movej(ROTATE_GRAB_SAFE_J, vel=vel, acc=acc)

    def rotate_plate(self, steps=None):
        """rotate_plate_once 를 steps 회 반복해 접시를 돌린다.

        기본값 3회 = 반바퀴.
        """
        steps = cfg.ROTATE_STEPS if steps is None else steps
        self._log.info(f"Rotate plate: {steps} step(s)")

        for i in range(steps):
            self._log.info(f"[rotate {i + 1}/{steps}]")
            self.rotate_plate_once()

        self._log.info("Rotate plate: done")

    # ================= Step 4: 접시 배치 =================
    def place_plate(self, via_j=None):
        """경유점 -> 접근 자세 -> 놓는 지점 -> release -> 접근 자세 복귀

        위아래 이동을 movel 로 하면 특이점에 걸리므로, 두 지점을 모두
        관절값(posj)으로 티칭해 movej 로만 이동한다.
        """
        self._log.info("Place plate: start")

        # 1) 경유점으로 이동 (배치 전용 안전 자세)
        via_j = PLACE_VIA_J if via_j is None else via_j
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
    def run_plate_task(self, rotate=True):
        """Pick -> 음식물 확인/배출 -> Wash -> (Rotate -> Wash) -> Place -> Home

        음식물 확인/배출과 그 뒤 '다음 작업 위치' 이동은 food_check 모듈이
        담당하며, 그 이동이 세척 위치 1 접근을 대신한다.
        rotate=True 면 세척 후 파지를 옮겨(rotate_plate) 그리퍼가 가렸던
        영역을 한 번 더 세척한다.
        """
        self._log.info("########## Plate task: START ##########")

        self.move_home()
        self.pick_plate()

        # 음식물 확인 -> (있으면) 배출 -> WASH1_APPROACH_J 로 이동
        # food_check 가 이미 접근점으로 옮겨놓으므로 첫 접근은 생략한다.
        food_check.run_food_check(self._log)

        self.wash_plate(skip_first_approach=True)

        if rotate:
            # 파지를 옮겨 그리퍼가 가렸던 영역을 한 번 더 세척
            self.rotate_plate()
            self.wash_plate()

        self.place_plate()
        self.move_home()

        self._log.info("########## Plate task: COMPLETE ##########")

    # ================= 테스트 유틸 =================
    def test_stiffness(self, force_sign, duration=20.0, interval=0.2):
        """강성이 실제로 걸리는지 확인한다(현재 자세 그대로).

        순응 제어만 켜고 로봇은 움직이지 않는다. 손으로 밀어보면서
        힘 축(FORCE_AXIS) 방향으로는 밀리고 나머지 방향으로는 안 밀리는지
        확인한다. 힘 값과 함께 base 기준 위치도 찍어서 실제로 밀렸는지 본다.
        """
        self._log.info(
            f"Test stiffness {duration}s: stiffness={cfg.WASH_STIFFNESS}, "
            f"force_axis={cfg.FORCE_AXIS}, sign={force_sign}")

        set_ref_coord(DR_TOOL)
        r1 = task_compliance_ctrl()
        r2 = set_stiffnessx(cfg.WASH_STIFFNESS, time=0.0)
        r3 = self._set_force(force_sign, cfg.APPROACH_FORCE)
        self._log.info(f"compliance={r1}, stiffness={r2}, force={r3}")

        start, _ = get_current_posx(ref=DR_BASE)
        try:
            for _ in range(int(duration / interval)):
                f = get_tool_force(DR_TOOL)
                cur, _ = get_current_posx(ref=DR_BASE)
                d = [cur[i] - start[i] for i in range(3)]
                self._log.info(
                    f"  F=[{f[0]:6.2f} {f[1]:6.2f} {f[2]:6.2f}]  "
                    f"moved(base)=[{d[0]:6.1f} {d[1]:6.1f} {d[2]:6.1f}]mm")
                wait(interval)
        finally:
            self._force_off()

        self._log.info("Test stiffness done")

    def monitor_force(self, duration=10.0, interval=0.2):
        """지정 시간 동안 툴 기준 힘을 계속 찍는다.

        임계값을 정하기 위한 진단용. 로봇을 세척 자세에 두고 손으로
        접시를 수세미에 붙였다 떼면서 값이 어떻게 변하는지 본다.
        """
        self._log.info(f"Monitor force for {duration}s "
                       f"(normal axis = {cfg.FORCE_AXIS})")
        n = int(duration / interval)
        vals = []
        for _ in range(n):
            vals.append(self.log_tool_force("  "))
            wait(interval)

        if vals:
            self._log.info(f"normal force: min={min(vals):.2f} "
                           f"max={max(vals):.2f} "
                           f"avg={sum(vals)/len(vals):.2f} N")
        return vals

    def probe_tool_axis(self, axis="x", amp=10.0):
        """툴 좌표계 축 방향으로 왕복. 어느 축이 접시 법선인지 확인용.

        get_current_posx 는 ref=DR_TOOL 을 받지 않으므로(Invalid value : ref(1))
        상대 이동(MOD_REL)만 사용한다.
        """
        idx = {"x": 0, "y": 1, "z": 2}[axis]
        d = [0.0] * 6
        d[idx] = amp

        self._log.info(f"Probe tool {axis}: +{amp}mm")
        movel(posx(*d), vel=10, acc=10, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        wait(1.0)

        d[idx] = -amp
        self._log.info(f"Probe tool {axis}: back")
        movel(posx(*d), vel=10, acc=10, ref=DR_TOOL, mod=DR_MV_MOD_REL)

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