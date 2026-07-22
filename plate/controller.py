"""접시 처리 컨트롤러. ROS Node가 아닌 일반 Python 클래스.

import 만으로는 로봇이 움직이지 않으며, 각 메서드를 호출할 때만 동작한다.
경로 계산(path_generator)과 로봇 제어(이 파일)는 분리되어 있다.
"""

from DSR_ROBOT2 import (
    set_tool, set_tcp, get_tool, get_tcp, get_current_posx,
    movej, movel,
    set_velj, set_accj, set_velx, set_accx,
    DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
)
from DR_common2 import posx

from . import config as cfg
from .gripper import Gripper
from .path_generator import generate_semicircle_path
from .waypoints import (
    HOME, PLATE_PICK, PLATE_PLACE,
    DISH1_VIA_J, DISH1_WASH_START_J,
    DISH2_VIA_J, DISH2_WASH_START_J,
)


class PlateController:
    """접시 파지 → 무게확인 → 세척 → 배치 담당."""

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

    # ================= 기본 이동 =================
    def move_home(self):
        self._log.info("Move to HOME")
        movej(HOME, vel=cfg.VEL_J, acc=cfg.ACC_J)

    def lift_vertical(self, dz, slow=True):
        """현재 자세를 유지한 채 base Z 방향으로만 dz[mm] 이동.

        회전 성분 0 → 파지 자세(접시 기울기) 유지.
        ref=DR_BASE  → 그리퍼가 기울어져 있어도 지면 수직 방향.
        양수 상승, 음수 하강.
        """
        vel = cfg.VEL_X_SLOW if slow else cfg.VEL_X
        acc = cfg.ACC_X_SLOW if slow else cfg.ACC_X

        movel(posx(0.0, 0.0, dz, 0.0, 0.0, 0.0),
              vel=vel, acc=acc,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

    # ================= 좌표계 확인용 테스트 =================
    def shake_in_coord(self, coord, amp=20.0, times=4, axis="x"):
        """현재 위치에서 사용자 좌표계 축을 따라 왕복 이동한다.

        좌표계 방향과 ref 전달이 제대로 되는지 확인하는 용도.
        현재 위치 기준 상대 이동이므로 자세는 그대로 유지된다.
        """
        self._log.info(f"Shake in coord {coord}: {axis}축 ±{amp}mm x{times}")

        # 현재 위치를 좌표계 기준으로 읽어 시작점 기록
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
            self._log.info(f"  step {i+1}/{times}")

        # 시작 위치로 복귀
        movel(start, vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
              ref=coord, mod=DR_MV_MOD_ABS)
        self._log.info("Shake done, returned to start")

    # ================= Step 1: 접시 파지 =================
    def pick_plate(self):
        """HOME → APPROACH → PICK → 파지 → 수직 RETREAT"""
        self._log.info("Pick plate: start")

        self.gripper.open()

        # 접근 지점까지는 movej (자세 변화가 크므로 직선 강제 회피)
        approach = self._offset_z(PLATE_PICK, cfg.PICK_APPROACH_HEIGHT)
        movel(approach, vel=cfg.VEL_X, acc=cfg.ACC_X, mod=DR_MV_MOD_ABS)
        

        # 파지 위치로 저속 직선 하강
        movel(PLATE_PICK, vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
              mod=DR_MV_MOD_ABS)

        self.gripper.close()
        self._log.info("Pick plate: grasped")

        # 자세 고정, base Z 수직 상승
        self.lift_vertical(cfg.PICK_RETREAT_HEIGHT)

        cur, sol = get_current_posx()
        self._log.info(f"Pick plate: done, pos={[round(v, 1) for v in cur]}, sol={sol}")

    # ================= Step 2: 무게 확인 =================
    def check_plate_weight(self):
        """접시 무게 측정.

        TODO: 측정 방식 미확정 (로봇 Force/Torque, 외부 Load Cell,
              또는 외부 모듈에서 정상/비정상 결과만 수신).
              방식이 정해져도 이 함수만 교체하면 되도록 분리해 둔다.
        """
        self._log.info("Check weight: (not implemented, assume normal)")
        return 0.0

    def is_abnormal_weight(self, weight):
        """무게 이상 여부 판정.

        TODO: EMPTY_PLATE_WEIGHT / WEIGHT_THRESHOLD 확정 후 구현
        """
        return False

    def request_content_disposal(self):
        """내용물 제거 요청.

        TODO: 다른 팀원 담당 기능과 연동 (ROS2 Service/Action 또는 직접 호출).
              현재는 음식물이 이미 제거된 것으로 가정하고 통과한다.
        """
        self._log.info("Content disposal: skipped (assumed already empty)")

    # ================= Step 3: 세척 =================

    def move_to_wash_start(self, via_j, start_j, label=""):
        """경유점을 거쳐 세척 시작점으로 이동한다.

        경유점과 시작점 모두 관절값(posj)으로 이동해 solution space
        불일치로 팔이 반대로 꺾이는 것을 막는다.
        """
        self._log.info(f"Move to wash start: {label}")

        self._log.info("  -> via point")
        movej(via_j, vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info("  -> wash start")
        movej(start_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)

        self._log.info(f"Move to wash start: {label} done")

    def leave_wash_area(self, via_j, label=""):
        """세척 면에서 경유점으로 이탈."""
        self._log.info(f"Leave wash area: {label}")
        movej(via_j, vel=cfg.VEL_J_SLOW, acc=cfg.ACC_J_SLOW)
            

    def wash_face(self, coord, via_j, start_j, label=""):
        """경유점 경유 → 세척 시작점 → 와이퍼 세척 → 경유점 이탈"""
        self._log.info(f"--- Wash face: {label} (coord={coord}) ---")

        self.move_to_wash_start(via_j, start_j, label)

        path = generate_semicircle_path(
            cfg.PLATE_RADIUS, cfg.BRUSH_EFFECTIVE_WIDTH,
            cfg.PATH_OVERLAP, cfg.EDGE_MARGIN, cfg.ANGLE_STEP,
            passes=cfg.WASH_PASSES,
        )
        total = sum(len(s) for s in path)
        self._log.info(f"  {len(path)} radii, {total} waypoints")

        self.execute_cleaning_path(coord, path)

        self.leave_wash_area(via_j, label)
        self._log.info(f"--- Wash face: {label} done ---")

    def wash_plate(self):
        """양면 세척: dish1(앞면) → dish2(뒷면)

        dish1 이탈 시 DISH1_VIA 로 돌아오고, wash_face 가 다시
        DISH2_VIA 를 거치므로 확인된 VIA1 -> VIA2 경로를 탄다.
        """
        self._log.info("===== Wash plate: start =====")

        self.wash_face(cfg.DISH1_COORD, DISH1_VIA_J, DISH1_WASH_START_J,
                       "dish1 (front)")
        self.wash_face(cfg.DISH2_COORD, DISH2_VIA_J, DISH2_WASH_START_J,
                       "dish2 (back)")

        self._log.info("===== Wash plate: done =====")

    def execute_cleaning_path(self, coord, path):
        """생성된 경로를 지정 좌표계에서 실행한다.

        path 는 반지름 단위로 묶인 waypoint 리스트이며, 각 그룹 안에서
        와이퍼처럼 왕복한다. 그룹이 바뀔 때는 직전에 멈춘 쪽에서
        이어지므로 별도의 복귀 이동이 없다.
        """
        for i, stroke in enumerate(path):
            self._log.info(f"  radius {i + 1}/{len(path)} ({len(stroke)} pts)")
            for (x, y) in stroke:
                self._move_in_coord(coord, x, y, cfg.WASH_PLANE_Z,
                                    vel=cfg.WASH_VEL)

    def _move_in_coord(self, coord, x, y, z, vel=None, acc=None,
                       use_movej=False):
        """사용자 좌표계 기준 (x, y, z) 로 이동한다.

        자세(rx, ry, rz)를 0 으로 두므로 엔드이펙터가 해당 좌표계의
        축 방향에 정렬된다.
        """
        vel = vel if vel is not None else cfg.WASH_VEL
        acc = acc if acc is not None else cfg.WASH_ACC

        target = posx(x, y, z, 0.0, 0.0, 0.0)
        if use_movej:
            movej(target, vel=cfg.VEL_J, acc=cfg.ACC_J, ref=coord)
        else:
            movel(target, vel=vel, acc=acc, ref=coord, mod=DR_MV_MOD_ABS)

    # ================= Step 4: 접시 배치 =================
    def place_plate(self):
        """PLACE_APPROACH → PLACE → release → 수직 RETREAT"""
        self._log.info("Place plate: start")

        # 배치 지점 위로 접근
        approach = self._offset_z(PLATE_PLACE, cfg.PLACE_APPROACH_HEIGHT)
        movel(approach, vel=cfg.VEL_X, acc=cfg.ACC_X, mod=DR_MV_MOD_ABS)

        # 저속 하강
        movel(PLATE_PLACE, vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
              mod=DR_MV_MOD_ABS)

        # 놓기
        self.gripper.open()
        self._log.info("Place plate: released")

        # 자세 고정, 수직 이탈
        self.lift_vertical(cfg.PLACE_RETREAT_HEIGHT)

        self._log.info("Place plate: done")

    # ================= 전체 시나리오 =================
    def run_plate_task(self):
        """접시 처리 전체: Pick → Weight → (Disposal) → Wash → Place → Home"""
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

    # ================= 유틸 =================
    @staticmethod
    def _offset_z(pos, dz):
        """주어진 posx 에서 base Z 만 dz 만큼 이동한 새 posx 반환."""
        return posx(pos[0], pos[1], pos[2] + dz, pos[3], pos[4], pos[5])