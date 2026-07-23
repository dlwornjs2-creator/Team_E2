#!/usr/bin/env python3
# ==============================================================================

import sys
import rclpy
import DR_init
import bowl_config as cfg

# 1. ROS 2 초기화 및 노드 등록 (DSR_ROBOT2 임포트 전 필수 수행!)
rclpy.init()
node = rclpy.create_node(cfg.NODE_NAME, namespace=cfg.ROBOT_ID)

DR_init.__dsr__node = node
DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL

from DSR_ROBOT2 import *
import bowl_skills as skills

# ------------------------------------------------------------------------------
# 좌표 딕셔너리를 로봇 전용 객체(posx, posj)로 변환하는 Helper 함수
# ------------------------------------------------------------------------------
def get_coord(name):
    data = cfg.COORD_DATA[name]
    return posj(*data["val"]) if data["type"] == "posj" else posx(*data["val"])

# ------------------------------------------------------------------------------
# 3대 핵심 시퀀스 함수
# ------------------------------------------------------------------------------
def pick_bowl(target_pos, approach_pos):
    print("[Sequence 1] 그릇 파지 시작")
    movel(approach_pos, v=cfg.VEL_FAST, a=cfg.ACC_FAST)
    movel(target_pos, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)
    
    print(" -> 그릇 가장자리 파지 중...")
    skills.grasp()
    print("[Sequence 1] 그릇 파지 완료")

    movel(approach_pos, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)

def scrub_bowl_on_sponge(sponge_pos):
    print("[Sequence 2] 원통형 수세미 정밀 세척 시작 (외부 + 내부)")
    
    # [PART A: 외부/하단 세척]
    print(" -> [PART A] 그릇 외부 및 하단 세척 중...")
    offset_app = [0, 0, 80, 0, 0, 0]
    pos_sponge_app = posx(*trans(sponge_pos, offset_app, DR_BASE, DR_BASE))
    
    movel(pos_sponge_app, v=cfg.VEL_FAST, a=cfg.ACC_FAST)
    movel(sponge_pos, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)
    
    # 2배 순응도 적용 강성 및 힘 제어
    task_compliance_ctrl(cfg.STIFFNESS_OUTER)
    set_ref_coord(DR_TOOL)
    set_desired_force(fd=[0.0, 0.0, 15.0, 0.0, 0.0, 0.0], dir=[0, 0, 1, 0, 0, 0], time=0.5, mod=DR_FC_MOD_REL)
    wait(1.0)
    
    move_periodic(amp=[20.0, 20.0, 0.0, 0.0, 0.0, 0.0], period=[1.5, 1.5, 0.0, 0.0, 0.0, 0.0], repeat=4, ref=DR_TOOL)
    
    release_force()
    release_compliance_ctrl()
    movel(pos_sponge_app, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)
    wait(0.5)

    # [PART B: 내부 바닥/내벽 세척]
    print(" -> [PART B] 그릇 내부(바닥 및 내벽) 세척 진입...")
    offset_inside = [30, 0, 0, 0, 0, 0]
    pos_inside_app = posx(*trans(sponge_pos, offset_inside, DR_BASE, DR_BASE))
    movel(pos_inside_app, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)

    task_compliance_ctrl(cfg.STIFFNESS_INNER)
    set_ref_coord(DR_TOOL)
    set_desired_force(fd=[0.0, 0.0, 10.0, 0.0, 0.0, 0.0], dir=[0, 0, 1, 0, 0, 0], time=0.5, mod=DR_FC_MOD_REL)
    wait(0.5)

    print(" -> 그릇 안쪽 바닥 및 내벽 훑는 중...")
    move_periodic(amp=[15.0, 15.0, 0.0, 0.0, 0.0, 0.0], period=[1.2, 1.2, 0.0, 0.0, 0.0, 0.0], repeat=3, ref=DR_TOOL)

    release_force()
    release_compliance_ctrl()
    movel(pos_sponge_app, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)
    print("[Sequence 2] 외부 + 내부 세척 완벽 완료")

def place_bowl_on_rack(rack_pos, way_pos, home_pos):
    print("[Sequence 3] 거치대 배치 시작")
    movel(way_pos, v=cfg.VEL_SLOW, a=cfg.ACC_SLOW)
    movej(rack_pos, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
    
    print(" -> 거치대 바닥 방향으로 접촉 감지 하강 시작...")
    task_compliance_ctrl(cfg.STIFFNESS_PLACE)
    
    # 모듈화된 바닥 접촉 감지 스킬 호출
    skills.descend_until_contact(target_force=-15.0, max_ext_force=12.0)
    
    print(" -> 거치대에 그릇 내려놓는 중...")
    skills.release()
    
    release_force()
    release_compliance_ctrl()
    
    print(" -> 그리퍼 안전 수직 상승 중...")
    movel([0, 0, 80, 0, 0, 0], v=cfg.VEL_SLOW, a=cfg.ACC_SLOW, mod=DR_MV_MOD_REL)

    print(" -> 조인트 값 기준 홈 자세로 복귀 중...")
    movej(home_pos, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
    wait(3.0)
    print("[Sequence 3] 거치 완료 및 로봇 복귀 완료")

# ------------------------------------------------------------------------------
# 4. 메인 실행 함수
# ------------------------------------------------------------------------------
def main():
    print("=== [Start] ROS 2 기반 그릇 세척 모듈 시작 ===")
    try:
        # 좌표 로드
        pos_home = get_coord("HOME")
        pos_pick = get_coord("PICK_BOWL")
        pos_pick_up = get_coord("PICK_UP")
        pos_sponge = get_coord("SPONGE")
        pos_way = get_coord("WAY")
        pos_rack = get_coord("RACK")

        skills.release()
        print(" -> [초기화] 로봇을 홈 자세로 이동 후 대기합니다.")
        movej(pos_home, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
        wait(3.00)

        set_velx(cfg.VEL_FAST)
        set_accx(cfg.ACC_FAST)
        
        # 3대 핵심 시퀀스 가동
        pick_bowl(pos_pick, pos_pick_up)
        scrub_bowl_on_sponge(pos_sponge)
        place_bowl_on_rack(pos_rack, pos_way, pos_home)
        
        print("=== [Success] 모든 시퀀스가 성공적으로 완료되었습니다! ===")
        
    except Exception as e:
        print(f"[Error] 시뮬레이션 중 오류 발생: {e}")
        
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()