#!/usr/bin/env python3
# ==============================================================================
# 그리퍼 개폐, 힘/순응 제어 서브루틴 및 바닥 감지 기술 모듈
# ==============================================================================

import bowl_config as cfg
from DSR_ROBOT2 import *

def release():
    """그릇을 놓는 함수 (DO 2: ON, DO 3: OFF)"""
    set_digital_output(cfg.PIN_RELEASE, ON)
    set_digital_output(cfg.PIN_GRASP, OFF)
    wait(cfg.GRIPPER_WAIT_TIME)

def grasp():
    """그릇을 잡는 함수 (DO 2: OFF, DO 3: ON)"""
    set_digital_output(cfg.PIN_RELEASE, OFF)
    set_digital_output(cfg.PIN_GRASP, ON)
    wait(cfg.GRIPPER_WAIT_TIME)

def descend_until_contact(target_force=-15.0, max_ext_force=12.0):
    """
    Z축 하방으로 힘 제어를 주어 하강하다가, 
    바닥 반발력이 임계값(max_ext_force)에 도달하면 감지하고 안착하는 서브루틴
    """
    set_ref_coord(DR_BASE)
    set_desired_force(fd=[0.0, 0.0, target_force, 0.0, 0.0, 0.0], 
                      dir=[0, 0, 1, 0, 0, 0], time=0.2, mod=DR_FC_MOD_REL)
    
    # 반발력이 임계치 이하일 동안 계속 하강 대기
    while check_force_condition(axis=DR_AXIS_Z, max=max_ext_force, ref=DR_BASE):
        wait(0.05)
        
    print(" -> [접촉 감지!] 바닥 반발력 확인, 안착 대기 중...")
    wait(0.5) # 수평 안착 안정화 대기