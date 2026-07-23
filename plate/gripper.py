"""2-Finger Gripper 제어 (디지털 출력 방식)."""

from DSR_ROBOT2 import set_digital_output, wait

from .config import GRIP_OPEN_CH, GRIP_CLOSE_CH, GRIP_WAIT

ON, OFF = 1, 0


class Gripper:
    """DO2=열기 / DO3=닫기 로 동작하는 2-finger 그리퍼.

    반대 채널을 먼저 끄고 켜서 두 신호가 동시에 ON 되는 순간을 없앤다.
    """

    def __init__(self, node=None):
        self.node = node

    def open(self, wait_time=None):
        set_digital_output(GRIP_CLOSE_CH, OFF)
        set_digital_output(GRIP_OPEN_CH, ON)
        wait(GRIP_WAIT if wait_time is None else wait_time)

    def close(self, wait_time=None):
        set_digital_output(GRIP_OPEN_CH, OFF)
        set_digital_output(GRIP_CLOSE_CH, ON)
        wait(GRIP_WAIT if wait_time is None else wait_time)