"""모션 가드 — 정지(abort) 요청을 모든 모션 호출에 강제한다.

배경: request_abort 의 move_stop 은 "지금 도는 모션 하나"만 끊는다.
컨트롤러/음식물 모듈의 시퀀스는 모션이 끝나면 곧장 다음 모션을 부르므로
단계 사이의 _check_abort 만으로는 그 사이 모션들(반원 pass 루프, 파지
하강 후의 그리퍼+상승, 털기 루프 등)이 정지 후에도 계속 실행된다.

여기서 DSR 모션 함수(movej/movel/movec/amovel)를 감싼 대체 함수를
제공한다. 각 호출 전후로 abort 플래그를 확인해:
  - 호출 전  : 정지 후 "다음 모션"이 새로 시작되는 것을 막고
  - 호출 후  : move_stop 으로 끊긴 직후 즉시 PlateAborted 로 빠져나온다

controller 와 food_check 는 DSR_ROBOT2 대신 이 모듈에서 모션 함수를
import 한다. 호출 지점은 하나도 바꾸지 않아도 전부 가드를 거친다.

주의: DSR_ROBOT2 를 import 하므로 이 모듈도 DR_init.__dsr__node 등록
이후(= controller import 시점)에만 import 돼야 한다.
"""

import threading

from DSR_ROBOT2 import (
    movej as _movej,
    movel as _movel,
    movec as _movec,
    amovel as _amovel,
)


class PlateAborted(Exception):
    """긴급정지 요청으로 작업이 중단됨."""


_ABORT = threading.Event()


def set_abort():
    """정지 플래그를 세운다. 이후 모든 모션 호출이 PlateAborted 를 던진다."""
    _ABORT.set()


def clear_abort():
    """정지 플래그 해제. 다음 작업 시작 전에 부른다."""
    _ABORT.clear()


def is_aborted():
    return _ABORT.is_set()


def check_abort():
    """정지 요청이 있으면 예외를 던져 작업을 끊는다."""
    if _ABORT.is_set():
        raise PlateAborted("aborted by request")


def _guarded(fn):
    def wrapper(*args, **kwargs):
        check_abort()                  # 정지 후 새 모션 시작 차단
        ret = fn(*args, **kwargs)
        check_abort()                  # 끊긴 모션에서 즉시 탈출
        return ret

    wrapper.__name__ = fn.__name__
    wrapper.__doc__ = f"abort 가드를 거치는 {fn.__name__}"
    return wrapper


movej = _guarded(_movej)
movel = _guarded(_movel)
movec = _guarded(_movec)
amovel = _guarded(_amovel)
