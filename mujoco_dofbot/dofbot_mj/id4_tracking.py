"""Bounded ID4 directional stiction compensation with live readback.

The extra command is collision checked and never changes servo registers.
Failure holds the measured position instead of continuing a robot trajectory.
"""
import time
from .protocol import read_position, request


def move_id4_checked(ser, goal, check_target, log=lambda row: None):
    def read():
        return read_position(ser,4)*300/4095

    def send(value, duration=700):
        check_target(value)
        ack=request(ser,f'MOVE 4 {value:.2f} {duration}','OK MOVE 4',timeout=3)
        log({'command_deg':value,'ack':ack})

    before=read()
    direction=1 if goal>before else -1
    if abs(goal-before)>2.01:
        raise RuntimeError('ID4 live step exceeds 2 degrees')
    if abs(goal-before)<=.5:
        return before
    send(goal)
    time.sleep(1.0)
    actual=read();log({'phase':'nominal','actual_deg':actual,'error_deg':actual-goal})
    if abs(actual-goal)<=.6:
        return actual
    if (actual-goal)*direction>=0:
        send(actual)
        raise RuntimeError('ID4 error is inconsistent with an underreached target')

    # Measured reversal dead zone: use a bounded extra target, remove on arrival.
    compensated=goal+direction*1.6
    if abs(compensated-actual)>3.6:
        send(actual)
        raise RuntimeError('ID4 compensation step exceeds envelope')
    send(compensated)
    deadline=time.monotonic()+2.5
    try:
        while time.monotonic()<deadline:
            actual=read();error=actual-goal
            log({'phase':'compensating','actual_deg':actual,'error_deg':error})
            if abs(error)<=.45:
                send(goal,300)
                time.sleep(.5)
                actual=read();log({'phase':'settled','actual_deg':actual,'error_deg':actual-goal})
                if abs(actual-goal)>.6:
                    raise RuntimeError('ID4 moved after compensation was removed')
                return actual
            if (actual-goal)*direction>.7 or (actual-before)*direction<-.5:
                raise RuntimeError('ID4 compensation overshoot')
            time.sleep(.1)
        raise RuntimeError('ID4 compensation timeout')
    except BaseException:
        send(read(),300)
        raise
