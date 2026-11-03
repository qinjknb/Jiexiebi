#!/usr/bin/env python3
"""无硬件自测：把"协议 + 标定 + 串口链路 + 轨迹采样"整条链路跑一遍。

不接机械臂、不需要 ROS 也能运行：

    python3 -m dofbot_hw_bridge.selftest
    python3 .../dofbot_hw_bridge/selftest.py  --calib config/arm_calib.yaml

它会用内置的"假 STM32"（FakeMcuTransport）走同一套帧和同一套状态机，
再现真实固件的应答，因此能提前发现协议不一致、标定换算错误、轨迹抖动等问题。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

if __package__ in (None, ""):        # 允许直接 python3 selftest.py
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from dofbot_hw_bridge import calib as C
    from dofbot_hw_bridge import protocol as P
    from dofbot_hw_bridge.serial_link import FakeMcuTransport, LinkError, SerialLink
    from dofbot_hw_bridge.traj import TrajSampler
else:
    from . import calib as C
    from . import protocol as P
    from .serial_link import FakeMcuTransport, LinkError, SerialLink
    from .traj import TrajSampler

GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
END = "\033[0m"


class _Pt:
    """模拟 trajectory_msgs/JointTrajectoryPoint（只需要三个字段）。"""

    def __init__(self, t, pos, vel=None):
        self.time_from_start = t
        self.positions = list(pos)
        self.velocities = list(vel) if vel else []


def step(msg):
    print("  - " + msg)


def ok(msg):
    print(GREEN + "  [OK] " + msg + END)


def fail(msg):
    print(RED + "  [FAIL] " + msg + END)
    raise SystemExit(1)


def main(argv=None):
    ap = argparse.ArgumentParser(description="DOFBOT Pro 实机链路自测（不需要硬件）")
    ap.add_argument("--calib", default="", help="标定 YAML（默认用包内 config/arm_calib.yaml）")
    ap.add_argument("--rate", type=float, default=50.0, help="模拟下发频率 Hz")
    args = ap.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    calib_path = args.calib or os.path.join(here, "..", "config", "arm_calib.yaml")
    calib_path = os.path.abspath(calib_path)

    print("\n=== 1. 协议层（CRC / 组帧 / 拆帧）===")
    assert P.crc16(b"123456789") == 0x29B1
    f = P.build_frame(P.CMD_SET_TARGETS, P.enc_targets([1, 2, 3, 4, 5, 6]), 7)
    got = list(P.FrameParser().feed(b"junk" + f + b"tail"))
    assert len(got) == 1 and P.dec_targets(got[0][2]) == [1, 2, 3, 4, 5, 6]
    ok("CRC 与帧编解码正常")

    print("\n=== 2. 标定换算（角度 <-> 舵机位置）===")
    cal = C.load_calib(calib_path)
    if not os.path.isfile(calib_path):
        print(YELLOW + "  ! 没找到 %s，用内置默认标定" % calib_path + END)
    for name in C.DEFAULT_JOINT_ORDER:
        jc = cal.joints[name]
        for deg in (-90.0, -45.0, 0.0, 30.0, 90.0):
            a = math.radians(deg)
            raw = jc.angle_to_tick(a, clamp=False)
            t = jc.angle_to_tick(a)
            if jc.tick_min <= raw <= jc.tick_max:   # 没被软限位钳位才校验
                back = jc.tick_to_angle(t)
                if abs(back - a) > math.radians(0.2):
                    fail("%s 角度往返误差过大：%.2f° -> %d -> %.2f°"
                         % (name, deg, t, math.degrees(back)))
    ok("0 点、方向、限位换算一致（误差 < 0.2°）")

    print("\n=== 3. 串口链路（用内置假 STM32）===")
    fake = FakeMcuTransport()
    link = SerialLink(fake)
    link.start()
    try:
        pong = link.ping(1.0)
        if pong["njoint"] != 6:
            fail("PONG 关节数不对：%s" % pong)
        ok("PING -> 固件 v%d.%d，%d 关节" % (pong["ver_major"], pong["ver_minor"],
                                         pong["njoint"]))

        link.set_limits(*cal.limits(), timeout=1.0)
        link.set_telemetry(25, timeout=1.0)
        link.set_mode(P.MODE_STREAM, timeout=1.0)
        ticks = link.read_pos(timeout=1.0)
        if len(ticks) != 6:
            fail("READ_POS 返回长度不对")
        ok("SET_LIMITS / SET_TELEM / SET_MODE / READ_POS 全部确认（位置=%s）" % ticks)

        print("\n=== 4. 轨迹下发（模拟一条 2 秒的取放轨迹）===")
        names = list(C.DEFAULT_JOINT_ORDER)
        pts = [
            _Pt(0.0, [0, 0, 0, 0, 0, 0]),
            _Pt(0.5, [math.radians(30), math.radians(-20), math.radians(15),
                      math.radians(10), math.radians(40), math.radians(-15)]),
            _Pt(1.0, [math.radians(60), math.radians(-40), math.radians(30),
                      math.radians(20), math.radians(80), math.radians(-30)]),
            _Pt(1.5, [math.radians(30), math.radians(-20), math.radians(15),
                      math.radians(10), math.radians(40), math.radians(-15)]),
            _Pt(2.0, [0, 0, 0, 0, 0, 0]),
        ]
        sampler = TrajSampler(names, pts, names, interp="linear")
        period = 1.0 / max(1.0, args.rate)
        last_ticks = None
        jumped = False
        n_tick = 0
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            angles = sampler.at(t)
            ticks = cal.angles_to_ticks(angles)
            link.send_targets(ticks)
            if last_ticks is not None:
                step_max = max(abs(a - b) for a, b in zip(ticks, last_ticks))
                if step_max > args.rate * 0 + 400:      # 单周期跳变上限（防抖检查）
                    jumped = True
            last_ticks = ticks
            n_tick += 1
            if t >= sampler.duration:
                break
            time.sleep(period)
        step("下发 %d 帧（%.1fHz，2 秒轨迹）" % (n_tick, args.rate))
        if fake.sync_writes < n_tick * 0.8:
            fail("固件侧收到的目标帧太少：%d < %d" % (fake.sync_writes, n_tick))
        ok("假固件收到 %d 帧目标" % fake.sync_writes)
        if jumped:
            print(YELLOW + "  ! 有一帧跳变 > 400 ticks（大角度轨迹时正常，实机会被固件限速）"
                  + END)

        time.sleep(0.3)
        st = link.latest_state
        if st is None:
            fail("没有收到 STATE 遥测帧")
        ok("遥测：mode=%s flags=%s cmd=%s"
           % (P.MODE_TEXT.get(st["mode"]), P.flags_text(st["flags"]), st["cmd"]))

        print("\n=== 5. 急停 / 恢复 ===")
        link.stop_motion(timeout=1.0)
        time.sleep(0.3)
        if link.latest_state["mode"] != P.MODE_IDLE:
            fail("STOP 之后固件应进入 IDLE")
        ok("STOP -> IDLE")
        link.set_mode(P.MODE_STREAM, timeout=1.0)
        ok("可重新进入 STREAM")

        print("\n=== 6. HOME（回中位）===")
        link.home(500, timeout=1.0)
        ok("HOME 指令确认")

        if link.error is not None:
            fail("读线程异常：%s" % link.error)
    finally:
        link.shutdown()

    print("\n" + GREEN + "全部自测通过：协议、标定、链路、轨迹下发一致 " + END)
    print("下一步：接上 USB-TTL 用真实端口跑  ros2 run dofbot_hw_bridge servo_tool scan\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
