#!/usr/bin/env python3
"""DOFBOT Pro 机械臂命令行调试/标定工具。

典型用法（先不接机械臂也能玩：加 --fake）：

    ros2 run dofbot_hw_bridge servo_tool scan                 # 扫总线上的舵机
    ros2 run dofbot_hw_bridge servo_tool read                 # 读 6 个舵机当前位置
    ros2 run dofbot_hw_bridge servo_tool move 2 20            # 关节 2 转到 +20°
    ros2 run dofbot_hw_bridge servo_tool goto 0,0,0,0,0,0     # 6 个关节同时到指定角度
    ros2 run dofbot_hw_bridge servo_tool raw 2047,2047,...    # 直接给舵机位置
    ros2 run dofbot_hw_bridge servo_tool home                 # 回中位
    ros2 run dofbot_hw_bridge servo_tool monitor               # 实时看状态帧
    ros2 run dofbot_hw_bridge servo_tool text "PING 1"        # 发原文本命令给固件

参数：
    --port /dev/ttyUSB0   --baud 115200   --calib <标定yaml>   --fake（不接硬件）
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from dofbot_hw_bridge import calib as C
    from dofbot_hw_bridge import protocol as P
    from dofbot_hw_bridge.serial_link import (FakeMcuTransport, LinkError, SerialLink,
                                              open_serial)
else:
    from . import calib as C
    from . import protocol as P
    from .serial_link import FakeMcuTransport, LinkError, SerialLink, open_serial

JOINTS = C.DEFAULT_JOINT_ORDER


# ---------------------------------------------------------------------------
def _joint_index(tok: str) -> int:
    """'2' / 'Arm2_Joint' / 'grip' -> 0..5"""
    tok = tok.strip()
    low = tok.lower()
    if low in ("grip", "gripper", "g", "夹爪"):
        return 5
    if low in JOINTS:
        return JOINTS.index(low)
    try:
        n = int(tok)
    except ValueError:
        raise SystemExit("关节名无法识别：%s（可用 1~6 或 Arm1_Joint..Arm5_Joint/grip_joint）" % tok)
    if not 1 <= n <= 6:
        raise SystemExit("关节序号应为 1~6")
    return n - 1


def _parse_float_list(text, n=6):
    vals = [float(x) for x in text.replace(",", " ").split()]
    if n and len(vals) != n:
        raise SystemExit("需要 %d 个数，收到 %d 个" % (n, len(vals)))
    return vals


def _connect(args) -> SerialLink:
    calib = C.load_calib(args.calib)
    if args.port:
        calib.port = args.port
    if args.baud:
        calib.baud = args.baud
    if args.fake:
        link = SerialLink(FakeMcuTransport())
        link.start()
        print("[--fake] 使用内置假 STM32，不访问串口")
        return link
    link = open_serial(calib.port, calib.baud)
    return link


def _report(pong):
    print("STM32 在线：固件 v%d.%d，关节数 %d，总线方式 %d"
          % (pong["ver_major"], pong["ver_minor"], pong["njoint"], pong["busmode"]))


def _print_ticks(calib, ticks, tag=""):
    print("%s位置(ticks)=%s" % (tag, ticks))
    for name, t in zip(JOINTS, ticks):
        if t == P.UNKNOWN_TICK:
            print("  %-11s :  未知（舵机没应答）" % name)
        else:
            print("  %-11s : id=%d tick=%4d  角度=%+7.2f°"
                  % (name, calib.joints[name].servo_id, t,
                     math.degrees(calib.joints[name].tick_to_angle(t))))


def _stream(link, calib, ticks, duration, rate=25.0):
    """按固定频率把同一目标连发一段时间，让固件保持在 STREAM 模式。"""
    period = 1.0 / rate
    end = time.monotonic() + max(0.2, duration)
    while time.monotonic() < end:
        link.send_targets(ticks)
        time.sleep(period)


def _move_to(link, calib, angles, extra=0.4):
    """平滑移到指定关节角（弧度 dict），等待完成。"""
    ticks = calib.angles_to_ticks(angles)
    try:
        cur = link.read_pos(0.5)
    except LinkError:
        cur = None
    if cur and all(t != P.UNKNOWN_TICK for t in cur):
        max_delta = max(abs(a - b) for a, b in zip(ticks, cur))
    else:
        max_delta = 4095
    # 固件限速 240°/s -> 约 3276 ticks/s，取 80% 估算时间
    dur = max(0.3, max_delta / (3276 * 0.8)) + extra
    link.set_mode(P.MODE_STREAM, 0.5)
    _stream(link, calib, ticks, dur)
    return ticks


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description="DOFBOT Pro 机械臂调试/标定工具")
    ap.add_argument("--port", default="", help="串口，如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=0, help="波特率，默认用标定文件里的")
    ap.add_argument("--calib", default=os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "config", "arm_calib.yaml"), help="标定 YAML")
    ap.add_argument("--fake", action="store_true", help="用内置假 STM32（不接硬件）")

    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="PING 并打印固件信息/状态")
    p = sub.add_parser("read", help="读 6 个舵机当前位置并换算成角度")
    p = sub.add_parser("scan", help="在总线 1..N 上挨个 PING 舵机")
    p.add_argument("--max", type=int, default=6)
    p = sub.add_parser("move", help="单个关节按角度移动")
    p.add_argument("joint")
    p.add_argument("deg", type=float)
    p = sub.add_parser("goto", help="6 个关节同时到指定角度，如 0,0,-30,0,0,-20")
    p.add_argument("angles")
    p = sub.add_parser("raw", help="直接给 6 个舵机位置(0~4095)")
    p.add_argument("ticks")
    p = sub.add_parser("home", help="回中位")
    p.add_argument("--ms", type=int, default=2000)
    sub.add_parser("stop", help="立即停止（保持当前位置）")
    p = sub.add_parser("limits", help="把标定文件里的软限位写入固件（或 --min/--max 覆盖）")
    p.add_argument("--min", default="")
    p.add_argument("--max", default="")
    p = sub.add_parser("busmode", help="切换总线下发方式 0=SYNC WRITE 1=逐个写 2=REG+ACTION")
    p.add_argument("mode", type=int)
    p = sub.add_parser("telem", help="设置状态帧频率 Hz")
    p.add_argument("hz", type=int)
    p = sub.add_parser("monitor", help="实时打印状态帧（Ctrl+C 退出）")
    p.add_argument("--hz", type=float, default=5.0)
    p = sub.add_parser("text", help='把一条原文本命令发给固件，如 "PING 1"、"MOVE 1 90 500"')
    p.add_argument("line")
    sub.add_parser("loopback", help="USB-TTL 回环自测（先把 TX 和 RX 短接起来再运行）")
    p = sub.add_parser("sniff", help="监听串口 N 秒，把收到的任何字节打出来（判断 MCU 是否在发数据）")
    p.add_argument("--seconds", type=float, default=5.0)

    args = ap.parse_args(argv)
    calib = C.load_calib(args.calib)

    if args.cmd == "text":
        return _text_console(args)
    if args.cmd == "scan":
        return _scan(args)
    if args.cmd == "loopback":
        return _loopback(args)
    if args.cmd == "sniff":
        return _sniff(args)

    link = _connect(args)
    try:
        try:
            pong = link.ping(1.0)
            _report(pong)
        except LinkError as exc:
            print("PING 失败：%s" % exc)
            print("排查：1) 固件是否烧录 2) TX->PA3 RX->PA2 GND->GND 3) 波特率 115200")
            return 1

        if args.cmd == "status":
            _print_ticks(calib, link.read_pos(0.8))
            link.set_telemetry(max(1, calib.telem_rate), 1.0)
            time.sleep(0.3)
            st = link.latest_state
            if st:
                print("状态帧：mode=%s flags=%s"
                      % (P.MODE_TEXT.get(st["mode"]), P.flags_text(st["flags"])))

        elif args.cmd == "read":
            _print_ticks(calib, link.read_pos(0.8))

        elif args.cmd == "move":
            idx = _joint_index(args.joint)
            angles = dict(calib.ticks_to_angles(link.read_pos(0.8)))
            for name in JOINTS:
                angles.setdefault(name, 0.0)
            angles[JOINTS[idx]] = math.radians(args.deg)
            print("关节 %s -> %+.1f°" % (JOINTS[idx], args.deg))
            ticks = _move_to(link, calib, angles)
            print("完成，目标 ticks=%s" % ticks)

        elif args.cmd == "goto":
            degs = _parse_float_list(args.angles, 6)
            angles = {n: math.radians(d) for n, d in zip(JOINTS, degs)}
            print("目标角度=%s" % ["%+.1f" % d for d in degs])
            ticks = _move_to(link, calib, angles)
            print("完成，目标 ticks=%s" % ticks)

        elif args.cmd == "raw":
            ticks = [int(round(v)) for v in _parse_float_list(args.ticks, 6)]
            print("目标 ticks=%s" % ticks)
            link.set_mode(P.MODE_STREAM, 0.5)
            _stream(link, calib, ticks, 1.5)
            print("已下发")

        elif args.cmd == "home":
            print("回中位（%d ms）..." % args.ms)
            link.home(args.ms, 1.0)
            time.sleep(args.ms / 1000.0)
            link.set_mode(P.MODE_STREAM, 0.5)
            _print_ticks(calib, link.read_pos(0.8), "到位后 ")

        elif args.cmd == "stop":
            link.stop_motion(1.0)
            print("已停止，舵机保持当前位置")

        elif args.cmd == "limits":
            tmin, tmax = calib.limits()
            if args.min:
                tmin = [int(v) for v in _parse_float_list(args.min, 6)]
            if args.max:
                tmax = [int(v) for v in _parse_float_list(args.max, 6)]
            link.set_limits(tmin, tmax, 1.0)
            print("软限位已写入：min=%s max=%s" % (tmin, tmax))

        elif args.cmd == "busmode":
            link.set_busmode(args.mode, 1.0)
            print("总线方式已切换为 %d（0=SYNC WRITE 1=逐个写 2=REG+ACTION）" % args.mode)

        elif args.cmd == "telem":
            link.set_telemetry(args.hz, 1.0)
            print("状态帧频率 = %d Hz" % args.hz)

        elif args.cmd == "monitor":
            link.set_telemetry(int(max(1, args.hz)), 1.0)
            print("实时状态（Ctrl+C 退出）：mode/flags/cmd/meas")
            try:
                while True:
                    try:
                        st = link.state_queue.get(timeout=1.0)
                    except Exception:
                        print("  (1 秒没有状态帧：固件在线吗？)")
                        continue
                    print("  %-6s %-10s cmd=%s meas=%s"
                          % (P.MODE_TEXT.get(st["mode"], st["mode"]),
                             P.flags_text(st["flags"]), st["cmd"], st["meas"]))
                    if args.hz and args.hz > 0:
                        time.sleep(1.0 / args.hz / 2)
            except KeyboardInterrupt:
                print("\n退出监控")
        else:
            ap.print_help()
            return 2
    finally:
        link.shutdown()
    return 0


# ---------------------------------------------------------------------------
def _open_raw(args):
    """打开裸串口（文本命令用），返回 (serial, None) 或 (None, 错误信息)。"""
    try:
        import serial
    except ImportError:
        return None, "没装 pyserial：pip3 install pyserial"
    calib = C.load_calib(args.calib)
    port = args.port or calib.port
    baud = args.baud or calib.baud
    try:
        ser = serial.serial_for_url(port, baudrate=baud, timeout=0.05)
    except Exception as exc:
        return None, "打不开串口 %s：%s" % (port, exc)
    time.sleep(0.25)            # 固件静默 200ms 后自动判定为文本模式
    return ser, None


def _drain(ser, wait: float = 0.4) -> str:
    """把串口里已经到达的数据读空（用于看固件开机问候/残留回复）。"""
    end = time.time() + wait
    buf = b""
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b"\n" in buf and len(buf) > 8:
                break
    return buf.decode("ascii", "replace").strip()


def _send_text(ser, line: str, wait: float = 0.6) -> str:
    ser.write((line + "\n").encode("ascii"))
    ser.flush()
    end = time.time() + wait
    buf = b""
    while time.time() < end:
        buf += ser.read(64)
        if b"\n" in buf:
            break
    return buf.decode("ascii", "replace").strip()


def _scan(args) -> int:
    if args.fake:
        print("[--fake] 扫描需要真实串口；假固件固定有 ID 1~6")
        return 0
    ser, err = _open_raw(args)
    if err:
        print(err)
        return 1
    try:
        banner = _drain(ser, 0.5)
        if banner:
            print("固件消息: %s" % banner.replace("\r", " ").replace("\n", " | "))
            if "ARM BRIDGE" in banner:
                print("        -> 已烧录本仓库新固件（V2.0）")
            elif "SERVO HOST" in banner:
                print("        -> 这是旧固件！请先烧录新固件（build_fw.ps1 -Flash）")
        else:
            print("固件消息: (无) 可能刚重启过；继续测试")

        print("逐个 PING 总线舵机（走固件文本命令通道）...")
        online = []
        for sid in range(1, max(1, args.max) + 1):
            rep = _send_text(ser, "PING %d" % sid, 0.45)
            clean = rep.replace("\r", " ").replace("\n", " | ").strip()
            if rep.startswith("OK PING"):
                print("  ID %-2d : 在线" % sid)
                online.append(sid)
            elif not rep:
                print("  ID %-2d : 无应答（固件没回话，见下方排查）" % sid)
            else:
                print("  ID %-2d : 固件回复 %r" % (sid, clean))
    finally:
        ser.close()

    print("在线舵机：%s" % (online if online else "无"))
    if not online:
        print("""
排查顺序（从上到下，哪条先不对就先修哪条）：
  1) 用二进制通道确认 STM32 与上位机通信正常：
       ros2 run dofbot_hw_bridge servo_tool --port %s status
     · 打印 "STM32 在线：固件 v2.0" -> MCU 链路正常，问题在舵机总线一侧，看 3)~6)
     · 超时无应答           -> 固件没烧 / TX-RX 接反 / 没共地 / 波特率不对
  2) 若上面把 '固件回复 ERR PING' 打出来 -> 固件在，舵机总线没应答，看 3)~6)
  3) 舵机驱动板是否上电（绿色端子 DC+/DC- 接 9~12.6V，PWR 灯亮）
  4) 板上小开关是否拨到 A(UART-SERVO)
  5) STM32 PA9 是否接到驱动板 UART 的数据线，且与驱动板共地
  6) 舵机 ID 是否真是 1~6，总线波特率是否 115200（可用 servo_tool text 试）：
       ros2 run dofbot_hw_bridge servo_tool --port %s text "PING 1"
       ros2 run dofbot_hw_bridge servo_tool --port %s text "MOVEP 1 2047 500"
""" % (args.port or "/dev/ttyUSB0", args.port or "/dev/ttyUSB0", args.port or "/dev/ttyUSB0"))
    return 0


def _loopback(args) -> int:
    """USB-TTL 回环自测：TX/RX 短接时应该能收到自己发的字节。"""
    if args.fake:
        print("[--fake] 回环自测需要真实串口")
        return 0
    print("请先把 USB-TTL 的 TX 和 RX 用一根杜邦线短接（从 STM32 上拔下来），然后重跑本命令。")
    ser, err = _open_raw(args)
    if err:
        print(err)
        return 1
    try:
        ser.reset_input_buffer()
        ser.write(b"LOOPBACK-TEST\r\n")
        ser.flush()
        time.sleep(0.3)
        got = ser.read(256)
    finally:
        ser.close()

    if b"LOOPBACK-TEST" in got:
        print("回环成功 -> USB-TTL 模块、虚拟机 USB 直通、串口号都是对的")
        print("           那么问题在 STM32 一侧：供电 / 接线 / 固件")
    else:
        print("回环失败（收到 %d 字节: %r）" % (len(got), got))
        print("  · 确认 TX 和 RX 真的短接了（不是接到 STM32 上）")
        print("  · 确认 --port 选的是这块 USB-TTL：ls -l /dev/serial/by-id/")
        print("  · VMware: 虚拟机 -> 可移动设备 -> 把 CH340 连接到虚拟机")
    return 0


def _sniff(args) -> int:
    """监听串口，看 MCU 到底有没有往外发数据。"""
    if args.fake:
        print("[--fake] 监听需要真实串口")
        return 0
    ser, err = _open_raw(args)
    if err:
        print(err)
        return 1
    print("监听 %.1f 秒……（这段时间里可以按一下 STM32 板上的复位键，"
          "正常会看到一条开机问候）" % args.seconds)
    total = 0
    end = time.time() + args.seconds
    ser.reset_input_buffer()
    try:
        while time.time() < end:
            data = ser.read(256)
            if not data:
                continue
            total += len(data)
            txt = data.decode("ascii", "replace").replace("\r", " ").replace("\n", " | ")
            print("  RX %3d 字节  %-40s  %r" % (len(data), txt.strip()[:40], data))
    finally:
        ser.close()

    print()
    print("共收到 %d 字节" % total)
    if total == 0:
        print("一个字节都没收到，说明 STM32 没有在发数据。按顺序查：")
        print("  1) STM32 有没有供电？（Blue Pill 板上的红色 PWR 灯要亮）")
        print("  2) USB-TTL RX 是否接到 STM32 PA2（USART2_TX），GND 是否共地")
        print("  3) 固件是否烧录成功（用 ST-Link 重烧一次最保险）")
        print("  4) 波特率是否 115200")
    return 0


def _text_console(args) -> int:
    """把一条文本命令发出去并打印回复（调试单个舵机最方便）。"""
    if args.fake:
        print("[--fake] 文本命令需要真实串口")
        return 0
    ser, err = _open_raw(args)
    if err:
        print(err)
        return 1
    try:
        rep = _send_text(ser, args.line)
    finally:
        ser.close()
    print(">> %s" % args.line)
    print("<< %s" % (rep if rep else "(无应答)"))
    if not rep:
        print("   提示：固件完全没回话 -> 先用 servo_tool status 确认 MCU 通信；")
        print("         固件回 ERR xxx -> MCU 正常，问题在舵机总线（供电/开关 A/PA9/ID）")
    return 0 if rep else 1


if __name__ == "__main__":
    raise SystemExit(main())
