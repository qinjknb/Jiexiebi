#!/usr/bin/env python3
"""串口链路体检工具（不依赖 ROS，也不依赖 dofbot_hw_bridge 包，只需要 pyserial）。

用途：当 servo_tool 收不到任何应答时，用它把「电脑 → USB-TTL → STM32」这条链路
一段一段测出来，判断到底是模块、接线、波特率、供电，还是固件的问题。

用法：
    python3 uart_probe.py                     # 体检 /dev/ttyUSB0（推荐先跑这个）
    python3 uart_probe.py --port /dev/ttyUSB1
    python3 uart_probe.py --list              # 只列出所有串口设备（含 VID:PID）
    python3 uart_probe.py --loopback          # 回环自测：先把 TX/RX 短接！
    python3 uart_probe.py --scan-baud         # 挨个波特率试，找舵机/固件真正的波特率

体检会依次做：
    1) 列设备，打印 VID:PID/序列号，确认 /dev/ttyUSB0 是哪块板
    2) 监听 8 秒，期间请按 STM32 的复位键，看有没有开机问候（固件是否在跑）
    3) 发文本 PING 1，看固件怎么回
    4) 发二进制 PING（本仓库固件的协议），看是否回 PONG
最后给结论。
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:                        # pragma: no cover
    print("缺少 pyserial：pip3 install pyserial   或   sudo apt install python3-serial")
    raise SystemExit(2)


# ---------------------------------------------------------------------------
# 协议：与固件 robot_link.h 一致（AA 55 | CMD | SEQ | LEN | PAYLOAD | CRC_H CRC_L）
# ---------------------------------------------------------------------------
def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd: int, payload: bytes = b"", seq: int = 1) -> bytes:
    body = bytes([cmd & 0xFF, seq & 0xFF, len(payload)]) + payload
    c = crc16(body)
    return bytes([0xAA, 0x55]) + body + bytes([c >> 8, c & 0xFF])


def show_ports() -> None:
    print("=== 串口设备 ===")
    comps = list(list_ports.comports())
    if not comps:
        print("  (没找到任何串口设备！)")
    for p in comps:
        print("  %-16s %s" % (p.device, p.description))
        print("  %-16s VID:PID=%04X:%04X  serial=%s" %
              ("", p.vid or 0, p.pid or 0, p.serial_number))
    # Linux 下更可靠的稳定路径
    try:
        import glob
        ids = glob.glob("/dev/serial/by-id/*")
        if ids:
            print("  稳定路径（推荐用它指定 --port）：")
            for i in ids:
                print("    " + i)
    except Exception:
        pass


def read_for(ser, seconds: float, quiet: bool = False) -> bytes:
    """读一段时间，返回收到的字节；每收到一块就打印。"""
    end = time.time() + seconds
    got = b""
    while time.time() < end:
        chunk = ser.read(256)
        if not chunk:
            continue
        got += chunk
        if not quiet:
            txt = chunk.decode("ascii", "replace").replace("\r", " ").replace("\n", " | ")
            print("    RX %3d 字节: %-46s | %s" % (len(chunk), txt.strip()[:46], chunk.hex(" ")))
    return got


def verdict_banner(banner: bytes) -> None:
    if b"ARM BRIDGE" in banner:
        print("  => 收到本仓库新固件开机问候，说明 STM32 供电/接线/固件全部正常")
    elif b"SERVO HOST" in banner:
        print("  => 收到**旧固件**问候：请先烧录新固件（GCWJ/build_fw.ps1 -Flash）")
    elif banner:
        print("  => 收到了数据但不像开机问候，可能是波特率不对或线路噪声")
    else:
        print("  => 没收到任何数据")


def probe(port: str, baud: int) -> int:
    print("\n=== 打开 %s @ %d ===" % (port, baud))
    try:
        ser = serial.serial_for_url(port, baudrate=baud, timeout=0.2)
    except Exception as exc:
        print("  打开失败：%s" % exc)
        print("  · 设备名对吗？ls -l /dev/serial/by-id/")
        print("  · 权限：sudo usermod -aG dialout $USER 后重新登录")
        print("  · VMware：虚拟机 -> 可移动设备 -> 把 CH340 连接到虚拟机")
        return 1

    try:
        # ---- 1) 开机问候（请用户按复位）----
        print("\n[1/3] 监听 8 秒 —— 现在请按一下 STM32 板上的复位键（RESET）")
        ser.reset_input_buffer()
        banner = read_for(ser, 8.0)
        verdict_banner(banner)

        # ---- 2) 文本 PING ----
        print("\n[2/3] 发文本命令  PING 1")
        ser.reset_input_buffer()
        ser.write(b"PING 1\n")
        ser.flush()
        rep = read_for(ser, 1.5)
        if rep:
            print("  => 固件有回复：%r" % rep.decode("ascii", "replace").strip())
            if b"OK PING" in rep:
                print("  => 舵机总线也通了，链路完全正常")
            elif b"ERR PING" in rep:
                print("  => STM32 正常，但舵机总线无应答：查驱动板供电/开关 A/PA9/舵机ID")
        else:
            print("  => 无回复")

        # ---- 3) 二进制 PING ----
        print("\n[3/3] 发二进制 PING（本仓库固件协议，AA 55 ...）")
        ser.reset_input_buffer()
        ser.write(frame(0x01))
        ser.flush()
        rep2 = read_for(ser, 1.5)
        if b"\xaa\x55" in rep2:
            print("  => 收到二进制应答，新固件在跑")
        elif rep2:
            print("  => 收到数据但不是二进制应答（可能是旧固件在按文本解析）")
        else:
            print("  => 无应答")
    finally:
        ser.close()

    # ---- 总结 ----
    print("\n=== 结论 ===")
    if not banner and not rep and not rep2:
        print("  三个测试全无回复 -> STM32 侧完全没在工作，按顺序查：")
        print("   1) Blue Pill 板上红色 PWR 灯亮不亮？（不亮 = 没供电）")
        print("      供电方式：USB-TTL 的 5V -> 板子 5V 引脚，或插着 ST-Link（3.3V）")
        print("   2) ST-Link 是否还插着并把芯片 halt 住了？拔掉 ST-Link 或按复位")
        print("   3) USB-TTL 的 RX 是否接到 STM32 的 PA2（USART2_TX）？GND 是否共地？")
        print("   4) 固件是否真的烧进去了？（用 ST-Link 重烧一次）")
        print("   5) 如果怀疑是电脑侧，做回环：拔掉 STM32，短接 TX-RX，"
              "运行 python3 uart_probe.py --loopback")
    elif banner and not rep and not rep2:
        print("  固件在跑但只发了开机问候、不回命令 -> 检查 RX 线（PC2PA2），"
              "以及是否用了旧固件")
    else:
        print("  链路基本正常。若舵机仍不动，问题在舵机总线（供电/开关A/PA9/ID）")
    return 0


def loopback(port: str, baud: int) -> int:
    print("\n=== 回环自测（必须先拔掉 STM32，并用杜邦线把 TX 和 RX 短接）===")
    try:
        ser = serial.serial_for_url(port, baudrate=baud, timeout=0.2)
    except Exception as exc:
        print("打开失败：%s" % exc)
        return 1
    try:
        ser.reset_input_buffer()
        marker = b"LOOPBACK-TEST-12345\r\n"
        ser.write(marker)
        ser.flush()
        time.sleep(0.3)
        got = ser.read(256)
    finally:
        ser.close()
    if b"LOOPBACK-TEST" in got:
        print("回环成功 -> USB-TTL、虚拟机 USB 直通、串口号都正常；问题在 STM32 一侧")
        return 0
    print("回环失败：收到 %d 字节 %r" % (len(got), got))
    print("  · TX/RX 真的短接了吗？（要拔掉 STM32，只短接模块自己两个脚）")
    print("  · --port 选的是这块 USB-TTL 吗？先跑 --list 看列表")
    return 1


def scan_baud(port: str) -> int:
    print("\n=== 挨个波特率试探（文本 PING + 二进制 PING）===")
    found = []
    for baud in (115200, 9600, 57600, 230400, 460800, 921600, 1000000, 500000):
        try:
            ser = serial.serial_for_url(port, baudrate=baud, timeout=0.15)
        except Exception as exc:
            print("  %7d : 打不开 %s" % (baud, exc))
            continue
        try:
            ser.reset_input_buffer()
            ser.write(b"PING 1\n")
            ser.flush()
            time.sleep(0.4)
            r1 = ser.read(256)
            ser.reset_input_buffer()
            ser.write(frame(0x01))
            ser.flush()
            time.sleep(0.4)
            r2 = ser.read(256)
        finally:
            ser.close()
        ok = bool(r1 or r2)
        print("  %7d : %s" % (baud, ("收到 %d/%d 字节 %r %r" % (len(r1), len(r2), r1, r2))
                              if ok else "无"))
        if ok:
            found.append(baud)
    if found:
        print("  可用波特率：%s" % found)
        print("  （注意：STM32 侧固定 115200；若这里显示别的值，说明你在直连驱动板或舵机）")
        return 0
    print("  所有波特率都没反应 -> 优先怀疑供电/接线，而不是波特率")
    return 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="串口链路体检（独立于 ROS）")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="只列出串口设备")
    ap.add_argument("--loopback", action="store_true", help="回环自测（先短接 TX/RX）")
    ap.add_argument("--scan-baud", action="store_true", help="试探多个波特率")
    args = ap.parse_args(argv)

    if args.list:
        show_ports()
        return 0
    if args.loopback:
        show_ports()
        return loopback(args.port, args.baud)
    if args.scan_baud:
        show_ports()
        return scan_baud(args.port)

    show_ports()
    return probe(args.port, args.baud)


if __name__ == "__main__":
    raise SystemExit(main())
