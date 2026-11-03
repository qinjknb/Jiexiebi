#!/usr/bin/env python3
"""DOFBOT Pro 串口链路体检（Windows 双击即可运行，自动找 CH340）。

检查内容：
  阶段0  监听 3 秒：本仓库固件上电后每 40ms 自发一帧 32 字节状态帧（25Hz），
         有数据流 => STM32 活着；完全没数据 => STM32 没在跑程序。
  阶段1  给你 15 秒按 STM32 的 RESET 键，抓开机问候，判断烧的是哪个固件。
  阶段2  逐个发文本命令（HELP / WIRING / JOINTS / PING 1~6）和二进制 PING，
         看 STM32 到底能不能"收到"命令。
  最后给出结论和下一步该查什么。

用法：双击 check_serial.bat，或命令行 python check_serial.py [COM口]
"""

from __future__ import annotations

import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial。请先在命令行执行：  pip install pyserial")
    pause()
    raise SystemExit(2)

BAUD = 115200


# ---------------------------------------------------------------- 协议
def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd: int, payload: bytes = b"", seq: int = 1) -> bytes:
    body = bytes([cmd, seq, len(payload)]) + payload
    c = crc16(body)
    return bytes([0xAA, 0x55]) + body + bytes([c >> 8, c & 0xFF])


# ---------------------------------------------------------------- 选口
def pick_port(argv) -> str:
    if len(argv) > 1 and argv[1]:
        return argv[1]
    cands = []
    for p in list_ports.comports():
        desc = (p.description or "").upper()
        if any(k in desc for k in ("CH340", "CH343", "CP210", "USB-SERIAL", "FT232")):
            cands.append(p.device)
    if len(cands) == 1:
        return cands[0]
    if len(cands) > 1:
        print("发现多个 USB 串口：%s，先用第一个" % cands)
        return cands[0]
    return None       # 没找到 USB 串口，不要乱选蓝牙串口


def pause(msg="\n按回车退出 ..."):
    try:
        input(msg)
    except (EOFError, KeyboardInterrupt):
        pass


def read_for(ser, seconds: float):
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
    return buf


def main() -> int:
    port = pick_port(sys.argv)
    print("=" * 62)
    print(" DOFBOT Pro 串口链路体检")
    print("=" * 62)
    print("\n所有串口设备：")
    for p in list_ports.comports():
        print("   %-8s %s" % (p.device, p.description))
    if not port:
        print("\n没找到 USB 串口（CH340/CP210x）！")
        print("  · USB-TTL 是否插在电脑上？")
        print("  · 是否被 VMware 虚拟机占用？(VM -> Removable Devices -> CH340 -> Disconnect)")
        print("  · 也可以指定： python check_serial.py COM7")
        pause()
        return 1
    print("\n准备使用： %s @ %d" % (port, BAUD))
    print("（请先在上位机里点【断开】，否则端口被占用打不开）\n")

    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as exc:
        print("打不开 %s：%s" % (port, exc))
        print("\n  · 提示“拒绝访问/Access denied” -> 上位机还连着，先点【断开】")
        print("  · 提示“找不到/FileNotFound”   -> 设备不在，检查 USB 是否接在 Windows 上")
        pause()
        return 1

    # DTR/RTS 拉低，避免个别模块用它们做复位/下载
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass

    verdict = []
    try:
        # ---------------- 阶段 0：是否在自发状态帧 ----------------
        print("[阶段0] 监听 3 秒，看 STM32 有没有主动发数据 ……")
        ser.reset_input_buffer()
        data = read_for(ser, 3.0)
        if data:
            print("        收到 %d 字节，前 32 字节：" % len(data))
            print("        " + data[:32].hex(" "))
            if b"\xaa\x55\x83" in data:
                print("        => 是 25Hz 状态帧(AA 55 83)：STM32 活着、固件是本仓库新版 ✔")
                verdict.append("STM32 活着（在发状态帧）")
            elif b"READY" in data:
                print("        => 是开机问候文本：STM32 活着 ✔")
                verdict.append("STM32 活着（收到问候）")
            else:
                print("        => 有数据但不像本仓库固件（可能是旧固件/波特率不对/噪声）")
                verdict.append("STM32 在发数据，但不匹配本仓库固件")
        else:
            print("        一个字节都没有。")
            print("        （固件默认 TELEM=0，不主动发状态帧，所以这里静默是正常的；")
            print("          关键看下面阶段1/2：按复位能不能出问候、发命令能不能回）")
            verdict.append("未主动发送数据（若固件 TELEM=0 属正常）")

        # ---------------- 阶段 1：按复位抓问候 ----------------
        print("\n[阶段1] 现在请按一下 STM32 板上的 RESET 键（有 15 秒时间）……")
        ser.reset_input_buffer()
        boot = read_for(ser, 15.0)
        if b"ARM BRIDGE" in boot:
            print("        => 收到 READY ... ARM BRIDGE V2.0：新固件，MCU 能正常复位运行 ✔")
            verdict.append("按复位后能发开机问候（新固件）")
        elif b"SERVO HOST" in boot:
            print("        => 收到 READY ... SERVO HOST V1.0：这是【旧固件】")
            verdict.append("按复位发的是旧固件问候")
        elif boot:
            print("        => 收到 %d 字节但含乱码/非问候：%s" % (len(boot), boot[:40]))
            verdict.append("按复位有输出但不是标准问候")
        else:
            print("        => 15 秒内什么都没有。")
            verdict.append("按复位后仍然没有任何输出")

        # ---------------- 阶段 2：命令能不能收进去 ----------------
        print("\n[阶段2] 发命令，看 STM32 能不能收到（这是关键）……")

        def cmd(line: str, wait: float = 1.0) -> bytes:
            ser.reset_input_buffer()
            ser.write(line.encode() + b"\n")
            ser.flush()
            return read_for(ser, wait)

        text_alive = False
        for line in ("HELP", "RANGE 300", "WIRING 2", "JOINTS"):
            r = cmd(line, 1.0)
            # 去掉可能夹在中间的状态帧，看看有没有文本回复
            printable = bytes(b for b in r if 32 <= b < 127 or b in (10, 13))
            print("        TX %-12s -> %s" % (line, printable.decode("ascii", "replace").strip()
                                              if printable.strip() else "(无文本回复)"))
            if any(k in printable for k in (b"OK", b"HELP", b"ERR", b"POS", b"JOINTS")):
                text_alive = True

        ser.reset_input_buffer()
        ser.write(frame(0x01))
        ser.flush()
        rb = read_for(ser, 1.2)
        print("        TX 二进制PING   -> %s" % (rb.hex(" ") if rb else "(无应答)"))
    finally:
        ser.close()

    # ---------------- 结论 ----------------
    print("\n" + "=" * 62)
    print(" 结论")
    print("=" * 62)
    for v in verdict:
        print("  · " + v)

    if text_alive:
        print("\n  ✔ STM32 能收到命令 -> 双向链路正常。")
        print("    接下来：在【自定义指令】里发 JOINTS 读位置、发 PING 1 找舵机。")
    else:
        print("\n  ✘ STM32 收不到命令。")
        if any("活着" in v for v in verdict) or any("问候" in v for v in verdict):
            print("    MCU 能发不能收 -> 几乎可以确定是【CH340 TX -> STM32 PA3】这根线/这个脚的问题。")
            print("    用万用表量：CH340 的 TX 对 GND 应为 ≈3.3V；再量 STM32 的 PA3 对 GND，")
            print("    如果 PA3 是 0V 或乱跳 -> 线没通；换一根杜邦线、或换到 PA3 另一个焊点。")
            print("    同时确认 TX 没有接到 PA2（对调）也没有接到 PA9/PA10（那是 USART1）。")
        else:
            print("    MCU 完全没输出 -> 先解决供电/固件：")
            print("    1) Blue Pill 红色 PWR 灯亮不亮？（不亮=没供电）")
            print("    2) 固件烧了吗？（GCWJ\\build_fw.ps1 -Flash）")
            print("    3) ST-Link 是否还插着把芯片 halt 住了？拔掉再按复位")
            print("    4) CH340 的 RX 是否接到 PA2、GND 是否共地")

    print("\n（本窗口结果可直接截图）")
    pause()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
