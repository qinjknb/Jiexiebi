#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比"裸 pyserial 发 MOVE" 与 "ServoHost GUI 发 MOVE" 的差异。

输出：串口参数、DTR/RTS 状态、写出的确切字节、收到的确切字节、以及位置变化。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVO_HOST = ROOT.parent / "ServoHost"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SERVO_HOST))

PORT = "COM9"
BAUD = 115200
TARGET = 150.0
TIME_MS = 600


def read_raw(ser, seconds=1.5):
    end = time.monotonic() + seconds
    data = b""
    while time.monotonic() < end:
        n = ser.in_waiting
        chunk = ser.read(n if n else 1)
        if chunk:
            data += chunk
            end = max(end, time.monotonic() + 0.25)
    return data


def show(tag, data):
    text = data.decode("ascii", errors="replace")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    print(f"   {tag}: {len(data)} 字节  {lines}")


def raw_test():
    import serial

    print("\n=== A) 裸 pyserial：打开 → RANGE → MOVE ===")
    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        print(f"   设置: {ser.get_settings()['baudrate']} 8N1  "
              f"dtr={getattr(ser, 'dtr', '?')} rts={getattr(ser, 'rts', '?')}")
        ser.reset_input_buffer()
        time.sleep(0.2)
        ser.write(b"RANGE 300\r\n")
        print("   TX: b'RANGE 300\\r\\n'")
        show("RX", read_raw(ser, 1.0))

        ser.write(b"READ 1\r\n")
        print("   TX: b'READ 1\\r\\n'")
        show("RX", read_raw(ser, 1.0))

        cmd = f"MOVE 1 {TARGET} {TIME_MS}\r\n".encode("ascii")
        ser.write(cmd)
        print(f"   TX: {cmd!r}")
        show("RX", read_raw(ser, 1.5))

        time.sleep(0.8)
        ser.write(b"READ 1\r\n")
        show("RX(回读)", read_raw(ser, 1.0))
    return True


def gui_test():
    import tkinter as tk

    import servo_host_gui as g

    print("\n=== B) ServoHost GUI 本体：连上 → 读 → MOVE ===")
    root = tk.Tk()
    root.withdraw()
    app = g.ServoHostApp(root)
    app.refresh_ports()
    name = next((n for n in app.com_box["values"] if PORT in n), None)
    app.com_var.set(name)
    app.baud_var.set(str(BAUD))
    app.mode_var.set(g.MODE_STM32)
    app.range_var.set("300")
    root.update()

    original_send = app._send_text
    def traced(text):
        print(f"   TX: {(text + chr(13) + chr(10)).encode('ascii')!r}")
        original_send(text)
    app._send_text = traced

    app._on_connect()
    for _ in range(60):
        root.update(); time.sleep(0.02)
    if app.ser is not None:
        print(f"   设置: dtr={getattr(app.ser, 'dtr', '?')} rts={getattr(app.ser, 'rts', '?')}")

    app.id_var.set("1")
    app.angle_entry_var.set(str(TARGET))
    app.time_var.set(str(TIME_MS))
    app._on_move()
    for _ in range(120):
        root.update(); time.sleep(0.02)
    app._on_read()
    for _ in range(60):
        root.update(); time.sleep(0.02)
    print(f"   读回: {app.pos_var.get()}")
    app._on_close()
    root.update()
    try:
        root.destroy()
    except Exception:
        pass


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass
    raw_test()
    time.sleep(0.5)
    gui_test()
