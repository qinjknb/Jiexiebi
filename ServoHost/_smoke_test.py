# -*- coding: utf-8 -*-
"""servo_host_gui.py 逻辑冒烟测试（伪造 pyserial，无需硬件）"""
import os
import sys

FAKE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_fake_pkg")
os.makedirs(os.path.join(FAKE, "serial", "tools"), exist_ok=True)

with open(os.path.join(FAKE, "serial", "__init__.py"), "w", encoding="utf-8") as f:
    f.write('''
class SerialException(Exception):
    pass

class Serial:
    def __init__(self, port=None, baudrate=9600, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.written = []
    def write(self, data):
        self.written.append(bytes(data))
    def read(self, n=1):
        return b""
    @property
    def in_waiting(self):
        return 0
    def close(self):
        self.is_open = False
''')

with open(os.path.join(FAKE, "serial", "tools", "__init__.py"), "w",
          encoding="utf-8") as f:
    f.write("")

with open(os.path.join(FAKE, "serial", "tools", "list_ports.py"), "w",
          encoding="utf-8") as f:
    f.write('''
def comports():
    class P:
        def __init__(self, d, desc):
            self.device = d
            self.description = desc
    return [P("COM3", "USB-SERIAL CH340 (COM3)")]
''')

sys.path.insert(0, FAKE)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import servo_host_gui as g  # noqa: E402
import tkinter as tk  # noqa: E402

root = tk.Tk()
root.withdraw()
app = g.ServoHostApp(root)

# 1. 端口刷新
app.refresh_ports()
assert "COM3" in app.com_var.get(), app.com_var.get()
print("1. 串口列表 OK:", app.com_var.get())

# 2. 连接（STM32 模式，应自动下发 RANGE）
app._on_connect()
assert app.ser is not None and app.ser.is_open
assert app.ser.written[-1] == b"RANGE 300\r\n", app.ser.written[-1]
print("2. 连接 OK, 自动下发:", app.ser.written[-1])

# 3. 模拟下位机问候 + MOVE + POS 应答
app.q.put(("line", "READY STM32F103 SERVO HOST V1.0"))
app._poll()
app.id_var.set("1")
app.angle_entry_var.set("150")
app.time_var.set("400")
app._on_move()
assert app.ser.written[-1] == b"MOVE 1 150.0 400\r\n", app.ser.written[-1]
print("3. MOVE 命令 OK:", app.ser.written[-1])

app.q.put(("line", "POS 1 2048"))
app._poll()
assert "2048" in app.pos_var.get(), app.pos_var.get()
print("   位置解析 OK:", app.pos_var.get())

# 4. 直连模式：MOVE 帧
app.mode_var.set(g.MODE_DIRECT)
app._on_mode_changed()
app._on_move()
frame = app.ser.written[-1]
expect = g.frame_move(1, 2048, 400)
assert frame == expect, (frame.hex(" "), expect.hex(" "))
print("4. 直连 MOVE 帧 OK:", frame.hex(" ").upper())

# 5. 直连模式：READ（回显过滤 + 应答解析）
app._on_read()
read_frame = app.ser.written[-1]
assert app._waiting is not None
reply = bytes([0xFF, 0xF5, 1, 0x04, 0x00, 0x08, 0x00, 0xF2])  # pos=2048
app.q.put(("raw", read_frame))   # 自发回显
app.q.put(("raw", reply))        # 舵机真实应答
app._poll()
assert app._waiting is None, "应答未解析"
assert "2048" in app.pos_var.get(), app.pos_var.get()
print("5. 直连 READ 应答 OK:", app.pos_var.get())

# 6. 直连模式：PING
app._on_ping()
ping_frame = app.ser.written[-1]
ping_reply = bytes([0xFF, 0xF5, 1, 0x02, 0x00, 0xFC])
app.q.put(("raw", ping_frame))
app.q.put(("raw", ping_reply))
app._poll()
assert app._waiting is None
print("6. 直连 PING OK")

# 7. 断开
app._on_close()
print("7. 断开/关闭 OK")
print("\nALL TESTS PASSED")
