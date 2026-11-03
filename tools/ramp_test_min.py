"""斜坡下发测试（极简版，直接粘进终端即可）。

用法： python3 ramp_test.py 串口 舵机ID 起点 终点 [秒数] [频率] [死区]
例：   python3 ramp_test.py /dev/ttyUSB0 6 1904 1050
       python3 ramp_test.py /dev/ttyUSB0 6 1904 1050 3 20
       python3 ramp_test.py /dev/ttyUSB0 6 1904 1050 3 50 3

用与桥接相同的二进制协议（SET_TARGETS @50Hz）只驱动一路舵机，
其余关节保持在"读到的当前位置"（不会乱跳）。
"""

import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
SERVO = int(sys.argv[2]) if len(sys.argv) > 2 else 6
START = int(sys.argv[3]) if len(sys.argv) > 3 else 2047
END = int(sys.argv[4]) if len(sys.argv) > 4 else 1904
SECS = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
RATE = float(sys.argv[6]) if len(sys.argv) > 6 else 50.0
STEP = int(sys.argv[7]) if len(sys.argv) > 7 else 0


def crc16(d, c=0xFFFF):
    for x in d:
        c ^= (x << 8) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def fr(cmd, pl=b"", seq=1):
    body = bytes([cmd, seq, len(pl)]) + pl
    c = crc16(body)
    return b"\xaa\x55" + body + bytes([c >> 8, c & 0xFF])


def enc(t):
    o = bytearray()
    for v in t:
        v = max(0, min(4095, int(v)))
        o += bytes([v >> 8, v & 0xFF])
    return bytes(o)


try:
    s = serial.Serial(PORT, 115200, timeout=0.1)
except Exception as e:
    print("打不开 %s: %s  (arm_real.launch.py 还占着？)" % (PORT, e))
    sys.exit(1)
try:
    s.dtr = False
    s.rts = False
except Exception:
    pass
time.sleep(0.2)


def readpos():
    s.reset_input_buffer()
    s.write(fr(0x06))
    s.flush()
    end = time.time() + 1.0
    buf = b""
    while time.time() < end:
        buf += s.read(128)
        i = buf.find(b"\xaa\x55\x84")
        if i >= 0:
            ln = buf[i + 4] if len(buf) > i + 4 else 0
            if len(buf) >= i + 5 + ln + 2:
                pl = buf[i + 5:i + 5 + ln]
                return [(pl[2 * k] << 8) | pl[2 * k + 1] for k in range(6)]
    return None


pos = readpos()
if not pos:
    print("读不到位置：固件没跑/串口被占/舵机没电")
    sys.exit(1)
base = [2047 if v == 0xFFFF else v for v in pos]
idx = SERVO - 1
print("当前位置:", pos)
print("舵机 %d: %d -> %d, %.1fs, %.0fHz, 死区 %d tick"
      % (SERVO, START, END, SECS, RATE, STEP))

s.write(fr(0x03, bytes([1])))          # 进入跟随模式
s.flush()
time.sleep(0.1)

n = max(1, int(SECS * RATE))
t0 = time.time()
last = None
sent = 0
for i in range(n + 1):
    v = int(round(START + (END - START) * i / n))
    if STEP and last is not None and abs(v - last) < STEP:
        v = last
    else:
        last = v
        sent += 1
    t = list(base)
    t[idx] = v
    s.write(fr(0x02, enc(t)))
    dt = t0 + (i + 1) / RATE - time.time()
    if dt > 0:
        time.sleep(dt)

t = list(base)
t[idx] = END
s.write(fr(0x02, enc(t)))
s.flush()
time.sleep(1.0)
print("实发 %d 帧，结束位置: %s" % (sent, readpos()))

s.write(fr(0x03, bytes([0])))          # 回 IDLE
s.flush()
s.close()
print("\n顺滑 -> 节奏没问题，问题在 MoveIt 给的位置（查 direction / 限位）")
print("抖动 -> 再试倒数第 2/3 个参数：20（降频）或 3（死区）")
