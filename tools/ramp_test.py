#!/usr/bin/env python3
"""斜坡下发测试：按固定频率（默认 50Hz）让某一路舵机从 A 线性走到 B。

用途：实机"动作过程中来回抖/开合"时，用它把两类原因分开——
  * 若本脚本也抖  -> 是"高频小步下发"这种节奏本身的问题（固件/舵机），
                     可用 --rate / --step-min 找到不抖的参数。
  * 若本脚本顺滑  -> 下发节奏没问题，是 MoveIt 给出来的目标位置本身有问题
                     （多见于 direction 反了、限位比机械行程宽，导致顶到端点）。

不依赖 ROS，只需要 pyserial。用法示例（先停掉 arm_real.launch.py 释放串口）：

    # 让 6 号舵机(夹爪)用 3 秒从 1904 走到 1050，50Hz 下发
    python3 ramp_test.py /dev/ttyUSB0 --servo 6 --from 1904 --to 1050 --seconds 3

    # 降频到 20Hz 再试（每次步子更大，看是否不抖了）
    python3 ramp_test.py /dev/ttyUSB0 --servo 6 --from 1904 --to 1050 --seconds 3 --rate 20

    # 加"死区"：目标变化不足 3 tick 就不发（减少重复命令）
    python3 ramp_test.py /dev/ttyUSB0 --servo 6 --from 1904 --to 1050 --seconds 3 --step-min 3

    # 只测一路、其它关节保持读到的初始位置（默认就是这样）
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial：pip3 install pyserial")
    raise SystemExit(2)

BAUD = 115200
CMD_PING = 0x01
CMD_SET_TARGETS = 0x02
CMD_SET_MODE = 0x03
CMD_SET_LIMITS = 0x05
CMD_READ_POS = 0x06
CMD_SET_TELEM = 0x07
CMD_STOP = 0x08
RSP_PONG, RSP_STATE, RSP_ACK, RSP_POS = 0x81, 0x83, 0x82, 0x84
MODE_IDLE, MODE_STREAM = 0, 1
UNKNOWN = 0xFFFF


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd: int, payload: bytes = b"", seq: int = 1) -> bytes:
    body = bytes([cmd, seq & 0xFF, len(payload)]) + payload
    c = crc16(body)
    return bytes([0xAA, 0x55]) + body + bytes([c >> 8, c & 0xFF])


class Link:
    def __init__(self, port: str, baud: int):
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0.15)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass
        self.seq = 0

    def _next(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def send(self, cmd: int, payload: bytes = b"") -> int:
        s = self._next()
        self.ser.write(frame(cmd, payload, s))
        self.ser.flush()
        return s

    def collect(self, wait: float):
        end = time.time() + wait
        buf = b""
        while time.time() < end:
            d = self.ser.read(256)
            if d:
                buf += d
        return buf

    def request(self, cmd: int, payload: bytes, want: int, wait: float = 0.6):
        self.ser.reset_input_buffer()
        s = self.send(cmd, payload)
        end = time.time() + wait
        buf = b""
        while time.time() < end:
            buf += self.ser.read(256)
            # 简单查找 AA 55 want seq
            for i in range(len(buf) - 2):
                if buf[i] == 0xAA and buf[i + 1] == 0x55 and buf[i + 2] == want:
                    return buf[i:]
        return b""

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def enc_targets(ticks) -> bytes:
    out = bytearray()
    for t in ticks:
        t = max(0, min(4095, int(t)))
        out += bytes([(t >> 8) & 0xFF, t & 0xFF])
    return bytes(out)


def dec_targets(payload) -> list:
    vals = []
    for i in range(6):
        vals.append((payload[i * 2] << 8) | payload[i * 2 + 1])
    return vals


def main() -> int:
    ap = argparse.ArgumentParser(description="50Hz 斜坡下发测试（绕过 ROS）")
    ap.add_argument("port", help="串口，如 /dev/ttyUSB0 或 COM7")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--servo", type=int, default=6, choices=range(1, 7),
                    help="要动的舵机 ID（1~6），默认 6=夹爪")
    ap.add_argument("--from", dest="frm", type=int, default=None, help="起始 tick（默认读当前值）")
    ap.add_argument("--to", dest="to", type=int, required=True, help="结束 tick")
    ap.add_argument("--seconds", type=float, default=3.0, help="总时长，默认 3 秒")
    ap.add_argument("--rate", type=float, default=50.0, help="下发频率 Hz，默认 50")
    ap.add_argument("--step-min", type=int, default=0,
                    help="死区：目标变化不足 N tick 就不发（0=每周期都发）")
    ap.add_argument("--back", action="store_true", help="到达后原路返回")
    args = ap.parse_args()

    try:
        link = Link(args.port, args.baud)
    except Exception as exc:
        print("打不开串口 %s：%s" % (args.port, exc))
        print("  · arm_real.launch.py / 上位机是否还占着这个口？")
        print("  · 设备名对不对：ls /dev/ttyUSB*")
        return 1
    try:
        # ---- 握手 ----
        r = link.request(CMD_PING, b"", RSP_PONG, 1.0)
        if not r:
            print("PING 无应答：检查串口/固件/接线（也可能被 arm_real.launch.py 占着）")
            return 1
        print("STM32 在线 ✔")

        # ---- 读当前位置 ----
        r = link.request(CMD_READ_POS, b"", RSP_POS, 1.0)
        if len(r) < 7:
            print("读位置失败")
            return 1
        base = dec_targets(r[5:])
        idx = args.servo - 1
        if base[idx] == UNKNOWN:
            print("舵机 %d 位置读不到（在线吗？）" % args.servo)
            return 1
        start = args.frm if args.frm is not None else base[idx]
        end = args.to
        print("基准位置: %s" % base)
        print("舵机 %d: %d -> %d，%.1f 秒，%.0fHz，死区 %d tick，共约 %d 帧"
              % (args.servo, start, end, args.seconds, args.rate,
                 args.step_min, int(args.seconds * args.rate)))

        # ---- 关遥测、进跟随 ----
        link.request(CMD_SET_TELEM, bytes([0]), RSP_ACK, 0.5)
        link.request(CMD_SET_MODE, bytes([MODE_STREAM]), RSP_ACK, 0.5)

        def ramp(a: int, b: int) -> None:
            n = max(1, int(args.seconds * args.rate))
            period = 1.0 / args.rate
            sent = 0
            last_sent = None
            t0 = time.time()
            for i in range(n + 1):
                frac = i / n
                val = int(round(a + (b - a) * frac))
                if args.step_min and last_sent is not None and abs(val - last_sent) < args.step_min:
                    val = last_sent          # 不足死区就不更新
                else:
                    last_sent = val
                    sent += 1
                ticks = list(base)
                ticks[idx] = val
                link.send(CMD_SET_TARGETS, enc_targets(ticks))
                # 按绝对时间对齐，避免累积误差
                target_t = t0 + (i + 1) * period
                dt = target_t - time.time()
                if dt > 0:
                    time.sleep(dt)
            print("  实发 %d 帧（每帧 %d 字节）" % (sent, len(enc_targets(base)) + 7))

        ramp(start, end)
        time.sleep(0.5)
        link.send(CMD_SET_TARGETS, enc_targets([end if i == idx else base[i] for i in range(6)]))
        time.sleep(0.8)
        if args.back:
            print("原路返回 ...")
            ramp(end, start)
            time.sleep(0.5)

        # ---- 收尾 ----
        time.sleep(0.5)
        r = link.request(CMD_READ_POS, b"", RSP_POS, 1.0)
        if len(r) >= 7:
            print("结束位置: %s" % dec_targets(r[5:]))
        link.request(CMD_SET_MODE, bytes([MODE_IDLE]), RSP_ACK, 0.5)
    finally:
        link.close()

    print("""
结果怎么判：
  顺滑            -> 下发节奏没问题，问题在 MoveIt 给的目标位置（查 direction / tick_min/max）
  仍然抖/来回     -> 是"高频小步下发"这个节奏本身的问题：
                     再试  --rate 20             （降频，每次步子更大）
                        或 --step-min 3          （加死区，减少重复命令）
                     这两个里有一个变顺滑，就说明该往固件里加降频/死区
""")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
