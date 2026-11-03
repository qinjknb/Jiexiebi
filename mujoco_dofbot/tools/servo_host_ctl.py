#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 ServoHost 的代码控制实物舵机（无界面版）。

直接 import ServoHost/servo_host_gui.py，复用它的：
  * 德晟 v4.01 组帧函数 frame_move / frame_read_pos / frame_ping / frame_reset / frame_set_id
  * 文本命令格式（\\r\\n 结尾）与应答分类规则
  * 满行程 300°、位置=角度×4095÷行程 的换算

比 dofbot_mj 里那套多出来的能力：
  * MOVEP  —— 直接按位置码 0~4095 驱动（绕开固件的角度换算，用来判定
              "是舵机不动还是 MOVE 这条路径有问题"）
  * RESET  —— 复位舵机（可能清掉输出级保护；注意也可能把 ID 恢复出厂）
  * DEMO   —— 让固件自己摆动 ID1，用来单独验证舵机本体是否还能转

用法：
    python tools/servo_host_ctl.py ping
    python tools/servo_host_ctl.py read
    python tools/servo_host_ctl.py move  --id 1 --angle 150 --time 400
    python tools/servo_host_ctl.py movep --id 1 --pos 2100 --time 400
    python tools/servo_host_ctl.py demo --watch 5
    python tools/servo_host_ctl.py raw "MOVE 1 150 400"
    python tools/servo_host_ctl.py reset --id 1        # 慎用
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVO_HOST = ROOT.parent / "ServoHost"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SERVO_HOST))

import servo_host_gui as host  # noqa: E402  复用 ServoHost 的协议实现


class HostLink:
    """把 ServoHost 的串口逻辑抽出来做成同步调用（去掉 tkinter 依赖）。"""

    def __init__(self, port: str, baud: int, range_deg: int = 300,
                 log=print, terminator: str = "\r\n"):
        self.port = port
        self.baud = baud
        self.range_deg = range_deg
        self.log = log
        self.terminator = terminator
        self.ser = None
        self._buf = ""

    # ------------------------------ 连接 ------------------------------
    def open(self):
        import serial

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        # 关键：CH340 重开端口时缓冲区里会残留上一次没读完的应答，
        # 不清掉会导致后续请求/应答错位（表现为"回了 OK 但舵机不动"）。
        self.ser.reset_input_buffer()
        time.sleep(0.2)
        self.log(f"[连接] {self.port} @ {self.baud}  满行程 {self.range_deg}°")
        self.send_text(f"RANGE {self.range_deg}")
        line = self.read_line(timeout=2.0)
        if line:
            self.log(f"[RX] {line}")
            if "READY" in line.upper():
                self.log("[连接] 收到下位机问候，链路正常")
        return self.ser.is_open

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ------------------------------ 收发 ------------------------------
    def send_text(self, text: str):
        data = (text + self.terminator).encode("ascii")
        self.ser.write(data)
        self.log(f"[TX] {text}")

    def read_line(self, timeout: float = 1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            data = self.ser.read(waiting if waiting else 1)
            if data:
                self._buf += data.decode("ascii", errors="replace")
                while "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    line = line.strip("\r").strip()
                    if line:
                        return line
        return None

    def request(self, command: str, timeout: float = 2.0):
        """发一条命令并等第一条应答（ServoHost 也是拿到就记录，不强制配对）。"""
        self.send_text(command)
        line = self.read_line(timeout=timeout)
        if line:
            self.log(f"[RX] {line}")
        else:
            self.log("[RX] (超时，无应答)")
        return line

    # ------------------------------ 查询 ------------------------------
    def ping(self, sid: int):
        return self.request(f"PING {sid}", timeout=2.0)

    def read_position(self, sid: int, retries: int = 3):
        for _ in range(retries):
            line = self.request(f"READ {sid}", timeout=1.5)
            if line and line.upper().startswith(f"POS {sid} "):
                try:
                    return int(line.split()[2])
                except (IndexError, ValueError):
                    continue
        return None

    def angle_of(self, pos: int) -> float:
        return pos * self.range_deg / 4095.0


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="用 ServoHost 的代码控制实物舵机")
    p.add_argument("--port", default="COM9")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--range", type=int, default=300, dest="range_deg")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")
    sub.add_parser("read")
    sub.add_parser("help")

    m = sub.add_parser("move")
    m.add_argument("--id", type=int, required=True)
    m.add_argument("--angle", type=float, required=True)
    m.add_argument("--time", type=int, default=400)

    mp = sub.add_parser("movep")
    mp.add_argument("--id", type=int, required=True)
    mp.add_argument("--pos", type=int, required=True)
    mp.add_argument("--time", type=int, default=400)

    r = sub.add_parser("reset")
    r.add_argument("--id", type=int, required=True)

    d = sub.add_parser("demo")
    d.add_argument("--watch", type=float, default=5.0)
    d.add_argument("--id", type=int, default=1)

    raw = sub.add_parser("raw")
    raw.add_argument("text")

    sc = sub.add_parser("scan", help="逐轴小幅度试走，判定哪几路舵机还能动")
    sc.add_argument("--step", type=float, default=5.0, help="每个轴试走的角度(°)")
    sc.add_argument("--time", type=int, default=500)
    sc.add_argument("--settle", type=float, default=1.0)

    args = p.parse_args(argv)

    link = HostLink(args.port, args.baud, args.range_deg)
    try:
        link.open()
    except Exception as exc:
        print(f"[错误] 打开串口失败：{type(exc).__name__}: {exc}")
        return 1

    code = 0
    try:
        if args.cmd == "help":
            print("固件帮助：")
            for line in ("HELP",):
                link.request(line, timeout=2.0)
                for _ in range(2):
                    extra = link.read_line(timeout=0.5)
                    if extra:
                        print("   ", extra)

        elif args.cmd == "ping":
            print(f"\n  {'ID':>3}  {'应答':<20}{'位置码':>8}{'角度°':>9}")
            for sid in range(1, 7):
                reply = link.ping(sid) or "-"
                pos = link.read_position(sid, retries=2)
                deg = f"{link.angle_of(pos):.2f}" if pos is not None else "-"
                print(f"  {sid:>3}  {reply:<20}{(pos if pos is not None else '-'):>8}{deg:>9}")

        elif args.cmd == "read":
            print(f"\n  {'ID':>3}{'位置码':>9}{'角度°':>10}")
            for sid in range(1, 7):
                pos = link.read_position(sid)
                if pos is None:
                    print(f"  {sid:>3}{'读取失败':>9}")
                else:
                    print(f"  {sid:>3}{pos:>9}{link.angle_of(pos):>10.2f}")

        elif args.cmd in ("move", "movep"):
            before = link.read_position(args.id)
            if args.cmd == "move":
                link.request(f"MOVE {args.id} {args.angle} {args.time}", timeout=2.5)
                target_deg = args.angle
            else:
                link.request(f"MOVEP {args.id} {args.pos} {args.time}", timeout=2.5)
                target_deg = link.angle_of(args.pos)
            time.sleep(max(0.8, args.time / 1000.0 + 0.5))
            after = link.read_position(args.id)
            if before is None or after is None:
                print("[结果] 读位置失败，无法判定")
                code = 1
            else:
                delta = link.angle_of(after) - link.angle_of(before)
                print(f"\n[结果] ID{args.id}: {link.angle_of(before):.2f}° → "
                      f"{link.angle_of(after):.2f}°  变化 {delta:+.2f}°  "
                      f"(目标 {target_deg:.2f}°)")
                print("       舵机已执行" if abs(delta) > 0.4 else "       舵机没动")

        elif args.cmd == "scan":
            step_code = int(round(args.step * 4095.0 / link.range_deg))
            print(f"\n  每轴试走 {args.step:.1f}°（{step_code} 个位置码），"
                  f"跑完自动回原位\n")
            print(f"  {'ID':>3}{'起始°':>9}{'试走后°':>10}{'变化°':>9}  判定")
            alive = []
            for sid in range(1, 7):
                before = link.read_position(sid)
                if before is None:
                    print(f"  {sid:>3}{'读不到':>9}")
                    continue
                target = before + step_code
                if not 0 <= target <= 4095:
                    target = before - step_code
                link.request(f"MOVEP {sid} {target} {args.time}", timeout=2.5)
                time.sleep(args.settle)
                after = link.read_position(sid)
                if after is None:
                    print(f"  {sid:>3}{link.angle_of(before):>9.2f}{'读不到':>10}")
                    continue
                delta = (after - before) * link.range_deg / 4095.0
                moved = abs(delta) >= 0.4
                if moved:
                    alive.append(sid)
                print(f"  {sid:>3}{link.angle_of(before):>9.2f}"
                      f"{link.angle_of(after):>10.2f}{delta:>+9.2f}  "
                      f"{'能动' if moved else '不动 ✗'}")
                link.request(f"MOVEP {sid} {before} {args.time}", timeout=2.5)
                time.sleep(0.4)
            print(f"\n[结果] 能动的轴：{alive if alive else '无'}")
            if len(alive) < 6:
                print("       不动的轴是舵机输出级停转（能 PING/READ 但不出力）：")
                print("       断电重启舵机供电，或带载测一下供电电压是否够（6~8.4V）")

        elif args.cmd == "reset":
            print(f"[注意] 即将复位 ID{args.id}；复位可能把 ID/波特率恢复出厂")
            link.request(f"RESET {args.id}", timeout=2.5)

        elif args.cmd == "demo":
            link.request("DEMO", timeout=2.0)
            print(f"\n[DEMO] 固件自己驱动 ID{args.id} 摆动，观察 {args.watch:.0f}s：")
            seen = []
            for index in range(int(args.watch / 0.3)):
                time.sleep(0.3)
                pos = link.read_position(args.id, retries=2)
                if pos is None:
                    print(f"   t={0.3*(index+1):4.1f}s  读取失败")
                    continue
                seen.append(pos)
                print(f"   t={0.3*(index+1):4.1f}s  {link.angle_of(pos):7.2f}°")
            if seen:
                span = (max(seen) - min(seen)) * link.range_deg / 4095.0
                print(f"\n[DEMO] 采样期间摆动幅度 {span:.2f}°  "
                      f"({'舵机本体能转' if span > 1.0 else '没动，舵机本体/供电有问题'})")
            link.request("PING 1", timeout=1.5)  # 任意命令退出演示模式

        elif args.cmd == "raw":
            link.request(args.text, timeout=2.5)

    finally:
        link.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
