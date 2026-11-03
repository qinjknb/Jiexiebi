#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用 ServoHost 的 GUI 本体（ServoHostApp）驱动六个舵机。

不是复刻协议，而是直接实例化 servo_host_gui.ServoHostApp，调它自己的
_on_connect / _on_move / _on_read，并跑 Tk 事件循环让它的 reader 线程和
_poll 正常工作——与手动点按钮完全同一条代码路径。

用法：
    # 六轴全部到 150°
    python tools/servo_host_gui_drive.py --moves "1:150,2:150,3:150,4:150,5:150,6:150"

    # 只读当前位置
    python tools/servo_host_gui_drive.py --read-only

    # 单轴微调
    python tools/servo_host_gui_drive.py --moves "1:155" --time 600
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVO_HOST = ROOT.parent / "ServoHost"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SERVO_HOST))

import tkinter as tk  # noqa: E402

import servo_host_gui as g  # noqa: E402


def pump(root: tk.Tk, seconds: float):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        root.update()
        time.sleep(0.02)


def parse_moves(text: str):
    moves = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        sid, _, angle = part.partition(":")
        moves.append((int(sid), float(angle)))
    return moves


def read_position(app, root, sid: int, range_deg: int, settle: float = 0.9):
    app.id_var.set(str(sid))
    app.range_var.set(str(range_deg))
    app._on_read()
    pump(root, settle)
    match = re.search(r"当前位置:\s*(\d+)", app.pos_var.get())
    if not match:
        return None
    return int(match.group(1))


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser()
    p.add_argument("--port", default="", help="留空=自动找 CH340")
    p.add_argument("--baud", default="115200")
    p.add_argument("--range", type=int, default=300)
    p.add_argument("--moves", default="", help='如 "1:150,2:150,..."')
    p.add_argument("--pose", default="",
                   help="按 config/poses.yaml 里的姿态名（如 scan）下发六个舵机角度")
    p.add_argument("--time", type=int, default=600, help="每条 MOVE 的运行时间 ms")
    p.add_argument("--read-only", action="store_true")
    p.add_argument("--settle", type=float, default=1.0, help="每条指令后等待秒数")
    args = p.parse_args(argv)

    # --pose 转成 --moves
    if args.pose:
        import yaml

        poses_file = ROOT / "config" / "poses.yaml"
        if not poses_file.is_file():
            print(f"[错误] 找不到 {poses_file}")
            return 1
        data = yaml.safe_load(poses_file.read_text(encoding="utf-8")) or {}
        entry = data.get(args.pose)
        if not entry or "servo_deg" not in entry:
            print(f"[错误] poses.yaml 里没有姿态 '{args.pose}'；"
                  f"可用：{', '.join(sorted(data)) or '无'}")
            return 1
        args.moves = ",".join(f"{sid}:{deg}"
                              for sid, deg in sorted(entry["servo_deg"].items()))
        print(f"[姿态] {args.pose} → {args.moves}")

    root = tk.Tk()
    root.withdraw()
    app = g.ServoHostApp(root)

    app.refresh_ports()
    names = list(app.com_box["values"])
    target = None
    if args.port:
        target = next((n for n in names if args.port in n), None)
    if target is None:
        from dofbot_mj.servo_link import detect_port

        want = detect_port()
        if want:
            target = next((n for n in names if want in n), None)
    if target is None and not args.port and names:
        target = names[0]
    if target is None:
        print(f"[错误] 没找到 {args.port or 'CH340'}，可用：{names}")
        root.destroy()
        return 1
    app.com_var.set(target)
    app.baud_var.set(str(args.baud))
    app.mode_var.set(g.MODE_STM32)
    app.range_var.set(str(args.range))
    pump(root, 0.2)

    print(f"[GUI] 连接 {target} @ {args.baud}（STM32 上位模式）")
    app._on_connect()
    pump(root, 1.5)
    if app.ser is None:
        print("[错误] 连接失败")
        root.destroy()
        return 1

    # ---- 读起点 ----
    before = {}
    print("\n起点：")
    for sid in range(1, 7):
        pos = read_position(app, root, sid, args.range)
        before[sid] = pos
        deg = f"{pos * args.range / 4095:.2f}°" if pos is not None else "读取失败"
        print(f"   ID{sid}  {pos if pos is not None else '-':>5}  {deg}")

    code = 0
    if not args.read_only and args.moves:
        moves = parse_moves(args.moves)
        print(f"\n下发 {len(moves)} 条 MOVE（每条 {args.time} ms）：")
        for sid, angle in moves:
            app.id_var.set(str(sid))
            app.angle_entry_var.set(f"{angle:g}")
            app.time_var.set(str(args.time))
            app.range_var.set(str(args.range))
            app._on_move()
            print(f"   ID{sid} → {angle:g}°")
            pump(root, max(0.4, args.time / 1000.0 + 0.3))

        pump(root, max(0.8, args.settle))

        print("\n读回：")
        moved = 0
        for sid, angle in moves:
            pos = read_position(app, root, sid, args.range)
            deg = pos * args.range / 4095 if pos is not None else None
            delta = (deg - before[sid] * args.range / 4095) if (deg is not None and before[sid] is not None) else float("nan")
            ok = deg is not None and abs(deg - angle) <= 1.5
            moved += ok
            print(f"   ID{sid}  起点 {before[sid]}  →  {pos if pos is not None else '-'}"
                  f"   ({deg if deg is None else round(deg, 2)}°)  变化 {delta:+.2f}°  "
                  f"{'到位' if ok else '未到位'}")
        print(f"\n[结果] {moved}/{len(moves)} 个到位")
        code = 0 if moved == len(moves) else 1

    # 让 _poll 不再重排，避免 destroy 后报 invalid command name
    app._poll = lambda: None
    app._on_close()
    pump(root, 0.3)
    try:
        root.destroy()
    except Exception:
        pass
    return code


if __name__ == "__main__":
    raise SystemExit(main())
