#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小 demo：MuJoCo 与实物舵机同步做一段缓慢的往返动作。

特点：
  * 驱动走 ServoHost 的 GUI 本体（ServoHostApp），与手点按钮同一条代码路径；
  * 仿真用「指令角」换算关节角，不反复读串口，所以画面是流畅的；
  * 幅度小、速度慢，几个轴同时动，适合第一次看联动效果。

用法：
    python tools/demo_sync.py                      # 默认 2 轮
    python tools/demo_sync.py --rounds 1 --speed 1.5
    python tools/demo_sync.py --port COM9 --no-view   # 不开窗口，只看命令
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SERVO_HOST = ROOT.parent / "ServoHost"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(SERVO_HOST))

import tkinter as tk  # noqa: E402

import mujoco  # noqa: E402

import servo_host_gui as g  # noqa: E402
from dofbot_mj.config import load_config  # noqa: E402
from dofbot_mj.model import RobotModel  # noqa: E402
from dofbot_mj.protocol import ServoMapping  # noqa: E402

# 从起始姿态出发的增量（度）：底座转、腕滚、夹爪开
DELTA = {1: 18.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: -18.0, 6: -26.0}


def read_all(gui, root, range_deg: int) -> dict:
    """逐轴读当前位置（走 GUI 自己的 _on_read）。"""
    import re

    result = {}
    for sid in range(1, 7):
        gui.id_var.set(str(sid))
        gui.range_var.set(str(range_deg))
        gui.pos_var.set("读取中")          # 清掉上一个轴的旧值，否则会串轴
        gui._on_read()
        end = time.monotonic() + 0.8
        value = None
        while time.monotonic() < end:
            root.update()
            time.sleep(0.02)
            match = re.search(r"当前位置:\s*(\d+)", gui.pos_var.get())
            if match:
                value = int(match.group(1)) * range_deg / 4095.0
                break
        result[sid] = value if value is not None else 150.0
    return result


def pump(root: tk.Tk):
    root.update()


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="MuJoCo + 实物舵机小 demo")
    p.add_argument("--config", default=str(ROOT / "config" / "calibrated.yaml"))
    p.add_argument("--port", default="COM9")
    p.add_argument("--baud", default="115200")
    p.add_argument("--rounds", type=int, default=2)
    p.add_argument("--leg", type=float, default=2.5, help="单程秒数")
    p.add_argument("--speed", type=float, default=1.0, help="整体速度倍率")
    p.add_argument("--hz", type=float, default=20.0, help="仿真刷新率")
    p.add_argument("--no-view", action="store_true", help="不开 MuJoCo 窗口")
    args = p.parse_args(argv)

    settings = load_config(args.config)
    mapping = ServoMapping(settings.servo.joints, settings.servo.range_deg)
    robot = RobotModel(settings.model.mjcf,
                       mimic=_load_mimic(settings.model.mjcf))
    print(f"[模型] {settings.model.mjcf.name}  {robot.summary()}")

    # ---- 连 ServoHost GUI ----
    root = tk.Tk()
    root.withdraw()
    gui = g.ServoHostApp(root)
    gui.refresh_ports()
    name = next((n for n in gui.com_box["values"] if args.port in n), None)
    if name is None:
        print(f"[错误] 没找到 {args.port}")
        root.destroy()
        return 1
    gui.com_var.set(name)
    gui.baud_var.set(args.baud)
    gui.mode_var.set(g.MODE_STM32)
    gui.range_var.set(str(settings.servo.range_deg))
    pump(root)
    gui._on_connect()
    for _ in range(40):
        pump(root)
        time.sleep(0.02)
    if gui.ser is None:
        print("[错误] 串口连接失败")
        root.destroy()
        return 1
    print(f"[舵机] {name} 已连接（{settings.servo.range_deg}° 满行程）")

    # ---- 起点用实测位置，避免一上来就跳 ----
    pose_a = read_all(gui, root, settings.servo.range_deg)
    rng = float(settings.servo.range_deg)
    pose_b = {sid: min(rng, max(0.0, pose_a[sid] + DELTA.get(sid, 0.0))) for sid in pose_a}
    print("[起点] " + " ".join(f"ID{s}:{d:6.1f}°" for s, d in sorted(pose_a.items())))
    print("[峰值] " + " ".join(f"ID{s}:{d:6.1f}°" for s, d in sorted(pose_b.items())))

    # ---- 轨迹 ----
    dt = 1.0 / args.hz
    n_seg = max(2, int(round(args.leg / args.speed / dt)))
    timeline = []
    for rnd in range(args.rounds):
        for src, dst in ((pose_a, pose_b), (pose_b, pose_a)):
            for i in range(1, n_seg + 1):
                k = 0.5 * (1.0 - math.cos(math.pi * i / n_seg))  # 余弦缓入缓出
                timeline.append({sid: src[sid] + (dst[sid] - src[sid]) * k
                                 for sid in src})
    total = len(timeline) * dt
    print(f"[轨迹] {args.rounds} 轮 × 2 段，共 {len(timeline)} 帧 / {total:.1f} 秒")

    def apply_to_sim(angles: dict):
        rad = mapping.positions_from_targets(angles)
        robot.apply_kinematic(rad)

    sent: dict[int, float] = {}
    moved = 0
    start = time.monotonic()
    handle = None
    context = None
    try:
        if not args.no_view:
            from mujoco import viewer as mj_viewer

            context = mj_viewer.launch_passive(robot.model, robot.data)
            handle = context.__enter__()
            handle.cam.lookat[:] = [0.0, 0.0, 0.20]
            handle.cam.distance = 0.72
            handle.cam.azimuth = 135.0
            handle.cam.elevation = -18.0

        last_send = 0.0
        for index, angles in enumerate(timeline):
            now = time.monotonic()
            # 仿真：每帧都更新
            apply_to_sim(angles)
            if handle is not None:
                if not handle.is_running():
                    print("[窗口] 已关闭，提前结束")
                    break
                handle.sync()

            # 实物：按死区 + 限速下发
            if now - last_send >= 0.12:
                pending = [(sid, deg) for sid, deg in angles.items()
                           if sid not in sent or abs(deg - sent[sid]) >= 1.0]
                for sid, deg in pending:
                    gui.id_var.set(str(sid))
                    gui.angle_entry_var.set(f"{deg:.1f}")
                    gui.time_var.set("180")
                    gui.range_var.set(str(settings.servo.range_deg))
                    gui._on_move()
                    sent[sid] = deg
                    moved += 1
                if pending:
                    last_send = now

            pump(root)
            remain = (index + 1) * dt - (time.monotonic() - start)
            if remain > 0:
                time.sleep(remain)
            if index % max(1, int(2 / dt)) == 0:
                print(f"    {100.0 * index / len(timeline):5.1f}%  " +
                      " ".join(f"ID{s}:{d:5.1f}" for s, d in sorted(angles.items())))

        # 收尾：末帧可能被限速吞掉，这里确保所有轴回到起点
        apply_to_sim(pose_a)
        if handle is not None:
            handle.sync()
        for sid, deg in pose_a.items():
            gui.id_var.set(str(sid))
            gui.angle_entry_var.set(f"{deg:.1f}")
            gui.time_var.set("400")
            gui.range_var.set(str(settings.servo.range_deg))
            gui._on_move()
            pump(root)
            moved += 1
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            pump(root)
            time.sleep(0.02)
        print("[收尾] 已回到起点姿态")
    except KeyboardInterrupt:
        print("\n[中断] Ctrl+C")
    finally:
        if handle is not None:
            context.__exit__(None, None, None)
        gui._poll = lambda: None
        gui._on_close()
        try:
            root.destroy()
        except Exception:
            pass
    print(f"[完成] 下发 {moved} 条 MOVE，用时 {time.monotonic() - start:.1f} s")
    return 0


def _load_mimic(mjcf_path: Path) -> dict:
    sidecar = mjcf_path.with_suffix(".json")
    if not sidecar.is_file():
        return {}
    import json

    return (json.loads(sidecar.read_text(encoding="utf-8")) or {}).get("mimic", {})


if __name__ == "__main__":
    raise SystemExit(main())
