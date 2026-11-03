#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 play_arm.py —— 一键运行机械臂（按仿真的双关节往返路径）
============================================================================
 只需运行本程序，真实机械臂就会按 MoveIt 仿真里的路径运动：
   起点(上电读取的舵机当前位置) → test 目标(-0.9286, -0.5294 rad) → 原路返回
   ... 循环 repeat 轮后停在起点。

 运行时不依赖 ROS / MoveIt，只需要 Python3 + pyserial（串口模式）。
 Ubuntu / Windows 均可运行。

 用法：
   python3 play_arm.py                       # 自动找串口，按内置路径跑 3 轮
   python3 play_arm.py --port /dev/ttyUSB0   # 指定串口（Windows 用 --port COM9）
   python3 play_arm.py --path 路径.json      # 回放用 record_path.py 录制的仿真路径
   python3 play_arm.py --dry-run             # 无硬件演练：只打印要发的命令
   python3 play_arm.py --repeat 1 --segment-time 4.0   # 1 轮、单程 4 秒
   python3 play_arm.py --joints 1            # 只动第 1 个关节（1 个舵机时）

 参数速查（常用）：
   --port        串口设备；不填自动找 CH340
   --baud        波特率，默认 115200
   --repeat      往返轮数，默认 3；回放文件模式默认 1
   --segment-time  去程/回程单段时长(秒)，默认 6.0（对应仿真 speed≈0.1 的节奏）
   --pause       端点停顿(秒)，默认 0.5
   --goal1/--goal2  目标关节角(rad)，默认仿真 test: -0.9286 / -0.5294
   --home1/--home2  起点舵机角度(°)，默认 150.0；串口模式下会先用 READ
                    读实际位置，读不到才用该值
   --offset1/--offset2  角度映射零点(°)，默认 150.0
   --scale1/--scale2    rad→° 比例，默认 57.2958（180/π）
   --dir1/--dir2        方向 ±1，默认 1
   --range       满行程角度，默认 300
   --dt-ms       指令间隔(ms)，默认 50（20 Hz）
   --timescale   时间缩放，默认 1.0（0.05 = 快 20 倍，仅供演练）

 映射公式（与 dofbot_servo_bridge 一致）：
   舵机角度(°) = offset + direction × 关节角(rad) × scale
 起点角度(°)=150 时对应 0 rad；目标 -0.9286 rad 对应约 96.8°（行程内）。
============================================================================
"""

import argparse
import json
import math
import os
import sys
import time

# 关节配置：sim 关节名 → (舵机 ID, 参数前缀)
JOINTS = [("Arm1_Joint", 1), ("Arm2_Joint", 2)]


# ------------------------------ 路径生成 -----------------------------------
def cosine_vector(start, goal, seg_ms, dt_ms):
    """余弦缓入缓出轨迹：两端速度为零、中间平滑，近似 MoveIt 平滑规划。"""
    n = max(2, int(round(seg_ms / dt_ms)))
    points = []
    for i in range(n + 1):
        u = i / float(n)
        k = (1.0 - math.cos(math.pi * u)) / 2.0
        points.append([a + (b - a) * k for a, b in zip(start, goal)])
    return points  # 含起点与终点，间隔 dt_ms


def build_builtin_cycle(start_rad, goal_rad, seg_ms, pause_ms, dt_ms, repeat):
    """内置路径：起点 → goal → 起点，repeat 轮。返回 [{'t_ms','rad'}]。"""
    waypoints = []
    t_ms = 0

    def add(rad):
        nonlocal t_ms
        waypoints.append({"t_ms": t_ms, "rad": [round(v, 6) for v in rad]})
        t_ms += dt_ms

    add(start_rad)
    for _ in range(repeat):
        for p in cosine_vector(start_rad, goal_rad, seg_ms, dt_ms)[1:]:
            add(p)
        t_ms += int(round(pause_ms * 1000))
        for p in cosine_vector(goal_rad, start_rad, seg_ms, dt_ms)[1:]:
            add(p)
        t_ms += int(round(pause_ms * 1000))
    return waypoints


def load_path_file(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    wps = data["waypoints"]
    names = data.get("joints", ["Arm1_Joint", "Arm2_Joint"])
    expected = [name for name, _ in JOINTS]
    if set(names) != set(expected):
        raise ValueError(f"路径文件的关节 {names} 与播放器期望 {expected} 不一致")
    if names != expected:
        # 关节顺序不同时按名字重排，保证 rad 顺序与舵机 ID 对应
        index = [names.index(n) for n in expected]
        wps = [{"t_ms": wp["t_ms"], "rad": [wp["rad"][i] for i in index]}
               for wp in wps]
    return wps


def resample(wps, dt_ms):
    """按固定间隔 dt_ms 线性重采样（文件录制频率与播放频率解耦）。"""
    if len(wps) < 2:
        return wps
    out = [wps[0]]
    t = dt_ms
    i = 1
    while t < wps[-1]["t_ms"] and i < len(wps):
        while i < len(wps) and wps[i]["t_ms"] < t:
            i += 1
        if i >= len(wps):
            break
        a, b = wps[i - 1], wps[i]
        span = b["t_ms"] - a["t_ms"]
        if span <= 0:
            i += 1
            continue
        frac = (t - a["t_ms"]) / float(span)
        rad = [a["rad"][j] + frac * (b["rad"][j] - a["rad"][j]) for j in range(len(a["rad"]))]
        out.append({"t_ms": t, "rad": [round(v, 6) for v in rad]})
        t += dt_ms
    out.append(wps[-1])
    return out


# ------------------------------ 串口工具 -----------------------------------
def detect_port():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for p in ports:
        if "CH340" in (p.description or "").upper():
            return p.device
    return ports[0].device


class SerialLineReader:
    """按行读取串口应答（READ 用），带字节缓冲。"""

    def __init__(self, ser):
        self.ser = ser
        self.buf = ""

    def read_line(self, timeout=0.6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n > 0 else 1)
            except OSError:
                return None
            if data:
                self.buf += data.decode("ascii", errors="replace")
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                line = line.strip("\r").strip()
                if line:
                    return line
        return None


def open_serial(port, baud):
    import serial  # 延迟导入：--dry-run 无需 pyserial
    for attempt in range(5):
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
            return ser
        except (serial.SerialException, OSError) as exc:
            print(f"[等待串口] 第 {attempt + 1} 次打开 {port} 失败: {exc}")
            time.sleep(2.0)
    raise SystemExit(1)


def drain_rx(ser, verbose):
    """读走 STM32 应答，避免缓冲堆积；verbose 时打印。"""
    lines = []
    try:
        for _ in range(4):
            n = ser.in_waiting
            data = ser.read(n if n > 0 else 1)
            if not data:
                break
            lines.extend(data.decode("ascii", errors="replace").splitlines())
    except OSError:
        pass
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if verbose or "READY" in line.upper():
            print(f"[RX] {line}")
        elif line.upper().startswith("ERR"):
            print(f"[WARN] {line}")


def read_servo_position(ser, reader, sid, range_deg, retries=8, verbose=False):
    """READ 舵机当前位置，返回角度(°)，失败返回 None。"""
    for attempt in range(retries):
        ser.write(f"READ {sid}\r\n".encode("ascii"))
        line = reader.read_line(timeout=0.6)
        if line and line.upper().startswith(f"POS {sid} "):
            parts = line.split()
            try:
                pos = int(parts[2])
                return pos * range_deg / 4095.0
            except (ValueError, IndexError):
                pass
        if line and line.upper().startswith("ERR"):
            if verbose:
                print(f"[WARN] {line}（重试 {attempt + 1}/{retries}）")
        time.sleep(0.25)
    return None


# ------------------------------ 主流程 -------------------------------------
def parse_args(argv):
    p = argparse.ArgumentParser(
        description="一键运行机械臂：按仿真的双关节往返路径运动（经 STM32 驱动德晟舵机）")
    p.add_argument("--port", default="", help="串口设备（不填自动找 CH340）")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--path", default="", help="回放的路径 JSON（record_path.py 录制）")
    p.add_argument("--repeat", type=int, default=None,
                   help="往返轮数；内置路径默认 3，回放文件默认 1")
    p.add_argument("--segment-time", type=float, default=6.0,
                   help="去程/回程单段时长(秒)，默认 6.0")
    p.add_argument("--pause", type=float, default=0.5, help="端点停顿(秒)")
    p.add_argument("--goal1", type=float, default=-0.9286, help="Arm1 目标角(rad)")
    p.add_argument("--goal2", type=float, default=-0.5294, help="Arm2 目标角(rad)")
    p.add_argument("--home1", type=float, default=150.0, help="Arm1 起点舵机角度(°)")
    p.add_argument("--home2", type=float, default=150.0, help="Arm2 起点舵机角度(°)")
    p.add_argument("--offset1", type=float, default=150.0)
    p.add_argument("--offset2", type=float, default=150.0)
    p.add_argument("--scale1", type=float, default=57.295779513)
    p.add_argument("--scale2", type=float, default=57.295779513)
    p.add_argument("--dir1", type=float, default=1.0)
    p.add_argument("--dir2", type=float, default=1.0)
    p.add_argument("--range", type=float, default=300.0, help="满行程角度")
    p.add_argument("--dt-ms", type=int, default=50, help="指令间隔(ms)，默认 50")
    p.add_argument("--joints", type=int, default=2, choices=(1, 2), help="驱动的关节数")
    p.add_argument("--timescale", type=float, default=1.0, help="时间缩放(演练用)")
    p.add_argument("--dry-run", action="store_true", help="无硬件演练：只打印命令")
    p.add_argument("--verbose", action="store_true", help="打印所有串口应答")
    return p.parse_args(argv)


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass
    args = parse_args(argv)

    joints = JOINTS[:args.joints]
    ids = [sid for _, sid in joints]
    offsets = [args.offset1, args.offset2]
    scales = [args.scale1, args.scale2]
    dirs = [args.dir1, args.dir2]
    homes = [args.home1, args.home2]
    goal_rad = [args.goal1, args.goal2]
    repeat = args.repeat if args.repeat is not None else (1 if args.path else 3)
    if args.segment_time <= 0 or args.pause < 0 or args.dt_ms <= 0 or args.timescale <= 0:
        raise SystemExit("--segment-time/--pause/--dt-ms/--timescale 参数非法")
    if not 0 < args.range <= 360:
        raise SystemExit("--range 必须在 1~360")

    # ---- 串口连接（dry-run 跳过）----
    ser = None
    if not args.dry_run:
        port = args.port or detect_port()
        if not port:
            raise SystemExit("没有找到可用串口：请用 --port COM9 或 --port /dev/ttyUSB0 指定")
        print(f"[连接] 打开串口 {port} @ {args.baud}")
        ser = open_serial(port, args.baud)
        time.sleep(0.3)
        ser.write(f"RANGE {int(args.range)}\r\n".encode("ascii"))
        print(f"[连接] 已发送 RANGE {int(args.range)}（满行程同步）")
        reader = SerialLineReader(ser)
        line = reader.read_line(timeout=2.0)
        if line and "READY" in line.upper():
            print(f"[连接] 下位机就绪：{line}")
    else:
        reader = None
        print("[DRY-RUN] 无硬件演练模式：只打印将发送的命令")

    # ---- 组装路径 ----
    if args.path:
        wps = resample(load_path_file(args.path), args.dt_ms)
        print(f"[路径] 回放文件 {args.path}：{len(wps)} 个路点，"
              f"总时长 {wps[-1]['t_ms'] / 1000.0:.1f} 秒，重复 {repeat} 轮")
        if repeat > 1:
            base = list(wps)
            for _ in range(repeat - 1):
                offset = wps[-1]["t_ms"] + args.dt_ms
                wps += [{"t_ms": wp["t_ms"] + offset, "rad": wp["rad"]} for wp in base]
    else:
        start_rad = []
        for j, (name, sid) in enumerate(joints):
            deg = None
            if ser is not None:
                deg = read_servo_position(ser, reader, sid, args.range, verbose=args.verbose)
                if deg is None:
                    print(f"[WARN] 读不到舵机 ID={sid} 当前位置，改用 --home{j + 1}={homes[j]}°")
                else:
                    print(f"[起点] 舵机 ID={sid} 当前位置 {deg:.1f}°")
            if deg is None:
                deg = homes[j]
            start_rad.append((deg - offsets[j]) / (dirs[j] * scales[j]))
        start_rad = [round(v, 6) for v in start_rad]
        seg_ms = int(round(args.segment_time * 1000))
        pause_ms = args.pause
        wps = build_builtin_cycle(start_rad, goal_rad, seg_ms, pause_ms, args.dt_ms, repeat)
        print(f"[路径] 内置仿真路径：起点 {start_rad} rad → 目标 {goal_rad} rad → 返回，"
              f"共 {repeat} 轮，总时长 {wps[-1]['t_ms'] / 1000.0:.1f} 秒")

    def map_deg(rad, off, sc, d):
        return max(0.0, min(args.range, off + d * rad * sc))

    # ---- 播放 ----
    print(f"[播放] 开始（{len(wps)} 条指令，按 Ctrl+C 停止）")
    start_mono = time.monotonic()
    next_at = start_mono
    last_report = start_mono
    total_ms = wps[-1]["t_ms"]
    sent = 0
    try:
        for i, wp in enumerate(wps):
            dt_ms = wp["t_ms"] - wps[i - 1]["t_ms"] if i else args.dt_ms
            move_ms = max(1, int(round(dt_ms * args.timescale)))
            for j, (name, sid) in enumerate(joints):
                deg = map_deg(wp["rad"][j], offsets[j], scales[j], dirs[j])
                line = f"MOVE {sid} {deg:.2f} {move_ms}"
                if ser is not None:
                    ser.write((line + "\r\n").encode("ascii"))
                    print(f"[TX] t={wp['t_ms'] / 1000.0:7.2f}s  {line}")
                else:
                    print(f"[DRY] t={wp['t_ms'] / 1000.0:7.2f}s  {line}")
                sent += 1
            if ser is not None:
                drain_rx(ser, args.verbose)
            # 按时间轴推进
            next_at += (dt_ms * args.timescale) / 1000.0
            now = time.monotonic()
            if now < next_at:
                time.sleep(next_at - now)
            # 每秒进度
            if time.monotonic() - last_report >= 1.0:
                last_report = time.monotonic()
                pct = 100.0 * wp["t_ms"] / max(1, total_ms)
                print(f"[进度] {pct:5.1f}%  (t={wp['t_ms'] / 1000.0:.1f}s/{total_ms / 1000.0:.1f}s)")
    except KeyboardInterrupt:
        print("\n[停止] 收到 Ctrl+C，已停止发送（舵机保持当前位置）")
    else:
        print(f"[完成] 路径播放完毕，共发送 {sent} 条 MOVE 指令")
    finally:
        if ser is not None:
            ser.close()
            print("[连接] 串口已关闭")


if __name__ == "__main__":
    main()
