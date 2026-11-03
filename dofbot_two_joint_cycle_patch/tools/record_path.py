#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 record_path.py —— 录制仿真路径（供 play_arm.py 回放）
============================================================================
 在 Ubuntu 上运行仿真循环的同时启动本脚本，把 /joint_states 里的
 Arm1_Joint / Arm2_Joint 关节角按时间轴录成 JSON，之后用
 play_arm.py --path <文件> 即可让真实机械臂复现这条仿真路径。

 用法：
   终端 1：ros2 launch dofbot_two_joint_cycle two_joint_cycle.launch.py repeat_count:=1
   终端 2：python3 record_path.py -o trajectories/sim_path.json
   结束后按 Ctrl+C，路径会保存到 -o 指定的文件。

 说明：
   - 仿真循环约 30 秒后开始动，之前录到的静止点无影响；
   - 录的是实际执行状态（含仿真控制器的跟踪误差），播放器会按 50ms 重采样；
   - 录制文件是纯 JSON，可在任意机器上回放。
============================================================================
"""

import argparse
import json
import math
import sys
import time

JOINT_NAMES = ["Arm1_Joint", "Arm2_Joint"]


def parse_args(argv):
    p = argparse.ArgumentParser(description="录制 ROS2 /joint_states 为 play_arm.py 可回放的路径文件")
    p.add_argument("-o", "--output", default="trajectories/sim_path.json",
                   help="输出 JSON 路径（默认 trajectories/sim_path.json）")
    p.add_argument("--topic", default="/joint_states", help="订阅的话题")
    p.add_argument("--dt-ms", type=int, default=20,
                   help="保存时的重采样间隔(ms)，默认 20")
    return p.parse_args(argv)


def resample(raw, dt_ms):
    """把原始采样线性重采样到固定间隔。"""
    if len(raw) < 2:
        return raw
    out = [raw[0]]
    t = raw[0]["t_ms"] + dt_ms
    i = 1
    while t < raw[-1]["t_ms"] and i < len(raw):
        while i < len(raw) and raw[i]["t_ms"] < t:
            i += 1
        if i >= len(raw):
            break
        a, b = raw[i - 1], raw[i]
        span = b["t_ms"] - a["t_ms"]
        if span <= 0:
            i += 1
            continue
        frac = (t - a["t_ms"]) / float(span)
        rad = [a["rad"][j] + frac * (b["rad"][j] - a["rad"][j])
               for j in range(len(a["rad"]))]
        out.append({"t_ms": int(t), "rad": [round(v, 6) for v in rad]})
        t += dt_ms
    out.append(raw[-1])
    return out


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass
    args = parse_args(argv)

    # ROS 依赖延迟导入：加 --help 或语法检查无需 ROS 环境
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("path_recorder")
    raw = []
    last = [0.0, 0.0]
    t0 = None

    def on_joint_states(msg):
        nonlocal t0
        if t0 is None:
            t0 = time.monotonic()
        t_ms = int(round((time.monotonic() - t0) * 1000))
        by_name = dict(zip(msg.name, msg.position))
        for j, name in enumerate(JOINT_NAMES):
            if name in by_name and math.isfinite(float(by_name[name])):
                last[j] = float(by_name[name])
        raw.append({"t_ms": t_ms, "rad": list(last)})

    node.create_subscription(JointState, args.topic, on_joint_states, 10)
    print(f"[录制] 订阅 {args.topic}，录制中…… 仿真循环结束后按 Ctrl+C 保存到 {args.output}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not raw:
        print("[录制] 一条消息都没收到，未保存文件")
        return 1

    waypoints = resample(raw, args.dt_ms)
    data = {
        "schema": 1,
        "source": f"recorded from {args.topic} by record_path.py",
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "joints": list(JOINT_NAMES),
        "dt_ms": args.dt_ms,
        "waypoints": waypoints,
    }
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    print(f"[录制] 已保存 {len(waypoints)} 个路点，总时长 "
          f"{waypoints[-1]['t_ms'] / 1000.0:.1f} 秒 → {args.output}")
    print(f"[提示] 回放：python3 play_arm.py --path {args.output} --repeat 1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
