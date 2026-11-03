#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 DOFBOT Pro · Windows + MuJoCo 双/多关节循环（替代 Ubuntu 上的 ROS 2 + MoveIt）
============================================================================
 一条命令跑起来：

   python run.py                          # MuJoCo 窗口 + 循环 + 舵机试运行(只打印)
   python run.py --servo off              # 纯仿真，完全不碰串口
   python run.py --servo live --calibration-confirmed
                                          # 仿真 ↔ 实物同步（COM9）
   python run.py --headless --speed 4     # 无窗口，4 倍速跑一遍自检
   python run.py --pose init --render out/init.png   # 只看某个姿态并出图

 常用参数：
   --config PATH          换一份配置（默认 config/default.yaml）
   --model PATH           换一个 MJCF（默认取配置里的 models/dofbot_pro.xml）
   --repeat N             覆盖重复轮数；0 = 一直跑
   --segment-seconds SEC  覆盖每段运动时长
   --speed X              播放速度倍率；0 = 不限速（配合 --headless）
   --headless             不开窗口（自检 / 服务器）
   --servo {auto,dry,live,off}
   --port COM9            串口名，留空自动找 CH340
   --list-poses / --list-joints   打印 SRDF 姿态 / 模型关节
============================================================================
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from dofbot_mj.config import load_config  # noqa: E402
from dofbot_mj.poses import parse_srdf  # noqa: E402


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="DOFBOT Pro MuJoCo 循环播放器（Windows，不依赖 ROS）")
    parser.add_argument("--config", default=str(ROOT / "config" / "default.yaml"))
    parser.add_argument("--model", default="", help="覆盖 MJCF 路径")
    parser.add_argument("--repeat", type=int, default=None, help="重复轮数，0=一直跑")
    parser.add_argument("--segment-seconds", type=float, default=None,
                        help="覆盖每段运动时长")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="播放速度倍率；0=不限速")
    parser.add_argument("--headless", action="store_true", help="不开 MuJoCo 窗口")
    parser.add_argument("--servo", choices=("auto", "dry", "live", "off"), default="auto",
                        help="auto=按配置；dry=只打印；live=真开串口；off=禁用")
    parser.add_argument("--port", default="", help="串口名，如 COM9；留空自动找 CH340")
    parser.add_argument("--calibration-confirmed", action="store_true",
                        help="确认六轴方向/零点已标定，允许实机运动")
    parser.add_argument("--pose", default="", help="只显示某个命名姿态并退出")
    parser.add_argument("--render", default="", help="配合 --pose 输出 PNG")
    parser.add_argument("--list-poses", action="store_true")
    parser.add_argument("--list-joints", action="store_true")
    parser.add_argument("--mode", choices=("kinematic", "physics"), default="",
                        help="覆盖控制模式")
    parser.add_argument("--start-from-servo", action="store_true",
                        help="先把仿真起点设成实机实测姿态（只读探测，不发运动指令）")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass
    args = parse_args(argv)

    settings = load_config(args.config)
    if args.model:
        settings.model.mjcf = Path(args.model).expanduser().resolve()
    if args.mode:
        settings.control.mode = args.mode
    if args.repeat is not None:
        settings.cycle.repeat = args.repeat
    if args.segment_seconds is not None:
        settings.cycle.segment_seconds = args.segment_seconds
    if args.port:
        settings.servo.port = args.port
    if args.calibration_confirmed:
        settings.servo.calibration_confirmed = True

    if args.servo == "off":
        settings.servo.enabled = False
    elif args.servo == "dry":
        settings.servo.enabled = True
        settings.servo.test_mode = True
    elif args.servo == "live":
        # 只表示"真的要开串口"，不代替标定确认——确认必须另外给
        # --calibration-confirmed 或改 config，避免手滑一次就驱动真机。
        settings.servo.enabled = True
        settings.servo.test_mode = False

    # --list-* 不需要加载 MuJoCo，先处理
    if args.list_poses:
        srdf = settings.model.srdf
        if not srdf.is_file():
            print(f"找不到 SRDF：{srdf}")
            return 2
        parsed = parse_srdf(srdf)
        print(f"SRDF: {srdf}")
        for name, values in parsed["states"].items():
            text = ", ".join(f"{j}={v:+.3f}" for j, v in values.items())
            print(f"  {name:<8} {text}")
        return 0

    if not settings.model.mjcf.is_file():
        print(f"找不到模型：{settings.model.mjcf}\n"
              f"先生成：python -m dofbot_mj.urdf2mjcf "
              f"../src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf -o models/dofbot_pro.xml")
        return 2

    from dofbot_mj.runner import SimApp

    initial = None
    if args.start_from_servo:
        from dofbot_mj.servo_link import probe_positions

        print(f"[探测] 只读读取 {settings.servo.port or '自动'} 上六个舵机的位置"
              f"（不发任何运动指令）")
        try:
            initial, _degrees = probe_positions(settings.servo, log=print)
        except Exception as exc:
            print(f"[错误] 探测失败：{type(exc).__name__}: {exc}")
            return 1
        print("[探测] 仿真将从实机当前姿态起步，两边走同样的角度差")

    app = SimApp(settings, speed=args.speed, initial=initial)

    if args.list_joints:
        print(f"模型: {settings.model.mjcf}")
        print(f"  {app.robot.summary()}")
        for name in app.robot.driver_joints:
            lo, hi = app.robot.limits(name)
            print(f"  {name:<14} [{lo:+.4f}, {hi:+.4f}] rad")
        for name, (driver, mult, offset) in app.robot.mimic.items():
            print(f"  {name:<14} mimic {driver} × {mult:+.0f} + {offset:+.0f}")
        return 0

    if args.pose:
        if not args.render:
            print("--pose 需要配合 --render 输出 PNG")
            return 2
        if args.pose == "measured":
            if not initial:
                print("--pose measured 需要同时给 --start-from-servo")
                return 2
            app.render_state(initial, args.render)
        else:
            app.render_pose(args.pose, args.render)
        return 0

    return app.run(repeat=settings.cycle.repeat, headless=args.headless)


if __name__ == "__main__":
    raise SystemExit(main())
