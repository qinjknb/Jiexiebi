#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读实机舵机当前角度，在 MuJoCo 里摆出同一姿态，用来核对"仿真像不像实物"。

用法：
    python tools/live_pose.py                      # 只打印对照表 + 限位检查
    python tools/live_pose.py --render out\\p.png   # 另外渲染一张图
    python tools/live_pose.py --view               # 打开交互窗口（可自由旋转查看）

判断依据（脚本会直接算给你）：
  * 关节角是否落在 URDF 限位内 —— 超出说明该轴 offset/direction 没标定；
  * 夹爪指尖在世界系的高度 —— 落到地面以下说明姿态不可能。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

SCALE = 57.295779513


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description="实机舵机角度 → MuJoCo 姿态对照")
    parser.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    parser.add_argument("--render", default="", help="渲染一张 PNG")
    parser.add_argument("--azimuth", type=float, default=180.0,
                        help="渲染观察方位角；180 度接近当前实物照片")
    parser.add_argument("--elevation", type=float, default=-18.0)
    parser.add_argument("--view", action="store_true", help="打开交互窗口保持该姿态")
    parser.add_argument("--follow", action="store_true",
                        help="实时跟随：持续读舵机并刷新窗口（用来直接看仿真与实物的残差）")
    parser.add_argument("--hz", type=float, default=5.0, help="跟随刷新率")
    args = parser.parse_args(argv)

    from dofbot_mj.config import load_config
    from dofbot_mj.runner import SimApp
    from dofbot_mj.servo_link import detect_port, probe_positions

    settings = load_config(args.config)
    print(f"[读舵机] {settings.servo.port} 上六个 ID（只读，不发运动指令）")
    try:
        rad, degrees = probe_positions(settings.servo, log=print)
    except Exception as exc:
        print(f"[错误] 探测失败：{type(exc).__name__}: {exc}")
        return 1
    if not settings.servo.port:
        settings.servo.port = detect_port()

    app = SimApp(settings, initial=rad, speed=0.0)
    robot = app.robot

    print()
    print(f"  {'关节':<14}{'舵机ID':>7}{'角度°':>10}{'换算rad':>11}"
          f"{'URDF限位':>20}  判定")
    violations = 0
    for joint in settings.servo.joints:
        name = joint.name
        if name not in rad:
            continue
        value = rad[name]
        lo, hi = robot.limits(name)
        # 舵机读数本身有 0.1° 级抖动，贴限位不算越限
        ok = lo - 0.01 <= value <= hi + 0.01
        if not ok:
            violations += 1
        edge = ok and (value < lo or value > hi)
        verdict = "OK" if ok and not edge else ("OK·贴限位" if edge else "越限 ✗")
        print(f"  {name:<14}{joint.sid:>7}{degrees[joint.sid]:>10.2f}{value:>11.4f}"
              f"{f'[{lo:+.2f}, {hi:+.2f}]':>20}  {verdict}")

    robot.apply_kinematic(rad)
    data = robot.data
    tip_id = next((i for i in range(robot.model.nbody)
                   if (robot.model.body(i).name or "") == "Gripping_point_Link"), -1)
    tip = data.xpos[tip_id] if tip_id >= 0 else data.xpos[-1]
    print()
    print(f"  夹爪指尖位置(世界系)：x={tip[0]:+.3f}  y={tip[1]:+.3f}  z={tip[2]:+.3f} m")
    if tip[2] < 0.0:
        print("  [!] 指尖在地面以下 —— 这个姿态物理上不可能，说明映射不对")
    print(f"  限位越界关节数：{violations} / {len(settings.servo.joints)}")
    if violations:
        print("  判定：MuJoCo 这一帧和实物不会一致，越界的那几轴 offset 需要标定")
    else:
        print("  判定：六轴都在限位内，仿真姿态可以作为实物姿态的参考")

    if args.render:
        app.render_state(rad, args.render, azimuth=args.azimuth,
                         elevation=args.elevation)

    if args.view:
        import mujoco.viewer

        print("\n[窗口] 已按实机角度摆好，可鼠标拖动旋转对照；关掉窗口即退出")
        with mujoco.viewer.launch_passive(robot.model, robot.data) as handle:
            handle.cam.lookat[:] = [0.0, 0.0, 0.20]
            handle.cam.distance = 0.75
            handle.cam.azimuth = args.azimuth
            handle.cam.elevation = args.elevation
            while handle.is_running():
                handle.sync()
                import time

                time.sleep(1.0 / 60.0)

    if args.follow:
        _follow(settings, robot, app, args.hz, args.azimuth, args.elevation)
    return 0


def _follow(settings, robot, app, hz: float, azimuth: float, elevation: float):
    """持续读舵机并刷新 MuJoCo 窗口：窗口里就是当前实物的姿态。"""
    import time

    import mujoco.viewer

    import serial
    from dofbot_mj.protocol import ProtocolError, ServoMapping, read_position, request

    cfg = settings.servo
    mapping = ServoMapping(cfg.joints, cfg.range_deg)
    period = 1.0 / max(0.5, hz)
    print(f"\n[实时跟随] {hz:.1f} Hz 读六个舵机并刷新窗口；关窗口或 Ctrl+C 结束")
    try:
        ser = serial.Serial(cfg.port, cfg.baudrate, timeout=0.2, write_timeout=1.0)
    except Exception as exc:
        print(f"[错误] 打开串口失败：{exc}")
        return
    try:
        ser.reset_input_buffer()
        request(ser, f"RANGE {cfg.range_deg}", f"OK RANGE {cfg.range_deg}", timeout=8.0)
        with mujoco.viewer.launch_passive(robot.model, robot.data) as handle:
            handle.cam.lookat[:] = [0.0, 0.0, 0.20]
            handle.cam.distance = 0.75
            handle.cam.azimuth = azimuth
            handle.cam.elevation = elevation
            last_report = 0.0
            while handle.is_running():
                started = time.monotonic()
                degrees = {}
                for sid in robot_ids(settings):
                    try:
                        degrees[sid] = read_position(ser, sid) * cfg.range_deg / 4095.0
                    except ProtocolError:
                        degrees = None
                        break
                if degrees:
                    robot.apply_kinematic(mapping.positions_from_targets(degrees))
                handle.sync()
                now = time.monotonic()
                if degrees and now - last_report > 2.0:
                    last_report = now
                    text = " ".join(f"ID{sid}:{deg:6.1f}°" for sid, deg in sorted(degrees.items()))
                    print(f"  {text}")
                remain = period - (time.monotonic() - started)
                if remain > 0:
                    time.sleep(remain)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("[实时跟随] 结束")


def robot_ids(settings):
    return [joint.sid for joint in settings.servo.joints]


if __name__ == "__main__":
    raise SystemExit(main())
