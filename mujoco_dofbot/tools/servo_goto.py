#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把指定舵机打到某个角度（或各自的 offset 零点），并读回确认。

用法：
    python tools/servo_goto.py                        # 六路都打到 150°
    python tools/servo_goto.py --angle 150 --ids 1 3
    python tools/servo_goto.py --to-zero              # 每轴打到各自 offset（=关节角0）
    python tools/servo_goto.py --angle 150 --dry-run

读回值 != 目标值，说明舵机没执行（输出级保护停转）或行程被挡。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description="把舵机打到指定角度并读回")
    parser.add_argument("--config", default=str(ROOT / "config" / "calibrated.yaml"))
    parser.add_argument("--angle", type=float, default=150.0, help="目标角度(°)")
    parser.add_argument("--to-zero", action="store_true",
                        help="改为把每个关节打到各自的 offset（即关节角 0，机械中位）")
    parser.add_argument("--ids", type=int, nargs="*", default=[1, 2, 3, 4, 5, 6])
    parser.add_argument("--move-ms", type=int, default=600, help="每条 MOVE 的运行时间")
    parser.add_argument("--spacing", type=float, default=1.2, help="两条指令间隔(秒)")
    parser.add_argument("--settle", type=float, default=1.5, help="全部发完后等待(秒)")
    parser.add_argument("--tolerance", type=float, default=1.5, help="判定到位容差(°)")
    parser.add_argument("--dry-run", action="store_true", help="只打印不发送")
    args = parser.parse_args(argv)

    from dofbot_mj.config import load_config
    from dofbot_mj.protocol import ProtocolError, read_position, request

    settings = load_config(args.config)
    cfg = settings.servo
    by_id = {joint.sid: joint for joint in cfg.joints}

    unknown = [sid for sid in args.ids if sid not in by_id]
    if unknown:
        print(f"[错误] 配置里没有 ID {unknown}")
        return 2

    if args.to_zero:
        targets = {sid: by_id[sid].offset_deg for sid in args.ids}
        print("[模式] 打到各轴 offset（关节角 0，机械中位）")
    else:
        if not 0 <= args.angle <= cfg.range_deg:
            print(f"[错误] 目标角 {args.angle}° 超出 0~{cfg.range_deg}°")
            return 2
        targets = {sid: args.angle for sid in args.ids}

    try:
        import serial
    except ImportError:
        print("[错误] 未安装 pyserial")
        return 1

    print(f"[串口] {cfg.port} @ {cfg.baudrate}  运行时间 {args.move_ms} ms")
    with serial.Serial(cfg.port, cfg.baudrate, timeout=0.25, write_timeout=1.0) as ser:
        ser.reset_input_buffer()
        request(ser, f"RANGE {cfg.range_deg}", f"OK RANGE {cfg.range_deg}", timeout=8.0)

        before = {}
        for sid in args.ids:
            try:
                before[sid] = read_position(ser, sid) * cfg.range_deg / 4095.0
            except ProtocolError as exc:
                before[sid] = None
                print(f"   ID{sid}: 读取失败 {exc}")

        print(f"\n  {'ID':>3}{'起始°':>9}{'目标°':>9}{'应答':>14}")
        for index, sid in enumerate(args.ids):
            line = f"MOVE {sid} {targets[sid]:.2f} {args.move_ms}"
            reply = "-"
            if not args.dry_run:
                try:
                    reply = request(ser, line, f"OK MOVE {sid}", timeout=2.5)
                except ProtocolError as exc:
                    reply = f"失败:{str(exc)[:18]}"
            start = before.get(sid)
            print(f"  {sid:>3}{(f'{start:8.2f}' if start is not None else '     ---'):>9}"
                  f"{targets[sid]:>9.1f}{reply:>14}")
            if not args.dry_run and index != len(args.ids) - 1:
                time.sleep(args.spacing)

        if args.dry_run:
            print("\n[DRY] 未发送")
            return 0

        print(f"\n[等待] {args.settle:.1f}s 后读回")
        time.sleep(args.settle)

        print(f"\n  {'ID':>3}{'起始°':>9}{'读回°':>9}{'变化°':>9}{'偏差°':>9}  判定")
        reached = 0
        for sid in args.ids:
            try:
                after = read_position(ser, sid) * cfg.range_deg / 4095.0
            except ProtocolError as exc:
                print(f"  {sid:>3}  读回失败 {exc}")
                continue
            start = before.get(sid)
            delta = after - start if start is not None else float("nan")
            error = after - targets[sid]
            ok = abs(error) <= args.tolerance
            reached += ok
            print(f"  {sid:>3}{(f'{start:8.2f}' if start is not None else '     ---'):>9}"
                  f"{after:>9.2f}{delta:>+9.2f}{error:>+9.2f}  "
                  f"{'到位' if ok else '没到位 ✗'}")

        print(f"\n[结果] {reached}/{len(args.ids)} 个到位。")
        if reached < len(args.ids):
            print("       没到位的多半是舵机输出级保护停转（能 PING/READ 但不执行 MOVE），")
            print("       需要把舵机外部供电断电重上电。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
