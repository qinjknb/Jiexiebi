#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读实机当前姿态并记成一个命名姿态（存到 config/poses.yaml）。

存两份数据：
  servo_deg —— 六个舵机的角度，用来让机械臂回到这个姿态；
  joint_rad —— 换算出的关节角，用来在 MuJoCo 里直接摆出来。

用法：
    python tools/record_pose.py --name scan --note "扫描位"
    python tools/record_pose.py --name scan --dry-run     # 只看不写
之后在 cycle.steps 里就能写 `arm: scan` 或 `scan: scan`。
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

    p = argparse.ArgumentParser(description="记录实机当前姿态为命名姿态")
    p.add_argument("--config", default=str(ROOT / "config" / "calibrated.yaml"))
    p.add_argument("--out", default=str(ROOT / "config" / "poses.yaml"))
    p.add_argument("--name", required=True, help="姿态名，例如 scan")
    p.add_argument("--note", default="", help="备注")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args(argv)

    import yaml

    from dofbot_mj.config import load_config
    from dofbot_mj.servo_link import probe_positions

    settings = load_config(args.config)
    print(f"[读取] 串口 {settings.servo.port or '自动'} 上的六个舵机")
    try:
        rad, degrees = probe_positions(settings.servo, log=print)
    except Exception as exc:
        print(f"[错误] 探测失败：{type(exc).__name__}: {exc}")
        return 1

    print()
    print(f"  {'关节':<14}{'ID':>4}{'舵机°':>10}{'关节rad':>11}")
    for joint in settings.servo.joints:
        if joint.name not in rad:
            continue
        print(f"  {joint.name:<14}{joint.sid:>4}{degrees[joint.sid]:>10.2f}"
              f"{rad[joint.name]:>11.4f}")

    out = Path(args.out)
    data = {}
    if out.is_file():
        try:
            data = yaml.safe_load(out.read_text(encoding="utf-8")) or {}
        except Exception:
            data = {}

    entry_order = [j.name for j in settings.servo.joints if j.name in rad]
    data[args.name] = {
        "note": args.note,
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source_config": Path(args.config).name,
        "servo_deg": {joint.sid: round(float(degrees[joint.sid]), 2)
                      for joint in settings.servo.joints if joint.sid in degrees},
        "joint_rad": {name: round(float(rad[name]), 5) for name in entry_order},
    }

    if args.dry_run:
        print("\n[DRY] 未写入：")
        print(yaml.safe_dump({args.name: data[args.name]},
                             allow_unicode=True, sort_keys=False))
        return 0

    out.parent.mkdir(parents=True, exist_ok=True)
    header = ("# 实机实测记录的命名姿态（SRDF 之外）\n"
              "# 由 tools/record_pose.py 生成；servo_deg 用来驱动实物，\n"
              "# joint_rad 用来在 MuJoCo 里摆出同一姿态。\n")
    out.write_text(header + yaml.safe_dump(data, allow_unicode=True,
                                           sort_keys=False), encoding="utf-8")
    print(f"\n[写入] {out}  →  姿态 '{args.name}'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
