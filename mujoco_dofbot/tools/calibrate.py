#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用一次已知姿态标定六个舵机的 offset（零点）。

原理：映射公式是 `舵机角 = offset + direction × 关节角 × scale`。
把机械臂摆成某个已知姿态时，每个关节角是已知的（例如 SRDF 的 `up` 全是 0），
于是：
        offset = 实测舵机角 − direction × 关节角 × scale
`up` 姿态下关节角全为 0，offset 就等于当时的实测读数，最省事。

⚠ 只能标 offset（零点）。**方向 direction 必须另外验证**：命令某轴 +5°，
看实物是不是朝"关节角增大"的方向转；反了就把它改成 -1。

用法：
    python tools/calibrate.py --pose up            # 按当前姿态=up 生成配置
    python tools/calibrate.py --pose init -o config/calibrated.yaml
它读 config/default.yaml，写出 config/calibrated.yaml（不改原文件）。
"""

from __future__ import annotations

import argparse
import re
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

    parser = argparse.ArgumentParser(description="按已知姿态标定舵机 offset")
    parser.add_argument("--config", default=str(ROOT / "config" / "calibrated.yaml"),
                        help="读哪份配置作为基准（默认 calibrated.yaml，"
                             "这样能保留已确认的 direction 与不参与标定关节的 offset）")
    parser.add_argument("-o", "--out", default=str(ROOT / "config" / "calibrated.yaml"))
    parser.add_argument("--pose", default="up", help="当前实物所处的 SRDF 姿态名")
    args = parser.parse_args(argv)

    from dofbot_mj.config import load_config
    from dofbot_mj.poses import parse_srdf
    from dofbot_mj.servo_link import probe_positions

    settings = load_config(args.config)

    pose = {}
    if settings.model.srdf.is_file():
        states = parse_srdf(settings.model.srdf)["states"]
        if args.pose not in states:
            print(f"SRDF 里没有姿态 '{args.pose}'，可用：{', '.join(sorted(states))}")
            return 2
        pose = states[args.pose]
    if args.pose not in ("up",) and not pose:
        print("找不到 SRDF，只能用 --pose up（全零）")
        return 2

    print(f"[标定] 前提：机械臂现在停在 SRDF 姿态 '{args.pose}' 上")
    print(f"       {args.pose} = " + (", ".join(f"{k}={v:+.3f}" for k, v in pose.items())
                                     or "全部关节角 0"))
    try:
        rad_now, degrees = probe_positions(settings.servo, log=print)
    except Exception as exc:
        print(f"[错误] 探测失败：{type(exc).__name__}: {exc}")
        return 1

    lines = []
    print()
    print(f"  {'关节':<14}{'ID':>4}{'实测°':>9}{'姿态rad':>10}{'新 offset°':>12}  说明")
    skipped = []
    for joint in settings.servo.joints:
        if joint.sid not in degrees:
            print(f"[错误] ID{joint.sid} 没有读数")
            return 1
        if joint.name not in pose:
            # 该姿态没有定义这个关节（例如 SRDF 的 up 只列了 Arm1~Arm5，
            # 不含夹爪）→ 不动它的 offset，否则会把夹爪标坏
            skipped.append(joint.name)
            lines.append(f"    - {{name: {joint.name}, id: {joint.sid}, "
                         f"offset_deg: {joint.offset_deg:.2f}, "
                         f"direction: {joint.direction:g}}}")
            print(f"  {joint.name:<14}{joint.sid:>4}{degrees[joint.sid]:>9.2f}"
                  f"{'—':>10}{joint.offset_deg:>12.2f}  保持不变（姿态未定义该关节）")
            continue
        angle = float(pose[joint.name])
        offset = degrees[joint.sid] - joint.direction * angle * joint.scale_deg_per_rad
        lines.append(f"    - {{name: {joint.name}, id: {joint.sid}, "
                     f"offset_deg: {offset:.2f}, direction: {joint.direction:g}}}")
        print(f"  {joint.name:<14}{joint.sid:>4}{degrees[joint.sid]:>9.2f}"
              f"{angle:>10.3f}{offset:>12.2f}  重算")
    if skipped:
        print(f"\n[注意] {', '.join(skipped)} 的 offset 未改（该姿态没定义它们）")

    text = Path(args.config).read_text(encoding="utf-8")
    block = "  joints:\n" + "\n".join(lines) + "\n"
    # 每行以 "    - {" 开头，允许行尾注释；块内还可能夹着 4 空格缩进的注释行
    new_text, count = re.subn(r"  joints:\n(?:    (?:- \{.*|#.*)\n)+", block, text)
    if count != 1:
        print(f"[错误] 没能在配置里定位 joints 段（匹配 {count} 处）")
        return 1
    new_text = new_text.replace(
        "calibration_confirmed: false", "calibration_confirmed: true")
    new_text = f"# 由 tools/calibrate.py 按姿态 '{args.pose}' 生成\n" + new_text

    out = Path(args.out)
    out.write_text(new_text, encoding="utf-8")
    try:
        shown = out.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        shown = str(out)
    print(f"\n[写出] {out}")
    print(f"[用法] python run.py --config {shown} --start-from-servo --servo live")
    print("[提醒] 方向 direction 还没验证：先单轴小幅度试一下，反了就改 -1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
