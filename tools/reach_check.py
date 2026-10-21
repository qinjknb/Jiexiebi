#!/usr/bin/env python3
"""DOFBOT Pro 抓取范围速查（离线计算，不接硬件、不用 ROS）。

用的是和 reach_test 完全同一套运动学（dofbot_hw_bridge/kin.py，几何直接读 URDF），
所以这里算出来的半径就是实机脚本会往那儿走的半径。

用法：
    python reach_check.py                # 默认表格
    python reach_check.py --z 2          # 只看某个高度
    python reach_check.py --detail       # 连"要抓半径 R 该给什么关节角"一起打印

输出两块：
  1) 几何包络：每个高度上、任意夹爪姿态下能到达的最大水平半径（只受关节限位约束）
  2) 抓取余量：给常见腕姿（90 水平 / 120 斜下 / 150 更朝下）各自的最大半径
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.abspath(os.path.join(HERE, "..", "My_JXB_test", "src", "dofbot_hw_bridge"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from dofbot_hw_bridge import calib as C          # noqa: E402
from dofbot_hw_bridge import kin as K           # noqa: E402

CALIB = os.path.join(PKG_SRC, "config", "arm_calib.yaml")
WRISTS = [90, 100, 110, 120, 130, 140, 150, 160, 170, 180,
          80, 70, 60, 50, 40, 30, 20, 10, 0]


def best_at(kin, calib, z_cm, wrists, min_margin=0.0, step=0.5, rmax=40.0):
    """在给定高度上，找能到的最远半径和对应姿态（内侧到不了不影响外侧）。"""
    lo, hi = kin.limits_rad(calib)
    best = None
    r = 5.0
    while r <= rmax + 1e-9:
        for wd in wrists:
            for el in (-1, 1):
                q, _ = kin.ik_radial(r / 100.0, z_cm / 100.0, 90.0, wrist_deg=wd,
                                     elbow=el, calib=calib)
                if q is None:
                    continue
                m = min(min(math.degrees(v - l), math.degrees(h - v))
                        for v, l, h in zip(q, lo, hi))
                if m < min_margin:
                    continue
                best = (r, wd, el, q, m)
                break
            else:
                continue
            break
        r += step
    return best


def main(argv=None):
    ap = argparse.ArgumentParser(description="DOFBOT Pro 抓取范围速查（离线）")
    ap.add_argument("--z", type=float, default=None, help="只看某个高度 cm")
    ap.add_argument("--detail", action="store_true", help="打印每个半径对应的关节角")
    ap.add_argument("--urdf", default="", help="URDF 路径")
    ap.add_argument("--calib", default=CALIB, help="标定 YAML")
    args = ap.parse_args(argv)

    kin = K.ArmKin(K.find_urdf(args.urdf or None))
    calib = C.load_calib(args.calib)
    print(kin.describe())
    print("标定：%s" % os.path.abspath(args.calib))
    print()

    zs = [args.z] if args.z is not None else [0, 1, 2, 3, 5, 8, 10, 12, 15, 18, 20, 25, 30]

    print("== 1) 几何包络：任意夹爪姿态下的最大水平半径（只看关节限位）==")
    print("  高度cm   最大半径cm   腕姿°  肘   限位余量")
    for z in zs:
        b = best_at(kin, calib, z, WRISTS)
        if b is None:
            print("  %5.0f     —— 不可达" % z)
            continue
        print("  %5.0f    %7.1f     %4.0f  %s   %5.1f°"
              % (z, b[0], b[1], {1: "外", -1: "内"}[b[2]], b[4]))

    print()
    print("== 2) 固定腕姿下的最大半径（夹爪越朝下，能伸的越近）==")
    print("  高度cm  " + "".join("%9s" % ("腕%d°" % w) for w in (90, 120, 150, 180)))
    for z in zs:
        row = []
        for w in (90, 120, 150, 180):
            b = best_at(kin, calib, z, [w])
            row.append("%8.1f " % b[0] if b else "     —— ")
        print("  %5.0f   %s" % (z, "".join(row)))

    if args.detail:
        from dofbot_hw_bridge.reach_test import Scanner, build_plan
        sc = Scanner(kin, calib)
        radii = [12, 14, 16, 18, 20, 22, 24, 26, 27, 28, 29, 30, 31, 32, 33]
        plan, skipped, transit = build_plan(sc, radii, 2.0, 90.0, "auto")
        print()
        print("== 3) 桌面(z=2cm) 逐半径的最优解（和 reach_test 实机脚本同一套选点）==")
        for p in plan:
            print("  %4.0f cm  腕%3.0f° 肘%s 余量%5.1f°  [%s]  ticks=%s"
                  % (p.radius_cm, p.wrist_deg, {1: "外", -1: "内"}[p.elbow], p.margin_deg,
                     " ".join("%+6.1f" % v for v in p.angles_deg()),
                     calib.angles_to_ticks({n: v for n, v in zip(C.ARM_JOINTS, p.q)})))
        for r, why in skipped:
            print("  %4.0f cm  不可达：%s" % (r, why))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
