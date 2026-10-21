#!/usr/bin/env python3
"""DOFBOT Pro —— 最大抓取范围实测（自动按半径扫描，人工判断成功与否）。

思路：
  把"夹爪抓取点"（URDF 里的 Gripping_point_Link，≈指尖）依次送到
  桌面上一串不同半径的位置，每个位置你放个物体、让它闭合夹爪，
  你回答"抓住没有"，最后得出**这台机器实际能抓到的最远半径**。

  半径/高度 -> 关节角是实时解逆运动学算出来的（不依赖 MoveIt、不用手填姿态），
  再走和 servo_tool 一样的二进制协议直接下发，所以：

      ros2 run dofbot_hw_bridge reach_test --port /dev/ttyUSB0

  只要串口没被 trajectory_bridge 占用就行（即**不要**同时开 arm_real.launch.py）。

安全：
  · 每个点位先算一遍正运动学校验；超出舵机软限位的点直接跳过，不会下发给机械臂。
  · 起始前要求你确认周围 40cm 无人；任何时候 Ctrl+C 都会立刻停发（固件保持当前位置）。
  · 速度由固件 240°/s 限速兜底，下发频率与标定文件 command_rate 一致。

常用参数：
  --radii 16,20,24,26,28,29,30,31   要试的半径（cm）
  --height 2                       物体抓取中心离桌面高度（cm）
  --azimuth 90                     方位角：90=正前方(+y)，0=+x
  --wrist auto                     夹爪俯仰：auto 自动挑（0=朝上,90=水平,180=朝下）
  --open-deg / --close-deg         夹爪张开/闭合角度（度，负值=闭合）
  --fake                           不接硬件，用假 STM32 走一遍流程
  --dry-run                        只打印点位计划，不动机械臂
  --yes                            跳过所有确认（配合 --answer 自动作答）
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from dofbot_hw_bridge import calib as C
    from dofbot_hw_bridge import kin as K
    from dofbot_hw_bridge import protocol as P
    from dofbot_hw_bridge.serial_link import (FakeMcuTransport, LinkError, SerialLink,
                                              open_serial)
else:
    from . import calib as C
    from . import kin as K
    from . import protocol as P
    from .serial_link import FakeMcuTransport, LinkError, SerialLink, open_serial

JOINTS = C.DEFAULT_JOINT_ORDER

# 夹爪俯仰的候选值（度，0=朝上 90=水平 180=朝下）。按"桌面抓取最自然"排序。
WRIST_CANDIDATES = [110, 105, 100, 95, 90, 115, 120, 85, 80, 125, 130, 75, 70,
                    135, 140, 65, 60, 150, 55, 50, 45, 160, 40, 170, 180, 30, 20, 10, 0]


# --------------------------------------------------------------------------- 计划
class Plan:
    __slots__ = ("radius_cm", "z_cm", "wrist_deg", "elbow", "q", "q_lift", "lift_cm",
                 "margin_deg", "tip", "tip_lift")

    def __init__(self, radius_cm, z_cm, wrist_deg, elbow, q, margin_deg, tip,
                 q_lift=None, lift_cm=0.0, tip_lift=None):
        self.radius_cm = radius_cm
        self.z_cm = z_cm
        self.wrist_deg = wrist_deg
        self.elbow = elbow
        self.q = q
        self.q_lift = q_lift
        self.lift_cm = lift_cm
        self.margin_deg = margin_deg
        self.tip = tip
        self.tip_lift = tip_lift

    def angles_deg(self):
        return [math.degrees(v) for v in self.q]


def _margin(q, lo, hi):
    """离最近的软限位还有多少度（越大越安全）。"""
    return min(min(math.degrees(v - l), math.degrees(h - v)) for v, l, h in zip(q, lo, hi))


def _solve_at(scanner, r_cm, z_cm, azimuth, wrist_pref=None, elbow_pref=None,
              min_margin_deg=3.0):
    """在给定 (半径, 高度) 找一个安全解：优先沿用指定的腕姿/肘向，不行就自由找。

    返回 (q, wrist_deg, elbow)；找不到返回 (None, None, None)。
    """
    lo, hi = scanner.limits_rad(scanner.calib)
    wrists = ([wrist_pref] if wrist_pref is not None else [])
    wrists += [w for w in WRIST_CANDIDATES if w != wrist_pref]
    elbows = ([elbow_pref] if elbow_pref else []) + [1, -1]
    for wd in wrists:
        for el in elbows:
            q, _ = scanner.ik_radial(r_cm / 100.0, z_cm / 100.0, azimuth,
                                     wrist_deg=wd, elbow=el, calib=scanner.calib)
            if q is not None and _margin(q, lo, hi) >= min_margin_deg:
                return q, wd, el
    return None, None, None


def build_plan(scanner, radii_cm, z_cm, azimuth, wrist_mode, lift_cm=12.0,
               min_margin_deg=3.0):
    lo, hi = scanner.limits_rad(scanner.calib)
    plan, skipped = [], []
    prev_wrist = 110.0
    for r in radii_cm:
        wrists = [float(wrist_mode)] if wrist_mode != "auto" else WRIST_CANDIDATES
        cands = []
        last_reason = ""
        tight = None                      # 可达但余量不够的最好attempt
        for wd in wrists:
            for el in (-1, +1):
                q, err = scanner.ik_radial(r / 100.0, z_cm / 100.0, azimuth,
                                           wrist_deg=wd, elbow=el, calib=scanner.calib)
                if q is None:
                    last_reason = err
                    continue
                m = _margin(q, lo, hi)
                if m < min_margin_deg:
                    if tight is None or m > tight[0]:
                        tight = (m, wd, el)
                    continue
                cands.append(Plan(r, z_cm, wd, el, q, m, scanner.fk(q)))
        if not cands:
            if tight is not None:
                skipped.append((r, "可达但限位余量只有 %.1f°（低于 --min-margin %.0f°，"
                                   "腕 %.0f°/肘%s）"
                                % (tight[0], min_margin_deg, tight[1],
                                   {1: "外", -1: "内"}[tight[2]])))
            else:
                skipped.append((r, last_reason or "无解"))
            continue
        # 既要余量大、又要相邻两档腕姿连续（避免机械臂在两点之间大幅甩动）
        pick = min(cands, key=lambda c: abs(c.wrist_deg - prev_wrist)
                   + max(0.0, 15.0 - c.margin_deg) * 4.0)
        # 接近点：同半径抬高，优先沿用目标腕姿
        for k in (1.0, 0.7, 0.5, 0.35, 0.25):
            q, _wd, _el = _solve_at(scanner, r, z_cm + lift_cm * k, azimuth,
                                    wrist_pref=pick.wrist_deg, elbow_pref=pick.elbow,
                                    min_margin_deg=min_margin_deg)
            if q is not None:
                pick.q_lift, pick.lift_cm, pick.tip_lift = q, lift_cm * k, scanner.fk(q)
                break
        prev_wrist = pick.wrist_deg
        plan.append(pick)

    # 全局中转位：离开桌面再换半径，杜绝两点之间横着扫过桌面
    transit = None
    for r_t, z_t in ((18.0, 22.0), (15.0, 20.0), (18.0, 16.0), (12.0, 18.0)):
        q, wd, el = _solve_at(scanner, r_t, z_t, azimuth)
        if q is not None:
            transit = (q, scanner.fk(q))
            break
    return plan, skipped, transit


def print_plan(plan, skipped, transit=None):
    if transit is not None:
        print("  中转位：抓取点 (%.3f, %.3f, %.3f) m（换半径前先回到这里，避免横扫桌面）"
              % (transit[1][0], transit[1][1], transit[1][2]))
        print()
    if not plan and not skipped:
        print("  （空）")
        return
    print("  %-8s %-6s %-4s %-7s %-22s %s" %
          ("半径cm", "腕姿°", "肘", "余量", "抓取点(m)", "关节角(deg) Arm1..5"))
    for p in plan:
        print("  %-8.1f %-6.0f %-4s %5.1f°  (%+.3f,%+.3f,%+.3f)  %s"
              % (p.radius_cm, p.wrist_deg, {1: "外", -1: "内"}[p.elbow], p.margin_deg,
                 p.tip[0], p.tip[1], p.tip[2],
                 "[%+.1f %+.1f %+.1f %+.1f %+.1f]" % tuple(p.angles_deg())))
        if p.q_lift is None:
            print("           接近：找不到抬高解，从中转位直接下探（慢一点）")
        else:
            print("           接近：抬 %.0f cm -> (%.3f,%.3f,%.3f) 再下探"
                  % (p.lift_cm, p.tip_lift[0], p.tip_lift[1], p.tip_lift[2]))
    for r, why in skipped:
        print("  %-8.1f 跳过：%s" % (r, why))


# --------------------------------------------------------------------------- 扫描器
class Scanner:
    def __init__(self, kin, calib):
        self.kin = kin
        self.calib = calib

    def fk(self, q):
        return self.kin.fk(q)

    def ik_radial(self, *a, **kw):
        return self.kin.ik_radial(*a, **kw)

    def limits_rad(self, calib):
        return self.kin.limits_rad(calib)

    def ticks_of(self, q_arm, grip_angle):
        angles = {n: v for n, v in zip(JOINTS[:5], q_arm)}
        angles[C.GRIP_JOINTS[0]] = grip_angle
        return self.calib.angles_to_ticks(angles)


# --------------------------------------------------------------------------- 动作
def _stream(link, ticks, duration, rate):
    """按 rate 连发同一目标 duration 秒，让固件保持在跟随模式。"""
    period = 1.0 / max(1.0, rate)
    end = time.monotonic() + max(0.15, duration)
    while time.monotonic() < end:
        link.send_targets(ticks)
        time.sleep(period)


def move_to(link, calib, ticks, extra=0.35, slow=1.0, verbose=True):
    """平滑移到目标（按最大 tick 差估时长），等它走完。slow>1 更慢更稳。"""
    try:
        cur = link.read_pos(0.6)
    except LinkError:
        cur = None
    if cur and all(t != P.UNKNOWN_TICK for t in cur):
        delta = max(abs(a - b) for a, b in zip(ticks, cur))
    else:
        delta = 4095
    # 固件限速 240°/s ≈ 3276 ticks/s，留 80% 余量
    dur = (max(0.4, delta / (3276 * 0.8)) + extra) * max(0.05, slow)
    if verbose:
        print("   移动中（最大 %d tick，约 %.1f s）..." % (delta, dur))
    link.set_mode(P.MODE_STREAM, 0.5)
    _stream(link, ticks, dur, calib.command_rate)
    time.sleep(0.15)


def ask(prompt, auto=None):
    if auto is not None:
        print("%s%s" % (prompt, auto))
        return auto
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return ""


def confirm(prompt, auto_yes=False):
    if auto_yes:
        print("%s（--yes 自动确认）" % prompt)
        return True
    return ask(prompt + " [y/N] ").startswith("y")


# --------------------------------------------------------------------------- 主流程
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="DOFBOT Pro 最大抓取范围实测（按半径自动扫描）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：ros2 run dofbot_hw_bridge reach_test --port /dev/ttyUSB0 "
               "--radii 20,24,26,28,29,30,31 --height 2")
    ap.add_argument("--port", default="", help="串口，如 /dev/ttyUSB0 / COM9")
    ap.add_argument("--baud", type=int, default=0, help="波特率，默认取标定文件")
    ap.add_argument("--calib", default=os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "config", "arm_calib.yaml"), help="标定 YAML")
    ap.add_argument("--urdf", default="", help="URDF（默认从 dofbot_pro_description 找）")
    ap.add_argument("--radii", default="16,20,24,26,28,29,30,31",
                    help="要试的半径 cm，逗号分隔（默认 16,20,24,26,28,29,30,31）")
    ap.add_argument("--height", type=float, default=2.0, help="抓取点离桌面高度 cm（默认 2）")
    ap.add_argument("--azimuth", type=float, default=90.0, help="方位角 deg，90=正前方(+y)")
    ap.add_argument("--wrist", default="auto",
                    help="夹爪俯仰 deg 或 auto（0=朝上 90=水平 180=朝下）")
    ap.add_argument("--lift", type=float, default=12.0,
                    help="接近时先抬多高 cm（默认 12，避免指尖扫到桌面）")
    ap.add_argument("--open-deg", type=float, default=-3.0, help="夹爪张开角度 deg（默认 -3≈全开）")
    ap.add_argument("--close-deg", type=float, default=-70.0, help="夹爪闭合角度 deg（默认 -70）")
    ap.add_argument("--no-grip", action="store_true", help="不操作夹爪（只测可达性）")
    ap.add_argument("--csv", default="", help="把结果写成 CSV（默认 ./reach_result_<时间>.csv）")
    ap.add_argument("--fake", action="store_true", help="用内置假 STM32，不接硬件")
    ap.add_argument("--dry-run", action="store_true", help="只打印点位计划，不动机械臂")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    ap.add_argument("--answer", default=None, help="自动回答闭合后的成败（y/n，测试用）")
    ap.add_argument("--slow", type=float, default=1.0, help="动作时长倍率（>1 更慢，默认 1.0）")
    ap.add_argument("--min-margin", type=float, default=3.0,
                    help="关节离软限位至少留多少度才接受（默认 3，0=只要不超限就用）")

    args = ap.parse_args(argv)

    # ---- 模型 ----
    urdf = K.find_urdf(args.urdf or None)
    calib = C.load_calib(args.calib)
    if args.port:
        calib.port = args.port
    if args.baud:
        calib.baud = args.baud
    kin = K.ArmKin(urdf)
    sc = Scanner(kin, calib)

    print("=" * 74)
    print("DOFBOT Pro 最大抓取范围实测")
    print("=" * 74)
    print(kin.describe())
    worst, cnt = kin.self_check(calib)
    if cnt < 20 or worst > 1e-3:
        print("  ⚠ 运动学自检异常：%d 个点，最大残差 %.2f mm（检查 URDF / arm_calib.yaml）"
              % (cnt, worst * 1e3))
    else:
        print("  运动学自检 OK：IK→FK 最大残差 %.3f mm（%d 个采样点）"
              % (worst * 1e3, cnt))
    print("  标定：%s" % os.path.abspath(args.calib))
    print()

    radii = [float(v) for v in args.radii.replace(" ", ",").split(",") if v]
    wrist_mode = "auto" if args.wrist == "auto" else float(args.wrist)
    plan, skipped, transit = build_plan(sc, radii, args.height, args.azimuth, wrist_mode,
                                        lift_cm=args.lift, min_margin_deg=args.min_margin)

    print("点位计划（抓取点离桌面 %.1f cm，方位 %s°）：" % (args.height, args.azimuth))
    print_plan(plan, skipped, transit)
    print()

    if not plan:
        print("没有可用的点位，换个半径/高度/腕姿再试。")
        return 1

    if args.dry_run:
        print("--dry-run：到此为止，没有动作。")
        return 0

    # ---- 连接 ----
    if args.fake:
        link = SerialLink(FakeMcuTransport())
        link.start()
        print("使用内置假 STM32（--fake），不会真的动机械臂")
    else:
        print("打开串口 %s @ %d ..." % (calib.port, calib.baud))
        try:
            link = open_serial(calib.port, calib.baud)
        except Exception as exc:
            print("串口打开失败：%s" % exc)
            print("提示：是不是 trajectory_bridge 还占着串口？先关掉它。")
            return 1

    grip_open = math.radians(args.open_deg)
    grip_close = math.radians(args.close_deg)
    results = []
    link_broken = False

    try:
        try:
            pong = link.ping(1.0)
            print("STM32 在线：固件 v%d.%d，关节数 %d，总线方式 %d"
                  % (pong["ver_major"], pong["ver_minor"], pong["njoint"], pong["busmode"]))
        except LinkError as exc:
            print("PING 失败：%s" % exc)
            print("排查：固件烧了吗？TX->PA3 / RX->PA2 / 共地 / 115200")
            return 1

        if not confirm("机械臂周围 40cm 内没有人、没有障碍物，台面清空。继续？", args.yes):
            print("已取消。")
            return 0

        print("\n先把夹爪张开并抬到中转位...")
        grip_hold = 0.0 if args.no_grip else grip_open
        if transit is not None:
            move_to(link, calib, sc.ticks_of(transit[0], grip_hold))
        else:
            first = plan[0]
            move_to(link, calib, sc.ticks_of(first.q_lift if first.q_lift else first.q,
                                             grip_hold))

        print()
        print("=" * 74)
        print("开始逐档测试。每档流程：")
        print("  ① 机械臂先抬到目标半径上方（这时你还能安全地把手伸过去）")
        print("  ② 你把物体放到指尖正下方 —— 台上标好半径处")
        print("  ③ 回车 -> 机械臂下探到抓取高度 z=%.1f cm，夹爪闭合" % args.height)
        print("  ④ 观察是否抓住，回答 y/n；机械臂随后张爪、抬高、回中转位")
        print("  Ctrl+C 随时中断（立刻停发，机械臂保持当前位置）。")
        print("=" * 74)

        def go_transit():
            if transit is not None:
                move_to(link, calib, sc.ticks_of(transit[0], grip_hold), verbose=False)

        for idx, p in enumerate(plan, 1):
            print("\n[%d/%d] 半径 %.1f cm  (腕姿 %.0f°, 肘%s, 限位余量 %.1f°)"
                  % (idx, len(plan), p.radius_cm, p.wrist_deg,
                     {1: "外", -1: "内"}[p.elbow], p.margin_deg))

            # 1) 中转位 -> 接近点（抬高）
            lift_q = p.q_lift if p.q_lift is not None else transit[0] if transit else p.q
            move_to(link, calib, sc.ticks_of(lift_q, grip_hold), extra=0.35, slow=args.slow)

            # 2) 人放物体
            ask("  把物体放到指尖正下方（半径 %.0f cm 处），按回车继续..."
                % p.radius_cm, "" if args.yes else None)

            # 3) 下探
            print("   下探到抓取高度 z=%.1f cm ..." % p.z_cm)
            move_to(link, calib, sc.ticks_of(p.q, grip_hold), extra=0.3, slow=args.slow)

            if not args.no_grip:
                print("   夹爪闭合中...")
                move_to(link, calib, sc.ticks_of(p.q, grip_close), extra=0.5, slow=args.slow)
                time.sleep(0.4)

            ans = ask("  抓住了吗？(y/n/s=跳过) ", args.answer)
            ok = ans.startswith("y")
            if ans.startswith("s"):
                print("  跳过这一档")
                if not args.no_grip:
                    move_to(link, calib, sc.ticks_of(p.q, grip_open), verbose=False)
                move_to(link, calib, sc.ticks_of(lift_q, grip_hold), verbose=False)
                go_transit()
                continue

            results.append((p.radius_cm, ok, p.wrist_deg, p.elbow))
            print("  -> %s" % ("抓住" if ok else "没抓住"))

            # 4) 张爪 + 抬高 + 回中转位，为下一档让路
            if not args.no_grip:
                move_to(link, calib, sc.ticks_of(p.q, grip_open), verbose=False)
            move_to(link, calib, sc.ticks_of(lift_q, grip_hold), verbose=False)
            go_transit()

    except KeyboardInterrupt:
        print("\n中断：停止下发，机械臂保持当前位置。")
        try:
            link.stop_motion(0.8)
        except Exception:
            pass
    except LinkError as exc:
        link_broken = True
        print("\n链路错误：%s" % exc)
    finally:
        try:
            link.stop_motion(0.8)
        except Exception:
            pass
        link.shutdown()

    # ---- 结果 ----
    print()
    print("=" * 74)
    if not results:
        print("没有采集到结果。")
        return 0
    print("结果（半径 cm -> 是否抓住）：")
    for r, ok, wd, el in results:
        print("  %5.1f cm   %s   （腕姿 %.0f°, 肘%s）"
              % (r, "✅ 抓住" if ok else "❌ 没抓住", wd, {1: "外", -1: "内"}[el]))

    okr = [r for r, ok, _, _ in results if ok]
    badr = [r for r, ok, _, _ in results if not ok]
    print()
    if okr:
        print("  实测最大抓取半径 = %.1f cm" % max(okr))
        if badr:
            print("  第一个失败的半径 = %.1f cm" % min([r for r in badr if r > max(okr)] or badr))
    else:
        print("  一档都没抓住：先确认物体放到位、夹爪没顶死，再重试。")

    path = args.csv or ("reach_result_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
    try:
        with open(path, "w", encoding="utf-8-sig") as f:
            f.write("radius_cm,height_cm,azimuth_deg,wrist_deg,elbow,success,"
                    "joints_deg\n")
            for r, ok, wd, el in results:
                p = next(x for x in plan if x.radius_cm == r)
                f.write("%.1f,%.1f,%.1f,%.0f,%d,%d,\"%s\"\n"
                        % (r, p.z_cm, args.azimuth, wd, el, 1 if ok else 0,
                           " ".join("%.2f" % v for v in p.angles_deg())))
        print("  结果已写入 %s" % os.path.abspath(path))
    except OSError as exc:
        print("  写 CSV 失败：%s" % exc)

    return 1 if link_broken else 0


if __name__ == "__main__":
    raise SystemExit(main())
