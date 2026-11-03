#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手眼标定（eye-in-hand）：解出相机相对机械臂末端连杆的安装变换。

原理
----
相机固定在腕部（URDF 里的 DaBai_DCW2_Link），桌面上放一个位置固定的 ArUco 标定板：

    每个姿态 i 都有：  T_base←marker = T_base←tool,i · X · T_cam←marker,i
    其中 T_base←tool,i 由关节角正解得到，T_cam←marker,i 由 ArUco 位姿估计得到，
    X = T_tool←cam 就是要求的安装变换（六自由度常量）。

这正是 OpenCV 的 eye-in-hand 手眼标定，用 cv2.calibrateHandEye 求解。

三个子命令
----------
    marker  生成可打印的 ArUco 图（含实际尺寸标注）
    record  在某一个姿态下采集一帧样本（读舵机 + 取图 + 检测标记）
    solve   用已采集的样本求解并校验

建议采集 8~12 个姿态，姿态之间要有明显的**转动**差异（纯平移对手眼标定帮助很小）。

用法
----
    python tools\\handeye_calib.py marker -o calib\\aruco_40mm.png --size-mm 40
    python tools\\handeye_calib.py record --name p01 --preview calib\\p01.png
    python tools\\handeye_calib.py solve
    python tools\\handeye_calib.py status
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
REPO = ROOT.parent
VISION = REPO / "yolo_pkg"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(VISION))

TOOL_LINK = "DaBai_DCW2_Link"
SAMPLES = ROOT / "calib" / "handeye_samples.yaml"
RESULT = ROOT / "config" / "handeye.yaml"

DICT_NAME = "DICT_4X4_50"


# ---------------------------------------------------------------- 工具
def load_yaml(path: Path) -> dict:
    import yaml

    if not path.is_file():
        return {}
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _plain(value):
    """把 numpy 标量/数组转成 Python 原生类型，否则 yaml.safe_dump 会崩。"""
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return _plain(value.tolist())
    return value


def dump_yaml(path: Path, data: dict, header: str = ""):
    import yaml

    path.parent.mkdir(parents=True, exist_ok=True)
    text = header + yaml.safe_dump(_plain(data), allow_unicode=True, sort_keys=False)
    path.write_text(text, encoding="utf-8")


def aruco_dict(name: str = DICT_NAME):
    import cv2

    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_object_points(size_mm: float) -> np.ndarray:
    """标记坐标系下的四个角点（与 OpenCV 检测顺序一致：左上→右上→右下→左下）。"""
    h = size_mm / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                    dtype=np.float64)


def robot_model(config: Path):
    from dofbot_mj.config import load_config
    from dofbot_mj.model import RobotModel

    settings = load_config(config)
    sidecar = settings.model.mjcf.with_suffix(".json")
    mimic = (json.loads(sidecar.read_text(encoding="utf-8")) or {}).get("mimic", {}) \
        if sidecar.is_file() else {}
    return settings, RobotModel(settings.model.mjcf, mimic=mimic)


def base_from_tool(robot, joint_rad: dict):
    """由关节角算 T_base←tool（4x4）。"""
    robot.apply_kinematic(joint_rad)
    body = next((i for i in range(robot.model.nbody)
                 if robot.model.body(i).name == TOOL_LINK), -1)
    if body < 0:
        raise RuntimeError(f"模型里没有连杆 {TOOL_LINK}")
    transform = np.eye(4)
    transform[:3, :3] = robot.data.xmat[body].reshape(3, 3)
    transform[:3, 3] = robot.data.xpos[body]
    return transform


def grab_frames(timeout_ms=3000):
    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
        time.sleep(1.2)
        for _ in range(20):
            pair = cam.get_frame_pair(timeout_ms=timeout_ms)
            if pair.get("depth") is not None and pair.get("color") is not None:
                return pair
    finally:
        try:
            cam.close()
        except Exception:
            pass
    return None


def aruco_board(spec: dict):
    """按 board 规格构建 OpenCV GridBoard。"""
    import cv2

    dictionary = aruco_dict(spec.get("dict", DICT_NAME))
    ids = np.arange(int(spec["cols"]) * int(spec["rows"]), dtype=np.int32)
    board = cv2.aruco.GridBoard(
        (int(spec["cols"]), int(spec["rows"])),
        float(spec["marker_mm"]) / 1000.0,
        float(spec["gap_mm"]) / 1000.0,
        dictionary, ids)
    return board, dictionary


def cmd_board(args) -> int:
    """生成一张 A4 多标记标定板（相机转大角度时总有标记在视野里）。"""
    import cv2

    spec = {"dict": args.dict, "cols": args.cols, "rows": args.rows,
            "marker_mm": args.marker_mm, "gap_mm": args.gap_mm}
    board, _dict = aruco_board(spec)

    width_mm = args.cols * args.marker_mm + (args.cols - 1) * args.gap_mm
    height_mm = args.rows * args.marker_mm + (args.rows - 1) * args.gap_mm
    margin_mm = args.margin_mm
    total_w = width_mm + 2 * margin_mm
    total_h = height_mm + 2 * margin_mm
    if total_w > 210 or total_h > 297:
        print(f"[错误] 板子 {total_w:.0f}x{total_h:.0f} mm 超出 A4（210x297），"
              f"请减小 --marker-mm / --gap-mm / --margin-mm")
        return 1

    px_per_mm = args.dpi / 25.4
    size_px = (int(round(total_w * px_per_mm)), int(round(total_h * px_per_mm)))
    image = board.generateImage(size_px, marginSize=int(round(margin_mm * px_per_mm)),
                                borderBits=1)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    caption = (f"{args.dict} {args.cols}x{args.rows}  marker {args.marker_mm:g}mm  "
               f"gap {args.gap_mm:g}mm")
    cv2.putText(image, caption, (10, image.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image)

    spec.update({"marker_ids": [int(i) for i in np.arange(args.cols * args.rows)],
                 "board_mm": [round(width_mm, 1), round(height_mm, 1)],
                 "total_mm": [round(total_w, 1), round(total_h, 1)],
                 "dpi": args.dpi})
    dump_yaml(ROOT / "calib" / "board.yaml", spec,
              header="# 多标记标定板规格（tools/handeye_calib.py board 生成）\n"
                     "# 打印时务必 100% 比例，marker_mm/gap_mm 是实际尺寸\n")

    print(f"[板] {out}  {total_w:.0f}x{total_h:.0f} mm  @{args.dpi}dpi")
    print(f"     {args.cols}x{args.rows} = {args.cols*args.rows} 个标记，"
          f"单标记 {args.marker_mm:g} mm，间距 {args.gap_mm:g} mm")
    print(f"     板面 {width_mm:.0f}x{height_mm:.0f} mm（不含白边）")
    print(f"[规格] {ROOT / 'calib' / 'board.yaml'}")
    print("     打印要求：100% 比例、勿缩放；打出来量任一标记黑框边长应为 "
          f"{args.marker_mm:g} mm")
    return 0


def detect_all_markers(color_bgr, dictionary):
    """检测画面里所有标记，返回 [(id, corners(4,2))]，按边长从大到小排。"""
    import cv2

    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _rej = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return []
    found = []
    for corner, mid in zip(corners, ids.flatten()):
        pts = corner.reshape(4, 2)
        side = float(np.mean([np.linalg.norm(pts[(k + 1) % 4] - pts[k])
                              for k in range(4)]))
        found.append((int(mid), pts, side))
    found.sort(key=lambda item: -item[2])
    return [(mid, pts) for mid, pts, _side in found]


def detect_marker(color_bgr, dictionary, marker_id=None):
    """返回 (corners(N,4,2) float, ids) 或 (None, None)。"""
    import cv2

    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _rejected = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None, None
    if marker_id is None:
        return corners[0].reshape(4, 2), int(ids[0])
    for corner, mid in zip(corners, ids.flatten()):
        if int(mid) == marker_id:
            return corner.reshape(4, 2), int(mid)
    return None, None


# ---------------------------------------------------------------- 子命令
def cmd_marker(args) -> int:
    import cv2

    dictionary = aruco_dict(args.dict)
    px = int(args.size_mm / 25.4 * args.dpi)
    image = cv2.aruco.generateImageMarker(dictionary, args.id, px)

    pad = int(px * 0.12)
    canvas = np.full((px + pad * 2 + 60, px + pad * 2), 255, dtype=np.uint8)
    canvas[pad:pad + px, pad:pad + px] = image
    cv2.putText(canvas, f"{args.dict} id={args.id}  {args.size_mm:g}mm",
                (8, px + pad * 2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 1, cv2.LINE_AA)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), canvas)
    print(f"[标记] {out}")
    print(f"       字典 {args.dict}   id={args.id}   边长 {args.size_mm:g} mm")
    print(f"       打印时请保证黑色方块外框实际边长 = {args.size_mm:g} mm"
          f"（在 {args.dpi} dpi 下图上约为 {px} 像素）")
    print("       打印后建议用卡尺量一下，把实测值传给 --size-mm")
    return 0


def cmd_record(args) -> int:
    import cv2

    from dofbot_mj.servo_link import probe_positions

    settings, robot = robot_model(Path(args.config))
    print(f"[舵机] 读取姿态（串口 {settings.servo.port or '自动'}）")
    try:
        rad, _deg = probe_positions(settings.servo, log=lambda *a: None)
    except Exception as exc:
        print(f"[错误] 读舵机失败：{exc}")
        return 1
    transform = base_from_tool(robot, rad)
    print(f"[姿态] {args.name}: " + " ".join(f"{k.split('_')[0]}={v:+.3f}"
                                             for k, v in rad.items()))

    pair = grab_frames()
    if pair is None:
        print("[错误] 取帧失败")
        return 1
    corners, mid = detect_marker(pair["color"], aruco_dict(args.dict), args.id)
    if corners is None:
        print(f"[错误] 没检测到 ArUco（id={args.id}）。"
              f"检查标记是否在画面内、大小是否合适、光照是否够")
        if args.preview:
            cv2.imwrite(args.preview, pair["color"])
            print(f"       已保存原始画面便于排查：{args.preview}")
        return 1
    print(f"[标记] 检测到 id={mid}，四角像素：")
    for index, (u, v) in enumerate(corners):
        print(f"        corner{index}: ({u:6.1f}, {v:6.1f})")
    edges = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
    side_px = float(np.mean(edges))
    print(f"       平均边长 {side_px:.1f} px", end="")
    if side_px < 60:
        print("  ← 太小！位姿估计会不准，建议把标记打大些或移近")
    elif side_px < 100:
        print("  ← 偏小，精度会打折（建议 ≥100 px）")
    else:
        print("  ← 尺寸合适")

    if args.preview:
        vis = pair["color"].copy()
        cv2.polylines(vis, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        for index, (u, v) in enumerate(corners):
            cv2.circle(vis, (int(u), int(v)), 4, (0, 0, 255), -1)
            cv2.putText(vis, str(index), (int(u) + 6, int(v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        Path(args.preview).parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(args.preview, vis)
        print(f"[预览] {args.preview}")

    data = load_yaml(SAMPLES)
    data.setdefault("marker", {
        "dict": args.dict, "id": args.id, "size_mm": args.size_mm,
        "image_size": [int(pair["color"].shape[1]), int(pair["color"].shape[0])],
    })
    samples = {s["name"]: s for s in data.get("samples", [])}
    samples[args.name] = {
        "name": args.name,
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "joint_rad": {k: round(float(v), 5) for k, v in rad.items()},
        "corners": [[round(float(u), 2), round(float(v), 2)] for u, v in corners],
        "marker_id": mid,
    }
    data["samples"] = list(samples.values())
    dump_yaml(SAMPLES, data,
              header="# 手眼标定样本（tools/handeye_calib.py record 生成）\n"
                     "# 每个样本 = 一组关节角 + 一张图上 ArUco 四角的像素坐标\n")
    print(f"[样本] 已保存 {args.name}（共 {len(data['samples'])} 个）→ {SAMPLES}")
    return 0


def _pose_of(rotation, translation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def _score_marker(corners):
    edges = [np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)]
    return float(np.mean(edges))


def _plan_poses(servo_joints, robot, base_rad: dict, count: int,
                max_drift: float = 0.10, seeds: list | None = None):
    """用正解挑姿态：保证相机朝向差异尽量大，同时位置漂移受控。

    为什么不能按固定增量给：Arm2/Arm3/Arm4 都是俯仰轴，同向变化会互相抵消
    （实测中位数旋转差只有 2°，等于白采）。这里对每个候选算 FK，
    再用「与已选姿态集合的最小旋转角最大」的贪心策略挑选。
    """
    joint_of_sid = {j.sid: j for j in servo_joints}
    base_T = base_from_tool(robot, base_rad)
    base_R, base_p = base_T[:3, :3], base_T[:3, 3]

    def rotation_angle(R1, R2):
        cos = (np.trace(R1.T @ R2) - 1.0) / 2.0
        return math.degrees(math.acos(max(-1.0, min(1.0, cos))))

    # 候选集合：单轴 + 两轴组合 + 三轴组合
    deltas_list = []
    for sid in (2, 3, 4, 5):
        for d in (-35, -25, -16, -10, 10, 16, 25, 35):
            deltas_list.append({sid: d})
    for s1, s2 in ((2, 4), (3, 5), (2, 5), (3, 4), (4, 5), (2, 3)):
        for d1 in (-22, -12, 12, 22):
            for d2 in (-30, -16, 16, 30):
                deltas_list.append({s1: d1, s2: d2})
    for d2 in (-14, 14):
        for d4 in (-18, 18):
            for d5 in (-25, 25):
                deltas_list.append({2: d2, 4: d4, 5: d5})

    candidates = []
    for deltas in deltas_list:
        rad = dict(base_rad)
        feasible = True
        for sid, delta in deltas.items():
            joint = joint_of_sid.get(sid)
            if joint is None or joint.name not in rad:
                feasible = False
                break
            rad[joint.name] = (base_rad[joint.name]
                               + delta / (joint.direction * joint.scale_deg_per_rad))
        if not feasible:
            continue
        transform = base_from_tool(robot, rad)
        drift = float(np.linalg.norm(transform[:3, 3] - base_p))
        if drift > max_drift:
            continue
        candidates.append({
            "deltas": deltas,
            "rad": rad,
            "R": transform[:3, :3],
            "drift": drift,
        })

    # 已选集合：先放上已有样本的旋转，避免新姿态和老的重复
    selected_R = [np.eye(3) if False else R for R in (seeds or [])]
    selected_R.append(base_R)

    picked = []
    while len(picked) < count and candidates:
        best = None
        for cand in candidates:
            gap = min(rotation_angle(cand["R"], R) for R in selected_R)
            score = gap - 20.0 * cand["drift"]      # 同等旋转收益下优先小漂移
            if best is None or score > best[0]:
                best = (score, gap, cand)
        if best is None:
            break
        _, gap, cand = best
        candidates.remove(cand)
        if gap < 8.0:                                # 再挑也没有新信息了
            break
        picked.append({
            "label": " ".join(f"ID{s}{d:+d}°" for s, d in sorted(cand["deltas"].items())),
            "rad": cand["rad"],
            "drift": cand["drift"],
            "gap": gap,
        })
        selected_R.append(cand["R"])
    return picked


def _candidate_poses(base_deg: dict, range_deg: float):
    """以当前姿态为基准，生成一批「转动差异大」的候选姿态（绝对舵机角）。

    只改俯仰/滚转类关节（ID2 肩、ID3 肘、ID4 腕俯仰、ID5 腕滚转），
    底座 ID1 对倾斜帮助小，不动。每个候选都是绝对目标，从基准算，避免误差累积。
    """
    def clamp(v):
        return max(0.0, min(range_deg, v))

    base = {int(k): float(v) for k, v in base_deg.items()}
    plan = []
    singles = [
        ("肩+18°", {2: 18}), ("肩−18°", {2: -18}),
        ("肘+18°", {3: 18}), ("肘−18°", {3: -18}),
        ("腕俯+22°", {4: 22}), ("腕俯−22°", {4: -22}),
        ("腕俯+38°", {4: 38}), ("腕俯−38°", {4: -38}),
        ("腕滚+40°", {5: 40}), ("腕滚−40°", {5: -40}),
    ]
    combos = [
        ("肩+12 腕俯+16", {2: 12, 4: 16}),
        ("肩−12 腕俯−16", {2: -12, 4: -16}),
        ("肘+12 腕滚+28", {3: 12, 5: 28}),
        ("肘−12 腕滚−28", {3: -12, 5: -28}),
        ("腕俯+20 腕滚+24", {4: 20, 5: 24}),
        ("腕俯−20 腕滚−24", {4: -20, 5: -24}),
        ("肩+16 肘−12 腕俯+14", {2: 16, 3: -12, 4: 14}),
        ("肩−16 肘+12 腕俯−14", {2: -16, 3: 12, 4: -14}),
    ]
    for label, deltas in singles + combos:
        targets = dict(base)
        for sid, delta in deltas.items():
            if sid in targets:
                targets[sid] = clamp(base[sid] + delta)
        if all(abs(targets[s] - base[s]) > 1.0 for s in deltas if s in base):
            plan.append((label, targets))
    return plan


def cmd_autosweep(args) -> int:
    """自动扫掠：程序自己移臂 → 停稳 → 取图检测 → 变姿态，直到采够样本。

    舵机通电即有保持力矩，只能靠指令移动；本命令把「移-停-拍-记」全自动化，
    人只需要盯着别让手臂撞到东西。
    """
    import tkinter as tk

    sys.path.insert(0, str(VISION.parent / "ServoHost"))
    import cv2

    import servo_host_gui as host
    from dofbot_mj.servo_link import probe_positions

    settings, robot = robot_model(Path(args.config))
    range_deg = float(settings.servo.range_deg)

    # 1) 先读当前姿态作基准（此时串口还没被 GUI 占用）
    print("[基准] 读取当前舵机角度 …")
    try:
        rad0, base_deg = probe_positions(settings.servo, log=lambda *a: None)
    except Exception as exc:
        print(f"[错误] 读舵机失败：{exc}")
        return 1
    print("       " + " ".join(f"ID{s}:{d:6.1f}°" for s, d in sorted(base_deg.items())))

    # 自适应扫掠：每次只转一小步，看得到就收录并继续往前推，看不到就退回换方向。
    # 固定大角度计划实测会把标记转出画面（8 个姿态只成功 2 个），所以改成试探式。
    existing = load_yaml(SAMPLES).get("samples", [])
    print(f"[计划] 自适应扫掠：每步 {args.step_deg:.0f}°，轴 2/3/4/5 正反各推进，"
          f"目标 {args.target} 个（已有 {len(existing)} 个）")
    if args.dry_run:
        print(f"       轴顺序：{args.axes}，每个方向最多 {args.per_direction} 步")
        print("\n[DRY] 只打印计划，不连串口、不开相机、不动机械臂")
        return 0
    # 2) 起 GUI（它持有串口，移臂和读位置都走它）
    root = tk.Tk()
    root.withdraw()
    gui = host.ServoHostApp(root)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            root.update()
            time.sleep(0.02)

    gui.refresh_ports()
    names = list(gui.com_box["values"])
    target = None
    if settings.servo.port:
        target = next((n for n in names if settings.servo.port in n), None)
    if target is None:
        from dofbot_mj.servo_link import detect_port

        want = detect_port()
        if want:
            target = next((n for n in names if want in n), None)
    if target is None and names:
        target = names[0]
    if target is None:
        print("[错误] 没有可用串口")
        root.destroy()
        return 1
    gui.com_var.set(target)
    gui.baud_var.set(str(settings.servo.baudrate))
    gui.mode_var.set(host.MODE_STM32)
    gui.range_var.set(str(int(range_deg)))
    pump(0.2)
    gui._on_connect()
    pump(1.5)
    if gui.ser is None:
        print("[错误] 串口连接失败")
        root.destroy()
        return 1
    print(f"[串口] {target} 已连接（{range_deg:.0f}° 满行程）")

    def read_axis(sid: int, settle: float = 0.6):
        import re

        gui.id_var.set(str(sid))
        gui.range_var.set(str(int(range_deg)))
        gui.pos_var.set("读取中")
        gui._on_read()
        end = time.monotonic() + settle
        while time.monotonic() < end:
            root.update()
            time.sleep(0.02)
            match = re.search(r"当前位置:\s*(\d+)", gui.pos_var.get())
            if match:
                return int(match.group(1)) * range_deg / 4095.0
        return None

    last_commanded = dict(base_deg)

    def move_to(targets: dict):
        # Send only joints that change; repeatedly commanding the sticky base
        # joint while sweeping pitch axes can move the arm unexpectedly.
        for sid, deg in sorted(targets.items()):
            if abs(deg - last_commanded[sid]) < 0.2:
                continue
            gui.id_var.set(str(sid))
            gui.angle_entry_var.set(f"{deg:.1f}")
            gui.time_var.set(str(args.move_ms))
            gui.range_var.set(str(int(range_deg)))
            gui._on_move()
            pump(0.05)
            last_commanded[sid] = deg
        pump(args.settle)

    # 自碰撞检查：每个候选姿态动之前先算，连中间插值路径一起查
    from dofbot_mj.collision import CollisionChecker, describe as describe_hits

    checker = CollisionChecker(args.collision_model)

    def pose_rad(servo_deg: dict) -> dict:
        rad = {}
        for joint in settings.servo.joints:
            if joint.sid in servo_deg:
                rad[joint.name] = ((servo_deg[joint.sid] - joint.offset_deg)
                                   / (joint.direction * joint.scale_deg_per_rad))
        return rad

    def collision_free(a: dict, b: dict) -> tuple[bool, str]:
        """检查从 a 到 b 的直线插值路径（含端点）是否全程无自碰撞。"""
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            blend = {sid: a[sid] + (b[sid] - a[sid]) * t
                     for sid in b if sid in a}
            joint_pose = pose_rad(blend)
            for name, value in joint_pose.items():
                lo, hi = robot.limits(name)
                if not lo <= value <= hi:
                    return False, f"{name} outside joint limits"
            if any(not 0 <= value <= range_deg for value in blend.values()):
                return False, "outside servo range"
            hits = checker.check(joint_pose)
            if hits:
                return False, f"t={t:.2f} {describe_hits(hits)}"
        return True, ""

    # 3) 开相机
    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    print("[相机] 打开 …")
    cam = OrbbecDCW2()
    cam.set_laser(True)
    cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
    time.sleep(1.2)

    data = load_yaml(SAMPLES)
    marker = data.get("marker", {})
    # 有多标记板规格就优先用它（转角能开大，精度更高）
    board_spec = load_yaml(ROOT / "calib" / "board.yaml") or None
    if board_spec:
        args.dict = board_spec.get("dict", args.dict)
        size_mm = float(board_spec["marker_mm"])
        marker_id = None
        print(f"[标记] 多标记板 {args.dict} {board_spec['cols']}x{board_spec['rows']}"
              f"，单标记 {size_mm:g} mm，间距 {board_spec['gap_mm']:g} mm")
    else:
        size_mm = float(args.size_mm if args.size_mm is not None
                        else marker.get("size_mm", 125.0))
        marker_id = args.id if args.id is not None else marker.get("id", 0)
        print(f"[标记] {args.dict} id={marker_id} 边长 {size_mm:g} mm")
    dictionary = aruco_dict(args.dict)

    saved = 0
    tried = 0
    current = dict(base_deg)

    def capture_and_save() -> bool:
        """取图+检测+读回角度+存档；失败返回 False（调用方负责退回）。"""
        nonlocal saved, tried
        tried += 1
        pair = None
        for _ in range(8):
            pair = cam.get_frame_pair(timeout_ms=2000)
            if pair.get("color") is not None:
                break
        if pair is None or pair.get("color") is None:
            print("      取帧失败")
            return False
        found = detect_all_markers(pair["color"], dictionary)
        if not found:
            print("      没检测到标记")
            return False
        side = _score_marker(found[0][1])
        if side < args.min_px:
            print(f"      标记只有 {side:.1f} px（<{args.min_px:g}）")
            return False
        actual = {}
        for sid in sorted(current):
            deg = None
            for _ in range(3):
                deg = read_axis(sid, settle=1.0)
                if deg is not None:
                    break
            if deg is not None:
                actual[sid] = deg
        if len(actual) < len(current):
            print("      读回角度不完整")
            return False
        rad = {}
        for joint in settings.servo.joints:
            if joint.sid in actual:
                rad[joint.name] = ((actual[joint.sid] - joint.offset_deg)
                                   / (joint.direction * joint.scale_deg_per_rad))
        # 去重：相机在 Arm4 上，转动 Arm5 不会移动相机；
        # 位置/朝向与已有样本太接近的姿态对求解毫无贡献，直接跳过。
        snapshot = base_from_tool(robot, rad)
        for existing in data.get("samples", []):
            try:
                other = base_from_tool(robot, existing["joint_rad"])
            except Exception:
                continue
            dist_m = float(np.linalg.norm(snapshot[:3, 3] - other[:3, 3]))
            cos = (np.trace(snapshot[:3, :3].T @ other[:3, :3]) - 1.0) / 2.0
            angle = float(np.degrees(np.arccos(max(-1.0, min(1.0, cos)))))
            if dist_m < args.min_separation_m and angle < args.min_separation_deg:
                print(f"      与 {existing.get('name')} 的相机位姿几乎相同"
                      f"（{dist_m*1000:.0f}mm / {angle:.1f}°），跳过")
                return False
        name = f"p{len(data.get('samples', [])) + 1:02d}"
        count = _save_sample(data, name, rad, found,
                             (pair["color"].shape[1], pair["color"].shape[0]),
                             board_spec, size_mm, args.dict, marker_id)
        saved += 1
        print(f"      [OK] {name}  标记 {len(found)} 个，最大边长 {side:.1f} px  "
              f"累计 {count} 个")
        return True

    try:
        for sid in args.axes:
            for sign in (1, -1):
                if saved >= args.target:
                    break
                # 每个方向都从基准姿态出发，避免累积漂移把标记逼到画面边缘
                if any(abs(current[k] - base_deg[k]) > 0.5 for k in current):
                    move_to(base_deg)
                    current = dict(base_deg)
                steps = 0
                while steps < args.per_direction and saved < args.target:
                    target = dict(current)
                    target[sid] = max(0.0, min(range_deg,
                                               current[sid] + sign * args.step_deg))
                    if abs(target[sid] - current[sid]) < 0.5:
                        break
                    print(f"\n--- ID{sid} {sign:+d} {args.step_deg:.0f}° "
                          f"第 {steps + 1} 步 ---")
                    safe, why = collision_free(current, target)
                    if not safe:
                        print(f"      [拒绝] 会自碰撞：{why}")
                        print("             换下一个方向")
                        break
                    move_to(target)
                    if capture_and_save():
                        current = target
                        steps += 1
                    else:
                        print("      退回上一步姿态")
                        move_to(current)
                        break
            if saved >= args.target:
                break
    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        print("\n[收尾] 回到基准姿态 …")
        try:
            move_to(base_deg)
        except Exception:
            pass
        cam.close()
        gui._poll = lambda: None
        gui._on_close()
        try:
            root.destroy()
        except Exception:
            pass

    total = len(load_yaml(SAMPLES).get("samples", []))
    print(f"\n[结束] 本次新增 {saved} 个（尝试 {tried} 个姿态），累计 {total} 个")
    if total >= 3:
        print("[下一步] python tools\\handeye_calib.py solve")
    return 0


def _save_sample(data: dict, name: str, rad: dict, markers, image_size,
                 board_spec: dict | None, size_mm: float, dict_name: str,
                 marker_id):
    """markers: [(id, corners(4,2))]，单标记时就是一条。"""
    samples = {s.get("name"): s for s in data.get("samples", []) if s.get("name")}
    biggest = markers[0][1] if markers else np.zeros((4, 2))
    samples[name] = {
        "name": name,
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "joint_rad": {k: round(float(v), 5) for k, v in rad.items()},
        "markers": [{"id": int(mid),
                     "corners": [[round(float(u), 2), round(float(v), 2)]
                                 for u, v in pts]}
                    for mid, pts in markers],
        # 兼容旧字段
        "corners": [[round(float(u), 2), round(float(v), 2)] for u, v in biggest],
        "marker_id": int(markers[0][0]) if markers else None,
    }
    if board_spec:
        data["board"] = board_spec
        data["marker"] = {"dict": board_spec["dict"], "id": marker_id,
                          "size_mm": float(board_spec["marker_mm"]),
                          "image_size": [int(image_size[0]), int(image_size[1])]}
    else:
        data["marker"] = {"dict": dict_name, "id": marker_id,
                          "size_mm": float(size_mm),
                          "image_size": [int(image_size[0]), int(image_size[1])]}
    data["samples"] = list(samples.values())
    dump_yaml(SAMPLES, data,
              header="# 手眼标定样本（tools/handeye_calib.py 生成）\n"
                     "# 每个样本 = 一组关节角 + 该帧检测到的全部 ArUco 四角像素坐标\n")
    return len(samples)


def cmd_record_board(args) -> int:
    """Record one multi-marker board frame at the current read-only servo pose."""
    import cv2
    from dofbot_mj.servo_link import probe_positions

    settings, _robot = robot_model(Path(args.config))
    board_spec = load_yaml(ROOT / "calib" / "board.yaml")
    if not board_spec:
        raise RuntimeError("Missing calib/board.yaml")
    rad, deg = probe_positions(settings.servo, log=lambda *a: None)
    print("[servo] " + " ".join(f"ID{i}:{v:.2f}" for i, v in sorted(deg.items())))

    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
        time.sleep(1.2)
        pair = None
        for _ in range(10):
            pair = cam.get_frame_pair(timeout_ms=1500)
            if pair.get("color") is not None:
                break
        if pair is None or pair.get("color") is None:
            raise RuntimeError("No board RGB frame")
        color = pair["color"]
    finally:
        cam.close()

    found = detect_all_markers(color, aruco_dict(board_spec["dict"]))
    found = [(mid, corners) for mid, corners in found if mid in board_spec["marker_ids"]]
    if len(found) < 3:
        raise RuntimeError(f"Only {len(found)} board markers visible; need at least three")
    largest = max(_score_marker(corners) for _, corners in found)
    if largest < args.min_px:
        raise RuntimeError(f"Largest marker only {largest:.1f} pixels")
    data = load_yaml(SAMPLES)
    count = _save_sample(data, args.name, rad, found,
                         (color.shape[1], color.shape[0]), board_spec,
                         float(board_spec["marker_mm"]), board_spec["dict"], None)
    preview = Path(args.preview) if args.preview else SAMPLES.with_name(args.name + "_board.png")
    preview.parent.mkdir(parents=True, exist_ok=True)
    view = color.copy()
    for mid, corners in found:
        cv2.polylines(view, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(view, str(mid), tuple(corners[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imwrite(str(preview), view)
    print(f"[board] {len(found)} markers, largest {largest:.1f}px; sample {count}: {SAMPLES}")
    print(f"[preview] {preview}")
    return 0


def cmd_wizard(args) -> int:
    """相机只开一次，每按一次回车采集一个姿态。"""
    import cv2

    from dofbot_mj.servo_link import probe_positions

    settings, robot = robot_model(Path(args.config))
    dictionary = aruco_dict(args.dict)

    data = load_yaml(SAMPLES)
    marker = data.get("marker", {})
    if args.size_mm is None:
        size_mm = float(marker.get("size_mm", 125.0))
    else:
        size_mm = float(args.size_mm)
    marker_id = marker.get("id", args.id)

    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    print("[相机] 打开（彩色+深度，D2C 对齐）…")
    cam = OrbbecDCW2()
    cam.set_laser(True)
    cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
    time.sleep(1.2)

    print()
    print("=" * 62)
    print("  采集向导：摆好一个姿态 → 停稳 → 按回车采集")
    print("  直接按回车 = 自动编号；输入 q 回车 = 结束并给出求解提示")
    print(f"  标记 {args.dict} id={marker_id} 边长 {size_mm:g} mm")
    print("=" * 62)
    print(f"  已有样本 {len(data.get('samples', []))} 个")

    saved = 0
    try:
        while True:
            raw = input("\n[回车采集 / q 结束] ").strip()
            if raw.lower() in ("q", "quit", "exit"):
                break
            name = raw or f"p{len(data.get('samples', [])) + 1:02d}"

            pair = None
            for _ in range(10):
                pair = cam.get_frame_pair(timeout_ms=2000)
                if pair.get("color") is not None:
                    break
            if pair is None or pair.get("color") is None:
                print("  [跳过] 取帧失败")
                continue

            corners, mid = detect_marker(pair["color"], dictionary, marker_id)
            if corners is None:
                print("  [跳过] 没检测到标记（检查是否在画面内、是否被遮挡）")
                continue
            side = _score_marker(corners)
            if side < args.min_px:
                print(f"  [跳过] 标记只有 {side:.1f} px，小于阈值 {args.min_px:g} px")
                continue

            try:
                rad, _deg = probe_positions(settings.servo, log=lambda *a: None)
            except Exception as exc:
                print(f"  [跳过] 读舵机失败：{exc}")
                continue

            count = _save_sample(data, name, rad, corners, mid,
                                 (pair["color"].shape[1], pair["color"].shape[0]),
                                 size_mm, args.dict, marker_id)
            saved += 1
            print(f"  [OK] {name}: 边长 {side:.1f} px，已存（共 {count} 个）")
            print("       姿态 " + " ".join(f"{k.split('_')[0]}={v:+.3f}"
                                            for k, v in rad.items()))
    except KeyboardInterrupt:
        print("\n[中断]")
    finally:
        cam.close()
        print(f"\n[结束] 本次新增 {saved} 个样本")

    total = len(load_yaml(SAMPLES).get("samples", []))
    if total >= 3:
        print(f"[下一步] 已累计 {total} 个样本，求解："
              f"python tools\\handeye_calib.py solve")
    else:
        print(f"[提示] 只有 {total} 个样本，至少需要 3 个（建议 8~12 个）")
    return 0


def _solve_nonlinear(R_g, t_g, R_c, t_c, fix_plate_z: bool = True):
    """带物理边界的非线性求解：min Σ‖T_base←tool,i · X · p_cam,i − P‖²。

    两个物理先验：
      1. 相机装在末端连杆上，平移不超过 25 cm；
      2. 标定板**平放在桌面上**，板心高度就是 z=0（fix_plate_z）。
    第 2 条很关键：滚转为主的运动对平移约束不足，把 P 的 z 固定为 0
    才能打破简并，否则解会漂到桌面上方几十厘米。
    """
    from scipy.optimize import least_squares

    import cv2

    # 统一成一维，否则 (3,1) 和 (3,) 混算会广播成 (3,3)
    R_g = [np.asarray(r, dtype=float) for r in R_g]
    t_g = [np.asarray(x, dtype=float).ravel() for x in t_g]
    R_c = [np.asarray(r, dtype=float) for r in R_c]
    t_c = [np.asarray(x, dtype=float).ravel() for x in t_c]

    def unpack(params):
        rvec = params[0:3]
        t = params[3:6]
        if fix_plate_z:
            P = np.array([params[6], params[7], 0.0])
        else:
            P = params[6:9]
        R, _ = cv2.Rodrigues(rvec)
        return R, t, P

    def residual(params):
        R, t, P = unpack(params)
        out = []
        for i in range(len(R_c)):
            p_base = R_g[i] @ (R @ t_c[i] + t) + t_g[i]
            out.extend(p_base - P)
        return np.array(out)

    # 初始猜测：旋转取零、平移取小量、板心位置取样本平均
    R0 = np.eye(3)
    t0 = np.array([0.02, 0.0, 0.0])
    P0 = np.mean([R_g[i] @ (R0 @ t_c[i] + t0) + t_g[i] for i in range(len(R_c))], axis=0)
    if fix_plate_z:
        x0 = np.concatenate([cv2.Rodrigues(R0)[0].ravel(), t0, P0[:2]])
        lower = np.concatenate([[-np.pi] * 3, [-0.25] * 3, [-0.9, -0.9]])
        upper = np.concatenate([[np.pi] * 3, [0.25] * 3, [0.9, 0.9]])
    else:
        x0 = np.concatenate([cv2.Rodrigues(R0)[0].ravel(), t0, P0])
        lower = np.concatenate([[-np.pi] * 3, [-0.25] * 3, [-0.8, -0.8, -0.15]])
        upper = np.concatenate([[np.pi] * 3, [0.25] * 3, [0.8, 0.8, 0.35]])
    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    result = least_squares(residual, x0, bounds=(lower, upper),
                           loss="soft_l1", f_scale=0.01, max_nfev=4000)
    R, t, P = unpack(result.x)
    residual_norm = float(np.linalg.norm(residual(result.x)))
    return R, t, P, residual_norm


def cmd_solve(args) -> int:
    import cv2

    data = load_yaml(SAMPLES)
    samples = data.get("samples", [])
    marker = data.get("marker", {})
    if len(samples) < 3:
        print(f"[错误] 只有 {len(samples)} 个样本，至少要 3 个（建议 8~12 个）")
        return 1

    # A tiny sweep can fit the board positions almost exactly while leaving the
    # camera rotation unconstrained.  In particular, samples from one pitch
    # servo must not be published as a usable hand-eye calibration.
    yaw = np.array([s["joint_rad"]["Arm1_Joint"] for s in samples])
    pitch = np.array([sum(s["joint_rad"][name] for name in
                          ("Arm2_Joint", "Arm3_Joint", "Arm4_Joint"))
                      for s in samples])
    yaw_span = float(np.degrees(np.ptp(yaw)))
    pitch_span = float(np.degrees(np.ptp(pitch)))
    if len(samples) < 6 or yaw_span < 8.0 or pitch_span < 10.0:
        print(f"[错误] 样本运动不足：{len(samples)} 视图，底座转角跨度 "
              f"{yaw_span:.1f}°，俯仰跨度 {pitch_span:.1f}°。"
              "至少需要 6 视图、8° 底座和 10° 俯仰跨度；未写出标定。")
        return 1

    size_mm = float(marker.get("size_mm", 40.0))
    image_size = tuple(int(v) for v in marker.get("image_size", [640, 480]))
    board_spec = data.get("board")

    # 物点一律用「米」：solvePnP 的 tvec 单位跟随物点单位，而机器人正解是米。
    if board_spec:
        board, _dict = aruco_board(board_spec)
        board_obj = board.getObjPoints()
        board_ids = [int(v) for v in board.getIds().flatten()]
        index_of_id = {mid: k for k, mid in enumerate(board_ids)}
        objpoints, imgpoints, usable = [], [], []
        for sample in samples:
            objs, imgs = [], []
            for entry in sample.get("markers", []):
                index = index_of_id.get(int(entry["id"]))
                if index is None:
                    continue
                objs.append(board_obj[index].astype(np.float32))
                imgs.append(np.array(entry["corners"], dtype=np.float32))
            if objs:
                objpoints.append(np.vstack(objs))
                imgpoints.append(np.vstack(imgs))
                usable.append(sample)
        samples = usable
        print(f"[标记] 多标记板 {board_spec['cols']}x{board_spec['rows']}，"
              f"{len(samples)} 个视图可用")
    else:
        objp = marker_object_points(size_mm / 1000.0).astype(np.float32)
        objpoints = [objp.copy() for _ in samples]
        imgpoints = [np.array(s["corners"], dtype=np.float32) for s in samples]

    if len(samples) < 3:
        print(f"[错误] 可用视图只有 {len(samples)} 个，至少要 3 个")
        return 1

    # 1) 相机内参：用这些视图自标定
    #    这里的视图只覆盖标定板的一个小角度范围，若让主点和纵横比自由变化，
    #    它们会和畸变互相补偿，把主点拉到画面外（实测 cy 跑到 297，中心是 240），
    #    进而歪曲每个视图的板子位姿。普通 RGB 相机是方形像素、主点约在中心，
    #    因此锁死这两项，只估焦距与径向畸变。
    print(f"[内参] 用 {len(samples)} 个视图标定相机内参（锁主点/纵横比）…")
    flags = (cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
             cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    rms, K, dist, _rv, _tv = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags)
    print(f"       fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  cx={K[0,2]:.1f}  cy={K[1,2]:.1f}"
          f"  重投影 RMS={rms:.3f} px")

    # 2) 每个视图的 T_cam←target（单标记=标记系；多标记=整块板坐标系）
    _, robot = robot_model(Path(args.config))
    R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
    for index, sample in enumerate(samples):
        ok, rvec, tvec = cv2.solvePnP(objpoints[index], imgpoints[index], K, dist)
        if not ok:
            print(f"[警告] 样本 {sample['name']} 位姿求解失败，跳过")
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        R_t2c.append(rotation)
        t_t2c.append(tvec.reshape(3, 1))
        tool = base_from_tool(robot, sample["joint_rad"])
        R_g2b.append(tool[:3, :3])
        t_g2b.append(tool[:3, 3].reshape(3, 1))

    if len(R_t2c) < 3:
        print("[错误] 有效样本不足 3 个")
        return 1

    # 3) 手眼标定：多种方法取最优
    print(f"[手眼] 用 {len(R_t2c)} 个有效样本求解 T_tool←cam …")
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    best = None
    for name, method in methods.items():
        try:
            R_x, t_x = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
        except cv2.error as exc:
            print(f"       {name}: 失败（{exc}）")
            continue
        X = _pose_of(R_x, t_x.reshape(3))
        # 校验：标记在基座系里的位置应当对所有姿态一致
        positions = []
        for index, sample in enumerate(samples[:len(R_t2c)]):
            cam_from_marker = _pose_of(R_t2c[index], t_t2c[index].reshape(3))
            tool = base_from_tool(robot, sample["joint_rad"])
            base_from_marker = tool @ X @ cam_from_marker
            positions.append(base_from_marker[:3, 3])
        positions = np.array(positions)
        spread = float(np.max(np.ptp(positions, axis=0)))
        std = float(np.mean(np.std(positions, axis=0)))
        print(f"       {name:<10} 标记基座位置离散：峰峰值 {spread*1000:7.1f} mm"
              f"  标准差 {std*1000:7.1f} mm")
        if best is None or std < best[0]:
            best = (std, name, X, positions)

    if best is None:
        print("[错误] 所有方法都失败")
        return 1
    std, method, X, positions = best
    print(f"\n[结果] OpenCV 最优 {method}，标记位置标准差 {std*1000:.1f} mm")

    # 4) 非线性求解（带物理边界），与上面的解析解比较
    print("[非线性] 带边界的整体最小二乘（相机距末端 ≤25 cm、标记在地面附近）…")
    R_nl, t_nl, P_nl, res_norm = _solve_nonlinear(R_g2b, t_g2b, R_t2c, t_t2c)
    X_nl = _pose_of(R_nl, t_nl)
    pos_nl = []
    for index, sample in enumerate(samples[:len(R_t2c)]):
        tool = base_from_tool(robot, sample["joint_rad"])
        cam_from_marker = _pose_of(R_t2c[index], t_t2c[index].reshape(3))
        pos_nl.append((tool @ X_nl @ cam_from_marker)[:3, 3])
    pos_nl = np.array(pos_nl)
    std_nl = float(np.mean(np.std(pos_nl, axis=0)))
    print(f"       残差范数 {res_norm*1000:.1f} mm（{len(R_t2c)} 姿态 ×3）  "
          f"标记位置标准差 {std_nl*1000:.2f} mm")
    print(f"       相机在 tool 系位置 {np.round(t_nl*1000, 1)} mm   "
          f"标记在基座系 {np.round(P_nl*1000, 1)} mm")

    if std_nl < std or abs(np.linalg.norm(t_nl)) < 0.25:
        std, method, X, positions = std_nl, "NL-BOUNDED", X_nl, pos_nl
        print("[结果] 采用非线性解（解析解的平移不可信）")

    print("       T_tool←cam =")
    for row in X:
        print("         " + " ".join(f"{v:9.4f}" for v in row))
    print(f"       相机在 tool 系下的位置：{np.round(X[:3,3]*1000, 1)} mm")
    print(f"       标记在基座系下的平均位置："
          f"{np.round(positions.mean(axis=0)*1000, 1)} mm")

    if std > args.max_std_mm / 1000.0:
        print(f"\n[警告] 离散度 {std*1000:.1f} mm 超过阈值 {args.max_std_mm:.0f} mm："
              f"姿态差异不够，或标记检测有误；建议补采更多、差异更大的姿态")

    dump_yaml(RESULT, {
        "tool_link": TOOL_LINK,
        "method": method,
        "marker": marker,
        "camera_matrix": {"fx": K[0, 0], "fy": K[1, 1], "cx": K[0, 2], "cy": K[1, 2],
                          "dist": dist.reshape(-1).tolist()},
        "T_tool_cam": X.reshape(4, 4).round(8).tolist(),
        "marker_in_base_m": positions.mean(axis=0).round(6).tolist(),
        "spread_std_mm": round(std * 1000, 2),
        "samples": len(R_t2c),
        "solved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }, header="# 手眼标定结果（tools/handeye_calib.py solve 生成）\n"
              "# T_tool_cam: 相机光学系 → DaBai_DCW2_Link 的 4x4 变换（米/弧度）\n")
    print(f"\n[写出] {RESULT}")
    return 0


def cmd_status(args) -> int:
    data = load_yaml(SAMPLES)
    samples = data.get("samples", [])
    marker = data.get("marker", {})
    print(f"样本文件：{SAMPLES}")
    if not samples:
        print("  还没有样本。先用 record 采集。")
    else:
        print(f"  标记：{marker.get('dict')} id={marker.get('id')} "
              f"边长 {marker.get('size_mm')} mm  图像 {marker.get('image_size')}")
        print(f"  样本 {len(samples)} 个：")
        for index, sample in enumerate(samples, 1):
            name = sample.get("name") or f"(未命名{index})"
            print(f"    {name:<8} {sample.get('recorded_at','')}")
    result = load_yaml(RESULT)
    if result:
        print(f"\n标定结果：{RESULT}")
        print(f"  方法 {result.get('method')}  样本 {result.get('samples')}  "
              f"离散 {result.get('spread_std_mm')} mm")
        print(f"  相机在 tool 系位置："
              f"{np.round(np.array(result['T_tool_cam'])[:3, 3] * 1000, 1)} mm")
    else:
        print("\n还没有标定结果。")
    return 0


def main(argv=None) -> int:
    global SAMPLES, RESULT
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="eye-in-hand 手眼标定（ArUco）")
    p.add_argument("--config", default=str(ROOT / "config" / "calibrated.yaml"))
    p.add_argument("--samples-file", type=Path, default=SAMPLES,
                   help="独立样本文件，避免混入旧标定样本")
    p.add_argument("--result-file", type=Path, default=RESULT)
    sub = p.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("marker", help="生成可打印的 ArUco 图")
    m.add_argument("-o", "--out", default=str(ROOT / "calib" / "aruco_40mm.png"))
    m.add_argument("--dict", default=DICT_NAME)
    m.add_argument("--id", type=int, default=0)
    m.add_argument("--size-mm", type=float, default=40.0)
    m.add_argument("--dpi", type=int, default=300)

    r = sub.add_parser("record", help="采集一个姿态的样本")
    r.add_argument("--name", required=True)
    r.add_argument("--dict", default=DICT_NAME)
    r.add_argument("--id", type=int, default=None)
    r.add_argument("--size-mm", type=float, default=None)
    r.add_argument("--preview", default="")

    rb = sub.add_parser("record-board", help="只读舵机并采集一帧多标记板")
    rb.add_argument("--name", required=True)
    rb.add_argument("--preview", default="")
    rb.add_argument("--min-px", type=float, default=60.0)

    s = sub.add_parser("solve", help="求解并校验")
    s.add_argument("--max-std-mm", type=float, default=5.0)

    w = sub.add_parser("wizard", help="采集向导：相机只开一次，每回车采一个样本")
    w.add_argument("--dict", default=DICT_NAME)
    w.add_argument("--id", type=int, default=None)
    w.add_argument("--size-mm", type=float, default=None)
    w.add_argument("--min-px", type=float, default=60.0,
                   help="标记边长低于该像素数就拒绝该样本")

    a = sub.add_parser("autosweep",
                       help="自动扫掠：程序自己移臂+采集（舵机通电无法手动摆）")
    a.add_argument("--dict", default=DICT_NAME)
    a.add_argument("--id", type=int, default=None)
    a.add_argument("--size-mm", type=float, default=None)
    a.add_argument("--min-px", type=float, default=60.0)
    a.add_argument("--target", type=int, default=10, help="目标采集样本数")
    a.add_argument("--max-poses", type=int, default=18, help="最多尝试多少个姿态")
    a.add_argument("--move-ms", type=int, default=700, help="每条 MOVE 运行时间")
    a.add_argument("--settle", type=float, default=1.0, help="移动后停稳等待秒数")
    a.add_argument("--step-deg", type=float, default=10.0, help="每次试探转多少度")
    a.add_argument("--per-direction", type=int, default=3, help="每个方向最多推进几步")
    a.add_argument("--min-separation-deg", type=float, default=5.0,
                   help="与已有相机姿态的最小角度差，较小扫掠可设 3°")
    a.add_argument("--min-separation-m", type=float, default=0.03)
    a.add_argument("--axes", type=int, nargs="*", default=[2, 3, 4, 5],
                   help="要试探的舵机 ID（默认 2 3 4 5）")
    a.add_argument("--dry-run", action="store_true",
                   help="只打印姿态计划，不连串口、不开相机、不动机械臂")
    a.add_argument("--collision-model", type=Path,
                   default=ROOT / "models" / "dofbot_pro_collision_visual.xml",
                   help="包含桌面碰撞体的机械臂碰撞模型")

    sub.add_parser("status", help="查看已有样本与结果")

    b = sub.add_parser("board", help="生成 A4 多标记标定板（推荐：转角可以开大）")
    b.add_argument("-o", "--out", default=str(ROOT / "calib" / "aruco_board.png"))
    b.add_argument("--dict", default=DICT_NAME)
    b.add_argument("--cols", type=int, default=2)
    b.add_argument("--rows", type=int, default=3)
    b.add_argument("--marker-mm", type=float, default=60.0)
    b.add_argument("--gap-mm", type=float, default=20.0)
    b.add_argument("--margin-mm", type=float, default=15.0)
    b.add_argument("--dpi", type=int, default=300)

    args = p.parse_args(argv)
    SAMPLES = Path(args.samples_file)
    RESULT = Path(args.result_file)

    if args.cmd == "record":
        data = load_yaml(SAMPLES)
        marker = data.get("marker", {})
        if args.size_mm is None:
            args.size_mm = float(marker.get("size_mm", 40.0))
        if args.id is None:
            args.id = marker.get("id", 0)

    handler = {"marker": cmd_marker, "record": cmd_record,
               "record-board": cmd_record_board, "solve": cmd_solve,
               "status": cmd_status, "wizard": cmd_wizard,
               "autosweep": cmd_autosweep, "board": cmd_board}[args.cmd]
    return handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
