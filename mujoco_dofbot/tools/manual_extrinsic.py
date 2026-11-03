#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人工/半自动外参：定出相机光轴在 DaBai_DCW2_Link 里的朝向与位置。

为什么能自动定朝向
------------------
桌面是水平的。把深度点云按某个候选朝向变换到基座坐标系后，如果朝向选对了，
点云里的主平面（桌面）就应该**垂直于世界 Z 轴**；选错了就会倾斜。
于是把 24 种正交朝向（3 个轴 × 置换 × 正负，行列式 +1）全试一遍，
用"平面法向与 Z 的夹角 + 平面拟合残差"打分即可。

平移的 z 分量也可以自动定：让拟合出的桌面正好落在 z=0。
剩下的横向偏移（光心相对连杆原点的 x、y）需要量一下，或者先给 0（误差几厘米）。

用法：
    python tools/manual_extrinsic.py --auto                 # 自动定朝向并写配置
    python tools/manual_extrinsic.py --offset-mm 5 0 20     # 指定光心偏移再定朝向
"""

from __future__ import annotations

import argparse
import itertools
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

OUT = ROOT / "config" / "extrinsic_manual.yaml"


def proper_rotations():
    """所有行列式为 +1 的轴置换/变号矩阵（共 24 个）。"""
    axes = np.eye(3)
    results = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([1, -1], repeat=3):
            m = np.zeros((3, 3))
            for col, (row, sign) in enumerate(zip(perm, signs)):
                m[row, col] = sign
            if np.linalg.det(m) > 0.5:
                results.append(m)
    return results


def plane_fit(points: np.ndarray):
    """返回 (法向, 偏置, 残差std)。法向为单位向量。"""
    centre = points.mean(axis=0)
    _u, _s, vt = np.linalg.svd(points - centre, full_matrices=False)
    normal = vt[-1]
    normal /= np.linalg.norm(normal) + 1e-12
    heights = (points - centre) @ normal
    return normal, float(centre @ normal), float(np.std(heights))


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="定出相机光轴朝向（半自动外参）")
    p.add_argument("--config", default=str(ROOT / "config" / "calibrated.yaml"))
    p.add_argument("--offset-mm", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   help="光心相对 DaBai_DCW2_Link 原点的偏移(mm)，连杆坐标系")
    p.add_argument("--min-mm", type=float, default=10.0)
    p.add_argument("--max-mm", type=float, default=1500.0)
    p.add_argument("--points", type=int, default=8000)
    args = p.parse_args(argv)

    from dofbot_mj.config import load_config
    from dofbot_mj.model import RobotModel
    from dofbot_mj.servo_link import probe_positions

    settings = load_config(args.config)
    sidecar = settings.model.mjcf.with_suffix(".json")
    mimic = (json.loads(sidecar.read_text(encoding="utf-8")) or {}).get("mimic", {}) \
        if sidecar.is_file() else {}
    robot = RobotModel(settings.model.mjcf, mimic=mimic)
    try:
        pose, _deg = probe_positions(settings.servo, log=lambda *a: None)
    except Exception as exc:
        print(f"[警告] 读舵机失败（{exc}），用零位")
        pose = {n: 0.0 for n in robot.driver_joints}
    robot.apply_kinematic(pose)

    body = next((i for i in range(robot.model.nbody)
                 if robot.model.body(i).name == "DaBai_DCW2_Link"), -1)
    if body < 0:
        print("[错误] 模型里没有 DaBai_DCW2_Link")
        return 1
    link_pos = robot.data.xpos[body].copy()
    link_mat = robot.data.xmat[body].reshape(3, 3).copy()
    offset = np.array(args.offset_mm, dtype=float) / 1000.0
    optical_centre = link_pos + link_mat @ offset

    # ---- 取一帧深度 ----
    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    print("[相机] 打开并取一帧 …")
    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
        time.sleep(1.2)
        pair = None
        for _ in range(20):
            pair = cam.get_frame_pair(timeout_ms=3000)
            if pair.get("depth") is not None:
                break
    finally:
        cam.close()
    if pair is None or pair.get("depth") is None:
        print("[错误] 取帧失败")
        return 1
    depth = pair["depth"]

    # 反投影到相机系（内参按已实测校准的那套）
    h, w = depth.shape
    fx = 482.8
    fy = 482.8
    cx, cy = 319.5, 239.5
    valid = (depth >= args.min_mm) & (depth <= args.max_mm)
    step = max(1, int(math.ceil(math.sqrt(valid.sum() / float(args.points)))))
    keep = np.zeros_like(valid)
    keep[::step, ::step] = True
    mask = valid & keep
    vs, us = np.nonzero(mask)
    z = depth[vs, us].astype(np.float64) / 1000.0
    pts_cam = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)
    print(f"[点云] {len(pts_cam)} 个点，深度 {z.min():.2f}~{z.max():.2f} m")

    # ---- 24 种朝向打分 ----
    # 相机系 (X右, Y下, Z前) → 连杆系；world = link_pos + link_mat @ (offset + R_cam2link @ p)
    best = None
    print(f"\n{'#':>3}{'朝向(相机Z轴→连杆)':>22}{'平面残差mm':>12}{'法向·Z':>9}{'得分':>8}")
    for index, rot in enumerate(proper_rotations()):
        world = optical_centre[None, :] + pts_cam @ (link_mat @ rot).T
        if len(world) < 200:
            continue
        normal, bias, resid = plane_fit(world)
        verticality = abs(float(normal[2]))
        score = resid * 1000.0 + (1.0 - verticality) * 200.0
        if best is None or score < best[0]:
            best = (score, rot, normal, bias, resid, verticality)
    _, rot, normal, bias, resid, verticality = best
    forward = rot[:, 2]
    print(f"\n[最优] 相机 Z 轴在连杆系下 = {np.round(forward, 3)}")
    print(f"       平面残差 {resid*1000:.1f} mm，法向竖直度 {verticality:.3f}")
    if verticality < 0.9:
        print("[警告] 最佳朝向的法向仍不够竖直，桌面可能不是画面里的主平面")

    # 让桌面落到 z=0：调整沿相机 down 轴的偏移
    # world 的 z 偏置 = bias；直接把整个点云沿世界 Z 平移 -bias 即可，
    # 等价于把光心高度改成 optical_centre_z - bias
    optical_centre_z = float(optical_centre[2] - bias)

    # ---- 输出 ----
    names = {}
    for name, vec in (("x", rot[:, 0]), ("y", rot[:, 1]), ("z", rot[:, 2])):
        axis = int(np.argmax(np.abs(vec)))
        names[name] = ("+" if vec[axis] > 0 else "-") + "xyz"[axis]
    print(f"\n[朝向] 相机 右={names['x']}  下={names['y']}  前={names['z']}  （相对连杆坐标系）")
    print(f"[平移] 光心偏移 {np.round(offset*1000,1)} mm（连杆系）")

    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = offset
    data = {
        "tool_link": "DaBai_DCW2_Link",
        "source": "manual/auto（按桌面平面定朝向）",
        "T_tool_cam": [[float(v) for v in row] for row in T.round(6)],
        "camera_axes_in_link": names,
        "cam_forward": names["z"], "cam_right": names["x"], "cam_down": names["y"],
        "optical_offset_mm": [float(round(float(v) * 1000, 2)) for v in offset],
        "plane_residual_mm": float(round(resid * 1000, 2)),
        "plane_verticality": float(round(verticality, 3)),
        "table_bias_mm": float(bias * 1000),
        "optical_centre_z_m": float(optical_centre_z),
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
    }
    import yaml

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        "# 相机外参（半自动：按桌面水平面定朝向）\n"
        "# 由 tools/manual_extrinsic.py 生成；不是手眼标定的结果\n"
        + yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"\n[写出] {OUT}")
    print(f"       depth_cloud 用法：--cam-forward {names['z']} "
          f"--cam-right {names['x']} --cam-down {names['y']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
