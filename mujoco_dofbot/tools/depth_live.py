#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""交互窗口：MuJoCo 里实时显示深度相机的点云（带彩色）。

  * 机械臂按实机当前舵机角度摆好（只读一次）；
  * 相机持续取帧，深度反投影成点云，用第一帧拟合的桌面基准摆正，
    之后沿用同一基准，所以画面不会逐帧抖动；
  * 点云颜色取自彩色图，深度盲区阈值沿用 strawberry_demo.py 的 DEPTH_VALID_MIN。

用法：
    python tools/depth_live.py                # 打开窗口
    python tools/depth_live.py --points 6000 --hz 10
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

import mujoco

ROOT = Path(__file__).resolve().parent.parent
REPO = ROOT.parent
VISION = REPO / "yolo_pkg"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(VISION))


def import_vision():
    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    return OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH


def depth_min_default() -> int:
    try:
        for line in (VISION / "strawberry_demo.py").read_text(
                encoding="utf-8").splitlines():
            if line.strip().startswith("DEPTH_VALID_MIN"):
                return int(line.split("=")[1].split("#")[0].strip())
    except Exception:
        pass
    return 10


def backproject(depth, color, min_mm, max_mm, hfov, vfov, stride):
    h, w = depth.shape
    fx = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    fy = (h / 2.0) / math.tan(math.radians(vfov) / 2.0)
    valid = (depth >= min_mm) & (depth <= max_mm)
    sub = np.zeros_like(valid)
    sub[::stride, ::stride] = True
    mask = valid & sub
    vs, us = np.nonzero(mask)
    if len(vs) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    z = depth[vs, us].astype(np.float32) / 1000.0
    pts = np.stack([(us - w / 2.0) * z / fx, (vs - h / 2.0) * z / fy, z], axis=1)
    if color is not None and color.shape[:2] == depth.shape:
        rgb = color[vs, us][:, ::-1].astype(np.float32) / 255.0
    else:
        rgb = np.full((len(z), 3), 0.7, dtype=np.float32)
    return pts, rgb


def level_basis(points):
    """用第一帧拟合桌面，返回 (right, forward, up, offset)。之后每帧沿用。"""
    sample = points[:: max(1, len(points) // 20000)]
    centroid = sample.mean(axis=0)
    _u, _s, vt = np.linalg.svd(sample - centroid, full_matrices=False)
    up = vt[-1]
    if up @ centroid > 0:
        up = -up
    up /= np.linalg.norm(up) + 1e-12
    ex = np.array([1.0, 0.0, 0.0])
    if abs(ex @ up) > 0.95:
        ex = np.array([0.0, 1.0, 0.0])
    right = ex - (ex @ up) * up
    right /= np.linalg.norm(right) + 1e-12
    forward = np.cross(up, right)
    offset = float(np.median(points @ up))
    return right, forward, up, offset


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="MuJoCo 窗口里实时看点云")
    p.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    p.add_argument("--points", type=int, default=6000)
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--hfov", type=float, default=64.0)
    p.add_argument("--vfov", type=float, default=55.0)
    p.add_argument("--center", action="store_true", default=True,
                   help="把点云摆到机器人正前方（缺手眼标定时的显示折中）")
    p.add_argument("--no-center", dest="center", action="store_false")
    p.add_argument("--no-servo", action="store_true")
    args = p.parse_args(argv)

    from dofbot_mj.config import load_config
    from dofbot_mj.model import RobotModel

    settings = load_config(args.config)
    sidecar = settings.model.mjcf.with_suffix(".json")
    mimic = (json.loads(sidecar.read_text(encoding="utf-8")) or {}).get("mimic", {}) \
        if sidecar.is_file() else {}
    robot = RobotModel(settings.model.mjcf, mimic=mimic)

    if args.no_servo:
        pose = {n: 0.0 for n in robot.driver_joints}
    else:
        from dofbot_mj.servo_link import probe_positions

        try:
            pose, _deg = probe_positions(settings.servo, log=lambda *a: None)
            print("[姿态] 已按实机舵机角度摆好")
        except Exception as exc:
            print(f"[警告] 读舵机失败（{exc}），用零位")
            pose = {n: 0.0 for n in robot.driver_joints}
    robot.apply_kinematic(pose)

    dmin = depth_min_default()
    print(f"[盲区] DEPTH_VALID_MIN = {dmin} mm")

    OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH = import_vision()
    cam = OrbbecDCW2()
    cam.set_laser(True)
    cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
    time.sleep(1.2)

    # 面积换算采样步长
    first = None
    for _ in range(20):
        pair = cam.get_frame_pair(timeout_ms=3000)
        if pair.get("depth") is not None:
            first = pair
            break
    if first is None:
        print("[错误] 取帧失败")
        cam.close()
        return 1

    depth = first["depth"]
    valid_est = int(((depth >= dmin) & (depth <= 4000)).sum())
    stride = max(1, int(math.ceil(math.sqrt(valid_est / float(args.points)))))

    pts, rgb = backproject(depth, first["color"], dmin, 4000,
                           args.hfov, args.vfov, stride)

    # 有外参文件就按「相机挂载」摆放（物理正确）；没有才退回拟合桌面+居中
    ext_path = ROOT / "config" / "extrinsic_manual.yaml"
    ext = {}
    if ext_path.is_file():
        try:
            import yaml as _yaml

            ext = _yaml.safe_load(ext_path.read_text(encoding="utf-8")) or {}
        except Exception:
            ext = {}

    if ext:
        view = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0),
                "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}
        mount = np.stack([np.array(view[ext["cam_right"]]),
                          np.array(view[ext["cam_down"]]),
                          np.array(view[ext["cam_forward"]])], axis=1)
        offset = np.array(ext.get("optical_offset_mm", [0, 0, 0]), float) / 1000.0
        body = next((i for i in range(robot.model.nbody)
                     if robot.model.body(i).name == "DaBai_DCW2_Link"), -1)
        cam_pos = robot.data.xpos[body].copy() + \
            robot.data.xmat[body].reshape(3, 3) @ offset
        cam_mat = robot.data.xmat[body].reshape(3, 3).copy()
        transform = cam_mat @ mount
        print(f"[外参] 按相机挂载摆放（前={ext['cam_forward']} "
              f"右={ext['cam_right']} 下={ext['cam_down']}）")

        def to_world(p):
            return p @ transform.T + cam_pos[None, :]

        world0 = to_world(pts)
        shift = np.zeros(3)
    else:
        right, forward, up, off = level_basis(pts)
        world0 = np.stack([pts @ right, pts @ forward, pts @ up - off], axis=1)
        shift = np.zeros(3)
        if args.center:
            shift = np.array([-world0[:, 0].mean(), 0.30 - world0[:, 1].mean(),
                              -world0[:, 2].mean()])
        print(f"[点云] 拟合桌面摆正，法向 {np.round(up, 2)}")

        def to_world(p):
            return np.stack([p @ right, p @ forward, p @ up - off], axis=1) + shift
    print(f"[点云] 第一帧 {len(pts)} 个点，采样步长 {stride}")
    print(f"[窗口] 实时刷新 {args.hz:g} Hz；关窗口退出")

    radius = 0.0018
    period = 1.0 / max(1.0, args.hz)
    from mujoco import viewer as mj_viewer

    try:
        with mj_viewer.launch_passive(robot.model, robot.data) as handle:
            handle.cam.lookat[:] = [0.0, 0.30, 0.10]
            handle.cam.distance = 0.85
            handle.cam.azimuth = 135.0
            handle.cam.elevation = -20.0
            frames = 0
            while handle.is_running():
                started = time.monotonic()
                pair = cam.get_frame_pair(timeout_ms=800)
                if pair.get("depth") is None:
                    handle.sync()
                    continue
                pts, rgb = backproject(pair["depth"], pair.get("color"), dmin,
                                       4000, args.hfov, args.vfov, stride)
                if len(pts) == 0:
                    handle.sync()
                    continue
                world = to_world(pts)
                scene = handle.user_scn
                scene.ngeom = 0
                count = 0
                for i in range(len(world)):
                    if count >= scene.maxgeom:
                        break
                    mujoco.mjv_initGeom(
                        scene.geoms[count], mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, 0.0, 0.0]), world[i],
                        np.eye(3).reshape(-1),
                        np.array([rgb[i][0], rgb[i][1], rgb[i][2], 1.0],
                                 dtype=np.float32))
                    count += 1
                scene.ngeom = count
                handle.sync()
                frames += 1
                if frames % 20 == 0:
                    print(f"    已刷新 {frames} 帧，当前 {count} 个点", flush=True)
                remain = period - (time.monotonic() - started)
                if remain > 0:
                    time.sleep(remain)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        print("[窗口] 已关闭")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
