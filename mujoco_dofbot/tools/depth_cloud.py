#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度点云 → MuJoCo 可视化。

流程：
  1) 用 yolo_pkg 的 OrbbecDCW2 开深度+彩色（D2C 对齐），取一帧；
  2) 按针孔模型把深度图反投影成点云（单位 m），颜色取彩色图；
  3) 读实机舵机角度，把机械臂摆到同一姿态；
  4) 点云挂到 URDF 里的 DaBai_DCW2_Link（相机模块）上，随手爪一起动；
  5) 用 MuJoCo 渲染，或用交互窗口实时看。

盲区阈值沿用 strawberry_demo.py 的 DEPTH_VALID_MIN（已改成 10 mm = 1 cm）。

用法：
    python tools/depth_cloud.py --render out\\cloud.png
    python tools/depth_cloud.py --view --points 8000
    python tools/depth_cloud.py --render out\\c.png --cam-forward +x --cam-right -y --cam-down -z
"""

from __future__ import annotations

import argparse
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

_VIEW = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0),
         "+z": (0, 0, 1), "-z": (0, 0, -1)}


def import_vision():
    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH  # noqa: E402

    return OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH


def load_depth_min(default=10) -> int:
    """从 strawberry_demo.py 里读当前盲区阈值，保证两边一致。"""
    try:
        text = (VISION / "strawberry_demo.py").read_text(encoding="utf-8")
        for line in text.splitlines():
            if line.strip().startswith("DEPTH_VALID_MIN"):
                return int(line.split("=")[1].split("#")[0].strip())
    except Exception:
        pass
    return default


def grab_pair(cam, timeout_ms=3000, tries=20):
    for _ in range(tries):
        pair = cam.get_frame_pair(timeout_ms=timeout_ms)
        if pair.get("depth") is not None:
            return pair
    return None


def camera_matrix(depth, hfov, vfov):
    h, w = depth.shape
    fx = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    fy = (h / 2.0) / math.tan(math.radians(vfov) / 2.0)
    return fx, fy, w / 2.0, h / 2.0


def build_cloud(depth, color, min_mm, max_mm, hfov, vfov, max_points):
    """返回 (points_cam (N,3) m, colors (N,3) float 0~1, stats)。"""
    fx, fy, cx, cy = camera_matrix(depth, hfov, vfov)
    valid = (depth >= min_mm) & (depth <= max_mm)
    n_valid = int(valid.sum())
    if n_valid == 0:
        return np.zeros((0, 3)), np.zeros((0, 3)), {"valid": 0}

    step = max(1, int(math.ceil(math.sqrt(n_valid / float(max_points)))))
    sample = np.zeros_like(valid)
    sample[::step, ::step] = True
    mask = valid & sample

    vs, us = np.nonzero(mask)
    z = depth[vs, us].astype(np.float32) / 1000.0
    x = (us.astype(np.float32) - cx) * z / fx
    y = (vs.astype(np.float32) - cy) * z / fy
    points = np.stack([x, y, z], axis=1)

    if color is not None and color.shape[:2] == depth.shape:
        rgb = color[vs, us][:, ::-1].astype(np.float32) / 255.0
    else:
        rgb = np.full((len(z), 3), 0.6, dtype=np.float32)

    stats = {
        "valid": n_valid,
        "density": 100.0 * n_valid / depth.size,
        "used": len(z),
        "step": step,
        "min_mm": int(depth[valid].min()),
        "max_mm": int(depth[valid].max()),
        "median_mm": int(np.median(depth[valid])),
    }
    return points, rgb, stats


def level_points(points: np.ndarray):
    """拟合点云里的主平面（桌面），把点云摆成「桌面水平、桌面在 z=0」。

    相机的安装朝向没有做过手眼标定，与其猜一个挂载角，不如用场景本身定标：
    桌面一定是水平的，把拟合出的平面法向当作世界 +Z 即可。

    返回 (world_points, info)。这里显式用点积算三个分量，不绕旋转矩阵，
    避免转置/行列方向的歧义。
    """
    if len(points) < 200:
        return points.copy(), {"ok": False}
    sample = points[:: max(1, len(points) // 20000)]
    centroid = sample.mean(axis=0)
    _u, s, vt = np.linalg.svd(sample - centroid, full_matrices=False)
    normal = vt[-1]
    if normal @ centroid > 0:          # 让法向从桌面指向相机
        normal = -normal
    up = normal / (np.linalg.norm(normal) + 1e-12)

    plane_offset = float(np.median(points @ up))
    ex = np.array([1.0, 0.0, 0.0])
    if abs(ex @ up) > 0.95:
        ex = np.array([0.0, 1.0, 0.0])
    right = ex - (ex @ up) * up
    right /= np.linalg.norm(right) + 1e-12
    forward = np.cross(up, right)

    world = np.stack([
        points @ right,
        points @ forward,
        points @ up - plane_offset,
    ], axis=1)
    residual = points @ up - plane_offset
    info = {
        "ok": True,
        "normal": up,
        "flat_1cm": float(100.0 * (np.abs(residual) < 0.01).mean()),
        "spread_p5": float(np.percentile(residual, 5)),
        "spread_p95": float(np.percentile(residual, 95)),
    }
    return world, info


def rot_from_axes(forward, right, down) -> np.ndarray:
    """相机三轴（右/下/前）在连杆坐标系里的方向 → 3x3 变换矩阵。"""
    f = np.array(_VIEW[forward], dtype=float)
    r = np.array(_VIEW[right], dtype=float)
    d = np.array(_VIEW[down], dtype=float)
    return np.stack([r, d, f], axis=1)   # 列 = 相机 X/Y/Z 在连杆系下的表示


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="深度点云 → MuJoCo")
    p.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    p.add_argument("--render", default="", help="输出 PNG")
    p.add_argument("--view", action="store_true", help="开交互窗口")
    p.add_argument("--points", type=int, default=6000, help="采样后的点上限")
    p.add_argument("--hfov", type=float, default=64.0, help="彩色视场角（对齐后用它）")
    p.add_argument("--vfov", type=float, default=55.0)

    # 有外参文件时默认按「相机挂载」摆放（物理正确）；没有才退回「拟合桌面摆正」
    ext_path = ROOT / "config" / "extrinsic_manual.yaml"
    ext = {}
    if ext_path.is_file():
        try:
            import yaml as _yaml

            ext = _yaml.safe_load(ext_path.read_text(encoding="utf-8")) or {}
        except Exception:
            ext = {}

    p.add_argument("--cam-forward", default=ext.get("cam_forward", "+x"),
                   choices=list(_VIEW))
    p.add_argument("--cam-right", default=ext.get("cam_right", "-y"), choices=list(_VIEW))
    p.add_argument("--cam-down", default=ext.get("cam_down", "-z"), choices=list(_VIEW))
    p.add_argument("--no-servo", action="store_true", help="不读舵机，用零位模型")
    p.add_argument("--level", action="store_true", default=not bool(ext),
                   help="拟合桌面摆正（无外参时的兜底；有外参时默认关闭）")
    p.add_argument("--no-level", dest="level", action="store_false")
    args = p.parse_args(argv)
    if bool(ext):
        print(f"[外参] 读取 {ext_path.name}：相机 前={args.cam_forward} "
              f"右={args.cam_right} 下={args.cam_down}")

    from dofbot_mj.config import load_config
    from dofbot_mj.model import RobotModel
    import json

    settings = load_config(args.config)
    sidecar = settings.model.mjcf.with_suffix(".json")
    mimic = (json.loads(sidecar.read_text(encoding="utf-8")) or {}).get("mimic", {}) \
        if sidecar.is_file() else {}
    robot = RobotModel(settings.model.mjcf, mimic=mimic)

    # ---- 1) 实机姿态 ----
    if args.no_servo:
        pose = {name: 0.0 for name in robot.driver_joints}
        print("[姿态] 未读舵机，用零位")
    else:
        from dofbot_mj.servo_link import probe_positions

        try:
            pose, degrees = probe_positions(settings.servo, log=print)
        except Exception as exc:
            print(f"[警告] 读舵机失败（{exc}），改用零位")
            pose = {name: 0.0 for name in robot.driver_joints}
    robot.apply_kinematic(pose)

    # ---- 2) 取深度 ----
    OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH = import_vision()
    dmin = load_depth_min()
    print(f"[相机] 盲区阈值 DEPTH_VALID_MIN = {dmin} mm（来自 strawberry_demo.py）")
    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
        time.sleep(1.2)
        pair = grab_pair(cam)
    finally:
        try:
            cam.close()
        except Exception:
            pass
    if pair is None:
        print("[错误] 取帧失败")
        return 1

    depth, color = pair["depth"], pair["color"]
    points, colors, stats = build_cloud(
        depth, color, dmin, 4000, args.hfov, args.vfov, args.points)
    print(f"[点云] 有效像素 {stats['valid']}（稠密 {stats['density']:.1f}%），"
          f"深度 {stats['min_mm']}~{stats['max_mm']} mm，中位 {stats['median_mm']} mm")
    print(f"       采样步长 {stats['step']} → {stats['used']} 个点")

    # ---- 3) 摆正点云 ----
    if args.level:
        world, info = level_points(points)
        if info.get("ok"):
            print(f"[摆正] 桌面法向 {np.round(info['normal'], 3)}，"
                  f"{info['flat_1cm']:.1f}% 的点在桌面 ±1cm 内；"
                  f"高度 P5/P95 = {info['spread_p5']:+.3f}/{info['spread_p95']:+.3f} m")
        else:
            print("[摆正] 点太少，跳过")
    else:
        body_id = next((i for i in range(robot.model.nbody)
                        if robot.model.body(i).name == "DaBai_DCW2_Link"), -1)
        if body_id < 0:
            print("[警告] 模型里没有 DaBai_DCW2_Link，点云放在世界原点")
            cam_pos, cam_mat = np.zeros(3), np.eye(3)
        else:
            cam_pos = robot.data.xpos[body_id].copy()
            cam_mat = robot.data.xmat[body_id].reshape(3, 3).copy()
        mount = rot_from_axes(args.cam_forward, args.cam_right, args.cam_down)
        world = cam_pos[None, :] + points @ (cam_mat @ mount).T
    print(f"[点云] 世界系范围 X {world[:,0].min():+.2f}~{world[:,0].max():+.2f}  "
          f"Y {world[:,1].min():+.2f}~{world[:,1].max():+.2f}  "
          f"Z {world[:,2].min():+.3f}~{world[:,2].max():+.3f} m")
    cam_pos = world.mean(axis=0)

    # ---- 4) 画点 ----
    radius = 0.0018

    def fill_scene(scene, max_points=None):
        # 追加到场景已有几何体之后，绝不能覆盖 ngeom（否则机器人会被点云顶掉）
        start = int(scene.ngeom)
        count = 0
        limit = max_points if max_points else len(world)
        for i in range(len(world)):
            if start + count >= scene.maxgeom or count >= limit:
                break
            geom = scene.geoms[start + count]
            mujoco.mjv_initGeom(
                geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([radius, 0.0, 0.0]),
                world[i], np.eye(3).reshape(-1),
                np.array([colors[i][0], colors[i][1], colors[i][2], 1.0], dtype=np.float32))
            count += 1
        scene.ngeom = start + count
        return count

    if args.render:
        renderer = mujoco.Renderer(robot.model, 720, 960)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(robot.model, camera)
        # 按「机械臂 + 点云」的整体包围盒取景，避免点云飞出画面
        scene_pts = np.vstack([world, robot.data.xpos[1:]])
        center = scene_pts.mean(axis=0)
        extent = float(np.max(np.ptp(scene_pts, axis=0)))
        camera.lookat[:] = center
        camera.distance = max(0.6, extent * 1.9)
        camera.azimuth = 135.0
        camera.elevation = -14.0
        renderer.update_scene(robot.data, camera=camera)
        drawn = fill_scene(renderer.scene, args.points)
        img = renderer.render()
        out = Path(args.render)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            from PIL import Image

            Image.fromarray(img).save(out)
        except ImportError:
            import cv2

            cv2.imwrite(str(out), img[:, :, ::-1])
        print(f"[渲染] {out}（画了 {drawn} 个点）")

    if args.view:
        from mujoco import viewer as mj_viewer

        print("[窗口] 点云固定在当前姿态；关窗退出")
        with mj_viewer.launch_passive(robot.model, robot.data) as handle:
            handle.cam.lookat[:] = cam_pos + np.array([0.0, 0.0, 0.12])
            handle.cam.distance = 0.5
            handle.cam.azimuth = 135.0
            handle.cam.elevation = -12.0
            while handle.is_running():
                scene = handle.user_scn
                fill_scene(scene, args.points)
                handle.sync()
                time.sleep(1.0 / 30.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
