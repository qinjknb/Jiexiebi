#!/usr/bin/env python3
"""Fuse YOLO, aligned RGB-D and a table plane into a MuJoCo scene.

The fruit cloud is observed surface geometry. The ellipsoid is a simple
simulation proxy inferred from that cloud, not a contact-validated grasp model.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel
from dofbot_mj.protocol import ServoMapping
from dofbot_mj.servo_link import probe_positions
from dofbot_mj.vision_geometry import camera_pose


def sampled_points(depth: np.ndarray, color: np.ndarray, mask: np.ndarray,
                   stride: int, max_points: int):
    h, w = depth.shape
    grid = np.zeros((h, w), bool)
    grid[::stride, ::stride] = True
    vv, uu = np.nonzero(mask & grid)
    if len(vv) > max_points:
        ids = np.linspace(0, len(vv) - 1, max_points, dtype=int)
        vv, uu = vv[ids], uu[ids]
    z = depth[vv, uu].astype(float) / 1000.0
    fx = (w / 2) / math.tan(math.radians(64) / 2)
    fy = (h / 2) / math.tan(math.radians(55) / 2)
    xyz = np.column_stack(((uu - w / 2) * z / fx,
                           (vv - h / 2) * z / fy, z))
    rgb = color[vv, uu][:, ::-1].astype(float) / 255
    return xyz, rgb


def point_geom(scene, xyz, rgb, radius: float, alpha: float) -> int:
    available = scene.maxgeom - scene.ngeom
    count = min(len(xyz), available)
    ident = np.eye(3).reshape(-1)
    for i in range(count):
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom],
                           mujoco.mjtGeom.mjGEOM_SPHERE,
                           np.array([radius, 0., 0.]), xyz[i], ident,
                           np.array([*rgb[i], alpha], dtype=np.float32))
        scene.ngeom += 1
    return count


def write_ply(path: Path, table_xyz, table_rgb, fruit_xyz, fruit_rgb):
    points = np.vstack((table_xyz, fruit_xyz))
    colors = np.clip(np.vstack((table_rgb, fruit_rgb)) * 255, 0, 255).astype(np.uint8)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        stream.write("property float x\nproperty float y\nproperty float z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for p, c in zip(points, colors):
            stream.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {c[0]} {c[1]} {c[2]}\n")


def make_scene_xml(source: Path, destination: Path, center, radii):
    xml = source.read_text(encoding="utf-8")
    if xml.count("</worldbody>") != 1:
        raise RuntimeError("Expected exactly one MuJoCo worldbody")
    body = (f'    <body name="observed_strawberry" pos="{center[0]:.6f} '
            f'{center[1]:.6f} {center[2]:.6f}">\n'
            f'      <geom name="strawberry_proxy" type="ellipsoid" '
            f'size="{radii[0]:.6f} {radii[1]:.6f} {radii[2]:.6f}" '
            f'rgba="0.85 0.06 0.04 0.78" contype="0" conaffinity="0"/>\n'
            f'    </body>\n')
    destination.write_text(xml.replace("  </worldbody>", body + "  </worldbody>"),
                           encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--targets", type=Path, required=True)
    ap.add_argument("--rgb", type=Path, required=True)
    ap.add_argument("--depth", type=Path, required=True)
    ap.add_argument("--table-plane", type=Path, required=True)
    ap.add_argument("--target-id", type=int, default=1)
    ap.add_argument("--plan", type=Path, help="Use the starting pose of a pick plan")
    ap.add_argument("--servo-deg", type=float, nargs=6)
    ap.add_argument("--read-servo", action="store_true")
    ap.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "strawberry_pointcloud_scene")
    ap.add_argument("--view", action="store_true", help="Open an interactive MuJoCo window")
    ap.add_argument("--infer-fruit-from-table", action="store_true",
                    help="Simulation only: create a fruit surface proxy when fruit depth is invalid")
    args = ap.parse_args()
    if sum((bool(args.plan), bool(args.servo_deg), args.read_servo)) != 1:
        ap.error("Choose exactly one of --plan, --servo-deg, or --read-servo")

    payload = json.loads(args.targets.read_text(encoding="utf-8"))
    target = next((x for x in payload["targets"] if x["id"] == args.target_id), None)
    if target is None or (target["camera_xyz_m"] is None and not args.infer_fruit_from_table):
        raise RuntimeError("Selected YOLO target has no valid aligned depth")
    if target["confidence"] < 0.5 or (target["depth_valid_fraction"] < 0.7
                                       and not args.infer_fruit_from_table):
        raise RuntimeError("Selected YOLO target failed confidence/depth quality gates")
    if args.infer_fruit_from_table and not args.plan:
        raise RuntimeError("Inferred fruit cloud requires --plan for the same target")
    plane = json.loads(args.table_plane.read_text(encoding="utf-8"))
    color = cv2.imread(str(args.rgb))
    depth = np.load(args.depth)
    if color is None or color.shape[:2] != depth.shape:
        raise RuntimeError("RGB and depth dimensions do not match")
    cfg = load_config(args.config)
    mimic = json.loads(cfg.model.mjcf.with_suffix(".json").read_text(encoding="utf-8"))["mimic"]
    robot = RobotModel(cfg.model.mjcf, mimic=mimic)
    if args.plan:
        plan = json.loads(args.plan.read_text(encoding="utf-8"))
        pose = plan["frames"][0]["q_rad"]
    elif args.servo_deg:
        pose = ServoMapping(cfg.servo.joints, cfg.servo.range_deg).positions_from_targets(
            {i + 1: angle for i, angle in enumerate(args.servo_deg)})
    else:
        pose, _ = probe_positions(cfg.servo)
    robot.apply_kinematic(pose)
    camera_xyz, camera_rot, registration = camera_pose(robot, plane)

    h, w = depth.shape
    yy, xx = np.mgrid[:h, :w]
    z = depth.astype(float) / 1000
    fx = (w / 2) / math.tan(math.radians(64) / 2)
    fy = (h / 2) / math.tan(math.radians(55) / 2)
    x = (xx - w / 2) * z / fx
    y = (yy - h / 2) * z / fy
    height = (plane["normal_camera"][0] * x +
              plane["normal_camera"][1] * y +
              plane["normal_camera"][2] * z + plane["offset_m"])
    valid = (depth > 180) & (depth < 850)
    bbox = np.zeros((h, w), bool)
    x1, y1, x2, y2 = map(int, target["box_xyxy"])
    bbox[max(0, y1):min(h, y2 + 1), max(0, x1):min(w, x2 + 1)] = True
    b, g, r = cv2.split(color.astype(np.float32))
    red = (r > 75) & (r > 1.25 * g) & (r > 1.25 * b)
    fruit_mask = (bbox & red if args.infer_fruit_from_table else
                  bbox & red & valid & (height > 0.005) & (height < 0.08))
    table_roi = (xx >= 170) & (xx < min(w, 620)) & (yy >= 90) & (yy < min(h, 400))
    table_mask = table_roi & valid & (np.abs(height) < 0.005) & ~bbox
    if fruit_mask.sum() < 100 or table_mask.sum() < 500:
        raise RuntimeError("Insufficient strawberry/table points after RGB-D filtering")

    if args.infer_fruit_from_table:
        vv, uu = np.nonzero(fruit_mask & ((yy % 2 == 0) & (xx % 2 == 0)))
        if len(vv) > 1200:
            keep = np.linspace(0, len(vv) - 1, 1200, dtype=int)
            vv, uu = vv[keep], uu[keep]
        rays = np.column_stack(((uu - w / 2) / fx, (vv - h / 2) / fy,
                                np.ones(len(vv))))
        normal = np.asarray(plane["normal_camera"], float)
        distance = -float(plane["offset_m"]) / (rays @ normal)
        ru = (uu - (x1 + x2) / 2) / max(1, (x2 - x1) / 2)
        rv = (vv - (y1 + y2) / 2) / max(1, (y2 - y1) / 2)
        surface = 0.04 * np.sqrt(np.clip(1 - ru**2 - rv**2, 0, 1))
        fruit_cam = rays * distance[:, None] + surface[:, None] * normal
        fruit_rgb = color[vv, uu][:, ::-1].astype(float) / 255
    else:
        fruit_cam, fruit_rgb = sampled_points(depth, color, fruit_mask, 2, 1200)
    table_cam, table_rgb = sampled_points(depth, color, table_mask, 6, 3500)
    fruit_world = fruit_cam @ camera_rot.T + camera_xyz
    table_world = table_cam @ camera_rot.T + camera_xyz
    target_world = (np.asarray(plan["target_base_xyz_m"], float) if args.infer_fruit_from_table
                    else camera_xyz + camera_rot @ np.array(target["camera_xyz_m"], float))
    center = target_world.copy() if args.infer_fruit_from_table else np.median(fruit_world, axis=0)
    center[2] = max(0.017, min(0.03, target_world[2]))
    spread = np.percentile(fruit_world, [5, 95], axis=0)
    radii = np.clip((spread[1] - spread[0]) / 2, [0.012, 0.012, 0.016],
                    [0.025, 0.025, 0.025])
    radii[2] = min(radii[2], center[2])
    if args.infer_fruit_from_table:
        radii[2] = 0.021

    args.out.mkdir(parents=True, exist_ok=True)
    overlay = color.copy()
    overlay[fruit_mask] = (0.35 * overlay[fruit_mask] +
                           0.65 * np.array([255, 0, 255])).astype(np.uint8)
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.imwrite(str(args.out / "fruit_mask.png"), overlay)
    np.savez_compressed(args.out / "pointcloud.npz", table_xyz=table_world,
                        table_rgb=table_rgb, strawberry_xyz=fruit_world,
                        strawberry_rgb=fruit_rgb)
    write_ply(args.out / "pointcloud.ply", table_world, table_rgb,
              fruit_world, fruit_rgb)
    scene_xml = cfg.model.mjcf.parent / "strawberry_scene_snapshot.xml"
    make_scene_xml(cfg.model.mjcf, scene_xml, center, radii)
    scene_robot = RobotModel(scene_xml, mimic=mimic)
    scene_robot.apply_kinematic(pose)

    metadata = {
        "schema": "dofbot.strawberry_pointcloud_scene.v1",
        "execution": "simulation_only",
        "source_rgb": str(args.rgb.resolve()),
        "source_depth": str(args.depth.resolve()),
        "source_targets": str(args.targets.resolve()),
        "target_id": args.target_id,
        "target_base_xyz_m": target_world.tolist(),
        "fruit_center_base_xyz_m": center.tolist(),
        "fruit_ellipsoid_radii_m": radii.tolist(),
        "fruit_observed_points": 0 if args.infer_fruit_from_table else int(len(fruit_world)),
        "fruit_inferred_proxy_points": int(len(fruit_world)) if args.infer_fruit_from_table else 0,
        "table_observed_points": int(len(table_world)),
        "camera_registration": registration,
        "model_xml": str(scene_xml.resolve()),
        "limitations": ("fruit depth invalid: surface points and ellipsoid inferred from RGB box and table plane; approximate camera X/Y and yaw"
                        if args.infer_fruit_from_table else
                        "visible fruit surface only; ellipsoid proxy; approximate camera X/Y and yaw"),
    }
    (args.out / "scene.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(scene_robot.model, camera)
    camera.lookat[:] = [0., -0.19, 0.10]
    camera.distance = 0.67
    camera.azimuth = 160
    camera.elevation = -32
    renderer = mujoco.Renderer(scene_robot.model, 720, 960)
    try:
        renderer.update_scene(scene_robot.data, camera=camera)
        scene = renderer.scene
        table_drawn = point_geom(scene, table_world, table_rgb, 0.0014, 0.55)
        fruit_drawn = point_geom(scene, fruit_world, fruit_rgb, 0.0025, 1.0)
        Image.fromarray(renderer.render()).save(args.out / "mujoco_pointcloud.png")
    finally:
        renderer.close()
    print(f"[scene] fruit {fruit_drawn} points, table {table_drawn} points")
    print(f"[scene] target={target_world.round(3)}, ellipsoid center={center.round(3)} "
          f"radii={radii.round(3)}")
    print(f"[scene] {scene_xml}; {args.out / 'mujoco_pointcloud.png'}")

    if args.view:
        from mujoco import viewer
        with viewer.launch_passive(scene_robot.model, scene_robot.data) as handle:
            handle.cam.lookat[:] = camera.lookat
            handle.cam.distance = camera.distance
            handle.cam.azimuth = camera.azimuth
            handle.cam.elevation = camera.elevation
            scene = handle.user_scn
            scene.ngeom = 0
            point_geom(scene, table_world, table_rgb, 0.0014, 0.55)
            point_geom(scene, fruit_world, fruit_rgb, 0.0025, 1.0)
            handle.sync()
            print("[viewer] static YOLO/RGB-D scene; close window to stop", flush=True)
            while handle.is_running():
                time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
