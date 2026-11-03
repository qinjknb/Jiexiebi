#!/usr/bin/env python3
"""Replay the recorded MuJoCo grasp with YOLO/RGB-D points in a viewer."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
from mujoco import viewer

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel


def add_points(scene, xyz, rgb, radius, alpha):
    ident = np.eye(3).reshape(-1)
    for point, color in zip(xyz, rgb):
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom],
                           mujoco.mjtGeom.mjGEOM_SPHERE,
                           np.array([radius, 0., 0.]), point,
                           ident, np.array([*color, alpha], np.float32))
        scene.ngeom += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", type=Path, required=True)
    ap.add_argument("--result", type=Path, required=True)
    ap.add_argument("--scene", type=Path, required=True)
    ap.add_argument("--cloud", type=Path, required=True)
    ap.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    result = json.loads(args.result.read_text(encoding="utf-8"))
    scene_info = json.loads(args.scene.read_text(encoding="utf-8"))
    frames = plan["frames"]
    history = result["frame_history"]
    if len(frames) != len(history) or not all("fruit_freejoint_qpos" in x for x in history):
        raise RuntimeError("Physics result does not contain full fruit pose history")
    cfg = load_config(args.config)
    mimic = json.loads(cfg.model.mjcf.with_suffix(".json").read_text(encoding="utf-8"))["mimic"]
    physics_xml = Path(scene_info["model_xml"]).parent / "strawberry_grasp_physics.xml"
    robot = RobotModel(physics_xml, mimic=mimic)
    fruit_qadr = int(robot.model.jnt_qposadr[robot.model.joint("strawberry_free").id])
    fruit_body = robot.model.body("observed_strawberry").id
    cloud = np.load(args.cloud)
    table_xyz = cloud["table_xyz"][::4]
    table_rgb = cloud["table_rgb"][::4]
    fruit_local = cloud["strawberry_xyz"][::3] - np.array(scene_info["fruit_center_base_xyz_m"])
    fruit_rgb = cloud["strawberry_rgb"][::3]

    with viewer.launch_passive(robot.model, robot.data) as handle:
        handle.cam.lookat[:] = [0., -0.19, 0.10]
        handle.cam.distance = 0.67
        handle.cam.azimuth = 160
        handle.cam.elevation = -32
        print("[viewer] replaying MuJoCo grasp with point cloud; close window to stop", flush=True)
        picks = list(range(0, len(frames), 5))
        if picks[-1] != len(frames) - 1:
            picks.append(len(frames) - 1)
        while handle.is_running():
            for index in picks:
                if not handle.is_running():
                    break
                robot.data.qpos[fruit_qadr:fruit_qadr + 7] = history[index]["fruit_freejoint_qpos"]
                robot.apply_kinematic(frames[index]["q_rad"])
                fruit_world = (fruit_local @ robot.data.xmat[fruit_body].reshape(3, 3).T +
                               robot.data.xpos[fruit_body])
                user_scene = handle.user_scn
                user_scene.ngeom = 0
                add_points(user_scene, table_xyz, table_rgb, 0.0016, 0.65)
                add_points(user_scene, fruit_world, fruit_rgb, 0.0022, 0.9)
                handle.sync()
                time.sleep(1 / 12)
            time.sleep(0.8)
            if args.once:
                break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
