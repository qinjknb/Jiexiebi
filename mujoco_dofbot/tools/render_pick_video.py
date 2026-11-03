#!/usr/bin/env python3
"""Render a simulation-only strawberry pick plan as a reviewable MP4."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("plan", type=Path)
    ap.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    if plan.get("schema") != "dofbot.strawberry_pick_plan.v1":
        raise RuntimeError("Unsupported plan schema")
    if plan.get("execution") != "simulation_only":
        raise RuntimeError("Refusing to render a non-simulation plan")
    frames = plan["frames"]
    if not frames:
        raise RuntimeError("Plan has no frames")

    cfg = load_config(args.config)
    mimic = json.loads(cfg.model.mjcf.with_suffix(".json").read_text(encoding="utf-8"))["mimic"]
    robot = RobotModel(cfg.model.mjcf, mimic=mimic)
    target = np.array(plan["target_base_xyz_m"], float)
    grasp_state = next(stage["q_rad"] for stage in plan["stages"]
                       if stage["name"] == "grasp")
    robot.apply_kinematic(grasp_state)
    tip_id = robot.model.body("Gripping_point_Link").id
    grasp_tip = robot.data.xpos[tip_id].copy()
    out = args.out or args.plan.with_suffix(".mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), 12, (640, 480))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open MP4 writer: {out}")
    renderer = mujoco.Renderer(robot.model, 480, 640)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(robot.model, camera)
    camera.lookat[:] = [0., -0.13, 0.20]
    camera.distance = 0.72
    camera.azimuth = 165
    camera.elevation = -18
    try:
        # 305 planning samples become 61 video frames. All planned frames
        # were already checked for joint limits and tip-floor clearance.
        picks = list(range(0, len(frames), 5))
        if picks[-1] != len(frames) - 1:
            picks.append(len(frames) - 1)
        for idx in picks:
            frame = frames[idx]
            robot.apply_kinematic(frame["q_rad"])
            fruit = target
            if frame["stage"] == "lift":
                fruit = target + robot.data.xpos[tip_id] - grasp_tip
            renderer.update_scene(robot.data, camera=camera)
            scene = renderer.scene
            if scene.ngeom >= scene.maxgeom:
                raise RuntimeError("MuJoCo scene geom budget exhausted")
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],
                               mujoco.mjtGeom.mjGEOM_SPHERE,
                               np.array([0.018, 0., 0.]), fruit,
                               np.eye(3).reshape(-1),
                               np.array([0.95, 0.12, 0.10, 1.]))
            scene.ngeom += 1
            bgr = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(bgr, f"SIMULATION | {frame['stage']} | {idx+1}/{len(frames)}",
                        (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2, cv2.LINE_AA)
            writer.write(bgr)
    finally:
        writer.release()
        renderer.close()
    print(f"Saved {len(picks)} frames to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
