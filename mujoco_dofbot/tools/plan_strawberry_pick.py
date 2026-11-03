#!/usr/bin/env python3
"""Plan a simulation-only strawberry pick from YOLO RGB-D targets.

This uses bounded numerical IK on the MuJoCo model, then smooth joint-space
segments for open, pregrasp, grasp, close and lift. It never sends MOVE.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel
from dofbot_mj.protocol import ServoMapping
from dofbot_mj.servo_link import probe_positions
from dofbot_mj.vision_geometry import camera_pose

ARM = [f"Arm{i}_Joint" for i in range(1, 6)]
def tip_xyz(robot: RobotModel) -> np.ndarray:
    return robot.data.xpos[robot.model.body("Gripping_point_Link").id].copy()


def solve_ik(robot: RobotModel, start: dict, target: np.ndarray) -> tuple[dict, float]:
    lower = np.array([robot.limits(name)[0] for name in ARM])
    upper = np.array([robot.limits(name)[1] for name in ARM])
    seed = np.clip(np.array([start[name] for name in ARM]), lower + 1e-5, upper - 1e-5)
    rng = np.random.default_rng(24)
    seeds = [seed, np.clip(np.zeros(5), lower + 1e-5, upper - 1e-5)]
    seeds += [np.clip(seed + rng.normal(0, 0.45, 5), lower + 1e-5, upper - 1e-5)
              for _ in range(8)]
    best = None
    for trial in seeds:
        def residual(q):
            state = dict(start)
            state.update(zip(ARM, q))
            robot.apply_kinematic(state)
            return np.r_[100 * (tip_xyz(robot) - target), 0.025 * (q - seed)]

        result = least_squares(residual, trial, bounds=(lower, upper),
                               max_nfev=180, ftol=1e-8, xtol=1e-8)
        state = dict(start)
        state.update(zip(ARM, result.x))
        robot.apply_kinematic(state)
        error = float(np.linalg.norm(tip_xyz(robot) - target))
        score = error + 0.002 * float(np.linalg.norm(result.x - seed))
        if best is None or score < best[0]:
            best = (score, state, error)
    assert best is not None
    if best[2] > 0.006:
        raise RuntimeError(f"IK target {target.round(3)} misses by {best[2]*1000:.1f} mm")
    return best[1], best[2]


def render(robot: RobotModel, state: dict, target: np.ndarray, out: Path) -> None:
    robot.apply_kinematic(state)
    renderer = mujoco.Renderer(robot.model, 720, 960)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(robot.model, camera)
    camera.lookat[:] = [0., -0.13, 0.20]
    camera.distance = 0.72
    camera.azimuth = 165
    camera.elevation = -18
    renderer.update_scene(robot.data, camera=camera)
    scene = renderer.scene
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                       np.array([0.018, 0., 0.]), target,
                       np.eye(3).reshape(-1), np.array([0.95, 0.12, 0.10, 1.]))
    scene.ngeom += 1
    Image.fromarray(renderer.render()).save(out)
    renderer.close()


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--targets", type=Path, help="capture_targets.py JSON")
    p.add_argument("--target-id", type=int, default=1)
    p.add_argument("--table-plane", type=Path,
                   help="Table plane JSON estimated from the same RGB-D frame")
    p.add_argument("--world-target", type=float, nargs=3,
                   help="Known base-frame XYZ metres for simulation testing")
    p.add_argument("--servo-deg", type=float, nargs=6,
                   help="Offline starting servo angles ID1..ID6")
    p.add_argument("--read-servo", action="store_true", help="Read current COM9 pose")
    p.add_argument("--allow-unknown", action="store_true",
                   help="Allow an unclassified detection for simulation only")
    p.add_argument("--infer-from-table", action="store_true",
                   help="Simulation only: intersect target pixel ray with measured table plane when fruit depth is missing")
    p.add_argument("--handeye", type=Path)
    p.add_argument("--grasp-height-offset-m", type=float, default=0.0,
                   help="Raise the planned tool point above the detected fruit centre")
    p.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    p.add_argument("--out", type=Path, default=ROOT / "out" / "strawberry_pick")
    args = p.parse_args()
    if bool(args.targets) == bool(args.world_target):
        p.error("Provide exactly one of --targets or --world-target")
    if args.read_servo and args.servo_deg:
        p.error("Choose --read-servo or --servo-deg")
    if args.table_plane and not args.targets:
        p.error("--table-plane requires --targets")
    if args.infer_from_table and not args.table_plane:
        p.error("--infer-from-table requires --table-plane")
    if not 0 <= args.grasp_height_offset_m <= 0.03:
        p.error("--grasp-height-offset-m must be in 0..0.03")

    cfg = load_config(args.config)
    handeye = None
    if args.handeye:
        import yaml
        handeye = yaml.safe_load(args.handeye.read_text(encoding='utf-8'))
    mimic = json.loads(cfg.model.mjcf.with_suffix(".json").read_text(encoding="utf-8"))["mimic"]
    robot = RobotModel(cfg.model.mjcf, mimic=mimic)
    if args.read_servo:
        start, degrees = probe_positions(cfg.servo)
        print("[servo] current angles", degrees)
    elif args.servo_deg:
        start = ServoMapping(cfg.servo.joints, cfg.servo.range_deg).positions_from_targets(
            {i + 1: angle for i, angle in enumerate(args.servo_deg)})
    else:
        start = {name: 0.0 for name in robot.driver_joints}
    robot.apply_kinematic(start)

    camera_details = None
    if args.world_target:
        target = np.array(args.world_target, float)
        source = "known base-frame target"
    else:
        payload = json.loads(args.targets.read_text(encoding="utf-8"))
        if payload.get("schema") != "dofbot.strawberry_targets.v1":
            raise RuntimeError("Unsupported target JSON schema")
        found = next((item for item in payload["targets"] if item["id"] == args.target_id), None)
        if found is None:
            raise RuntimeError("Target missing")
        if found["confidence"] < 0.5 or (
            found["ripeness"] != "good" and not args.allow_unknown
        ):
            raise RuntimeError("YOLO target is low confidence or not confirmed ripe; no pick planned")
        plane = json.loads(args.table_plane.read_text(encoding="utf-8")) if args.table_plane else None
        if args.infer_from_table:
            w, h = payload["frame_size"]
            u, v = found["pixel_uv"]
            fx = (w / 2) / math.tan(math.radians(64) / 2)
            fy = (h / 2) / math.tan(math.radians(55) / 2)
            ray = np.array([(u - w / 2) / fx, (v - h / 2) / fy, 1.0])
            if handeye is not None:
                import cv2
                k = handeye['camera_matrix']
                K = np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
                uv = cv2.undistortPoints(np.array([[[u,v]]],dtype=float),K,np.array(k['dist']))[0,0]
                ray = np.r_[uv,1.]
            normal = np.array(plane["normal_camera"], float)
            distance = -float(plane["offset_m"]) / float(normal @ ray)
            if not 0.1 < distance < 0.85:
                raise RuntimeError("Fruit ray does not intersect measured table plausibly")
            optical = ray * distance + normal * 0.022
        else:
            if found["camera_xyz_m"] is None or found.get("depth_valid_fraction", 1.0) < 0.7:
                raise RuntimeError("Target has insufficient aligned depth coverage")
            optical = np.array(found["camera_xyz_m"], float)
        camera_pos, camera_rot, camera_details = camera_pose(robot, plane, handeye)
        source = "camera target via approximate visual mount"
        if args.table_plane:
            source = "camera target with measured table plane; model X/Y remain approximate"
        if args.infer_from_table:
            source = "simulation-only fruit center inferred from target pixel and table plane; fruit depth invalid"
        target = camera_pos + camera_rot @ optical
    if not np.isfinite(target).all() or target[2] < 0.015:
        raise RuntimeError(f"Target {target} is below the allowed grasp height")

    tip0 = tip_xyz(robot)
    direction = target - tip0
    direction[2] = 0
    direction = direction / (np.linalg.norm(direction) + 1e-9)
    if np.linalg.norm(direction) < 0.1:
        direction = np.array([0., -1., 0.])
    pregrasp = target - 0.055 * direction + np.array([0., 0., 0.02 + args.grasp_height_offset_m])
    grasp = target - 0.008 * direction + np.array([0., 0., args.grasp_height_offset_m])
    lift = grasp + np.array([0., 0., 0.06])

    stages = []
    current = dict(start)
    current["grip_joint"] = -0.2  # SRDF open
    stages.append(("open", current.copy(), tip0.copy(), 0.0))
    for label, point in (("pregrasp", pregrasp), ("grasp", grasp), ("lift", lift)):
        solved, error = solve_ik(robot, current, point)
        if label == "lift":
            current["grip_joint"] = -1.2  # SRDF close
            stages.append(("close", current.copy(), grasp.copy(), 0.0))
            solved["grip_joint"] = -1.2
        stages.append((label, solved.copy(), point.copy(), error))
        current = solved

    # Joint-limit and floor checks at every point of the interpolated path.
    previous = dict(start)
    frames = []
    for label, state, _, _ in stages:
        for u in np.linspace(0, 1, 61):
            t = 0.5 * (1 - math.cos(math.pi * u))
            frame = {name: previous[name] + (state[name] - previous[name]) * t
                     for name in robot.driver_joints}
            robot.apply_kinematic(frame)
            if tip_xyz(robot)[2] < 0.012:
                raise RuntimeError(f"Floor clearance failed during {label}")
            for name in robot.driver_joints:
                lo, hi = robot.limits(name)
                if not lo - 1e-6 <= frame[name] <= hi + 1e-6:
                    raise RuntimeError(f"Joint limit failed: {name} during {label}")
            frames.append({"stage": label, "q_rad": frame})
        previous = state

    args.out.mkdir(parents=True, exist_ok=True)
    grasp_state = next(state for label, state, _, _ in stages if label == "grasp")
    robot.apply_kinematic(grasp_state)
    grasp_tip = tip_xyz(robot)
    for label, state, _, _ in stages:
        if label in ("pregrasp", "grasp", "lift"):
            fruit = target
            if label == "lift":
                robot.apply_kinematic(state)
                fruit = target + tip_xyz(robot) - grasp_tip
            render(robot, state, fruit, args.out / f"{label}.png")
    plan = {
        "schema": "dofbot.strawberry_pick_plan.v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "execution": "simulation_only",
        "target_source": source,
        "target_base_xyz_m": target.tolist(),
        "grasp_height_offset_m": args.grasp_height_offset_m,
        "camera_registration": camera_details,
        "calibration": source + "; not verified for hardware",
        "collision_check": "joint limits and 12 mm tip-to-floor clearance only",
        "fruit_rendering": "idealized attachment after close; no contact-force simulation",
        "stages": [{"name": name, "target_xyz_m": xyz.tolist(),
                    "ik_error_m": error, "q_rad": q}
                   for name, q, xyz, error in stages],
        "frames": frames,
    }
    out = args.out / "plan.json"
    out.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[plan] {source}: target={target.round(3)}, {len(stages)} stages, "
          f"{len(frames)} frames")
    for name, _, _, error in stages:
        print(f"  {name}: IK error {error*1000:.1f} mm")
    print(f"[plan] saved {out}; simulation only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
