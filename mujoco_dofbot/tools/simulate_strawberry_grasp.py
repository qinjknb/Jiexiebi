#!/usr/bin/env python3
"""Run a contact-based MuJoCo strawberry grasp and record a point-cloud MP4.

The arm follows planned joint coordinates kinematically. The strawberry is a
free MuJoCo body with gravity and contacts. No commands are sent to hardware.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel


def make_physics_xml(source: Path, destination: Path) -> None:
    xml = source.read_text(encoding="utf-8")
    pads = {
        'mesh="rlink2" material="mat0"/>':
            'mesh="rlink2" material="mat0"/>\n'
            '                    <geom name="right_pad" type="capsule" '
            'fromto="0.029 0.0115 -0.0045 0.051 0.0115 -0.0045" '
            'size="0.003" contype="1" conaffinity="1" '
            'friction="2.5 0.01 0.0001" rgba="0.2 0.2 0.2 0.25"/>',
        'mesh="llink2" material="mat0"/>':
            'mesh="llink2" material="mat0"/>\n'
            '                    <geom name="left_pad" type="capsule" '
            'fromto="0.029 -0.0115 -0.0045 0.051 -0.0115 -0.0045" '
            'size="0.003" contype="1" conaffinity="1" '
            'friction="2.5 0.01 0.0001" rgba="0.2 0.2 0.2 0.25"/>',
    }
    for old, new in pads.items():
        if xml.count(old) != 1:
            raise RuntimeError(f"Expected one finger geom: {old}")
        xml = xml.replace(old, new)
    old = '<geom name="strawberry_proxy" type="ellipsoid"'
    if xml.count(old) != 1:
        raise RuntimeError("Expected one strawberry proxy")
    xml = xml.replace(old, '<freejoint name="strawberry_free"/>\n'
                           '      <geom name="strawberry_proxy" type="ellipsoid"')
    xml = xml.replace('rgba="0.85 0.06 0.04 0.78" contype="0" conaffinity="0"/>',
                      'rgba="0.85 0.06 0.04 1" mass="0.012" '
                      'contype="1" conaffinity="1" '
                      'friction="2.5 0.01 0.0001"/>')
    xml = xml.replace('  <actuator>',
                      '  <equality>\n'
                      '    <weld name="grasp_assist" body1="Gripping_point_Link" '
                      'body2="observed_strawberry" active="false" torquescale="0.01"/>\n'
                      '  </equality>\n'
                      '  <actuator>', 1)
    destination.write_text(xml, encoding="utf-8")


def add_cloud(scene, table_xyz, table_rgb, fruit_xyz, fruit_rgb) -> None:
    # renderer.update_scene already inserted the robot, floor and strawberry.
    # Append observed points without clearing those MuJoCo model geoms.
    ident = np.eye(3).reshape(-1)
    for points, colors, radius, alpha in (
        (table_xyz, table_rgb, 0.0016, 0.65),
        (fruit_xyz, fruit_rgb, 0.0022, 0.9),
    ):
        for point, color in zip(points, colors):
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
    ap.add_argument("--scene", type=Path, required=True,
                    help="scene.json from model_strawberry_pointcloud.py")
    ap.add_argument("--cloud", type=Path, required=True,
                    help="pointcloud.npz from the same scene")
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "strawberry_physics_pick")
    ap.add_argument("--no-video", action="store_true", help="Physics diagnostic only")
    ap.add_argument("--no-latch", action="store_true",
                    help="Test contact-only grasp without contact-triggered weld")
    ap.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    args = ap.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    scene_info = json.loads(args.scene.read_text(encoding="utf-8"))
    if plan.get("execution") != "simulation_only" or scene_info.get("execution") != "simulation_only":
        raise RuntimeError("Only simulation plans and scenes are accepted")
    if np.linalg.norm(np.array(plan["target_base_xyz_m"]) -
                      np.array(scene_info["target_base_xyz_m"])) > 0.01:
        raise RuntimeError("Plan and point-cloud scene refer to different targets")
    frames = plan["frames"]
    args.out.mkdir(parents=True, exist_ok=True)
    source_xml = Path(scene_info["model_xml"])
    physics_xml = source_xml.parent / "strawberry_grasp_physics.xml"
    make_physics_xml(source_xml, physics_xml)
    cfg = load_config(args.config)
    mimic = json.loads(cfg.model.mjcf.with_suffix(".json").read_text(encoding="utf-8"))["mimic"]
    robot = RobotModel(physics_xml, mimic=mimic)
    model, data = robot.model, robot.data
    fruit_body = model.body("observed_strawberry").id
    fruit_joint = model.joint("strawberry_free").id
    fruit_qadr = int(model.jnt_qposadr[fruit_joint])
    fruit_dadr = int(model.jnt_dofadr[fruit_joint])
    fruit_geom = model.geom("strawberry_proxy").id
    weld_id = model.equality("grasp_assist").id
    gripper_body = model.body("Gripping_point_Link").id
    right_pad = model.geom("right_pad").id
    left_pad = model.geom("left_pad").id
    radii = np.array(scene_info["fruit_ellipsoid_radii_m"])
    cloud = np.load(args.cloud)
    table_xyz = cloud["table_xyz"][::3]
    table_rgb = cloud["table_rgb"][::3]
    fruit_local = cloud["strawberry_xyz"][::2] - np.array(scene_info["fruit_center_base_xyz_m"])
    fruit_rgb = cloud["strawberry_rgb"][::2]

    def set_arm(q, velocity=None):
        q = robot.expand(q)
        for name, value in q.items():
            if name == "strawberry_free":
                continue
            jid = robot.joint_ids[name]
            data.qpos[model.jnt_qposadr[jid]] = value
            data.qvel[model.jnt_dofadr[jid]] = 0.0 if velocity is None else velocity.get(name, 0.0)
            aid = robot.actuator_ids.get(f"act_{name}")
            if aid is not None:
                data.ctrl[aid] = value

    set_arm(frames[0]["q_rad"])
    mujoco.mj_forward(model, data)
    initial_fruit = data.qpos[fruit_qadr:fruit_qadr + 3].copy()
    print(f"[physics] initial fruit={initial_fruit.round(3)}, radius={radii.round(3)}", flush=True)
    # Let the fruit settle onto the table before moving the gripper.
    for _ in range(150):
        set_arm(frames[0]["q_rad"])
        mujoco.mj_step(model, data)
    settled_fruit = data.qpos[fruit_qadr:fruit_qadr + 3].copy()

    renderer = writer = None
    if not args.no_video:
        writer = cv2.VideoWriter(str(args.out / "grasp_pointcloud.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), 12, (800, 600))
        if not writer.isOpened():
            raise RuntimeError("Cannot create MP4 writer")
        renderer = mujoco.Renderer(model, 600, 800)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, camera)
        camera.lookat[:] = [0., -0.19, 0.10]
        camera.distance = 0.67
        camera.azimuth = 160
        camera.elevation = -32

    contact_frames = 0
    both_pad_frames = 0
    consecutive_both = 0
    latch_event = None
    history = []
    previous = frames[0]["q_rad"]
    try:
        for index, frame in enumerate(frames):
            current = frame["q_rad"]
            for step in range(10):
                alpha = (step + 1) / 10
                q = {name: previous[name] + alpha * (current[name] - previous[name])
                     for name in previous}
                velocity = {name: (current[name] - previous[name]) / 0.02
                            for name in previous}
                set_arm(q, velocity)
                mujoco.mj_step(model, data)
                touch = set()
                for contact in data.contact:
                    pair = {int(contact.geom1), int(contact.geom2)}
                    if fruit_geom in pair:
                        if right_pad in pair:
                            touch.add("right")
                        if left_pad in pair:
                            touch.add("left")
                contact_frames += bool(touch)
                both_pad_frames += len(touch) == 2
                consecutive_both = consecutive_both + 1 if len(touch) == 2 else 0
                if (not args.no_latch and latch_event is None and
                        frame["stage"] == "close" and consecutive_both >= 3 and
                        np.linalg.norm(data.xpos[fruit_body] -
                                       np.array(scene_info["fruit_center_base_xyz_m"])) < 0.025):
                    mujoco.mj_forward(model, data)
                    p1, p2 = data.xpos[gripper_body], data.xpos[fruit_body]
                    r1 = Rotation.from_matrix(data.xmat[gripper_body].reshape(3, 3))
                    r2 = Rotation.from_matrix(data.xmat[fruit_body].reshape(3, 3))
                    rel = (r1.inv() * r2).as_quat()
                    model.eq_data[weld_id, 3:6] = r1.inv().apply(p2 - p1)
                    model.eq_data[weld_id, 6:10] = [rel[3], *rel[:3]]
                    data.eq_active[weld_id] = True
                    latch_event = {"frame": index, "stage": frame["stage"],
                                   "fruit_xyz_m": p2.tolist(),
                                   "trigger": "both finger pads in contact for three physics steps"}
                    print(f"[grasp] contact-triggered weld at frame {index}", flush=True)
            previous = current
            fruit_pos = data.qpos[fruit_qadr:fruit_qadr + 3].copy()
            history.append({"stage": frame["stage"], "fruit_xyz_m": fruit_pos.tolist(),
                            "fruit_freejoint_qpos": data.qpos[fruit_qadr:fruit_qadr + 7].tolist()})
            if renderer and (index % 5 == 0 or index == len(frames) - 1):
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                fruit_world = fruit_local @ data.xmat[fruit_body].reshape(3, 3).T + data.xpos[fruit_body]
                add_cloud(renderer.scene, table_xyz, table_rgb, fruit_world, fruit_rgb)
                image = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                label = f"MuJoCo | {frame['stage']} | {index+1}/{len(frames)}"
                cv2.putText(image, label, (15, 34), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (255, 255, 255), 2, cv2.LINE_AA)
                mode_label = ("two-pad contact latch active" if latch_event else
                              "dynamic fruit / contact only")
                cv2.putText(image, mode_label, (15, 61), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (110, 235, 255), 2, cv2.LINE_AA)
                writer.write(image)
                if index in (0, 120, 180, 235, 300, len(frames) - 1):
                    cv2.imwrite(str(args.out / f"frame_{index:03d}.png"), image)
        final_z = float(data.qpos[fruit_qadr + 2])
        lifted = final_z - settled_fruit[2] > 0.04
        results = {
            "schema": "dofbot.strawberry_physics_pick.v1",
            "mode": "contact_only" if args.no_latch else "contact_triggered_weld",
            "fruit_initial_xyz_m": initial_fruit.tolist(),
            "fruit_settled_xyz_m": settled_fruit.tolist(),
            "fruit_final_xyz_m": data.qpos[fruit_qadr:fruit_qadr + 3].tolist(),
            "fruit_lifted": bool(lifted),
            "latch_event": latch_event,
            "fruit_contact_substeps": int(contact_frames),
            "both_pad_contact_substeps": int(both_pad_frames),
            "frame_history": history,
        }
        (args.out / "physics_result.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"[physics] final fruit z={final_z:.3f} m; lifted={lifted}; "
              f"pad contacts={contact_frames}, both pads={both_pad_frames}", flush=True)
        if writer:
            print(f"[video] {args.out / 'grasp_pointcloud.mp4'}", flush=True)
    finally:
        if renderer:
            renderer.close()
        if writer:
            writer.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
