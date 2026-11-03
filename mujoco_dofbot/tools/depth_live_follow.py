#!/usr/bin/env python3
"""Show live Orbbec points and read-only servo tracking in one MuJoCo viewer.

The camera-to-link axes are a visual estimate. Precise registration still
requires a measured hand-eye transform.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from depth_live import backproject, depth_min_default, import_vision
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel
from dofbot_mj.protocol import ServoMapping, read_position, request
from dofbot_mj.servo_link import detect_port, probe_positions


def main() -> int:
    p = argparse.ArgumentParser(description="MuJoCo live arm + Orbbec point cloud")
    p.add_argument("--config", default=str(ROOT / "config" / "visual_match.yaml"))
    p.add_argument("--hz", type=float, default=5.0)
    p.add_argument("--points", type=int, default=3000)
    p.add_argument("--max-depth-mm", type=int, default=1200)
    args = p.parse_args()
    if args.hz <= 0 or args.points <= 0 or args.max_depth_mm <= 0:
        raise ValueError("hz, points and max-depth-mm must be positive")

    cfg = load_config(args.config)
    sidecar = cfg.model.mjcf.with_suffix(".json")
    mimic = json.loads(sidecar.read_text(encoding="utf-8")).get("mimic", {})
    robot = RobotModel(cfg.model.mjcf, mimic=mimic)
    pose, degrees = probe_positions(cfg.servo)
    robot.apply_kinematic(pose)
    port = cfg.servo.port or detect_port()
    if not port:
        raise RuntimeError("No servo serial port found")
    print(f"[servo] {port}, six IDs read; no MOVE commands will be sent", flush=True)

    camera_body = robot.model.body("DaBai_DCW2_Link").id
    # Optical frame X right/Y down/Z forward in the camera link. The former
    # manual axes pointed the cloud through the floor after the mount fix.
    mount = np.array([[-1., 0., 0.],
                      [0., 0., 1.],
                      [0., 1., 0.]])
    print("[cloud] approximate mount: right=-x, down=+z, forward=+y", flush=True)
    dmin = depth_min_default()
    mapping = ServoMapping(cfg.servo.joints, cfg.servo.range_deg)
    period = 1.0 / args.hz

    OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH = import_vision()
    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
        time.sleep(1.2)
        import serial
        from mujoco import viewer

        with serial.Serial(port, cfg.servo.baudrate, timeout=0.2,
                           write_timeout=1.0) as ser:
            ser.reset_input_buffer()
            request(ser, f"RANGE {cfg.servo.range_deg}",
                    f"OK RANGE {cfg.servo.range_deg}", timeout=8.0)
            with viewer.launch_passive(robot.model, robot.data) as handle:
                handle.cam.lookat[:] = [0.0, -0.27, 0.17]
                handle.cam.distance = 0.9
                handle.cam.azimuth = 180.0
                handle.cam.elevation = -20.0
                print("[viewer] live arm and point cloud; close window to stop", flush=True)
                count_frames = 0
                while handle.is_running():
                    started = time.monotonic()
                    try:
                        degrees = {
                            sid: mapping.degrees_from_position(read_position(ser, sid))
                            for sid in mapping.ids
                        }
                        robot.apply_kinematic(mapping.positions_from_targets(degrees))
                    except Exception as exc:
                        print(f"[servo] read failed, keeping last pose: {exc}", flush=True)

                    pair = cam.get_frame_pair(timeout_ms=800)
                    if pair.get("depth") is not None:
                        depth = pair["depth"]
                        valid = int(((depth >= dmin) &
                                     (depth <= args.max_depth_mm)).sum())
                        stride = max(1, int(np.ceil(np.sqrt(valid / args.points))))
                        pts, rgb = backproject(depth, pair.get("color"), dmin,
                                               args.max_depth_mm, 64.0, 55.0, stride)
                        if len(pts):
                            body_mat = robot.data.xmat[camera_body].reshape(3, 3)
                            body_pos = robot.data.xpos[camera_body]
                            world = pts @ (body_mat @ mount).T + body_pos
                            scene = handle.user_scn
                            scene.ngeom = 0
                            drawn = min(len(world), scene.maxgeom)
                            for i in range(drawn):
                                mujoco.mjv_initGeom(
                                    scene.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    np.array([0.0025, 0., 0.]), world[i],
                                    np.eye(3).reshape(-1),
                                    np.array([*rgb[i], 1.], dtype=np.float32))
                            scene.ngeom = drawn
                            if count_frames % 20 == 0:
                                print(f"[frame] {count_frames} servo={degrees} "
                                      f"points={drawn} depth_valid={valid}", flush=True)
                    handle.sync()
                    count_frames += 1
                    remain = period - (time.monotonic() - started)
                    if remain > 0:
                        time.sleep(remain)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        print("[viewer] closed; camera and serial released", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
