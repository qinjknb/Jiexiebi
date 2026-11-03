#!/usr/bin/env python3
"""Read six servos, move one axis by a small amount, and capture a depth cloud.

Run with hardware access: python tools/demo_live_depth.py
Only the selected ID receives MOVE. The script checks its start position and returns it
to the measured angle even if rendering or camera capture fails.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from dofbot_mj.config import load_config
from dofbot_mj.protocol import ServoMapping, read_position, request
from dofbot_mj.runner import SimApp
from dofbot_mj.servo_link import detect_port, probe_positions


def capture_cloud(out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    from depth_cloud import build_cloud, grab_pair, import_vision, load_depth_min

    OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH = import_vision()
    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
        time.sleep(1.2)
        pair = grab_pair(cam)
    finally:
        cam.close()
    if pair is None:
        raise RuntimeError("Depth camera did not return a frame")

    points, colors, stats = build_cloud(
        pair["depth"], pair["color"], load_depth_min(), 1400, 64, 55, 9000)
    if len(points) == 0:
        raise RuntimeError("Depth frame had no valid points")
    print(f"[cloud] valid={stats['valid']} sampled={len(points)} "
          f"median={stats['median_mm']} mm")

    # Camera frame: X right, Y down, Z forward. Keep this in the camera frame
    # because the current hand-eye extrinsic is only an approximate estimate.
    fig = plt.figure(figsize=(11, 8), facecolor="#f7f9fc")
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:, 0], points[:, 2], -points[:, 1],
               c=np.clip(colors, 0, 1), s=1.8, alpha=0.9, linewidths=0)
    ax.set_xlabel("X right (m)")
    ax.set_ylabel("Z forward (m)")
    ax.set_zlabel("Y up (m)")
    ax.set_title("Orbbec DCW2 depth point cloud · live demo peak")
    ax.view_init(elev=24, azim=-68)
    ax.set_box_aspect((1.2, 1.7, 1.0))
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)
    print(f"[cloud] {out}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--id", type=int, default=1, choices=range(1, 7))
    p.add_argument("--delta-deg", type=float, default=3.0)
    p.add_argument("--out", type=Path, default=ROOT / "out" / "live_depth_demo")
    args = p.parse_args()
    if not 0 < abs(args.delta_deg) <= 5:
        raise ValueError("absolute delta-deg must be greater than 0 and at most 5")
    args.out.mkdir(parents=True, exist_ok=True)

    cfg = load_config(ROOT / "config" / "calibrated.yaml")
    port = cfg.servo.port or detect_port()
    if not port:
        raise RuntimeError("No serial port found")
    cfg.servo.port = port
    pose, degrees = probe_positions(cfg.servo)
    start = degrees[args.id]
    peak = start + args.delta_deg
    mapping = ServoMapping(cfg.servo.joints, cfg.servo.range_deg)
    joint = next(j for j in cfg.servo.joints if j.sid == args.id)
    peak_rad = (peak - joint.offset_deg) / (joint.direction * joint.scale_deg_per_rad)
    app = SimApp(cfg, initial=pose)
    low, high = app.robot.limits(joint.name)
    if not (15 < start < 285 and low <= peak_rad <= high):
        raise RuntimeError("Arm1 target fails servo or model limits")
    print(f"[preflight] {port} ID{args.id} {start:.2f} -> {peak:.2f} deg; "
          f"model limit [{low:.3f}, {high:.3f}] rad")
    app.render_state(pose, str(args.out / "arm_before.png"))

    import serial
    motion_sent = False
    actual = None
    with serial.Serial(port, cfg.servo.baudrate, timeout=0.2, write_timeout=1.0) as ser:
        try:
            ser.reset_input_buffer()
            request(ser, f"PING {args.id}", f"OK PING {args.id}")
            current = mapping.degrees_from_position(read_position(ser, args.id))
            if abs(current - start) > 1.0:
                raise RuntimeError(f"ID{args.id} start changed to {current:.2f} deg")
            request(ser, f"MOVE {args.id} {peak:.2f} 900", f"OK MOVE {args.id}")
            motion_sent = True
            started = time.monotonic()
            for i in range(51):
                blend = 0.5 * (1 - math.cos(math.pi * i / 50))
                frame = dict(pose)
                frame[joint.name] = pose[joint.name] + (peak_rad - pose[joint.name]) * blend
                app.robot.apply_kinematic(frame)
                wait = started + i / 50 - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
            app.render_state(frame, str(args.out / "arm_peak.png"))
            actual = mapping.degrees_from_position(read_position(ser, args.id))
            print(f"[peak] target={peak:.2f} actual={actual:.2f} deg")
            capture_cloud(args.out / "depth_cloud_peak.png")
        finally:
            if motion_sent:
                request(ser, f"MOVE {args.id} {start:.2f} 900", f"OK MOVE {args.id}")
                time.sleep(1.2)
                end = mapping.degrees_from_position(read_position(ser, args.id))
                app.robot.apply_kinematic(pose)
                print(f"[return] target={start:.2f} actual={end:.2f} deg")
                if abs(end - start) > 2.0:
                    raise RuntimeError(f"ID{args.id} did not return to its start position")
    if actual is None or abs(actual - start) < 1.0:
        raise RuntimeError(f"ID{args.id} acknowledged MOVE but did not physically move")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
