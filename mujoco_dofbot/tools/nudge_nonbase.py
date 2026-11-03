#!/usr/bin/env python3
"""Small, collision-checked manual vision adjustment on ID2/3/4 only."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.collision import CollisionChecker, describe
from dofbot_mj.config import load_config
from dofbot_mj.protocol import ServoMapping, read_position, request
from dofbot_mj.servo_link import detect_port


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--sid", type=int, choices=(2, 3, 4), required=True)
    p.add_argument("--delta-deg", type=float, required=True)
    p.add_argument("--move-ms", type=int, default=1200)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    if not 0 < abs(args.delta_deg) <= 3:
        p.error("A single adjustment must be at most 3 degrees")
    if not 300 <= args.move_ms <= 1500:
        p.error("--move-ms must be in 300..1500")

    cfg = load_config(ROOT / "config" / "visual_match.yaml")
    mapping = ServoMapping(cfg.servo.joints, cfg.servo.range_deg)
    checker = CollisionChecker(ROOT / "models" / "dofbot_pro_collision_visual.xml")
    port = cfg.servo.port or detect_port()
    with serial.Serial(port, cfg.servo.baudrate, timeout=.25, write_timeout=1) as ser:
        ser.reset_input_buffer()
        time.sleep(1.5)
        request(ser, f"RANGE {cfg.servo.range_deg}",
                f"OK RANGE {cfg.servo.range_deg}", timeout=8)
        deg = {j.sid: read_position(ser, j.sid) * cfg.servo.range_deg / 4095
               for j in cfg.servo.joints}
        start_q = mapping.positions_from_targets(deg)
        target = deg[args.sid] + args.delta_deg
        print("Current " + " ".join(f"ID{k}={v:.2f}°" for k, v in sorted(deg.items())))
        for i in range(11):
            value = deg[args.sid] + args.delta_deg * i / 10
            q = dict(start_q, **mapping.positions_from_targets({args.sid: value}))
            hits = checker.check(q)
            if hits:
                raise RuntimeError(f"Rejected at {i/10:.1f}: {describe(hits)}")
        print(f"ID{args.sid}: {deg[args.sid]:.2f}° -> {target:.2f}°, tabletop path clear")
        if args.dry_run:
            return 0
        before = deg[args.sid]
        request(ser, f"MOVE {args.sid} {target:.2f} {args.move_ms}",
                f"OK MOVE {args.sid}", timeout=3)
        time.sleep(args.move_ms / 1000 + 0.4)
        after = read_position(ser, args.sid) * cfg.servo.range_deg / 4095
    print(f"ID{args.sid} readback {after:.2f}°, change {after-before:+.2f}°")
    # A 1-degree probe must not report success when the servo stayed still.
    if abs(after - before) < max(0.4, 0.5 * abs(args.delta_deg)) or abs(after - target) > 1.0:
        raise RuntimeError("Servo did not reach target; stop further motion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
