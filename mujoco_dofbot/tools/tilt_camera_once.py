#!/usr/bin/env python3
"""Move only ID4 a small, checked amount to aim the wrist camera downward.

This is a one-time manual adjustment, separate from automatic trajectory motion.
The visual servo mapping is approximate; the program reads back the real angle.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.model import RobotModel
from dofbot_mj.protocol import ServoMapping, read_position, request
from dofbot_mj.servo_link import detect_port, probe_positions


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--delta-deg", type=float, default=5.0,
                    help="ID4 angle increment, positive points the modeled camera down")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if not 0 < args.delta_deg <= 5:
        ap.error("Only a positive adjustment of at most 5 degrees is allowed")

    cfg = load_config(ROOT / "config" / "visual_match.yaml")
    state, angles = probe_positions(cfg.servo)
    target_deg = angles[4] + args.delta_deg
    joint = next(j for j in cfg.servo.joints if j.sid == 4)
    mapping = ServoMapping(cfg.servo.joints, cfg.servo.range_deg)
    goal = dict(state)
    goal.update(mapping.positions_from_targets({4: target_deg}))

    mimic = json.loads(cfg.model.mjcf.with_suffix(".json").read_text(encoding="utf-8"))["mimic"]
    robot = RobotModel(cfg.model.mjcf, mimic=mimic)
    q4 = goal[joint.name]
    lo, hi = robot.limits(joint.name)
    if not lo + 0.05 < q4 < hi - 0.05:
        raise RuntimeError(f"ID4 target would exceed joint margin: q4={q4:.3f} rad")
    robot.apply_kinematic(goal)
    tip_z = robot.data.xpos[robot.model.body("Gripping_point_Link").id][2]
    if tip_z < 0.07:
        raise RuntimeError(f"Predicted tip clearance too small: {tip_z:.3f} m")
    print(f"ID4 {angles[4]:.2f} -> {target_deg:.2f} deg; "
          f"modeled q4={q4:.3f} rad, tip z={tip_z:.3f} m")
    if args.dry_run:
        return 0

    port = cfg.servo.port or detect_port()
    if not port:
        raise RuntimeError("No servo serial port found")
    with serial.Serial(port, cfg.servo.baudrate, timeout=0.25, write_timeout=1.0) as ser:
        ser.reset_input_buffer()
        request(ser, f"RANGE {cfg.servo.range_deg}",
                f"OK RANGE {cfg.servo.range_deg}", timeout=8.0)
        before = read_position(ser, 4) * cfg.servo.range_deg / 4095
        if abs(before - angles[4]) > 2:
            raise RuntimeError("ID4 changed since preflight; refusing stale motion")
        request(ser, f"MOVE 4 {target_deg:.2f} 1500", "OK MOVE 4", timeout=3.0)
        time.sleep(2.0)
        after = read_position(ser, 4) * cfg.servo.range_deg / 4095
    print(f"ID4 readback {after:.2f} deg, change {after-before:+.2f} deg")
    if abs(after - target_deg) > 1.5:
        raise RuntimeError("ID4 did not reach the commanded angle; stop and inspect power/servo")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
