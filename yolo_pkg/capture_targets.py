#!/usr/bin/env python3
"""Capture one aligned RGB-D frame and export YOLO strawberry targets.

Output coordinates are in the camera optical frame, in metres: X right,
Y down, Z forward. No robot motion is performed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "Ultralytics"))
from strawberry_demo import (ALIGN_MODE, COLOR_HFOV, COLOR_PROFILE_INDEX,
                             COLOR_VFOV, DEFAULT_MODEL, DEPTH_PROFILE_INDEX,
                             DEPTH_VALID_MAX, DEPTH_VALID_MIN, RIPENESS_IDS,
                             STRAWBERRY_ID, STREAM_COLOR, STREAM_DEPTH,
                             OrbbecDCW2)


def aligned_depth(depth: np.ndarray, box: np.ndarray) -> tuple[int, int, int, float]:
    h, w = depth.shape
    x1, y1, x2, y2 = map(float, box)
    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
    half = min(12, max(4, int(min(x2 - x1, y2 - y1) * 0.18)))
    patch = depth[max(0, cy - half):min(h, cy + half + 1),
                  max(0, cx - half):min(w, cx + half + 1)]
    values = patch[(patch >= DEPTH_VALID_MIN) & (patch <= DEPTH_VALID_MAX)]
    valid_fraction = float(len(values) / patch.size) if patch.size else 0.0
    # A few background pixels inside a depth hole must not be treated as
    # the fruit surface. This occurs when the object is too close to the D2C
    # camera or when the box straddles a depth discontinuity.
    if len(values) < 30 or valid_fraction < 0.7:
        return cx, cy, 0, valid_fraction
    return cx, cy, int(np.median(values)), valid_fraction


def capture_frame() -> tuple[np.ndarray, np.ndarray]:
    cam = OrbbecDCW2()
    try:
        cam.set_laser(True)
        cam.enable_streams([(STREAM_COLOR, COLOR_PROFILE_INDEX),
                            (STREAM_DEPTH, DEPTH_PROFILE_INDEX)],
                           align_mode=ALIGN_MODE)
        time.sleep(1.2)
        for _ in range(25):
            pair = cam.get_frame_pair(timeout_ms=1500)
            color, depth = pair.get("color"), pair.get("depth")
            if color is not None and depth is not None and color.shape[:2] == depth.shape:
                return color.copy(), depth.copy()
        raise RuntimeError("No aligned RGB-D frame received")
    finally:
        cam.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", type=Path, help="Offline RGB image")
    ap.add_argument("--depth", type=Path, help="Offline aligned depth .npy (mm)")
    ap.add_argument("--model", type=Path, default=Path(DEFAULT_MODEL))
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--out", type=Path, default=ROOT / "out" / "targets.json")
    args = ap.parse_args()
    if bool(args.image) != bool(args.depth):
        ap.error("--image and --depth must be provided together")

    if args.image:
        color = cv2.imread(str(args.image))
        if color is None:
            raise RuntimeError(f"Cannot read {args.image}")
        depth = np.load(args.depth)
    else:
        color, depth = capture_frame()
    if color.shape[:2] != depth.shape:
        raise RuntimeError("Color and depth are not pixel aligned")

    from ultralytics import YOLO
    model = YOLO(str(args.model))
    result = model(color, conf=args.conf, verbose=False)[0]
    detections = []
    if result.boxes is not None:
        for box, cls, confidence in zip(result.boxes.xyxy.cpu().numpy(),
                                        result.boxes.cls.cpu().numpy(),
                                        result.boxes.conf.cpu().numpy()):
            detections.append({"box": box, "cls": int(cls),
                               "confidence": float(confidence)})

    h, w = depth.shape
    fx = (w / 2) / math.tan(math.radians(COLOR_HFOV) / 2)
    fy = (h / 2) / math.tan(math.radians(COLOR_VFOV) / 2)
    names = {1: "good", 2: "raw", 3: "medium", 4: "overripe"}
    targets = []
    preview = color.copy()
    for det in detections:
        if det["cls"] != STRAWBERRY_ID:
            continue
        x1, y1, x2, y2 = det["box"]
        ranks = []
        for other in detections:
            if other["cls"] not in RIPENESS_IDS:
                continue
            a, b, c, d = other["box"]
            if x1 <= (a + c) / 2 <= x2 and y1 <= (b + d) / 2 <= y2:
                ranks.append(other)
        ripe = max(ranks, key=lambda item: item["confidence"], default=None)
        u, v, z_mm, depth_fraction = aligned_depth(depth, det["box"])
        point = None
        if z_mm:
            z = z_mm / 1000.0
            point = [round((u - w / 2) * z / fx, 5),
                     round((v - h / 2) * z / fy, 5), round(z, 5)]
        targets.append({
            "id": len(targets) + 1,
            "box_xyxy": [round(float(x), 1) for x in det["box"]],
            "pixel_uv": [u, v],
            "confidence": round(det["confidence"], 4),
            "ripeness": names.get(ripe["cls"], "unknown") if ripe else "unknown",
            "ripeness_confidence": round(ripe["confidence"], 4) if ripe else 0.0,
            "depth_mm": z_mm,
            "depth_valid_fraction": round(depth_fraction, 3),
            "camera_xyz_m": point,
        })
        cv2.rectangle(preview, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(preview, f"#{len(targets)} {targets[-1]['ripeness']} {z_mm}mm",
                    (int(x1), max(18, int(y1) - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

    payload = {
        "schema": "dofbot.strawberry_targets.v1",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "source": "offline" if args.image else "Orbbec DaBai DCW2",
        "frame_size": [w, h],
        "coordinate_frame": "camera_optical_x_right_y_down_z_forward",
        "units": "m",
        "depth_alignment": "D2C",
        "targets": targets,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    cv2.imwrite(str(args.out.with_suffix(".png")), preview)
    cv2.imwrite(str(args.out.with_suffix(".rgb.png")), color)
    np.save(args.out.with_suffix(".depth.npy"), depth)
    print(f"Saved {len(targets)} targets to {args.out}")
    for target in targets:
        print(f"  #{target['id']} {target['ripeness']} "
              f"conf={target['confidence']:.2f} camera={target['camera_xyz_m']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
