#!/usr/bin/env python3
"""Estimate the cutting-mat plane from one aligned RGB-D frame.

This provides a camera pitch/roll and height check for simulation. It is not
a hand-eye calibration and must not authorize physical motion.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from strawberry_demo import COLOR_HFOV, COLOR_VFOV


def plane_from_depth(depth: np.ndarray, stride: int = 5, intrinsics=None) -> dict:
    h, w = depth.shape
    yy, xx = np.mgrid[90:min(h, 400):stride, 170:min(w, 620):stride]
    z = depth[yy, xx].astype(float) / 1000.0
    keep = (z > 0.20) & (z < 0.85)
    fx = (w / 2) / math.tan(math.radians(COLOR_HFOV) / 2)
    fy = (h / 2) / math.tan(math.radians(COLOR_VFOV) / 2)
    points = np.stack(((xx[keep] - w / 2) * z[keep] / fx,
                       (yy[keep] - h / 2) * z[keep] / fy,
                       z[keep]), axis=1)
    if intrinsics is not None:
        import cv2
        k = intrinsics
        K = np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
        uv = np.column_stack((xx[keep],yy[keep])).astype(float).reshape(-1,1,2)
        rays = cv2.undistortPoints(uv,K,np.array(k['dist'])).reshape(-1,2)
        points = np.column_stack((rays*z[keep,None],z[keep]))
    if len(points) < 300:
        raise RuntimeError("Too few valid table depth points")
    rng = np.random.default_rng(31)
    best = np.zeros(len(points), bool)
    for _ in range(700):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        residual = np.abs((points - a) @ normal)
        inliers = residual < 0.006
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 0.4 * len(points):
        raise RuntimeError("No dominant plane in the table ROI")
    center = np.mean(points[best], axis=0)
    _, _, vh = np.linalg.svd(points[best] - center, full_matrices=False)
    normal = vh[-1]
    # The table normal points towards the camera. Optical +Y points down.
    if normal[1] > 0:
        normal = -normal
    d = -float(normal @ center)
    errors = np.abs(points[best] @ normal + d)
    return {
        "normal_camera": normal.tolist(),
        "offset_m": d,
        "camera_height_above_table_m": d,
        "inlier_count": int(best.sum()),
        "sample_count": int(len(points)),
        "inlier_fraction": float(best.mean()),
        "median_residual_m": float(np.median(errors)),
        "roi_uvxy": [170, 90, min(w, 620), min(h, 400)],
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("depth", type=Path, help="Aligned depth .npy in millimetres")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--intrinsics", type=Path)
    args = ap.parse_args()
    k = None
    if args.intrinsics:
        import yaml
        k = yaml.safe_load(args.intrinsics.read_text(encoding='utf-8'))['camera_matrix']
    result = plane_from_depth(np.load(args.depth), intrinsics=k)
    out = args.out or args.depth.with_suffix(".table.json")
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
