#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""相机实时预览 + ArUco 检测（用来摆标定板、确认能不能看到）。

窗口里显示彩色画面；检测到标记会把它框成绿色并标出 ID，检测不到则顶部红色提示。

用法：
    python tools/camera_preview.py
    python tools/camera_preview.py --id 0 --dict DICT_4X4_50
    python tools/camera_preview.py --save calib\snap.png     # 按 s 时保存到该路径

按键：q / ESC 退出，s 保存当前帧
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
REPO = ROOT.parent
VISION = REPO / "yolo_pkg"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(VISION))


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="相机预览 + ArUco 检测")
    p.add_argument("--dict", default="DICT_4X4_50")
    p.add_argument("--id", type=int, default=None, help="只认这个 ID；不填则任意")
    p.add_argument("--save", default=str(ROOT / "calib" / "snapshot.png"))
    p.add_argument("--depth", action="store_true", help="右侧同时显示深度伪彩")
    args = p.parse_args(argv)

    import cv2

    os.environ.setdefault("ORBBEC_SDK_DIR", str(VISION / "sdk"))
    from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    print(f"[相机] 打开 DaBai DCW2（彩色 + 深度，D2C 对齐）…")
    cam = OrbbecDCW2()
    cam.set_laser(True)
    cam.enable_streams([(STREAM_COLOR, 0), (STREAM_DEPTH, 0)], align_mode=2)
    time.sleep(1.2)
    print(f"[检测] 字典 {args.dict}" + (f"，只认 id={args.id}" if args.id is not None else ""))
    print("[窗口] q / ESC 退出，s 保存当前帧")

    window = "DOFBOT camera preview"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 860, 500)

    last = time.monotonic()
    fps = 0.0
    hits = 0
    frames = 0
    try:
        while True:
            pair = cam.get_frame_pair(timeout_ms=2000)
            if pair.get("color") is None:
                key = cv2.waitKey(10) & 0xFF
                if key in (27, ord("q")):
                    break
                continue

            color = pair["color"]
            depth = pair.get("depth")
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            corners, ids, _rej = detector.detectMarkers(gray)

            view = color.copy()
            shown = None
            side_px = 0.0
            if ids is not None and len(ids):
                for corner, mid in zip(corners, ids.flatten()):
                    if args.id is not None and int(mid) != args.id:
                        continue
                    shown = int(mid)
                    pts = corner.reshape(4, 2)
                    cv2.polylines(view, [pts.astype(np.int32)], True, (0, 255, 0), 2)
                    for index, (u, v) in enumerate(pts.astype(np.int32)):
                        cv2.circle(view, (int(u), int(v)), 4, (0, 0, 255), -1)
                        cv2.putText(view, str(index), (int(u) + 6, int(v) - 6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    # 四条边的平均像素长度：位姿估计精度主要由它决定
                    edges = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
                    side_px = float(np.mean(edges))

            if shown is not None:
                hits += 1
                # 边长 <60px 太小、60~100 勉强、>=100 合适
                if side_px >= 100:
                    tone, note = (20, 120, 20), "good"
                elif side_px >= 60:
                    tone, note = (20, 110, 160), "small"
                else:
                    tone, note = (30, 30, 180), "TOO SMALL"
                banner = f"OK  ArUco id={shown}  {side_px:5.1f} px  ({note})"
                cv2.rectangle(view, (0, 0), (view.shape[1], 34), tone, -1)
                cv2.putText(view, banner, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.66, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.rectangle(view, (0, 0), (view.shape[1], 34), (30, 30, 180), -1)
                cv2.putText(view, "NO ArUco detected", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            frames += 1
            now = time.monotonic()
            fps = 0.9 * fps + 0.1 / max(1e-6, now - last)
            last = now
            cv2.putText(view, f"{fps:4.1f} fps", (8, view.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            if args.depth and depth is not None:
                valid = depth > 0
                if valid.any():
                    lo, hi = np.percentile(depth[valid], 2), np.percentile(depth[valid], 98)
                else:
                    lo, hi = 0, 1
                norm = np.clip((depth.astype(np.float32) - lo) / max(1e-6, hi - lo), 0, 1)
                colored = cv2.applyColorMap((norm * 255).astype(np.uint8),
                                            cv2.COLORMAP_TURBO)
                colored[~valid] = (20, 20, 20)
                view = np.hstack([view, colored])

            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                out = Path(args.save)
                out.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out), view)
                print(f"[保存] {out}")
            if not _window_closed(window, cv2):
                print("[窗口] 已关闭")
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print(f"[结束] 共 {frames} 帧，其中 {hits} 帧检测到标记")
    return 0


def _window_closed(name, cv2, quiet_after: int = 1) -> bool:
    """检测窗口是否被用户关掉（cv2 没有直接 API）。"""
    try:
        if cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
            return False
    except cv2.error:
        return False
    return True


if __name__ == "__main__":
    raise SystemExit(main())
