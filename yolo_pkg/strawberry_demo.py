 # -*- coding: utf-8 -*-
"""
草莓实时识别 + 位置输出演示程序

功能：
    实时用 YOLO 识别草莓（品种 + 成熟度），并用深度相机测算每颗草莓的：
        距离 Z (mm) 与相机坐标系下的 3D 坐标 (X, Y)
    结果同时显示在画面上、打印到控制台。

硬件：
    Orbbec DaBai DCW2（RGB-D）
    注意：相机需距离目标 0.4m 以上，否则深度大面积无效（结构光盲区）

运行：
    .\\.venv\\Scripts\\python.exe strawberry_demo.py
    .\\.venv\\Scripts\\python.exe strawberry_demo.py --no-show        # 只看控制台
    .\\.venv\\Scripts\\python.exe strawberry_demo.py --model 路径.pt  # 换模型

按键：
    q / ESC  退出
    s        保存当前帧（彩色+深度）
    d        切换深度图叠加显示
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from orbbec_v1 import OrbbecDCW2, STREAM_COLOR, STREAM_DEPTH   # noqa: E402

# ---------------------------------------------------------------- 配置

DEFAULT_MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "weights", "best.pt")

# 模型类别（与 mydata.yaml 一致）
CLASS_NAMES = ["strawberry", "good", "raw", "medium", "overripe"]
RIPENESS_IDS = [1, 2, 3, 4]          # 成熟度类别
STRAWBERRY_ID = 0                     # 定位框类别

# 成熟度中文名与显示颜色 (B, G, R)
RIPENESS_INFO = {
    1: ("good/好", (0, 200, 0)),
    2: ("raw/生", (0, 200, 200)),
    3: ("medium/中", (0, 165, 255)),
    4: ("overripe/过熟", (0, 0, 255)),
}

# 相机视场角（用于彩色图与深度图的坐标换算，来自官方规格）
COLOR_HFOV, COLOR_VFOV = 64.0, 55.0     # 4:3 模式
DEPTH_HFOV, DEPTH_VFOV = 91.0, 62.0

# 彩色流 profile 索引：0 = 640x480@15 MJPG
# （实测 MJPG 只需 37KB/帧，深度流稳定 6/6；
#   而 RGB888 要 921KB/帧，USB2.0 带宽不足导致深度只有 4/6）
COLOR_PROFILE_INDEX = 0
DEPTH_PROFILE_INDEX = 0                  # 640x400 @15fps

# D2C 对齐模式（实测本机可用值）
#   1 -> 深度变 640x480，稠密度 89.9%
#   2 -> 深度变 640x480，稠密度 93.2%   <- 推荐
#   3 -> 保持 640x400（未真正对齐）
ALIGN_MODE = 2

CONF_THRESHOLD = 0.35                    # YOLO 置信度阈值
DEPTH_VALID_MIN = 10                     # 小于此深度(mm)视为不可靠（盲区 1cm）
DEPTH_VALID_MAX = 4000


# ---------------------------------------------------------------- 工具

def color_box_to_depth(box, color_shape, depth_shape):
    """把彩色图上的检测框映射到深度图坐标系。

    两路流的视场角不同，按 FOV 比例做线性映射（近似但实用）。
    """
    x1, y1, x2, y2 = box
    cw, chh = color_shape[1], color_shape[0]
    dw, dh = depth_shape[1], depth_shape[0]

    def map_axis(v, csize, dsize, cfov, dfov):
        # 像素 -> 归一化角度 -> 目标图像素
        ang = (v / float(csize) - 0.5) * cfov       # 相对光轴的角度
        return (ang / dfov + 0.5) * dsize

    return (
        map_axis(x1, cw, dw, COLOR_HFOV, DEPTH_HFOV),
        map_axis(y1, chh, dh, COLOR_VFOV, DEPTH_VFOV),
        map_axis(x2, cw, dw, COLOR_HFOV, DEPTH_HFOV),
        map_axis(y2, chh, dh, COLOR_VFOV, DEPTH_VFOV),
    )


def measure_diameter(depth, cx, cy, z_center, half=60, tol=0.12,
                     hfov=None, vfov=None):
    """在深度图上测量以 (cx,cy) 为中心的目标直径（mm）。

    做法：取目标深度上下 12% 容差范围内的像素，统计其最大跨度。
    这是"俯瞰球体"的近似测量，精度有限但对演示足够。

    hfov/vfov: 该深度图对应的视场角。D2C 对齐后深度图与彩色同坐标系，
               应传彩色的 64x55；未对齐时用深度的 91x62。
    """
    h, w = depth.shape
    if hfov is None:
        hfov = DEPTH_HFOV
    if vfov is None:
        vfov = DEPTH_VFOV
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    if x1 <= x0 or y1 <= y0 or z_center <= 0:
        return 0, 0

    roi = depth[y0:y1, x0:x1].astype(np.float32)
    lo = z_center * (1.0 - tol)
    hi = z_center * (1.0 + tol)
    mask = (roi >= lo) & (roi <= hi)

    cols = np.where(mask.any(axis=0))[0]
    rows = np.where(mask.any(axis=1))[0]
    if cols.size == 0 or rows.size == 0:
        return 0, 0

    # 像素跨度 -> 物理尺寸（用该深度的像素当量换算）
    import math
    fx = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    fy = (h / 2.0) / math.tan(math.radians(vfov) / 2.0)
    w_mm = (cols.max() - cols.min() + 1) * z_center / fx
    h_mm = (rows.max() - rows.min() + 1) * z_center / fy
    return w_mm, h_mm


def pick_best_depth(depth, cx, cy, window=8):
    """在检测框中心附近取最可靠的深度值。

    结构光对光滑表面会产生空洞，所以扩大搜索窗口并用中位数抗噪。
    """
    h, w = depth.shape
    x0, x1 = max(0, cx - window), min(w, cx + window + 1)
    y0, y1 = max(0, cy - window), min(h, cy + window + 1)
    patch = depth[y0:y1, x0:x1]
    v = patch[(patch >= DEPTH_VALID_MIN) & (patch <= DEPTH_VALID_MAX)]
    if v.size == 0:
        return 0
    return int(np.median(v))


# ---------------------------------------------------------------- 主程序

def main():
    ap = argparse.ArgumentParser(description="草莓实时识别 + 位置输出")
    ap.add_argument("--model", default=DEFAULT_MODEL, help="YOLO 模型路径")
    ap.add_argument("--conf", type=float, default=CONF_THRESHOLD,
                    help="置信度阈值")
    ap.add_argument("--no-show", action="store_true", help="不显示窗口")
    ap.add_argument("--quiet", action="store_true", help="控制台不逐帧打印")
    args = ap.parse_args()

    print("=" * 72)
    print("  草莓实时识别 + 位置输出")
    print("=" * 72)

    # ---- 加载模型
    if not os.path.isfile(args.model):
        print("[错误] 找不到模型: %s" % args.model)
        return 1
    from ultralytics import YOLO
    print("加载模型: %s" % args.model)
    model = YOLO(args.model)
    print("模型类别: %s" % CLASS_NAMES)

    # ---- 连接深度相机（一次性启用彩色+深度，并开启 D2C 对齐）
    print("\n连接深度相机 ...")
    cam = OrbbecDCW2()
    print("  设备: %s (SN=%s)" % (cam.device_name, cam.serial))
    # D2C 对齐：深度图会变成与彩色相同的 640x480 且坐标系一致，
    # 这样 YOLO 检测框的像素坐标可直接用于查深度，无需坐标映射。
    cam.enable_streams([(STREAM_COLOR, COLOR_PROFILE_INDEX),
                        (STREAM_DEPTH, DEPTH_PROFILE_INDEX)],
                       align_mode=ALIGN_MODE)
    time.sleep(2.0)

    color_img = None
    depth = None
    for _ in range(40):
        fr = cam.get_frame_pair(timeout_ms=1500)
        if fr["color"] is not None:
            color_img = fr["color"]
        if fr["depth"] is not None and (fr["depth"] > 0).sum() > 5000:
            depth = fr["depth"]
        if color_img is not None and depth is not None:
            break
    if color_img is None:
        print("[错误] 彩色流取帧失败")
        cam.close()
        return 1
    if depth is None:
        print("[错误] 深度流取帧失败")
        cam.close()
        return 1

    ch, cw = color_img.shape[:2]
    dh, dw = depth.shape
    s = cam.depth_stats(depth)
    aligned = (dw == cw and dh == ch)
    print("  彩色流: %dx%d" % (cw, ch))
    print("  深度流: %dx%d  稠密度=%.1f%%  中位=%smm"
          % (dw, dh, s["density"], s.get("median")))
    print("  D2C 对齐: %s" % ("已对齐（坐标系一致，坐标可直接对应）"
                              if aligned else
                              "未对齐（将按视场角比例映射）"))
    if not aligned:
        print("  [提示] 深度与彩色尺寸不同，映射精度会受影响。")

    if s["density"] < 10:
        print("\n[警告] 深度稠密度偏低(%.1f%%)！" % s["density"])
        print("       请把相机对准 0.4~1.5m 外的目标（<30cm 会测不出深度）。")

    # 测尺寸时该用哪个视场角：对齐后深度图是彩色坐标系 -> 用彩色 FOV
    if aligned:
        _mhfov, _mvfov = COLOR_HFOV, COLOR_VFOV
    else:
        _mhfov, _mvfov = DEPTH_HFOV, DEPTH_VFOV

    # ---- 窗口
    win = "Strawberry Localization (q=quit, s=save, d=depth)"
    if not args.no_show:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, cw, ch)

    print("\n" + "=" * 72)
    print("  开始实时检测（按 q 退出）")
    print("=" * 72)

    frame_id = 0
    fps_t0 = time.time()
    fps_cnt = 0
    fps = 0.0
    show_depth = False

    try:
        while True:
            # ---------- 同步取彩色 + 深度（同一 frameset，时空对齐）
            fr = cam.get_frame_pair(timeout_ms=1500)
            img = fr["color"]
            if fr["depth"] is not None:
                depth = fr["depth"]
            if img is None:
                continue

            # ---------- YOLO 推理
            results = model(img, conf=args.conf, verbose=False)
            boxes = results[0].boxes

            # 解析检测框
            dets = []
            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clss = boxes.cls.cpu().numpy().astype(int)
                for i in range(len(xyxy)):
                    dets.append({
                        "box": xyxy[i],
                        "conf": float(confs[i]),
                        "cls": int(clss[i]),
                    })

            # 配对：每颗草莓 = 一个 strawberry 框 + 框内最佳成熟度
            strawberries = [x for x in dets if x["cls"] == STRAWBERRY_ID]
            ripeness = [x for x in dets if x["cls"] in RIPENESS_IDS]

            items = []
            for sb in strawberries:
                sx1, sy1, sx2, sy2 = sb["box"]
                best = None
                for rp in ripeness:
                    rx1, ry1, rx2, ry2 = rp["box"]
                    ccx, ccy = (rx1 + rx2) / 2, (ry1 + ry2) / 2
                    if sx1 <= ccx <= sx2 and sy1 <= ccy <= sy2:
                        if best is None or rp["conf"] > best["conf"]:
                            best = rp
                items.append({"strawberry": sb, "ripeness": best})

            # ---------- 逐颗定位
            frame_id += 1
            found = []
            for it in items:
                sx1, sy1, sx2, sy2 = it["strawberry"]["box"]
                # 彩色图上的中心
                ccx = int((sx1 + sx2) / 2)
                ccy = int((sy1 + sy2) / 2)

                # D2C 对齐后深度与彩色同尺寸同坐标系，直接用；
                # 未对齐时按视场角比例映射。
                if depth.shape[1] == img.shape[1] and depth.shape[0] == img.shape[0]:
                    dcx = int(np.clip(ccx, 0, depth.shape[1] - 1))
                    dcy = int(np.clip(ccy, 0, depth.shape[0] - 1))
                else:
                    dx1, dy1, dx2, dy2 = color_box_to_depth(
                        (sx1, sy1, sx2, sy2), img.shape, depth.shape)
                    dcx = int(np.clip((dx1 + dx2) / 2, 0, dw - 1))
                    dcy = int(np.clip((dy1 + dy2) / 2, 0, dh - 1))

                z = pick_best_depth(depth, dcx, dcy, window=10)
                p3 = cam.pixel_to_3d(depth, dcx, dcy) if z > 0 else None
                if p3 is None and z > 0:
                    p3 = (0.0, 0.0, float(z))

                dia_w, dia_h = (measure_diameter(depth, dcx, dcy, z,
                                                 hfov=_mhfov, vfov=_mvfov)
                                if z > 0 else (0, 0))

                rcls = it["ripeness"]["cls"] if it["ripeness"] else None
                rconf = it["ripeness"]["conf"] if it["ripeness"] else 0.0

                found.append({
                    "cx": ccx, "cy": ccy,
                    "box": (sx1, sy1, sx2, sy2),
                    "conf": it["strawberry"]["conf"],
                    "ripeness": rcls,
                    "rconf": rconf,
                    "z": z,
                    "p3": p3,
                    "dia": (dia_w, dia_h),
                })

            # ---------- 控制台输出
            if not args.quiet and (frame_id % 10 == 0 or found):
                t = time.strftime("%H:%M:%S")
                if not found:
                    print("[%s] 未检出草莓" % t)
                else:
                    print("[%s] 检出 %d 颗草莓：" % (t, len(found)))
                    for k, f in enumerate(found):
                        rname = (RIPENESS_INFO.get(f["ripeness"],
                                                   ("未定级", (0, 0, 0)))[0]
                                 if f["ripeness"] else "未定级")
                        if f["z"] > 0 and f["p3"]:
                            print("   #%d %-10s 置信%.2f | 距离 %4d mm | "
                                  "坐标 X=%6.0f Y=%6.0f Z=%4.0f mm | 直径 %.0f mm"
                                  % (k + 1, rname, f["conf"], f["z"],
                                     f["p3"][0], f["p3"][1], f["p3"][2],
                                     max(f["dia"])))
                        else:
                            print("   #%d %-10s 置信%.2f | 深度无效（<%dmm 盲区"
                                  " 或表面无回波）"
                                  % (k + 1, rname, f["conf"], DEPTH_VALID_MIN))

            # ---------- 绘制
            vis = img.copy()
            for f in found:
                x1, y1, x2, y2 = [int(v) for v in f["box"]]
                if f["ripeness"]:
                    rname, color = RIPENESS_INFO.get(f["ripeness"],
                                                     ("?", (255, 255, 255)))
                else:
                    rname, color = "?", (200, 200, 200)

                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

                if f["z"] > 0 and f["p3"]:
                    label = "%s Z=%dmm" % (rname.split("/")[0], f["z"])
                else:
                    label = "%s Z=N/A" % rname.split("/")[0]
                cv2.putText(vis, label, (x1, max(14, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # 中心点标记
                cv2.drawMarker(vis, (f["cx"], f["cy"]), color,
                               cv2.MARKER_CROSS, 12, 2)

                # 深度有效时标出 3D 坐标
                if f["z"] > 0 and f["p3"]:
                    cv2.putText(vis, "X=%.0f Y=%.0f" % (f["p3"][0], f["p3"][1]),
                                (x1, min(ch - 4, y2 + 16)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            # FPS 与状态
            fps_cnt += 1
            if time.time() - fps_t0 >= 1.0:
                fps = fps_cnt / (time.time() - fps_t0)
                fps_t0 = time.time()
                fps_cnt = 0
            s = cam.depth_stats(depth)
            cv2.putText(vis, "FPS %.1f | depth %.0f%% | %d found | %s"
                        % (fps, s["density"], len(found),
                           "DEPTH VIEW" if show_depth else "RGB VIEW"),
                        (8, ch - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1)

            if show_depth:
                dvis = cv2.normalize(depth, None, 0, 255,
                                     cv2.NORM_MINMAX).astype(np.uint8)
                dvis = cv2.applyColorMap(dvis, cv2.COLORMAP_JET)
                dvis[depth == 0] = (0, 0, 0)
                # 把两个视图并排
                dh2 = int(dvis.shape[0] * vis.shape[1] / dvis.shape[1])
                dvis = cv2.resize(dvis, (vis.shape[1], dh2))
                vis = np.vstack([vis, dvis])

            if not args.no_show:
                cv2.imshow(win, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key == ord("s"):
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite("shot_%s_rgb.png" % ts, vis)
                    cv2.imwrite("shot_%s_depth.png" % ts,
                                cv2.normalize(depth, None, 0, 65535,
                                              cv2.NORM_MINMAX).astype(np.uint16))
                    print("  已保存 shot_%s_*.png" % ts)
                elif key == ord("d"):
                    show_depth = not show_depth
            else:
                if frame_id % 30 == 0:
                    time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        cam.close()
        if not args.no_show:
            cv2.destroyAllWindows()

    print("\n结束。共处理 %d 帧。" % frame_id)
    return 0


if __name__ == "__main__":
    sys.exit(main())
