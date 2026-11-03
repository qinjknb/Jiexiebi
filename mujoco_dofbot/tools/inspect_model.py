#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 MJCF 模型：编译、列出关节、渲染预览图。

用法：
    python tools/inspect_model.py models/dofbot_pro.xml
    python tools/inspect_model.py models/dofbot_pro.xml --render out.png --pose up
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco  # noqa: E402

from dofbot_mj.poses import resolve_pose  # noqa: E402


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", nargs="?", default="models/dofbot_pro.xml")
    parser.add_argument("--render", default="", help="输出预览 PNG")
    parser.add_argument("--pose", default="up")
    parser.add_argument("--srdf", default="../src/My_JXB_moveit/config/DOFBOT_Pro-V24.srdf")
    args = parser.parse_args(argv)

    path = Path(args.model)
    if not path.is_file():
        print(f"找不到模型: {path}")
        return 2

    started = time.time()
    model = mujoco.MjModel.from_xml_path(str(path))
    compile_s = time.time() - started
    data = mujoco.MjData(model)

    triangles = int(model.mesh_facenum.sum()) if model.mesh_facenum.size else 0
    print(f"模型: {path}")
    print(f"  编译耗时 {compile_s:.2f}s   nq={model.nq}  nbody={model.nbody}  ngeom={model.ngeom}")
    print(f"  网格三角形 {triangles:,}   总质量 {model.body_mass.sum():.3f} kg")
    print(f"  关节 {model.njnt} 个 / 执行器 {model.nu} 个")
    print()
    print(f"  {'关节名':<16}{'类型':<8}{'下限':>9}{'上限':>9}{'qpos索引':>9}")
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        jtype = model.jnt_type[jid]
        kind = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(int(jtype), "?")
        limited = bool(model.jnt_limited[jid])
        lo, hi = (model.jnt_range[jid] if limited else (np.nan, np.nan))
        print(f"  {name:<16}{kind:<8}{lo:>9.4f}{hi:>9.4f}{int(model.jnt_qposadr[jid]):>9}")

    if args.render:
        pose_name, q = resolve_pose(args.srdf, args.pose, model)
        if q is None:
            print(f"\n[警告] 姿态 '{args.pose}' 解析失败，用零位渲染")
            data.qpos[:] = 0.0
        else:
            data.qpos[:] = 0.0
            for jid, value in q.items():
                data.qpos[model.jnt_qposadr[jid]] = value
            print(f"\n渲染姿态 '{pose_name}': " + ", ".join(
                f"{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)}={v:.3f}"
                for j, v in q.items()))
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 720, 960)
        renderer.update_scene(data, camera="view")
        image = renderer.render()
        out = Path(args.render)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            from PIL import Image

            Image.fromarray(image).save(out)
        except ImportError:
            import struct
            import zlib

            def write_png(filename, rgb):
                h, w, _ = rgb.shape
                raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

                def chunk(tag, payload):
                    body = tag + payload
                    return (struct.pack(">I", len(payload)) + body
                            + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

                png = (b"\x89PNG\r\n\x1a\n"
                       + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                       + chunk(b"IDAT", zlib.compress(raw, 6))
                       + chunk(b"IEND", b""))
                Path(filename).write_bytes(png)

            write_png(out, image)
        print(f"预览图: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
