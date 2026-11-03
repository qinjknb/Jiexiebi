#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 URDF 生成「碰撞代理」MJCF：每个连杆用一根胶囊包住，用于自碰撞检查。

为什么不用原始网格做碰撞：
  14 个 STL 共 100 多万三角面，MuJoCo 要为每个算凸包，编译要几十秒且碰撞检测很慢。
  自碰撞检查只需要"连杆大致占多大空间"，用包住网格的胶囊（主轴 + 半径）既快又够用。

产物：models/dofbot_pro_collision.xml
  * 每个连杆一根 capsule（contype=1 conaffinity=1，参与碰撞）
  * 相邻连杆（父子）之间不检测——真实关节本来就连在一起
  * 带一个地面，用于检查连杆是否插进桌面

用法：
    python tools/build_collision_model.py
    python tools/build_collision_model.py --radius-scale 0.9
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from dofbot_mj.urdf2mjcf import (  # noqa: E402
    _fmt, _quat_attr, build_children, find_mesh_path, inertia_to_diag,
    parse_urdf, rpy_to_quat, rpy_to_matrix,
)

URDF = ROOT.parent / "src" / "dofbot_pro_description" / "urdf" / "DOFBOT_Pro-V24.urdf"


def stl_vertices(path: Path) -> np.ndarray:
    """读 STL 的全部顶点 (N,3)。"""
    raw = path.read_bytes()
    if len(raw) < 84:
        return np.zeros((0, 3))
    count = struct.unpack("<I", raw[80:84])[0]
    if 84 + count * 50 == len(raw):
        arr = np.frombuffer(raw[84:], dtype=np.uint8).reshape(count, 50)
        return arr[:, 12:48].copy().view("<f4").astype(np.float64).reshape(-1, 3)
    pts = []
    for line in raw.decode("ascii", errors="replace").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            try:
                pts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            except ValueError:
                continue
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 3))


def fit_capsule(points: np.ndarray, radius_scale: float = 1.0):
    """用主轴 + 半径拟合一根胶囊，返回 (p1, p2, radius)。"""
    if len(points) < 8:
        return None
    centroid = points.mean(axis=0)
    centred = points - centroid
    # 主轴 = 协方差最大特征值的特征向量
    _vals, vecs = np.linalg.eigh(centred.T @ centred)
    axis = vecs[:, -1]
    axis /= np.linalg.norm(axis) + 1e-12
    proj = centred @ axis
    lo, hi = float(proj.min()), float(proj.max())
    radial = np.linalg.norm(centred - np.outer(proj, axis), axis=1)
    # 用 92 分位而不是最大值，避免个别凸起把胶囊撑肥
    radius = float(np.percentile(radial, 92)) * radius_scale
    mid = centroid + axis * (lo + hi) / 2.0
    half = max(0.002, (hi - lo) / 2.0)
    p1 = mid - axis * half
    p2 = mid + axis * half
    return p1, p2, max(0.004, radius)


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    p = argparse.ArgumentParser(description="生成胶囊碰撞代理模型")
    p.add_argument("--urdf", default=str(URDF))
    p.add_argument("-o", "--out", default=str(ROOT / "models" / "dofbot_pro_collision.xml"))
    p.add_argument("--radius-scale", type=float, default=1.0)
    p.add_argument("--ground", action="store_true",
                   help="加一个地面，用来查连杆是否插进桌面")
    args = p.parse_args(argv)

    urdf_path = Path(args.urdf)
    if not urdf_path.is_file():
        print(f"[错误] 找不到 URDF: {urdf_path}")
        return 1

    name, links, joints = parse_urdf(urdf_path)
    children, roots = build_children(links, joints)
    urdf_dir = urdf_path.parent

    # 每个连杆：把所有视觉网格顶点合到连杆坐标系，再拟合胶囊
    capsules = {}
    for link_name, link in links.items():
        clouds = []
        for filename, xyz, rpy, _rgba in link.visuals:
            mesh_path = find_mesh_path(filename, [urdf_dir.parent], urdf_dir)
            if mesh_path is None:
                continue
            verts = stl_vertices(mesh_path)
            if len(verts) == 0:
                continue
            rotated = verts @ rpy_to_matrix(*rpy).T
            clouds.append(rotated + np.array(xyz))
        if not clouds:
            continue
        fitted = fit_capsule(np.vstack(clouds), args.radius_scale)
        if fitted is not None:
            capsules[link_name] = fitted

    lines = [f'<mujoco model="{name}_collision">',
             "  <!-- 由 tools/build_collision_model.py 生成：每个连杆一根胶囊，仅用于自碰撞检查 -->",
             '  <compiler angle="radian" autolimits="true" balanceinertia="true"/>',
             '  <option timestep="0.002" gravity="0 0 -9.81"/>',
             '  <size nconmax="400"/>',
             "  <worldbody>"]
    if args.ground:
        lines.append('    <geom name="floor" type="plane" size="1.5 1.5 0.05" pos="0 0 0"'
                     ' contype="1" conaffinity="1"/>')

    def emit(link_name: str, depth: int, transform: str = "", joint_markup: str = ""):
        pad = "  " * depth
        link = links[link_name]
        lines.append(f'{pad}<body name="{link_name}"{transform}>')
        if link.mass > 0.0:
            diag, quat = inertia_to_diag(*link.inertia,
                                         rot=rpy_to_matrix(*link.inertia_rpy))
            lines.append(f'{pad}  <inertial pos="{_fmt(link.com)}" mass="{_fmt(link.mass)}"'
                         f' diaginertia="{_fmt(diag)}"{_quat_attr(quat)}/>')
        if joint_markup:
            lines.append(f"{pad}  {joint_markup}")
        if link_name in capsules:
            p1, p2, radius = capsules[link_name]
            lines.append(
                f'{pad}  <geom name="c_{link_name}" type="capsule"'
                f' fromto="{_fmt(p1)} {_fmt(p2)}" size="{radius:.4f}"'
                f' contype="1" conaffinity="1" rgba="0 0.7 1 0.25"/>')
        for joint in children.get(link_name, []):
            if joint.child not in links:
                continue
            move = f' pos="{_fmt(joint.xyz)}"{_quat_attr(rpy_to_quat(*joint.rpy))}'
            if joint.type == "fixed":
                emit(joint.child, depth + 1, move, "")
            else:
                kind = "hinge" if joint.type in ("revolute", "continuous") else "slide"
                if joint.type == "continuous":
                    markup = (f'<joint name="{joint.name}" type="{kind}"'
                              f' axis="{_fmt(joint.axis)}" limited="false"/>')
                else:
                    markup = (f'<joint name="{joint.name}" type="{kind}"'
                              f' axis="{_fmt(joint.axis)}"'
                              f' range="{_fmt(joint.lower)} {_fmt(joint.upper)}" limited="true"/>')
                emit(joint.child, depth + 1, move, markup)
        lines.append(f"{pad}</body>")

    for root_link in roots:
        if root_link in links:
            emit(root_link, 2)
    lines.append("  </worldbody>")
    lines.append("</mujoco>")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"[OK] {out}")
    print(f"     连杆胶囊 {len(capsules)} 个：")
    for link_name, (p1, p2, radius) in capsules.items():
        length = float(np.linalg.norm(p2 - p1)) + 2 * radius
        print(f"       {link_name:<20} 长度 {length*100:5.1f} cm  半径 {radius*100:4.1f} cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
