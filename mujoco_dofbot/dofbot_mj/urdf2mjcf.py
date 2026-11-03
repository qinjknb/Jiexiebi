#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 urdf2mjcf.py —— URDF → MJCF 转换器（DOFBOT Pro 用，通用可实现）
============================================================================
 为什么不用 MuJoCo 自带的 URDF 导入：
   * 自带导入不会生成执行器（actuator），位置控制无从下手；
   * package:// 形式的网格路径在 Windows 上必然失效；
   * URDF 的 mimic 关节不被识别，夹爪会散架。
 所以这里自己做一遍确定性的转换，产物是纯 MJCF 文本，随时可手改。

 转换规则：
   链接 → <body>（嵌套），关节 origin → body 的 pos/quat
   revolute/continuous 关节 → hinge，limit → range
   视觉网格 → mesh geom 资源；collision 默认丢弃（不做碰撞，见 --collision）
   惯性 → <inertial>（惯性张量先特征分解成 diaginertia + quat，避免
            fullinertia 字段顺序歧义）
   mimic 关节 → 保留为 hinge，并写入同名 .json sidecar，播放器按
           q_mimic = multiplier × q_driver + offset 驱动
   非 fixed 关节 → 位置执行器（physics 模式用）

 用法：
   python -m dofbot_mj.urdf2mjcf src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf \
       -o models/dofbot_pro.xml
============================================================================
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


# --------------------------------------------------------------------------
# 基础数学
# --------------------------------------------------------------------------
def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """URDF 的固定轴 roll-pitch-yaw（RPY，等价 XYZ）→ 四元数 (w, x, y, z)。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def matrix_to_quat(m: np.ndarray) -> tuple[float, float, float, float]:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / norm, x / norm, y / norm, z / norm)


def inertia_to_diag(ixx, iyy, izz, ixy, ixz, iyz, rot: np.ndarray):
    """完整惯性张量 → MuJoCo 的 diaginertia + 主轴旋转四元数。

    URDF 的 inertial rpy 非零时，张量先旋转到 link 系；再特征分解取主轴。
    """
    tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)
    tensor = rot @ tensor @ rot.T
    values, vectors = np.linalg.eigh(tensor)
    order = np.argsort(values)[::-1]
    values, vectors = values[order], vectors[:, order]
    if np.linalg.det(vectors) < 0:  # 保证右手系，四元数才唯一
        vectors[:, 2] *= -1.0
    values = np.maximum(values, 1e-12)  # 退化惯量兜底，否则 MuJoCo 编译报错
    return tuple(float(v) for v in values), matrix_to_quat(vectors)


# --------------------------------------------------------------------------
# STL 包围盒（判断网格单位是米还是毫米）
# --------------------------------------------------------------------------
def stl_bounds(path: Path):
    """返回 (min, max, 三角面数)；读失败返回 None。"""
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    if len(raw) < 84:
        return None
    count = struct.unpack("<I", raw[80:84])[0]
    if 84 + count * 50 == len(raw):  # 二进制 STL
        arr = np.frombuffer(raw[84:], dtype=np.uint8).reshape(count, 50)
        verts = arr[:, 12:48].copy().view("<f4").astype(np.float64).reshape(-1, 3)
    else:  # ASCII STL
        pts = []
        for line in raw.decode("ascii", errors="replace").splitlines():
            parts = line.strip().split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                try:
                    pts.append([float(parts[1]), float(parts[2]), float(parts[3])])
                except ValueError:
                    continue
        if not pts:
            return None
        verts = np.array(pts, dtype=np.float64)
    if verts.size == 0:
        return None
    return verts.min(axis=0), verts.max(axis=0), len(verts) // 3


# MuJoCo 单个 STL 的硬上限是 20 万面。超过就按面数切成多份——
# 切分不改变任何三角形，形状零损失，只是多几个 geom。
MAX_STL_FACES = 190000


def split_stl(path: Path, out_dir: Path, max_faces: int = MAX_STL_FACES):
    """把超限的 STL 按三角面顺序切成若干份二进制 STL，返回各份路径。

    二进制 STL 每条记录固定 50 字节（法线 12 + 三顶点 36 + 属性 2），所以可以
    按字节切片，原样保留法线与顶点，不做任何几何运算。
    """
    raw = path.read_bytes()
    if len(raw) < 84:
        return [path]
    count = struct.unpack("<I", raw[80:84])[0]
    if 84 + count * 50 != len(raw) or count <= max_faces:
        return [path]  # 非二进制或未超限，原样使用
    pieces = (count + max_faces - 1) // max_faces
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    body = raw[84:]
    for index in range(pieces):
        start = index * max_faces
        end = min(count, start + max_faces)
        chunk = body[start * 50:end * 50]
        header = f"split part {index + 1}/{pieces} of {path.name}".encode("ascii")[:80]
        dst = out_dir / f"{path.stem}_p{index}.STL"
        dst.write_bytes(header.ljust(80, b" ") + struct.pack("<I", end - start) + chunk)
        written.append(dst)
    return written


# --------------------------------------------------------------------------
# URDF 解析
# --------------------------------------------------------------------------
@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: tuple[float, float, float] = (0.0, 0.0, 0.0)
    inertia: tuple[float, ...] = (0.0,) * 6
    inertia_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    visuals: list = field(default_factory=list)
    collisions: list = field(default_factory=list)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float
    effort: float
    velocity: float
    mimic: tuple | None = None


def _floats(text, default):
    if not text:
        return default
    try:
        return tuple(float(v) for v in text.split())
    except ValueError:
        return default


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    name = root.get("name", "robot")
    links: dict[str, Link] = {}
    for el in root.findall("link"):
        link = Link(el.get("name", ""))
        inertial = el.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            link.mass = float(mass.get("value", 0.0)) if mass is not None else 0.0
            org = inertial.find("origin")
            if org is not None:
                link.com = _floats(org.get("xyz"), (0.0, 0.0, 0.0))
                link.inertia_rpy = _floats(org.get("rpy"), (0.0, 0.0, 0.0))
            ine = inertial.find("inertia")
            if ine is not None:
                link.inertia = (
                    float(ine.get("ixx", 0.0)), float(ine.get("iyy", 0.0)),
                    float(ine.get("izz", 0.0)), float(ine.get("ixy", 0.0)),
                    float(ine.get("ixz", 0.0)), float(ine.get("iyz", 0.0)),
                )
        for kind, bucket in (("visual", link.visuals), ("collision", link.collisions)):
            for vis in el.findall(kind):
                geo = vis.find("geometry")
                mesh = geo.find("mesh") if geo is not None else None
                if mesh is None:
                    continue
                org = vis.find("origin")
                xyz = _floats(org.get("xyz"), (0.0, 0.0, 0.0)) if org is not None else (0.0, 0.0, 0.0)
                rpy = _floats(org.get("rpy"), (0.0, 0.0, 0.0)) if org is not None else (0.0, 0.0, 0.0)
                rgba = None
                mat = vis.find("material")
                if mat is not None:
                    col = mat.find("color")
                    if col is not None:
                        rgba = _floats(col.get("rgba"), None)
                bucket.append((mesh.get("filename", ""), xyz, rpy, rgba))
        links[el.get("name", "")] = link

    joints: list[Joint] = []
    for el in root.findall("joint"):
        jtype = el.get("type", "fixed")
        org, axis, lim, mimic = el.find("origin"), el.find("axis"), el.find("limit"), el.find("mimic")
        lower, upper, effort, velocity = -math.pi, math.pi, 100.0, 1.0
        if lim is not None:
            if lim.get("lower") is not None:
                lower = float(lim.get("lower"))
            if lim.get("upper") is not None:
                upper = float(lim.get("upper"))
            effort = float(lim.get("effort", 100.0))
            velocity = float(lim.get("velocity", 1.0))
        joints.append(
            Joint(
                name=el.get("name", ""),
                type=jtype,
                parent=el.find("parent").get("link"),
                child=el.find("child").get("link"),
                xyz=_floats(org.get("xyz"), (0.0, 0.0, 0.0)) if org is not None else (0.0, 0.0, 0.0),
                rpy=_floats(org.get("rpy"), (0.0, 0.0, 0.0)) if org is not None else (0.0, 0.0, 0.0),
                axis=_floats(axis.get("xyz"), (1.0, 0.0, 0.0)) if axis is not None else (1.0, 0.0, 0.0),
                lower=lower, upper=upper, effort=effort, velocity=velocity,
                mimic=(
                    (mimic.get("joint"), float(mimic.get("multiplier", 1.0)),
                     float(mimic.get("offset", 0.0)))
                    if mimic is not None else None
                ),
            )
        )
    return name, links, joints


def find_mesh_path(filename: str, mesh_roots: list[Path], urdf_dir: Path):
    cleaned = filename.replace("\\", "/")
    if cleaned.startswith("file://"):
        cleaned = cleaned[7:]
    if cleaned.startswith("package://"):
        cleaned = cleaned[len("package://"):]
        cleaned = cleaned.split("/", 1)[1] if "/" in cleaned else cleaned
    candidates = [urdf_dir / cleaned, urdf_dir / Path(cleaned).name]
    for root in mesh_roots:
        candidates += [root / cleaned, root / Path(cleaned).name,
                       root / "meshes" / Path(cleaned).name]
    for cand in candidates:
        if cand.is_file():
            return cand.resolve()
    return None


# --------------------------------------------------------------------------
# MJCF 生成
# --------------------------------------------------------------------------
def _fmt(values) -> str:
    if isinstance(values, (int, float)):
        return f"{values:.6g}"
    return " ".join(f"{v:.6g}" for v in values)


def _quat_attr(quat) -> str:
    return "" if quat == (1.0, 0.0, 0.0, 0.0) else f' quat="{_fmt(quat)}"'


def _joint_element(joint: Joint) -> str:
    kind = "hinge" if joint.type in ("revolute", "continuous") else "slide"
    if joint.type == "continuous":
        return f'<joint name="{joint.name}" type="{kind}" axis="{_fmt(joint.axis)}" limited="false"/>'
    return (
        f'<joint name="{joint.name}" type="{kind}" axis="{_fmt(joint.axis)}"'
        f' range="{_fmt(joint.lower)} {_fmt(joint.upper)}" limited="true"/>'
    )


def build_children(links, joints):
    children = {name: [] for name in links}
    child_links = set()
    for joint in joints:
        if joint.parent in children:
            children[joint.parent].append(joint)
        child_links.add(joint.child)
    roots = [name for name in links if name not in child_links]
    return children, roots


def convert(
    urdf_path: Path,
    out_path: Path,
    mesh_roots: list[Path],
    meshdir: str | None = None,
    copy_meshes: Path | None = None,
    collision: bool = False,
    camera_distance: float = 0.75,
    ground: bool = True,
) -> dict:
    robot_name, links, joints = parse_urdf(urdf_path)
    children, roots = build_children(links, joints)
    urdf_dir = urdf_path.parent

    # ---- 网格 ----
    mesh_parts: dict[str, list[tuple[str, Path]]] = {}
    missing: list[str] = []
    sizes: list[float] = []
    split_names: list[str] = []
    mesh_min_z: dict[str, float] = {}
    for link in links.values():
        for filename, _xyz, _rpy, _rgba in link.visuals:
            if not filename:
                continue
            key = Path(filename.replace("\\", "/")).name
            stem = Path(key).stem
            if stem in mesh_parts:
                continue
            resolved = find_mesh_path(filename, mesh_roots, urdf_dir)
            if resolved is None:
                missing.append(filename)
                continue
            info = stl_bounds(resolved)
            if info is not None:
                size = float(np.max(info[1] - info[0]))
                if size > 0:
                    sizes.append(size)
                faces = info[2]
                mesh_min_z[stem] = float(info[0][2])
            else:
                faces = 0
            if faces > MAX_STL_FACES:
                pieces = split_stl(resolved, out_path.parent / "mesh_parts", MAX_STL_FACES)
                mesh_parts[stem] = [(f"{stem}_p{i}", p) for i, p in enumerate(pieces)]
                split_names.append(f"{stem}({faces}面→{len(pieces)}份)")
            else:
                mesh_parts[stem] = [(stem, resolved)]

    # 单位自检：DOFBOT 连杆在 0.09 m 量级。网格明显大于 2 m 基本就是毫米/厘米。
    mesh_scale = 1.0
    if sizes:
        median = float(np.median(sizes))
        if median > 20.0:
            mesh_scale = 0.001
        elif median > 2.0:
            mesh_scale = 0.01

    if copy_meshes is not None:
        copy_meshes.mkdir(parents=True, exist_ok=True)
        for stem, parts in list(mesh_parts.items()):
            copied = []
            for asset, src in parts:
                dst = copy_meshes / (asset + src.suffix)
                if not dst.exists() or dst.stat().st_size != src.stat().st_size:
                    dst.write_bytes(src.read_bytes())
                copied.append((asset, dst))
            mesh_parts[stem] = copied

    materials: dict[str, int] = {}
    for link in links.values():
        for _fn, _xyz, _rpy, rgba in link.visuals:
            if rgba:
                materials.setdefault(_fmt(rgba), len(materials))

    actuated = [j for j in joints if j.type != "fixed"]

    lines: list[str] = []
    add = lines.append
    add(f'<mujoco model="{robot_name}">')
    add(f"  <!-- 由 urdf2mjcf.py 从 {urdf_path.name} 生成；可直接手改 -->")
    add('  <compiler angle="radian" autolimits="true" balanceinertia="true"'
        ' discardvisual="false"/>')
    add('  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>')
    add('  <size nconmax="120" njmax="400"/>')
    add("  <visual>")
    add('    <global offwidth="1280" offheight="720" fovy="45"/>')
    add('    <headlight ambient="0.35 0.35 0.38" diffuse="0.65 0.65 0.65" specular="0.1 0.1 0.1"/>')
    add('    <map znear="0.01" zfar="20" shadowclip="0.5"/>')
    add('    <quality shadowsize="4096" offsamples="4"/>')
    add("  </visual>")
    add("  <asset>")
    for stem, parts in sorted(mesh_parts.items()):
        for asset, path in parts:
            if copy_meshes is not None:
                rel = path.name
            elif meshdir:
                rel = f"{meshdir.rstrip('/')}/{path.name}"
            else:
                rel = os.path.relpath(path, out_path.parent).replace("\\", "/")
            scale = ("" if mesh_scale == 1.0
                     else f' scale="{mesh_scale} {mesh_scale} {mesh_scale}"')
            add(f'    <mesh name="{asset}" file="{rel}"{scale}/>')
    for rgba, idx in materials.items():
        add(f'    <material name="mat{idx}" rgba="{rgba}"/>')
    add('    <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.25 0.29 0.34"'
        ' rgb2="0.06 0.07 0.09" width="512" height="512"/>')
    add('    <texture name="grid" type="2d" builtin="checker" rgb1="0.16 0.17 0.19"'
        ' rgb2="0.11 0.12 0.13" width="512" height="512"/>')
    add('    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>')
    add("  </asset>")
    add("  <default>")
    add('    <joint damping="0.15" armature="0.004" frictionloss="0.02"/>')
    add('    <geom type="mesh" contype="0" conaffinity="0" group="2"/>')
    add('    <default class="collide">')
    add('      <geom type="mesh" contype="1" conaffinity="1" group="3" rgba="0 0.6 1 0.25"/>')
    add("    </default>")
    add('    <default class="motor">')
    add('      <position kp="80" kv="5" forcerange="-100 100"/>')
    add("    </default>")
    add("  </default>")
    add("  <worldbody>")
    add('    <light pos="0.6 -0.6 1.4" dir="-0.4 0.4 -1" directional="true"'
        ' diffuse="0.5 0.5 0.5" specular="0.2 0.2 0.2" castshadow="true"/>')
    add('    <geom name="floor" type="plane" size="2 2 0.05" pos="0 0 0"'
        ' material="grid" contype="1" conaffinity="1" group="1"/>')
    add(f'    <camera name="view" pos="{camera_distance:.3f} {-camera_distance:.3f} '
        f'{camera_distance * 0.8:.3f}" xyaxes="0.7 0.7 0 -0.3 0.3 1"/>')

    def emit_body(link_name: str, depth: int, transform: str = "", joint_markup: str = ""):
        pad = "  " * depth
        link = links[link_name]
        add(f'{pad}<body name="{link_name}"{transform}>')
        if link.mass > 0.0:
            diag, quat = inertia_to_diag(*link.inertia, rot=rpy_to_matrix(*link.inertia_rpy))
            add(f'{pad}  <inertial pos="{_fmt(link.com)}" mass="{_fmt(link.mass)}"'
                f' diaginertia="{_fmt(diag)}"{_quat_attr(quat)}/>')
        if joint_markup:
            add(f"{pad}  {joint_markup}")
        for filename, xyz, rpy, rgba in link.visuals:
            stem = Path(filename.replace("\\", "/")).stem
            if stem not in mesh_parts:
                continue
            mat = f' material="mat{materials[_fmt(rgba)]}"' if rgba else ""
            pos_attr = f' pos="{_fmt(xyz)}"' if any(xyz) else ""
            quat_attr = _quat_attr(rpy_to_quat(*rpy))
            for asset, _path in mesh_parts[stem]:
                add(f'{pad}  <geom mesh="{asset}"{pos_attr}{quat_attr}{mat}/>')
        if collision:
            for filename, xyz, rpy, _rgba in link.collisions:
                stem = Path(filename.replace("\\", "/")).stem
                if stem not in mesh_parts:
                    continue
                pos_attr = f' pos="{_fmt(xyz)}"' if any(xyz) else ""
                quat_attr = _quat_attr(rpy_to_quat(*rpy))
                for asset, _path in mesh_parts[stem]:
                    add(f'{pad}  <geom class="collide" mesh="{asset}"{pos_attr}{quat_attr}/>')
        for joint in children.get(link_name, []):
            if joint.child not in links:
                continue
            move = f' pos="{_fmt(joint.xyz)}"{_quat_attr(rpy_to_quat(*joint.rpy))}'
            markup = "" if joint.type == "fixed" else _joint_element(joint)
            emit_body(joint.child, depth + 1, transform=move, joint_markup=markup)
        add(f"{pad}</body>")

    for root_link in roots:
        if root_link not in links:
            continue
        # URDF 的 root 原点常在地面附近，底盘网格会陷进地板 15 mm 左右；
        # 按网格最低点把根链接抬起来，视觉上正好坐在地面上。
        lift = 0.0
        if ground:
            lows = [
                xyz[2] + mesh_min_z[Path(fn.replace("\\", "/")).stem] * mesh_scale
                for fn, xyz, _rpy, _rgba in links[root_link].visuals
                if Path(fn.replace("\\", "/")).stem in mesh_min_z
            ]
            lift = -min(lows) if lows else 0.0
        emit_body(root_link, 2, transform=f' pos="0 0 {_fmt(lift)}"' if abs(lift) > 1e-9 else "")
    add("  </worldbody>")

    add("  <actuator>")
    for joint in actuated:
        add(f'    <position class="motor" name="act_{joint.name}" joint="{joint.name}"'
            f' ctrlrange="{_fmt(joint.lower)} {_fmt(joint.upper)}"/>')
    add("  </actuator>")
    add("</mujoco>")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    mimic = {
        j.name: {"driver": j.mimic[0], "multiplier": j.mimic[1], "offset": j.mimic[2]}
        for j in joints if j.mimic is not None
    }
    sidecar = {
        "robot": robot_name,
        "source_urdf": str(urdf_path),
        "mjcf": out_path.name,
        "mesh_scale": mesh_scale,
        "missing_meshes": missing,
        "split_meshes": split_names,
        "joints": [
            {
                "name": j.name, "type": j.type, "parent": j.parent, "child": j.child,
                "lower": j.lower, "upper": j.upper, "axis": list(j.axis),
                "mimic": (
                    {"driver": j.mimic[0], "multiplier": j.mimic[1], "offset": j.mimic[2]}
                    if j.mimic else None
                ),
            }
            for j in joints
        ],
        "actuated": [j.name for j in actuated],
        "arm_joints": [j.name for j in actuated if not j.mimic],
        "mimic": mimic,
    }
    out_path.with_suffix(".json").write_text(
        json.dumps(sidecar, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return sidecar


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="URDF → MJCF（DOFBOT Pro）")
    parser.add_argument("urdf", help="URDF 文件路径")
    parser.add_argument("-o", "--out", default="models/dofbot_pro.xml", help="输出的 MJCF")
    parser.add_argument("--mesh-root", action="append", default=[], help="网格搜索目录，可重复")
    parser.add_argument("--copy-meshes", default="", help="把网格拷到该目录（自包含）")
    parser.add_argument("--meshdir", default="", help="MJCF 里的 meshdir 前缀")
    parser.add_argument("--collision", action="store_true", help="额外生成碰撞网格（很慢）")
    parser.add_argument("--camera-distance", type=float, default=0.75)
    parser.add_argument("--no-ground", action="store_true", help="不把根链接抬到地面")
    args = parser.parse_args(argv)

    urdf_path = Path(args.urdf).expanduser().resolve()
    if not urdf_path.is_file():
        print(f"找不到 URDF: {urdf_path}", file=sys.stderr)
        return 2
    roots = [Path(r).expanduser().resolve() for r in args.mesh_root] or [urdf_path.parent.parent]
    out_path = Path(args.out).expanduser()
    if not out_path.is_absolute():
        out_path = Path.cwd() / out_path

    sidecar = convert(
        urdf_path, out_path, mesh_roots=roots,
        meshdir=args.meshdir or None,
        copy_meshes=Path(args.copy_meshes).resolve() if args.copy_meshes else None,
        collision=args.collision, camera_distance=args.camera_distance,
        ground=not args.no_ground,
    )
    print(f"[OK] {out_path}")
    print(f"     机器人: {sidecar['robot']}")
    print(f"     关节 {len(sidecar['joints'])} 个，执行器 {len(sidecar['actuated'])} 个")
    print(f"     mimic: {', '.join(sidecar['mimic']) or '无'}")
    print(f"     网格缩放: {sidecar['mesh_scale']}")
    if sidecar["missing_meshes"]:
        print(f"[警告] 缺 {len(sidecar['missing_meshes'])} 个网格")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
