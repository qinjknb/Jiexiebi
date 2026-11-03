#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 MoveIt 的 SRDF 读命名姿态（group_state），供 MuJoCo 播放器复用。

这样仿真里的 `init` / `up` / `close` / `open` 与 Ubuntu 上 MoveIt 用的是同一份
定义，不会出现两边角度对不上。
"""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET


def parse_srdf(path: Path) -> dict:
    """返回 {'states': {姿态名: {关节名: 弧度}}, 'groups': {组名: 关节名列表}}。"""
    root = ET.parse(path).getroot()
    states: dict[str, dict[str, float]] = {}
    groups: dict[str, list[str]] = {}
    for group in root.findall("group"):
        name = group.get("name", "")
        joint_names = [j.get("name") for j in group.findall("joint") if j.get("name")]
        groups[name] = joint_names
    for state in root.findall("group_state"):
        name = state.get("name", "")
        values = {}
        for joint in state.findall("joint"):
            jname = joint.get("name")
            if not jname:
                continue
            try:
                values[jname] = float(joint.get("value", "0"))
            except ValueError:
                continue
        states[name] = values
    return {"states": states, "groups": groups}


def resolve_pose(srdf_path, pose_name: str, model):
    """把姿态名解析成 {关节 id: 弧度}；找不到返回 (None, None)。"""
    import mujoco

    srdf = Path(srdf_path)
    if not srdf.is_file():
        return None, None
    try:
        parsed = parse_srdf(srdf)
    except ET.ParseError:
        return None, None
    values = parsed["states"].get(pose_name)
    if values is None:
        return None, None
    result = {}
    for joint_name, angle in values.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            continue
        result[jid] = angle
    return pose_name, result


def limit_report(model) -> list[dict]:
    """列出每个受限关节的名与范围，便于做越限检查。"""
    import mujoco

    rows = []
    for jid in range(model.njnt):
        if not bool(model.jnt_limited[jid]):
            continue
        rows.append({
            "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid),
            "lower": float(model.jnt_range[jid][0]),
            "upper": float(model.jnt_range[jid][1]),
            "id": jid,
        })
    return rows


def clamp_to_limits(model, joint_id: int, value: float) -> float:
    if not bool(model.jnt_limited[joint_id]):
        return value
    lo, hi = model.jnt_range[joint_id]
    return min(max(value, float(lo)), float(hi))
