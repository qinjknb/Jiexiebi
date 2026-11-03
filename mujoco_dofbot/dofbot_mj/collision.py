#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自碰撞检查：给定关节角，判断机械臂各连杆之间（以及和桌面）有没有撞上。

用 models/dofbot_pro_collision.xml（每个连杆一根胶囊代理）做快速几何检查，
可以在**下发任何运动指令之前**就发现会撞的姿态。

被忽略的接触：
  * 同一个 body 内部的接触；
  * 父子连杆之间的接触（它们本来就连在一起）；
  * base_link 与地面（机器人就是立在地上的）。
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

DEFAULT_MODEL = Path(__file__).resolve().parent.parent / "models" / "dofbot_pro_collision.xml"

# 夹爪是个紧凑组件：几根手指的胶囊必然互相重叠，它们之间的接触没有意义。
# 手指与手臂/相机之间的接触仍然检查。
FINGER_LINKS = {"rlink1", "rlink2", "rlink3", "llink1", "llink2", "llink3",
                "Gripping_point_Link"}


class CollisionChecker:
    def __init__(self, model_path: Path | str = DEFAULT_MODEL):
        self.path = Path(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.data = mujoco.MjData(self.model)
        self.qadr = {}
        for jid in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if name:
                self.qadr[name] = int(self.model.jnt_qposadr[jid])
        # 父子关系（用于跳过相邻连杆的接触）
        self.parent_body = {}
        for bid in range(1, self.model.nbody):
            self.parent_body[bid] = int(self.model.body_parentid[bid])
        self.body_name = {
            bid: (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body{bid}")
            for bid in range(self.model.nbody)
        }

    def _adjacent(self, a: int, b: int) -> bool:
        return (self.parent_body.get(a) == b) or (self.parent_body.get(b) == a)

    def _chain_distance(self, a: int, b: int, limit: int = 8) -> int:
        """两个 body 在运动链上隔了几节（沿父子链走）。"""
        chain_a = []
        node = a
        for _ in range(limit):
            chain_a.append(node)
            node = self.parent_body.get(node, 0)
            if node == 0:
                break
        node = b
        for dist_b in range(limit):
            if node in chain_a:
                return dist_b + chain_a.index(node)
            node = self.parent_body.get(node, 0)
            if node == 0:
                break
        return 99

    def check(self, joint_rad: dict, penetration_tol: float = 0.002,
              min_chain_gap: int = 3):
        """返回 [(连杆A, 连杆B, 穿透深度mm)]，空列表表示无碰撞。

        min_chain_gap：运动链上相隔少于这么多节的连杆之间的接触不算碰撞。
        相邻（1 节）在关节处必然贴着，隔一节（2 节）也常常物理重叠；
        实测取 3 之后，零位/扫描位不再误报，而真正折过头的姿态仍能检出。
        """
        self.data.qpos[:] = 0.0
        for name, value in joint_rad.items():
            adr = self.qadr.get(name)
            if adr is not None:
                self.data.qpos[adr] = float(value)
        mujoco.mj_forward(self.model, self.data)

        results = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if contact.dist > penetration_tol:
                continue
            b1 = int(self.model.geom_bodyid[contact.geom1])
            b2 = int(self.model.geom_bodyid[contact.geom2])
            if b1 == b2:
                continue
            if self._chain_distance(b1, b2) < min_chain_gap:
                continue
            name1, name2 = self.body_name.get(b1, "?"), self.body_name.get(b2, "?")
            if name1 in FINGER_LINKS and name2 in FINGER_LINKS:
                continue
            geom1 = self.model.geom(contact.geom1).name or ""
            geom2 = self.model.geom(contact.geom2).name or ""
            if "base_link" in (name1, name2) and (
                "floor" in (geom1, geom2) or "tabletop" in (geom1, geom2)
            ):
                continue
            results.append((name1, name2, float(-contact.dist) * 1000.0))
        results.sort(key=lambda item: -item[2])
        return results

    def worst(self, joint_rad: dict):
        hits = self.check(joint_rad)
        return hits[0] if hits else None


def describe(hits) -> str:
    if not hits:
        return "无自碰撞"
    return "，".join(f"{a}←→{b}({d:.1f}mm)" for a, b, d in hits[:4])
