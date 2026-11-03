#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 模型封装：关节索引、mimic 展开、两种控制模式的状态注入。"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


class RobotModel:
    """把 MJCF 里的关节索引、mimic 关系和执行器索引一次算好。"""

    def __init__(self, mjcf_path: Path, mimic: dict | None = None):
        self.path = Path(mjcf_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.data = mujoco.MjData(self.model)

        self.joint_ids: dict[str, int] = {}
        self.qadr: dict[str, int] = {}
        for jid in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if not name:
                continue
            self.joint_ids[name] = jid
            self.qadr[name] = int(self.model.jnt_qposadr[jid])

        self.actuator_ids: dict[str, int] = {}
        for aid in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            if not name:
                continue
            self.actuator_ids[name] = aid

        # mimic 关系：q_mimic = multiplier × q_driver + offset
        self.mimic: dict[str, tuple[str, float, float]] = {}
        for name, spec in (mimic or {}).items():
            if name in self.qadr:
                self.mimic[name] = (
                    spec["driver"], float(spec["multiplier"]), float(spec["offset"]),
                )

        # 驱动关节 = 除 mimic 外的全部关节，按模型顺序
        self.driver_joints = [
            name for name in self.joint_ids if name not in self.mimic
        ]
        self.dim = len(self.driver_joints)

    # ------------------------------ 基础查询 ------------------------------
    def limits(self, name: str) -> tuple[float, float]:
        jid = self.joint_ids[name]
        if not bool(self.model.jnt_limited[jid]):
            return (-float("inf"), float("inf"))
        lo, hi = self.model.jnt_range[jid]
        return float(lo), float(hi)

    def clamp(self, values: dict) -> dict:
        out = {}
        for name, value in values.items():
            lo, hi = self.limits(name)
            out[name] = float(min(max(value, lo), hi))
        return out

    def expand(self, values: dict) -> dict:
        """把 mimic 关节按驱动关节算出来。"""
        out = dict(values)
        for name, (driver, mult, offset) in self.mimic.items():
            if driver in out:
                out[name] = mult * out[driver] + offset
        return out

    def vector(self, values: dict) -> np.ndarray:
        return np.array([values[name] for name in self.driver_joints], dtype=float)

    def as_dict(self, vector: np.ndarray) -> dict:
        return {name: float(v) for name, v in zip(self.driver_joints, vector)}

    def measure(self, values: dict) -> dict:
        """以模型当前状态为准，读回驱动关节的实际角度。"""
        out = {}
        for name in self.driver_joints:
            out[name] = float(self.data.qpos[self.qadr[name]])
        return out

    # ------------------------------ 状态注入 ------------------------------
    def apply_kinematic(self, values: dict):
        """直接把关节摆到目标角并刷新正运动学（等价 mock_components）。"""
        expanded = self.expand(values)
        for name, value in expanded.items():
            self.data.qpos[self.qadr[name]] = value
        mujoco.mj_forward(self.model, self.data)

    def apply_physics(self, values: dict):
        """给位置执行器下发目标，由物理积分去跟随。"""
        expanded = self.expand(values)
        for name, value in expanded.items():
            act = self.actuator_ids.get(f"act_{name}")
            if act is not None:
                self.data.ctrl[act] = value

    def step(self, seconds: float):
        """推进物理时间 seconds（kinematic 模式下只刷新一次正运动学）。"""
        if seconds <= 0:
            return
        n = max(1, int(round(seconds / self.model.opt.timestep)))
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def reset_to(self, values: dict):
        expanded = self.expand(values)
        self.data.qpos[:] = 0.0
        for name, value in expanded.items():
            self.data.qpos[self.qadr[name]] = value
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------ 诊断 ------------------------------
    def summary(self) -> str:
        triangles = int(self.model.mesh_facenum.sum()) if self.model.mesh_facenum.size else 0
        return (f"关节 {self.model.njnt} 个（驱动 {self.dim}，mimic {len(self.mimic)}），"
                f"执行器 {self.model.nu} 个，网格 {triangles:,} 三角形，"
                f"总质量 {self.model.body_mass.sum():.3f} kg")
