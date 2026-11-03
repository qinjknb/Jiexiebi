#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节空间轨迹生成。

对应 Ubuntu 上的 MoveIt 规划：MoveIt 给的是平滑的关节轨迹，这里用余弦缓入缓出
（两端速度为零）复现同样的手感；回到起点的回程直接倒放同一段轨迹，与
two_joint_cycle.cpp 的「把指令轨迹反向播放」是同一个思路。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def cosine_segment(q0: np.ndarray, q1: np.ndarray, samples: int) -> np.ndarray:
    """余弦缓入缓出，返回 (samples, dim)，含起点与终点，两端速度为零。"""
    samples = max(2, int(samples))
    u = np.linspace(0.0, 1.0, samples)
    blend = 0.5 * (1.0 - np.cos(np.pi * u))
    return q0[None, :] + (q1 - q0)[None, :] * blend[:, None]


def linear_segment(q0: np.ndarray, q1: np.ndarray, samples: int) -> np.ndarray:
    samples = max(2, int(samples))
    u = np.linspace(0.0, 1.0, samples)
    return q0[None, :] + (q1 - q0)[None, :] * u[:, None]


@dataclass
class Timeline:
    frames: np.ndarray          # (N, dim) 每个控制周期一帧
    labels: list[str]           # 每帧所属动作名（日志用）
    duration: float             # 总时长（秒）
    step_ends: list[int]        # 每段运动的结束帧索引
    step_targets: list[np.ndarray]  # 每段目标（终点校验用）

    @property
    def n_frames(self) -> int:
        return len(self.frames)


def build_timeline(plan: list[dict], dt: float, profile: str = "cosine") -> Timeline:
    """把动作计划展开成逐帧时间线。

    plan 里每一项：{"label": str, "from": ndarray, "to": ndarray,
                    "seconds": float, "hold": float}
    """
    maker = cosine_segment if profile == "cosine" else linear_segment
    frames: list[np.ndarray] = []
    labels: list[str] = []
    step_ends: list[int] = []
    step_targets: list[np.ndarray] = []

    for index, item in enumerate(plan):
        q0 = np.asarray(item["from"], dtype=float)
        q1 = np.asarray(item["to"], dtype=float)
        seconds = float(item.get("seconds", 0.0))
        hold = float(item.get("hold", 0.0))
        label = str(item.get("label", f"step{index + 1}"))
        n = max(2, int(round(seconds / dt))) if seconds > 0 else 1
        seg = maker(q0, q1, n)
        if frames:                 # 相邻两段共用边界点，去掉重复那一帧
            seg = seg[1:]
        frames.extend(seg)
        labels.extend([label] * len(seg))
        if hold > 0.0:
            hold_frames = max(1, int(round(hold / dt)))
            hold_q = q1.copy()
            frames.extend([hold_q] * hold_frames)
            labels.extend([f"{label}（停顿）"] * hold_frames)
        step_ends.append(len(frames) - 1)
        step_targets.append(q1)

    if not frames:
        raise ValueError("动作计划为空，检查 config 里的 cycle.steps")
    return Timeline(
        frames=np.asarray(frames, dtype=float),
        labels=labels,
        duration=len(frames) * dt,
        step_ends=step_ends,
        step_targets=step_targets,
    )


def reverse_path(frames: np.ndarray) -> np.ndarray:
    """回程 = 同一段指令轨迹倒放（对应 C++ 里的 std::reverse）。"""
    return np.ascontiguousarray(frames[::-1])
