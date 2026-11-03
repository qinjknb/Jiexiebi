"""轨迹采样：把时间参数化路点变成"任意时刻可采样"的函数。

MoveIt 发下来的 ``trajectory_msgs/JointTrajectory`` 是一串
``(time_from_start, positions, velocities)``。本模块负责：

* 校验关节名、把时间戳排好序；
* 支持两种插值：
  - ``linear``：路点之间线性插值（最稳，默认兜底）
  - ``cubic`` ：三次 Hermite（用路点自带速度，与 ros2_control 的
                JointTrajectoryController 基本一致，速度连续更顺）
  - ``auto``  ：给了非零速度就用 cubic，否则 linear

本模块不依赖 ROS，方便离线自测：``python3 traj.py``
"""

from __future__ import annotations

import bisect
import math


def _finite(v, default=0.0):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def _dur_to_sec(d) -> float:
    """builtin_interfaces/Duration 或 float -> 秒。"""
    if d is None:
        return 0.0
    if isinstance(d, (int, float)):
        return float(d)
    sec = getattr(d, "sec", None)
    nsec = getattr(d, "nanosec", None)
    if sec is None and hasattr(d, "to_sec"):
        return float(d.to_sec())
    return float(sec or 0) + float(nsec or 0) * 1e-9


class TrajSampler:
    """对一条轨迹做时间采样。"""

    def __init__(self, joint_names, points, wanted_joints=None, interp="auto"):
        self.joint_names = list(joint_names)
        self.joints = list(wanted_joints) if wanted_joints else list(self.joint_names)
        self.interp = (interp or "auto").lower()

        idx = []
        for j in self.joints:
            if j not in self.joint_names:
                raise ValueError("轨迹里缺少关节 %s（现有：%s）"
                                 % (j, ", ".join(self.joint_names)))
            idx.append(self.joint_names.index(j))

        pts = list(points)
        if not pts:
            raise ValueError("轨迹没有任何路点")

        self.times = []
        self.pos = {j: [] for j in self.joints}
        self.vel = {j: [] for j in self.joints}
        has_vel = True
        n_names = len(self.joint_names)

        for pt in pts:
            self.times.append(_dur_to_sec(getattr(pt, "time_from_start", 0.0)))
            positions = list(getattr(pt, "positions", []))
            velocities = list(getattr(pt, "velocities", []))
            for j, i in zip(self.joints, idx):
                self.pos[j].append(_finite(positions[i]) if i < len(positions) else 0.0)
                if len(velocities) == n_names:
                    self.vel[j].append(_finite(velocities[i]))
                else:
                    self.vel[j].append(0.0)
                    has_vel = False

        # 时间必须严格递增
        for i in range(1, len(self.times)):
            if self.times[i] <= self.times[i - 1]:
                self.times[i] = self.times[i - 1] + 1e-3

        if self.interp == "linear":
            has_vel = False
        elif self.interp == "cubic":
            has_vel = True
        else:
            has_vel = has_vel and any(
                abs(v) > 1e-9 for j in self.joints for v in self.vel[j])
        self.use_vel = has_vel

    # ---- 基本信息 ----
    @property
    def duration(self) -> float:
        return self.times[-1]

    @property
    def n_points(self) -> int:
        return len(self.times)

    # ---- 采样 ----
    def at(self, t: float) -> dict:
        """返回 {关节名: 角度rad}"""
        out = {}
        for j in self.joints:
            out[j] = self._one(self.pos[j], self.vel[j] if self.use_vel else None, t)
        return out

    def at_point(self, k: int) -> dict:
        k = max(0, min(len(self.times) - 1, int(k)))
        return {j: self.pos[j][k] for j in self.joints}

    def _one(self, p, v, t: float) -> float:
        times = self.times
        if t <= times[0]:
            return p[0]
        if t >= times[-1]:
            return p[-1]
        i = bisect.bisect_right(times, t) - 1
        i = max(0, min(len(times) - 2, i))
        h = times[i + 1] - times[i]
        s = 0.0 if h <= 0.0 else (t - times[i]) / h
        if v is None:
            return p[i] + (p[i + 1] - p[i]) * s
        s2 = s * s
        s3 = s2 * s
        return ((2 * s3 - 3 * s2 + 1) * p[i]
                + (s3 - 2 * s2 + s) * h * v[i]
                + (-2 * s3 + 3 * s2) * p[i + 1]
                + (s3 - s2) * h * v[i + 1])


# ---------------------------------------------------------------------------
# 自测：python3 traj.py
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    class Pt:
        def __init__(self, t, pos, vel=None):
            self.time_from_start = t
            self.positions = pos
            self.velocities = vel if vel is not None else []

    # 线性：0s 在 0，1s 到 1
    s = TrajSampler(["a", "b"], [
        Pt(0.0, [0.0, 0.0]),
        Pt(1.0, [1.0, -1.0]),
    ], wanted_joints=["a"], interp="linear")
    assert abs(s.at(0.0)["a"] - 0.0) < 1e-9
    assert abs(s.at(0.5)["a"] - 0.5) < 1e-9
    assert abs(s.at(1.0)["a"] - 1.0) < 1e-9
    assert abs(s.at(0.25)["a"] - 0.25) < 1e-9
    assert abs(s.at(-1.0)["a"] - 0.0) < 1e-9      # 越界钳位
    assert abs(s.at(9.0)["a"] - 1.0) < 1e-9

    # 三次 Hermite：两端速度为 0 时，中点是 0.5 且速度连续
    s = TrajSampler(["a"], [
        Pt(0.0, [0.0], [0.0]),
        Pt(1.0, [1.0], [0.0]),
    ], interp="cubic")
    assert abs(s.at(0.5)["a"] - 0.5) < 1e-9
    assert abs(s.at(0.25)["a"] - (3 * 0.25 ** 2 - 2 * 0.25 ** 3)) < 1e-9
    # 端点导数应为 0（用差分近似）
    d = (s.at(0.001)["a"] - s.at(0.0)["a"]) / 0.001
    assert abs(d) < 1e-2, d

    # 匀速路点（速度非零）时 Hermite 退化为直线
    s = TrajSampler(["a"], [
        Pt(0.0, [0.0], [1.0]),
        Pt(1.0, [1.0], [1.0]),
    ], interp="cubic")
    for t in (0.1, 0.3, 0.5, 0.9):
        assert abs(s.at(t)["a"] - t) < 1e-9, (t, s.at(t))

    # 关节名不匹配要报错
    try:
        TrajSampler(["a"], [Pt(0.0, [0.0])], wanted_joints=["zzz"])
        raise AssertionError("应当抛出 ValueError")
    except ValueError:
        pass

    # 空轨迹
    try:
        TrajSampler(["a"], [])
        raise AssertionError("应当抛出 ValueError")
    except ValueError:
        pass

    print("traj.py 自测全部通过")
