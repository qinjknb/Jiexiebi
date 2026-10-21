"""关节标定：MoveIt 关节角（弧度） <-> 总线舵机原始位置（0~4095）。

标定文件示例（config/arm_calib.yaml）：

    bus:
      port: /dev/ttyUSB0
      baud: 115200
      command_rate: 50.0
      state_rate: 50.0
      max_velocity: 3000      # ticks/s，PC 侧安全限速（0 = 不限制）
    joints:
      Arm1_Joint:
        servo_id: 1
        center: 2047          # 关节角为 0 时舵机的原始位置
        direction: 1          # +1/-1：关节角增大时舵机位置增大/减小
        tick_min: 0           # 软限位（可选，默认 0/4095）
        tick_max: 4095
        range_deg: 300        # 舵机满行程角度

换算：
    tick  = center + direction * (angle_rad * 180/pi) * 4095 / range_deg
    angle = direction * (tick - center) * (range_deg/4095) * pi/180
"""

from __future__ import annotations

import math
import os

# 机械臂 6 个关节（顺序就是 /joint_states 与 SET_TARGETS 的顺序）
DEFAULT_JOINT_ORDER = [
    "Arm1_Joint",
    "Arm2_Joint",
    "Arm3_Joint",
    "Arm4_Joint",
    "Arm5_Joint",
    "grip_joint",
]

ARM_JOINTS = DEFAULT_JOINT_ORDER[:5]
GRIP_JOINTS = DEFAULT_JOINT_ORDER[5:]

DEFAULT_RANGE_DEG = 300.0
TICK_MAX = 4095


class JointCalib:
    """单个关节的标定参数。"""

    __slots__ = ("name", "servo_id", "center", "direction", "tick_min",
                 "tick_max", "range_deg")

    def __init__(self, name, servo_id=1, center=2047, direction=1,
                 tick_min=0, tick_max=TICK_MAX, range_deg=DEFAULT_RANGE_DEG):
        self.name = name
        self.servo_id = int(servo_id)
        self.center = float(center)
        self.direction = 1.0 if direction >= 0 else -1.0
        self.tick_min = int(tick_min)
        self.tick_max = int(tick_max)
        self.range_deg = float(range_deg)
        if self.range_deg <= 1.0:
            self.range_deg = DEFAULT_RANGE_DEG
        if self.tick_min > self.tick_max:
            self.tick_min, self.tick_max = self.tick_max, self.tick_min

    @property
    def ticks_per_rad(self) -> float:
        return TICK_MAX / (self.range_deg * math.pi / 180.0)

    def angle_to_tick(self, angle_rad: float, clamp: bool = True) -> int:
        t = self.center + self.direction * float(angle_rad) * self.ticks_per_rad
        ti = int(round(t))
        if clamp:
            ti = max(self.tick_min, min(self.tick_max, ti))
        return ti

    def tick_to_angle(self, tick: float) -> float:
        return self.direction * (float(tick) - self.center) / self.ticks_per_rad

    def __repr__(self) -> str:
        return (f"JointCalib({self.name} id={self.servo_id} center={self.center:.0f} "
                f"dir={self.direction:+.0f} ticks=[{self.tick_min},{self.tick_max}] "
                f"range={self.range_deg:g})")


class ArmCalib:
    """整机标定：总线参数 + 6 个关节。"""

    def __init__(self, joints=None, bus=None, joint_limits=None):
        self.joints = {}
        # 可选：URDF 里的关节角限位（弧度），只用于把"发布出去的角度"钳在合法范围内
        # （舵机回读有 ±几个 tick 的噪声，超界会让 MoveIt 的 CheckStartStateBounds 拒绝规划）
        self.joint_limits = {}
        for k, v in (joint_limits or {}).items():
            try:
                if isinstance(v, dict):
                    lo = float(v["min"])
                    hi = float(v["max"])
                else:                       # 兼容 [min, max] 列表写法
                    lo = float(v[0])
                    hi = float(v[1])
                self.joint_limits[k] = (min(lo, hi), max(lo, hi))
            except Exception:
                pass
        bus = dict(bus or {})
        self.port = str(bus.get("port", "/dev/ttyUSB0"))
        self.baud = int(bus.get("baud", 115200))
        self.command_rate = float(bus.get("command_rate", 50.0))
        self.state_rate = float(bus.get("state_rate", 50.0))
        self.max_velocity = float(bus.get("max_velocity", 3000.0))   # ticks/s
        self.telem_rate = int(bus.get("telem_rate", 25))
        self.link_timeout = float(bus.get("link_timeout", 0.4))

        order = DEFAULT_JOINT_ORDER
        for i, name in enumerate(order):
            src = None
            if joints:
                src = joints.get(name)
            if src is None:
                # 没写就用默认：ID=i+1，中位 2047，方向 +1
                self.joints[name] = JointCalib(name, servo_id=i + 1)
            else:
                self.joints[name] = JointCalib(
                    name,
                    servo_id=src.get("servo_id", i + 1),
                    center=src.get("center", 2047),
                    direction=src.get("direction", 1),
                    tick_min=src.get("tick_min", 0),
                    tick_max=src.get("tick_max", TICK_MAX),
                    range_deg=src.get("range_deg", DEFAULT_RANGE_DEG),
                )
        self.order = list(order)

    # ---- 角度钳位（发布用，不影响下发）----
    def clamp_rad(self, name: str, value: float) -> float:
        lim = self.joint_limits.get(name)
        if not lim:
            return value
        lo, hi = lim
        eps = 1e-4                      # 往里收 0.0001 rad，避免浮点误差
        if value < lo + eps:
            return lo + eps
        if value > hi - eps:
            return hi - eps
        return value

    def limit_slack(self, name: str, value: float) -> float:
        """返回越界量（0 表示没越界），用于打印告警。"""
        lim = self.joint_limits.get(name)
        if not lim:
            return 0.0
        lo, hi = lim
        return max(0.0, lo - value, value - hi)

    # ---- 角度 <-> 位置 ----
    def angles_to_ticks(self, angles: dict) -> list:
        out = []
        for name in self.order:
            a = float(angles.get(name, 0.0))
            out.append(self.joints[name].angle_to_tick(a))
        return out

    def ticks_to_angles(self, ticks) -> dict:
        out = {}
        for name, t in zip(self.order, ticks):
            if t == 0xFFFF:
                continue
            out[name] = self.joints[name].tick_to_angle(t)
        return out

    def limits(self):
        """-> (tick_min[6], tick_max[6])，用于下发固件软限位。"""
        return ([self.joints[n].tick_min for n in self.order],
                [self.joints[n].tick_max for n in self.order])

    def home_ticks(self) -> list:
        return [int(round(self.joints[n].center)) for n in self.order]

    def __repr__(self) -> str:
        lines = [f"ArmCalib(port={self.port} baud={self.baud} "
                 f"cmd_rate={self.command_rate}Hz state_rate={self.state_rate}Hz)"]
        for n in self.order:
            lines.append("  " + repr(self.joints[n]))
        return "\n".join(lines)


def load_calib(path: str) -> ArmCalib:
    """读取 YAML 标定文件（用 ROS 自带的 yaml，无需额外依赖）。"""
    joints = None
    bus = None
    limits = None
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        data = _safe_load_yaml(text)
        if isinstance(data, dict):
            bus = data.get("bus")
            joints = data.get("joints")
            limits = data.get("joint_limits")
    return ArmCalib(joints=joints, bus=bus, joint_limits=limits)


def _safe_load_yaml(text: str):
    try:
        import yaml
        return yaml.safe_load(text)
    except ImportError:
        return _mini_yaml(text)


def _mini_yaml(text: str):
    """极简 YAML 解析（没有 pyyaml 时的兜底，支持嵌套 map + 标量 + 注释）。"""
    root = {}
    stack = [(-1, root)]
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, sep, val = line.strip().partition(":")
        if not sep:
            continue
        val = val.strip()
        while len(stack) > 1 and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if val:
            parent[key] = _scalar(val)
        else:
            node = {}
            parent[key] = node
            stack.append((indent, node))
    return root


def _scalar(v: str):
    if len(v) >= 2 and v[0] == v[-1] and v[0] in "'\"":
        return v[1:-1]
    low = v.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v
