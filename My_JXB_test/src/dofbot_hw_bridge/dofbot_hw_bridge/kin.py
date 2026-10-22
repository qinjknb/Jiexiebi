#!/usr/bin/env python3
"""DOFBOT Pro 正逆运动学 —— 全部几何直接从 URDF 读，不写死。

为什么要它：
  "机械臂能抓多远"这个问题，本质是**夹爪抓取点**能到哪儿。
  URDF 里 `Gripping_point_Link` 就是抓取点（比 Arm5 原点再沿 +y 出去 9cm，
  大致等于指尖）。把它当作目标点做正/逆运动学，就能算出可达半径、
  也能反解出"要去某个半径应该给什么关节角"。

几何特点（从 URDF 反推，代码里会 assert 校验）：
  Arm1  底座偏航，绕 z
  Arm2/3/4  三个俯仰关节，全部绕 x  —— 所以是平面 3R 臂
  Arm5  绕 y 滚转；抓取点正好在 Arm5 的 +y 轴上，
        **所以 Arm5 转动不改变抓取点位置**，位置只由 Arm1~Arm4 决定。
  Arm4 的 origin 带 +90° 的 rpy，于是手部绝对俯仰
        α3 = α0 - (q2 + q3 + 90° + q4)，α0≈89.27°（手部带 2mm 偏心）。
  实测含义：q4=0 时夹爪朝上；q4=-90° 且臂伸直时夹爪水平前伸；
        要把夹爪压成竖直朝下(α3=180°)，必须把肘部折起来
        （q2+q3+q4 ≈ -180.7°），此时可达半径会明显变小。

坐标约定：
  base_link 原点在底座底面上，z 向上；"正前方"是 +y（全 0 姿态朝上，
  Arm2 = -90° 时水平伸向 +y）。
  平面内用 (u, w) = (水平半径, 高度)，角 α 从 +z 转向 +y 为正。
"""

from __future__ import annotations

import math
import os
import struct
import xml.etree.ElementTree as ET

ARM_JOINTS = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint"]
GRIP_JOINT = "grip_joint"
TIP_LINK = "Gripping_point_Link"


# ------------------------------------------------------------------ 3x3 小工具
def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return _mat_mul(_mat_mul(Rz, Ry), Rx)


def _axis_angle(axis, ang):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(ang), math.sin(ang)
    C = 1.0 - c
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _floats(text, n):
    if not text:
        return [0.0] * n
    vals = [float(v) for v in text.replace(",", " ").split()]
    return (vals + [0.0] * n)[:n]


# ------------------------------------------------------------------ 运动学
class ArmKin:
    """URDF 驱动的 DOFBOT Pro 运动学。关节顺序固定为 ARM_JOINTS。"""

    def __init__(self, urdf_path, tip_link=TIP_LINK, joint_names=None):
        self.urdf_path = os.path.abspath(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        self.joints = {j.get("name"): j for j in root.findall("joint")}
        self.joint_names = list(joint_names or ARM_JOINTS)

        self.chain = []
        for name in self.joint_names:
            j = self.joints[name]
            o = j.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None, 3)
            rpy = _floats(o.get("rpy") if o is not None else None, 3)
            ax = j.find("axis")
            axis = _floats(ax.get("xyz") if ax is not None else None, 3)
            lim = j.find("limit")
            lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else -math.pi
            hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else math.pi
            self.chain.append({"name": name, "xyz": xyz, "R": _rpy_to_mat(*rpy),
                               "axis": axis, "lo": lo, "hi": hi})

        # 末端固定段：Arm5_Link -> 抓取点
        gp = None
        for j in root.findall("joint"):
            if j.get("type") == "fixed" and j.find("child") is not None \
                    and j.find("child").get("link") == tip_link:
                gp = j
                break
        if gp is None:
            raise ValueError("URDF 里找不到通向 %s 的固定关节" % tip_link)
        self.tip_xyz = _floats(gp.find("origin").get("xyz"), 3)
        self.tip_parent = gp.find("parent").get("link")

        self._derive_planar()

    # ---------------- 几何量（派生 + 校验） ----------------
    def _derive_planar(self):
        """把 5 个关节的几何折算成平面 3R 臂的参数，并校验结构假设。"""
        c = self.chain
        # 肩部高度 = Arm1/J2 两级 origin 的 z 累加
        self.shoulder_z = c[0]["xyz"][2] + c[1]["xyz"][2]
        self.L1 = math.sqrt(sum(v * v for v in c[2]["xyz"]))      # 肩 -> 肘
        self.L2 = math.sqrt(sum(v * v for v in c[3]["xyz"]))      # 肘 -> 腕

        # 手部：Arm4 -> Arm5 的 origin 再叠上抓取点偏移
        # 结构上 Arm5 的 +y 就是抓取方向，滚转不改变抓取点，所以直接相加。
        seg = [c[4]["xyz"][i] + self.tip_xyz[i] for i in range(3)]
        self.seg3 = (seg[1], seg[2])          # (d, e)：平面内 (y, z) 分量
        self.seg3_len = math.hypot(*self.seg3)
        # 手部向量自身的初始夹角（从 +z 转向 +y），≈89.27°，差的那 0.73° 就是 e 偏心
        self.hand_alpha0 = math.atan2(self.seg3[0], self.seg3[1])
        # 手部绝对旋转角 φ = q2+q3+90°+q4；而手部方向 α3 = α0 - φ
        self.q4_offset = self.hand_alpha0 - math.pi / 2.0

        # 结构假设校验：只在俯仰/偏航平面内，且 J4 之前有 +90° 固定旋转
        ok = True
        for i in (1, 2, 3):
            ax = c[i]["axis"]
            ok = ok and abs(abs(ax[0]) - 1.0) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6
        ok = ok and abs(abs(c[0]["axis"][2]) - 1.0) < 1e-6
        self.planar_ok = bool(ok)
        # J4 前面的固定俯仰角（normally +90°）
        R3 = c[3]["R"]
        self.j4_fixed_pitch = math.atan2(-R3[2][1], R3[1][1])

    def describe(self):
        return (
            "URDF %s\n"
            "  肩部高度 %.4f m   上臂 %.4f m   小臂 %.4f m   手部(含指尖) %.4f m\n"
            "  手部向量 (y,z)=(%.5f, %.5f)   平面 3R 结构: %s"
            % (self.urdf_path, self.shoulder_z, self.L1, self.L2, self.seg3_len,
               self.seg3[0], self.seg3[1], "是" if self.planar_ok else "否(URDF 可能改过，IK 结果需复核)")
        )

    # ---------------- 正运动学 ----------------
    def fk(self, q):
        """q: 5 个关节角（弧度）-> 抓取点 (x, y, z)，base_link 坐标系。"""
        R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        t = [0.0, 0.0, 0.0]
        for i, j in enumerate(self.chain):
            # 先叠 origin
            t = [t[k] + sum(R[k][m] * j["xyz"][m] for m in range(3)) for k in range(3)]
            R = _mat_mul(R, j["R"])
            # 再叠关节旋转
            R = _mat_mul(R, _axis_angle(j["axis"], q[i]))
        t = [t[k] + sum(R[k][m] * self.tip_xyz[m] for m in range(3)) for k in range(3)]
        return t

    def fk_all(self, q):
        """额外给出手部（Arm5 原点）位置，方便看腕部余量。"""
        return self.fk(q), None

    def wrist_pos(self, q):
        """Arm5 原点（腕部）位置。"""
        saved = self.tip_xyz
        try:
            self.tip_xyz = [0.0, 0.0, 0.0]
            return self.fk(q)
        finally:
            self.tip_xyz = saved

    # ---------------- 限位 ----------------
    def limits_rad(self, calib=None):
        """URDF 限位 ∩ 标定限位。calib 是 calib.ArmCalib。"""
        lo, hi = [], []
        for j in self.chain:
            a, b = j["lo"], j["hi"]
            if calib is not None:
                c = calib.joints.get(j["name"])
                if c is not None:
                    tpr = c.ticks_per_rad
                    t1 = c.direction * (c.tick_min - c.center) / tpr
                    t2 = c.direction * (c.tick_max - c.center) / tpr
                    a = max(a, min(t1, t2))
                    b = min(b, max(t1, t2))
            lo.append(a)
            hi.append(b)
        return lo, hi

    # ---------------- 逆运动学 ----------------
    def ik_point(self, target, wrist_deg=90.0, elbow=+1.0, roll_rad=0.0,
                 calib=None, eps_deg=0.0):
        """目标点 (x,y,z) -> (q5, err_m)；不可达返回 (None, 原因字符串)。

        wrist_deg：第 3 段（腕->指尖）与 +z 的夹角，从 +z 转向正前方为正。
                   90°=夹爪水平前伸（这个臂最常用的抓取姿态）
                   0°=夹爪竖直朝上；受 q4 限位，**不能到 180°（竖直朝下）**。
        elbow：+1 / -1 选择肘部里/外两个解。
        eps_deg：把解往限位内侧收一点，避免顶着限位抖。
        """
        x, y, z = float(target[0]), float(target[1]), float(target[2])
        r = math.hypot(x, y)
        q1 = 0.0 if r < 1e-9 else math.atan2(-x, y)

        a3 = math.radians(wrist_deg)
        # 手部向量（腕点 -> 抓取点）长度固定，方向就是 α3。
        # 注意别用 (d,e) 直接旋转：手部带 2mm 偏心，必须按"整段方向"处理。
        v3u = self.seg3_len * math.sin(a3)
        v3w = self.seg3_len * math.cos(a3)

        Wu = r - v3u
        Ww = z - self.shoulder_z - v3w
        D = math.hypot(Wu, Ww)
        if D > self.L1 + self.L2 + 1e-9:
            return None, "超出臂展（腕点需 %.3f m，最大 %.3f m）" % (D, self.L1 + self.L2)
        if D < abs(self.L1 - self.L2) - 1e-9:
            return None, "太近，进不去（腕点 %.3f m < 最小 %.3f m）" % (D, abs(self.L1 - self.L2))

        beta = math.atan2(Wu, Ww)
        cosg = (D * D + self.L1 * self.L1 - self.L2 * self.L2) / (2.0 * self.L1 * D)
        g = math.acos(max(-1.0, min(1.0, cosg)))
        a1 = beta + elbow * g
        p1u, p1w = self.L1 * math.sin(a1), self.L1 * math.cos(a1)
        a2 = math.atan2(Wu - p1u, Ww - p1w)

        q2, q3 = -a1, -(a2 - a1)
        # 手部绝对旋转 φ = α0 - α3 = q2+q3+90°+q4  =>  q4 = φ - 90° - q2 - q3
        q4 = (self.hand_alpha0 - a3) - math.pi / 2.0 - q2 - q3
        q = [q1, q2, q3, q4, float(roll_rad)]

        lo, hi = self.limits_rad(calib)
        eps = math.radians(eps_deg)
        for i, v in enumerate(q):
            if v < lo[i] - 1e-6 or v > hi[i] + 1e-6:
                return None, "关节 %s 超限（%+.1f° 不在 %+.1f~%+.1f°）" % (
                    self.joint_names[i], math.degrees(v),
                    math.degrees(lo[i]), math.degrees(hi[i]))
        if eps > 0:
            q = [min(max(v, l + eps), h - eps) for v, l, h in zip(q, lo, hi)]

        tip = self.fk(q)
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(tip, target)))
        if err > 3e-3:
            return None, "解析解复核失败（残差 %.1f mm），URDF 可能改过" % (err * 1e3)
        return q, err

    def ik_radial(self, radius, z, azimuth_deg=90.0, **kw):
        """按"半径 + 高度 + 方位角"求逆解。方位角 90° = 正前方(+y)。"""
        a = math.radians(azimuth_deg)
        return self.ik_point((radius * math.cos(a), radius * math.sin(a), z), **kw)

    # ---------------- 包络 ----------------
    def envelope(self, z_list, wrist_deg=90.0, calib=None, step_cm=1.0):
        """对每个高度给出可达半径 [外层, 内层]（粗扫，用来出表）。"""
        out = []
        for z in z_list:
            good = []
            r = 5.0
            while r <= 45.0 + 1e-9:
                q, err = self.ik_radial(r / 100.0, z / 100.0, 90.0,
                                        wrist_deg=wrist_deg, elbow=+1.0, calib=calib)
                if q is not None:
                    good.append(r)
                r += step_cm
            if not good:
                out.append((z, None, None))
            else:
                out.append((z, min(good), max(good)))
        return out


    def self_check(self, calib=None, n=200):
        """往返自检：随机取一组关节角 -> 正解出抓取点 -> 反解 -> 再正解，比位置。

        这能同时校验"平面 3R 模型"和"手部 α3 关系"两个推导，
        改过 URDF 或 arm_calib.yaml 后跑一下就知道还对不对。
        返回 (最大残差 m, 参与校验的点数)。
        """
        import random
        lo, hi = self.limits_rad(calib)
        rng = random.Random(12345)
        worst, cnt = 0.0, 0
        for _ in range(n):
            q = [rng.uniform(l, h) for l, h in zip(lo, hi)]
            tip = self.fk(q)
            if math.hypot(tip[0], tip[1]) < 0.01:      # 太靠近底座轴线，跳过
                continue
            a3 = self.hand_alpha0 - (q[1] + q[2] + math.pi / 2.0 + q[3])
            best = None
            for el in (1.0, -1.0):
                qq, _ = self.ik_point(tip, wrist_deg=math.degrees(a3), elbow=el,
                                      calib=calib)
                if qq is None:
                    continue
                t2 = self.fk(qq)
                # 只比"半径 + 高度"：URDF 里两级 origin 有 0.6mm 的 x 偏心，
                # 它只会让方位角偏 ~0.1°，不影响力臂，不算模型误差。
                e = math.hypot(math.hypot(t2[0], t2[1]) - math.hypot(tip[0], tip[1]),
                               t2[2] - tip[2])
                best = e if best is None else min(best, e)
            if best is None:
                continue
            cnt += 1
            worst = max(worst, best)
        return worst, cnt


# ------------------------------------------------------------------ 手部实体
class HandGeometry:
    """用 URDF 里的 STL 网格算"手部真实最低点"。

    为什么需要它：`Gripping_point_Link` 只是指尖上的**一个点**，
    腕部块（Arm5_Link）和夹爪指片在有些姿态下比这个点更低
    （实测 30 cm / 腕 120° 时低 8 mm）。判断"会不会蹭桌面"必须看实体最低点。
    """

    DEFAULT_LINKS = ("Arm5_Link", "DaBai_DCW2_Link", "rlink1", "rlink2", "rlink3",
                     "llink1", "llink2", "llink3", "Gripping_point_Link")

    def __init__(self, urdf_path, links=None, max_tris=1500):
        self.urdf_path = os.path.abspath(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        self.joints = {j.get("name"): j for j in root.findall("joint")}
        self.by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        self.links = {}          # name -> [(x,y,z), ...]  已转换到 link 坐标系
        self.missing = []
        for L in root.findall("link"):
            name = L.get("name")
            if links is not None and name not in links:
                continue
            if links is None and name not in self.DEFAULT_LINKS:
                continue
            me = L.find("visual/geometry/mesh")
            if me is None:
                continue
            try:
                verts = self._load_mesh(self._resolve(me.get("filename")), max_tris)
            except Exception:
                self.missing.append(name)
                continue
            vo = L.find("visual/origin")
            if vo is not None:
                xyz = _floats(vo.get("xyz"), 3)
                Rv = _rpy_to_mat(*_floats(vo.get("rpy"), 3))
                verts = [[Rv[i][0] * v[0] + Rv[i][1] * v[1] + Rv[i][2] * v[2] + xyz[i]
                          for i in range(3)] for v in verts]
            self.links[name] = verts

    # ---- 资源定位 ----
    def _resolve(self, uri):
        if uri.startswith("package://"):
            parts = uri[len("package://"):].split("/")
            try:
                from ament_index_python.packages import get_package_share_directory
                return os.path.join(get_package_share_directory(parts[0]), *parts[1:])
            except Exception:
                return os.path.abspath(os.path.join(
                    os.path.dirname(self.urdf_path), "..", *parts[1:]))
        if uri.startswith("file://"):
            return uri[len("file://"):]
        return os.path.abspath(uri)

    @staticmethod
    def _load_mesh(path, max_tris):
        with open(path, "rb") as f:
            b = f.read()
        if len(b) >= 84:
            n = struct.unpack("<I", b[80:84])[0]
            if n > 0 and 84 + n * 50 == len(b):
                k = max(1, n // max_tris)
                out = []
                for i in range(0, n, k):
                    f12 = struct.unpack("<12f", b[84 + i * 50:84 + i * 50 + 48])
                    out.append((f12[3], f12[4], f12[5]))
                    out.append((f12[6], f12[7], f12[8]))
                    out.append((f12[9], f12[10], f12[11]))
                return out
        # 兜底：ASCII STL
        txt = b.decode("ascii", "ignore").split()
        out = []
        for i, t in enumerate(txt):
            if t == "vertex" and i + 3 < len(txt):
                out.append((float(txt[i + 1]), float(txt[i + 2]), float(txt[i + 3])))
        if not out:
            raise ValueError("无法解析 STL：%s" % path)
        return out

    # ---- 正运动学 ----
    def _q(self, j, qmap):
        if j.get("type") == "fixed":
            return 0.0
        m = j.find("mimic")
        if m is not None:
            src = self.joints.get(m.get("joint"))
            return (self._q(src, qmap) * float(m.get("multiplier", "1"))
                    + float(m.get("offset", "0")))
        return float(qmap.get(j.get("name"), 0.0))

    def fk_link(self, link, qmap):
        chain, cur = [], link
        while cur in self.by_child:
            j = self.by_child[cur]
            chain.append(j)
            cur = j.find("parent").get("link")
        chain.reverse()
        R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        t = [0.0, 0.0, 0.0]
        for j in chain:
            o = j.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None, 3)
            rpy = _floats(o.get("rpy") if o is not None else None, 3)
            t = [t[k] + sum(R[k][m] * xyz[m] for m in range(3)) for k in range(3)]
            R = _mat_mul(R, _rpy_to_mat(*rpy))
            ax = j.find("axis")
            if ax is not None:
                R = _mat_mul(R, _axis_angle(_floats(ax.get("xyz"), 3), self._q(j, qmap)))
        return R, t

    def lowest_z(self, q_arm, grip_rad=0.0):
        """手部最低点 -> (z, 哪个零件)。q_arm 是 5 个臂关节角（弧度）。"""
        qmap = {n: v for n, v in zip(ARM_JOINTS, q_arm)}
        qmap[GRIP_JOINT] = float(grip_rad)
        best_z, best_link = 9.0, ""
        for name, verts in self.links.items():
            R, t = self.fk_link(name, qmap)
            r2x, r2y, r2z = R[2]
            tz = t[2]
            for v in verts:
                z = tz + r2x * v[0] + r2y * v[1] + r2z * v[2]
                if z < best_z:
                    best_z, best_link = z, name
        return best_z, best_link


# ------------------------------------------------------------------ 定位 URDF
def find_urdf(explicit=None):
    """按优先级找 DOFBOT 的 urdf：参数 > 包共享目录 > 仓库相对路径。"""
    if explicit:
        p = os.path.abspath(os.path.expanduser(explicit))
        if os.path.isfile(p):
            return p
        raise FileNotFoundError("指定的 URDF 不存在：%s" % explicit)

    try:                                   # ROS 环境
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("dofbot_pro_description"),
                         "urdf", "DOFBOT_Pro-V24.urdf")
        if os.path.isfile(p):
            return p
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    for up in (2, 3, 4, 5):                # src/dofbot_hw_bridge/dofbot_hw_bridge -> src/..
        p = os.path.join(here, *[".."] * up, "dofbot_pro_description", "urdf",
                         "DOFBOT_Pro-V24.urdf")
        p = os.path.abspath(p)
        if os.path.isfile(p):
            return p
    raise FileNotFoundError("找不到 DOFBOT_Pro-V24.urdf，请用 --urdf 指定")
