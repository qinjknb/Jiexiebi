#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无 ROS 自检：转换器、SRDF 姿态、角度映射、轨迹、两种控制模式、dry 命令流。

运行：
    cd mujoco_dofbot
    python tests/test_smoke.py          # 或  python -m unittest discover -s tests
"""

from __future__ import annotations

import io
import shutil
import sys
import unittest
import contextlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402

from dofbot_mj.config import load_config  # noqa: E402
from dofbot_mj.motion import build_timeline  # noqa: E402
from dofbot_mj.poses import parse_srdf  # noqa: E402
from dofbot_mj.protocol import ServoMapping  # noqa: E402
from dofbot_mj.urdf2mjcf import convert  # noqa: E402

URDF = ROOT.parent / "src" / "dofbot_pro_description" / "urdf" / "DOFBOT_Pro-V24.urdf"
SRDF = ROOT.parent / "src" / "My_JXB_moveit" / "config" / "DOFBOT_Pro-V24.srdf"
BASELINE = ROOT / "models" / "dofbot_pro.xml"
SCALE = 180.0 / np.pi
TMP_ROOT = ROOT / "build" / "test_tmp"   # 工程内临时目录（系统 temp 可能被策略限制）


def quiet_app(**overrides):
    """构造一个静音、小步长的 SimApp，供各测试复用。"""
    from dofbot_mj.runner import SimApp

    settings = load_config(ROOT / "config" / "default.yaml")
    settings.cycle.segment_seconds = overrides.pop("segment_seconds", 0.4)
    settings.cycle.repeat = 1
    settings.cycle.pause_seconds = overrides.pop("pause_seconds", 0.0)
    settings.servo.enabled = overrides.pop("servo", False)
    settings.servo.test_mode = True
    if "mode" in overrides:
        settings.control.mode = overrides.pop("mode")
    for key, value in overrides.items():
        obj, attr = key.split("__", 1)
        setattr(getattr(settings, obj), attr, value)
    logs: list[str] = []
    app = SimApp(settings, log=logs.append, speed=0.0)
    return app, logs


class TestConversion(unittest.TestCase):
    def setUp(self):
        # 不用 tempfile：它建的目录带受限 ACL，子目录会创建失败
        self.out_dir = TMP_ROOT / "convert"
        shutil.rmtree(self.out_dir, ignore_errors=True)
        self.out_dir.mkdir(parents=True, exist_ok=True)

    def test_urdf_to_mjcf(self):
        self.assertTrue(URDF.is_file(), f"缺少 URDF: {URDF}")
        out = self.out_dir / "robot.xml"
        sidecar = convert(URDF, out, mesh_roots=[URDF.parent.parent])
        self.assertTrue(out.is_file())
        self.assertEqual(sidecar["robot"], "DOFBOT_Pro-V24")
        self.assertEqual(sidecar["mesh_scale"], 1.0, "网格单位判断错误")
        self.assertFalse(sidecar["missing_meshes"], "有网格没找到")

        model = mujoco.MjModel.from_xml_path(str(out))
        names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                 for i in range(model.njnt)}
        for expected in ("Arm1_Joint", "Arm2_Joint", "Arm3_Joint",
                         "Arm4_Joint", "Arm5_Joint", "grip_joint"):
            self.assertIn(expected, names)
        self.assertEqual(model.nu, 11, "执行器数量不对")
        self.assertGreater(float(model.body_mass.sum()), 0.4)

    def test_oversized_mesh_is_split(self):
        out = self.out_dir / "robot.xml"
        sidecar = convert(URDF, out, mesh_roots=[URDF.parent.parent])
        # Arm4_Link 有 21.8 万面，超过 MuJoCo 的 20 万上限，应被拆成多份
        self.assertTrue(sidecar["split_meshes"], "超限网格没有被拆分")
        self.assertTrue((self.out_dir / "mesh_parts").is_dir())

    def test_mimic_recorded(self):
        out = self.out_dir / "robot.xml"
        sidecar = convert(URDF, out, mesh_roots=[URDF.parent.parent])
        self.assertEqual(len(sidecar["mimic"]), 5)
        self.assertEqual(sidecar["mimic"]["rlink_joint2"]["multiplier"], 1.0)
        self.assertEqual(sidecar["mimic"]["rlink_joint3"]["multiplier"], -1.0)


class TestPoses(unittest.TestCase):
    def setUp(self):
        if not SRDF.is_file():
            self.skipTest(f"缺少 SRDF: {SRDF}")
        self.states = parse_srdf(SRDF)["states"]

    def test_named_poses(self):
        for name in ("up", "init", "close", "open"):
            self.assertIn(name, self.states)
        self.assertAlmostEqual(self.states["init"]["Arm2_Joint"], 0.5)
        self.assertAlmostEqual(self.states["init"]["Arm3_Joint"], -1.0)
        self.assertAlmostEqual(self.states["close"]["grip_joint"], -1.2)
        self.assertAlmostEqual(self.states["open"]["grip_joint"], -0.2)


class TestServoMapping(unittest.TestCase):
    def setUp(self):
        self.settings = load_config(ROOT / "config" / "default.yaml")
        self.mapping = ServoMapping(self.settings.servo.joints, 300)

    def test_known_angles(self):
        targets = self.mapping.targets({
            "Arm1_Joint": 0.0, "Arm2_Joint": 0.5, "Arm3_Joint": -1.0,
            "Arm4_Joint": -1.0, "Arm5_Joint": 0.0, "grip_joint": -1.2,
        })
        self.assertAlmostEqual(targets[1], 150.0, places=3)
        self.assertAlmostEqual(targets[2], 150.0 + 0.5 * SCALE, places=3)
        self.assertAlmostEqual(targets[3], 150.0 - 1.0 * SCALE, places=3)
        self.assertAlmostEqual(targets[6], 150.0 - 1.2 * SCALE, places=3)

    def test_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            self.mapping.targets({
                "Arm1_Joint": 0.0, "Arm2_Joint": 3.0, "Arm3_Joint": 0.0,
                "Arm4_Joint": 0.0, "Arm5_Joint": 0.0, "grip_joint": 0.0,
            })

    def test_position_roundtrip(self):
        self.assertAlmostEqual(self.mapping.degrees_from_position(4095), 300.0)
        self.assertAlmostEqual(self.mapping.degrees_from_position(0), 0.0)


class TestTimeline(unittest.TestCase):
    def test_cycle_returns_arm_to_start(self):
        app, _ = quiet_app()
        plan = app.build_plan()
        timeline = build_timeline(plan, 1.0 / 50.0, "cosine")
        start = np.zeros(app.robot.dim)
        self.assertTrue(np.allclose(timeline.frames[0], start, atol=1e-9))

        # cycle_demo 的序列是 arm→init → 夹爪闭合 → arm→up → 夹爪张开，
        # 所以一轮结束时手臂回到 up（全零），夹爪停在 open(-0.2) 而不是起点。
        expected = start.copy()
        grip = app.robot.driver_joints.index("grip_joint")
        expected[grip] = -0.2
        self.assertTrue(np.allclose(timeline.frames[-1], expected, atol=1e-6),
                        f"一轮结束姿态不对：{timeline.frames[-1]}")

    def test_frames_within_limits(self):
        app, _ = quiet_app()
        plan = app.build_plan()
        timeline = build_timeline(plan, 1.0 / 50.0, "cosine")
        for index, name in enumerate(app.robot.driver_joints):
            lo, hi = app.robot.limits(name)
            column = timeline.frames[:, index]
            self.assertGreaterEqual(column.min(), lo - 1e-9, name)
            self.assertLessEqual(column.max(), hi + 1e-9, name)

    def test_cosine_is_smooth(self):
        app, _ = quiet_app(segment_seconds=1.0, pause_seconds=0.0)
        plan = app.build_plan()
        timeline = build_timeline(plan, 1.0 / 100.0, "cosine")
        steps = np.abs(np.diff(timeline.frames, axis=0)).max(axis=1)
        peak = steps.max()
        # 余弦缓入缓出：起步那一帧的速度应远小于段中峰值（约 1/16）
        self.assertLess(steps[0], 0.3 * peak, "起步没有缓入")
        self.assertLess(steps[-1], 0.3 * peak, "结束没有缓出")


class TestControlModes(unittest.TestCase):
    def test_kinematic_reaches_target(self):
        app, logs = quiet_app()
        self.assertEqual(app.run(repeat=1, headless=True), 0)
        self.assertTrue(any("完成" in line for line in logs))

    def test_physics_follows(self):
        app, logs = quiet_app(mode="physics", segment_seconds=1.5)
        app.run(repeat=1, headless=True)
        # 物理模式跑完后检查最后一段是否真的到位
        measured = app.robot.vector(app.robot.measure({}))
        target = app.build_plan()[-1]["to"]
        error = float(np.max(np.abs(measured - target)))
        self.assertLess(error, 0.02, f"物理模式跟随误差过大: {error:.4f} rad")


class TestSafetyGates(unittest.TestCase):
    def test_uncalibrated_refuses_without_opening_port(self):
        """未确认标定时必须拒绝运动，且不能去打开串口。"""
        from dofbot_mj.config import load_config
        from dofbot_mj.servo_link import ServoLink

        settings = load_config(ROOT / "config" / "default.yaml")
        settings.servo.enabled = True
        settings.servo.enable_motion = True
        settings.servo.test_mode = False
        settings.servo.calibration_confirmed = False
        settings.servo.port = "COM_DOES_NOT_EXIST"
        logs: list[str] = []
        link = ServoLink(settings.servo, log=logs.append)
        link.start()
        link.stop()
        self.assertTrue(any("被阻止" in line or "标定" in line for line in logs),
                        f"没有给出拒绝提示：{logs}")


class TestDryServo(unittest.TestCase):
    def test_dry_commands(self):
        app, logs = quiet_app(servo=True)
        app.cfg.servo.test_mode = True
        app.cfg.servo.enabled = True
        app.run(repeat=1, headless=True)
        dry = [line for line in logs if "[DRY]" in line]
        self.assertTrue(dry, "dry 模式没有输出任何 MOVE")

        import re

        values: dict[int, list[float]] = {}
        for line in dry:
            for sid, deg in re.findall(r"MOVE (\d+) ([\d.\-]+) \d+", line):
                values.setdefault(int(sid), []).append(float(deg))
        self.assertEqual(set(values), {1, 2, 3, 4, 5, 6}, "6 路舵机应都出现过")
        for sid, angles in values.items():
            self.assertGreaterEqual(min(angles), 0.0, f"ID{sid} 越下限")
            self.assertLessEqual(max(angles), 300.0, f"ID{sid} 越上限")
        self.assertAlmostEqual(max(values[2]), 150.0 + 0.5 * SCALE, delta=0.3)
        self.assertAlmostEqual(min(values[3]), 150.0 - 1.0 * SCALE, delta=0.3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
