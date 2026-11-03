#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""播放器：把 config 里的动作序列在 MuJoCo 里跑起来，并同步驱动实物舵机。"""

from __future__ import annotations

import time

import mujoco
import numpy as np

from .config import Settings
from .model import RobotModel
from .motion import build_timeline
from .poses import parse_srdf
from .servo_link import ServoLink


class SimApp:
    def __init__(self, settings: Settings, log=print, speed: float = 1.0,
                 render_dir: str | None = None, initial: dict | None = None):
        self.cfg = settings
        self.log = log
        self.speed = speed
        self.render_dir = render_dir

        self.poses = self._load_poses(settings)
        mimic = {}
        sidecar = settings.model.mjcf.with_suffix(".json")
        if sidecar.is_file():
            import json

            mimic = (json.loads(sidecar.read_text(encoding="utf-8")) or {}).get("mimic", {})

        self.robot = RobotModel(settings.model.mjcf, mimic=mimic)
        # 起始姿态：默认零位（= SRDF 的 up）；给了 initial 就用它
        #（--start-from-servo 时是实机读回来的姿态）
        self.initial = {name: 0.0 for name in self.robot.driver_joints}
        if initial:
            for name, value in initial.items():
                if name in self.initial:
                    self.initial[name] = float(value)
        self.robot.reset_to(self.initial)

        self.servo = ServoLink(settings.servo, log=log)
        self._renderer = None
        self._sim_time = 0.0

    # ------------------------------ 姿态 ------------------------------
    def _load_poses(self, settings: Settings) -> dict:
        poses: dict = {}
        srdf = settings.model.srdf
        if srdf.is_file():
            try:
                poses.update(parse_srdf(srdf)["states"])
            except Exception as exc:  # SRDF 坏了不至于让仿真起不来
                self.log(f"[警告] 解析 SRDF 失败（{exc}），命名姿态不可用")
        # 合并实机实测记录的姿态（config/poses.yaml，由 tools/record_pose.py 生成）
        extra = settings.root / "config" / "poses.yaml"
        if extra.is_file():
            try:
                import yaml

                recorded = yaml.safe_load(extra.read_text(encoding="utf-8")) or {}
                for name, entry in recorded.items():
                    rad = (entry or {}).get("joint_rad")
                    if isinstance(rad, dict):
                        poses[name] = {k: float(v) for k, v in rad.items()}
                if recorded:
                    self.log(f"[姿态] 已载入实测姿态：{', '.join(sorted(recorded))}")
            except Exception as exc:
                self.log(f"[警告] 读取 {extra.name} 失败（{exc}）")
        return poses

    def pose_values(self, name: str) -> dict:
        if name not in self.poses:
            raise KeyError(
                f"没有姿态 '{name}'（SRDF 与 config/poses.yaml 里都没有）；"
                f"可用：{', '.join(sorted(self.poses)) or '无'}")
        # 只保留模型里真实存在的驱动关节
        return {
            joint: float(value)
            for joint, value in self.poses[name].items()
            if joint in self.robot.qadr and joint not in self.robot.mimic
        }

    # ------------------------------ 计划 ------------------------------
    def build_plan(self) -> list[dict]:
        cycle = self.cfg.cycle
        if not cycle.steps:
            raise ValueError("config 里 cycle.steps 为空")
        # 起点 = 初始姿态；每一步只改本步点名的关节，其余保持上一段的值
        #（和 MoveIt 里"只动某个 group"的语义一致）
        current = dict(self.initial)
        steps = []
        for index, step in enumerate(cycle.steps):
            target = dict(current)
            labels = []
            for key, value in step.items():
                if key in ("hold", "seconds"):
                    continue
                if key == "joints":          # 绝对关节角
                    for joint, angle in value.items():
                        if joint not in target:
                            raise KeyError(f"未知关节 '{joint}'（第 {index + 1} 步）")
                        target[joint] = float(angle)
                    labels.append("joints " + " ".join(
                        f"{j}={float(a):+.2f}" for j, a in value.items()))
                elif key == "relative":      # 相对初始姿态的增量
                    for joint, delta in value.items():
                        if joint not in target:
                            raise KeyError(f"未知关节 '{joint}'（第 {index + 1} 步）")
                        target[joint] = self.initial[joint] + float(delta)
                    labels.append("rel " + " ".join(
                        f"{j}{float(d):+.2f}" for j, d in value.items()))
                else:                        # SRDF 命名姿态
                    values = self.pose_values(str(value))
                    target.update(values)
                    labels.append(f"{key}→{value}")
            for joint, angle in target.items():
                lo, hi = self.robot.limits(joint)
                if not lo <= angle <= hi:
                    self.log(f"[警告] 第 {index + 1} 步 {joint}={angle:+.3f} "
                             f"超出 URDF 限位 [{lo:+.3f}, {hi:+.3f}]")
            steps.append({
                "label": f"第{index + 1}步 " + "，".join(labels) if labels else f"第{index + 1}步",
                "from": self.robot.vector(current),
                "to": self.robot.vector(target),
                "seconds": float(step.get("seconds", cycle.segment_seconds)),
                "hold": float(step.get("hold", cycle.pause_seconds)),
            })
            current = target
        return steps

    # ------------------------------ 运行 ------------------------------
    def _pace(self, target_time: float, start: float):
        if self.speed <= 0:
            return
        delay = target_time - (time.monotonic() - start)
        if delay > 0:
            time.sleep(delay)

    def run(self, repeat: int | None = None, headless: bool = False) -> int:
        cfg = self.cfg
        plan = self.build_plan()
        dt = 1.0 / cfg.control.rate_hz
        timeline = build_timeline(plan, dt, cfg.cycle.profile)
        repeat = cfg.cycle.repeat if repeat is None else repeat

        self.log(f"[模型] {self.robot.path.name}：{self.robot.summary()}")
        self.log(f"[计划] 每轮 {len(plan)} 步，{timeline.n_frames} 帧，"
                 f"{timeline.duration:.1f} 秒；重复 "
                 f"{'直到 Ctrl+C' if repeat == 0 else repeat} 轮")
        for item in plan:
            self.log(f"        {item['label']}：{item['seconds']:.1f}s"
                     f"{' + 停顿 %.1fs' % item['hold'] if item['hold'] > 0 else ''}")

        self.servo.start()
        entered = 0
        cycle = 0
        try:
            if headless:
                entered = self._run_headless(timeline, dt, repeat)
            else:
                entered = self._run_viewer(timeline, dt, repeat)
        except KeyboardInterrupt:
            self.log("\n[中断] 收到 Ctrl+C，停止")
        except Exception as exc:  # 模型/渲染/串口线程里的显式异常
            self.log(f"[错误] {type(exc).__name__}: {exc}")
            return 1
        finally:
            self.servo.stop()
            self.log(f"[统计] 完成 {entered} 轮，舵机指令 {self.servo.sent} 条")
        return 0

    def _loop_cycles(self, timeline, dt, repeat, step_frame):
        cycle = 0
        while repeat == 0 or cycle < repeat:
            cycle += 1
            self.log(f"[第 {cycle} 轮] 开始")
            self._step_cursor = 0
            start = time.monotonic()
            for index in range(timeline.n_frames):
                if not step_frame(index, timeline, dt, start):
                    return cycle - 1
                self._check_endpoint(index, timeline)
            if self._step_cursor < len(timeline.step_ends):
                self.log(f"[第 {cycle} 轮] 完成（末段未到终点，可能被中断）")
            else:
                self.log(f"[第 {cycle} 轮] 完成")
        return cycle

    def _check_endpoint(self, index: int, timeline):
        """每段动作结束处校验实际关节角（对应 C++ 的 endpoint_tolerance）。"""
        while (self._step_cursor < len(timeline.step_ends)
               and timeline.step_ends[self._step_cursor] == index):
            target = timeline.step_targets[self._step_cursor]
            measured = self.robot.vector(self.robot.measure({}))
            error = float(np.max(np.abs(measured - target)))
            tolerance = self.cfg.cycle.endpoint_tolerance_rad
            if error > tolerance:
                self.log(f"    [警告] 第 {self._step_cursor + 1} 段终点误差 "
                         f"{error:.4f} rad > {tolerance} rad（物理模式下常见）")
            self._step_cursor += 1

    def _apply(self, q_vector: np.ndarray, dt: float):
        values = self.robot.as_dict(q_vector)
        if self.cfg.control.mode == "physics":
            self.robot.apply_physics(values)
            self.robot.step(dt)
        else:
            self.robot.apply_kinematic(values)
        self._sim_time += dt
        self.servo.update(self.robot.measure(values))
        self.servo.tick(self._sim_time)

    def _run_headless(self, timeline, dt, repeat):
        def step_frame(index, tl, dt_, start):
            self._apply(tl.frames[index], dt_)
            if self.speed > 0:
                self._pace(index * dt_, start)
            if index % max(1, int(1.0 / dt_)) == 0:
                self.log(f"    {tl.labels[index]}  {100.0 * index / tl.n_frames:5.1f}%")
            return True

        return self._loop_cycles(timeline, dt, repeat, step_frame)

    def _run_viewer(self, timeline, dt, repeat):
        try:
            from mujoco import viewer
        except Exception as exc:  # 缺少 GLFW/OpenGL 的机器
            raise RuntimeError(
                f"无法加载 MuJoCo 交互窗口（{exc}）；改用 --headless 无窗口运行") from exc
        model, data = self.robot.model, self.robot.data
        with viewer.launch_passive(model, data) as handle:
            handle.cam.lookat[:] = [0.0, 0.0, 0.22]
            handle.cam.distance = 0.75
            handle.cam.azimuth = 135.0
            handle.cam.elevation = -20.0

            def step_frame(index, tl, dt_, start):
                self._apply(tl.frames[index], dt_)
                handle.sync()
                if not handle.is_running():
                    return False
                if self.speed > 0:
                    self._pace(index * dt_, start)
                return True

            entered = self._loop_cycles(timeline, dt, repeat, step_frame)
        return entered

    # ------------------------------ 静态渲染 ------------------------------
    def render_pose(self, pose_name: str, out_path: str, width: int = 960,
                    height: int = 720):
        values = {name: 0.0 for name in self.robot.driver_joints}
        values.update(self.pose_values(pose_name))
        self.robot.apply_kinematic(values)
        self._render(out_path, width, height)

    def render_state(self, values: dict, out_path: str, width: int = 960,
                     height: int = 720, azimuth: float = 135.0,
                     elevation: float = -18.0):
        """按给定关节角渲染（--start-from-servo 时可用实测姿态出图）。"""
        state = {name: 0.0 for name in self.robot.driver_joints}
        state.update({k: float(v) for k, v in values.items() if k in state})
        self.robot.apply_kinematic(state)
        self._render(out_path, width, height, azimuth, elevation)

    def _render(self, out_path: str, width: int = 960, height: int = 720,
                azimuth: float = 135.0, elevation: float = -18.0):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.robot.model, height, width)
        # 按当前姿态的机身包围盒自动取景，免得换了姿态就被切出画面
        data = self.robot.data
        points = data.xpos[1:] if len(data.xpos) > 1 else data.xpos
        center = points.mean(axis=0)
        extent = float(np.max(np.ptp(points, axis=0))) if len(points) else 0.3
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(self.robot.model, camera)
        camera.lookat[:] = center
        camera.distance = max(0.55, extent * 2.4)
        camera.azimuth = azimuth
        camera.elevation = elevation
        self._renderer.update_scene(data, camera=camera)
        image = self._renderer.render()
        self._save_png(out_path, image)
        self.log(f"[渲染] {out_path}")

    @staticmethod
    def _save_png(path: str, rgb: np.ndarray):
        from pathlib import Path

        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            from PIL import Image

            Image.fromarray(rgb).save(out)
            return
        except ImportError:
            pass
        import struct
        import zlib

        height, width, _ = rgb.shape
        raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))

        def chunk(tag, payload):
            body = tag + payload
            return (struct.pack(">I", len(payload)) + body
                    + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

        out.write_bytes(
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6))
            + chunk(b"IEND", b"")
        )
