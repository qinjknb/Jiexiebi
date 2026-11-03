#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 config/default.yaml，路径统一按工程根目录解析。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


DEFAULT_SCALE = 180.0 / math.pi


@dataclass
class ModelConfig:
    mjcf: Path
    srdf: Path


@dataclass
class ControlConfig:
    mode: str = "kinematic"
    rate_hz: float = 50.0


@dataclass
class CycleConfig:
    steps: list = field(default_factory=list)
    repeat: int = 3
    segment_seconds: float = 6.0
    pause_seconds: float = 0.5
    profile: str = "cosine"
    endpoint_tolerance_rad: float = 0.01


@dataclass
class ServoJoint:
    name: str
    sid: int
    offset_deg: float = 150.0
    direction: float = 1.0
    scale_deg_per_rad: float = DEFAULT_SCALE
    min_degrees: float = 0.0
    max_degrees: float = 300.0


@dataclass
class ServoConfig:
    enabled: bool = True
    port: str = "COM9"
    baudrate: int = 115200
    test_mode: bool = True
    enable_motion: bool = True
    calibration_confirmed: bool = False
    range_deg: int = 300
    max_rate_hz: float = 10.0
    move_time_ms: int = 120
    deadband_deg: float = 0.2
    state_timeout_sec: float = 0.5
    start_tolerance_deg: float = 10.0
    joints: list[ServoJoint] = field(default_factory=list)


@dataclass
class Settings:
    root: Path
    model: ModelConfig
    control: ControlConfig
    cycle: CycleConfig
    servo: ServoConfig


def _resolve(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (root / path).resolve()


def load_config(path: str | Path = "config/default.yaml") -> Settings:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        config_path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"
    config_path = config_path.resolve()
    root = config_path.parent.parent
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}

    model_raw = raw.get("model", {})
    model = ModelConfig(
        mjcf=_resolve(root, model_raw.get("mjcf", "models/dofbot_pro.xml")),
        srdf=_resolve(root, model_raw.get("srdf", "../src/My_JXB_moveit/config/DOFBOT_Pro-V24.srdf")),
    )

    control_raw = raw.get("control", {})
    control = ControlConfig(
        mode=str(control_raw.get("mode", "kinematic")),
        rate_hz=float(control_raw.get("rate_hz", 50.0)),
    )

    cycle_raw = raw.get("cycle", {})
    cycle = CycleConfig(
        steps=list(cycle_raw.get("steps", [])),
        repeat=int(cycle_raw.get("repeat", 3)),
        segment_seconds=float(cycle_raw.get("segment_seconds", 6.0)),
        pause_seconds=float(cycle_raw.get("pause_seconds", 0.5)),
        profile=str(cycle_raw.get("profile", "cosine")),
        endpoint_tolerance_rad=float(cycle_raw.get("endpoint_tolerance_rad", 0.01)),
    )

    servo_raw = raw.get("servo", {})
    joints = []
    for entry in servo_raw.get("joints", []):
        joints.append(
            ServoJoint(
                name=str(entry["name"]),
                sid=int(entry["id"]),
                offset_deg=float(entry.get("offset_deg", 150.0)),
                direction=float(entry.get("direction", 1.0)),
                scale_deg_per_rad=float(entry.get("scale_deg_per_rad", DEFAULT_SCALE)),
                min_degrees=float(entry.get("min_degrees", 0.0)),
                max_degrees=float(entry.get("max_degrees", servo_raw.get("range_deg", 300))),
            )
        )
    servo = ServoConfig(
        enabled=bool(servo_raw.get("enabled", True)),
        port=str(servo_raw.get("port", "COM9")),
        baudrate=int(servo_raw.get("baudrate", 115200)),
        test_mode=bool(servo_raw.get("test_mode", True)),
        enable_motion=bool(servo_raw.get("enable_motion", True)),
        calibration_confirmed=bool(servo_raw.get("calibration_confirmed", False)),
        range_deg=int(servo_raw.get("range_deg", 300)),
        max_rate_hz=float(servo_raw.get("max_rate_hz", 10.0)),
        move_time_ms=int(servo_raw.get("move_time_ms", 120)),
        deadband_deg=float(servo_raw.get("deadband_deg", 0.2)),
        state_timeout_sec=float(servo_raw.get("state_timeout_sec", 0.5)),
        start_tolerance_deg=float(servo_raw.get("start_tolerance_deg", 10.0)),
        joints=joints,
    )

    if not 1 <= servo.range_deg <= 360:
        raise ValueError(f"range_deg 必须在 1~360：{servo.range_deg}")
    if not 1 <= servo.move_time_ms <= 65535:
        raise ValueError(f"move_time_ms 必须在 1~65535：{servo.move_time_ms}")
    if servo.max_rate_hz <= 0:
        raise ValueError("max_rate_hz 必须为正数")
    if control.mode not in ("kinematic", "physics"):
        raise ValueError(f"control.mode 只能是 kinematic/physics：{control.mode}")
    if cycle.profile not in ("cosine", "linear"):
        raise ValueError(f"cycle.profile 只能是 cosine/linear：{cycle.profile}")

    return Settings(root=root, model=model, control=control, cycle=cycle, servo=servo)
