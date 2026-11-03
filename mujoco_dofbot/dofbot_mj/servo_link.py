#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真 → 实物舵机桥（Windows 版，不经过 ROS）。

对应 Ubuntu 上的 dofbot_servo_bridge 节点，逻辑逐条移植：
  * 订阅者角色由 update() 承担——仿真每个控制周期把关节角喂进来；
  * live 模式用独立线程按 max_rate_hz 取最新状态，死区过滤后发 MOVE；
  * dry 模式由仿真循环调 tick() 驱动，节拍按仿真时间走，--speed 0 也准；
  * 三道闸：test_mode（只打印）、enable_motion（允许动）、
    calibration_confirmed（确认真机已标定）；
  * 连接后先 RANGE 同步满行程、PING 每个 ID，再做「起点对齐」检查：
    实机读数和仿真目标差超过 start_tolerance_deg 就拒绝运动，
    避免上一次停在别处时一上电就猛地拽过去。
"""

from __future__ import annotations

import threading
import time
from datetime import datetime

from .config import ServoConfig
from .protocol import ProtocolError, ServoMapping, read_position, request

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # 未安装 pyserial：test_mode 与 --servo off 仍可用
    serial = None


def _stamp() -> str:
    return datetime.now().strftime("%H:%M:%S")


def detect_port() -> str:
    """自动找 CH340（DOFBOT 用的 USB-TTL 一般是它）。"""
    if serial is None:
        return ""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "CH340" in (port.description or "").upper():
            return port.device
    return ports[0].device if ports else ""


def probe_positions(cfg: ServoConfig, log=print):
    """只读探测：打开串口做 RANGE + PING + READ，不发任何运动指令。

    返回 (rad_positions, deg_targets)；任一步失败抛异常，由调用方决定怎么办。
    """
    if serial is None:
        raise RuntimeError("未安装 pyserial")
    mapping = ServoMapping(cfg.joints, cfg.range_deg)
    port = cfg.port or detect_port()
    if not port:
        raise RuntimeError("找不到串口，请用 --port COM9 指定")
    with serial.Serial(port, cfg.baudrate, timeout=0.2, write_timeout=1.0) as ser:
        ser.reset_input_buffer()
        request(ser, f"RANGE {mapping.range_deg}",
                f"OK RANGE {mapping.range_deg}", timeout=8.0)
        for sid in mapping.ids:
            # PING 在总线上也会偶发失败，重试几次再判定
            last = None
            for _ in range(4):
                try:
                    request(ser, f"PING {sid}", f"OK PING {sid}")
                    last = None
                    break
                except ProtocolError as exc:
                    last = exc
                    time.sleep(0.06)
            if last is not None:
                raise last
        targets = {}
        for sid in mapping.ids:
            degrees = mapping.degrees_from_position(read_position(ser, sid))
            targets[sid] = degrees
            log(f"   ID{sid:<2} 位置 {degrees:7.2f}°")
    return mapping.positions_from_targets(targets), targets


class ServoLink:
    def __init__(self, cfg: ServoConfig, log=print):
        self.cfg = cfg
        self.log = log
        self.mapping = ServoMapping(cfg.joints, cfg.range_deg)
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._state_at = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_sent: dict[int, float] = {}
        self._last_tick = 0.0
        self.sent = 0
        self.aligned = False
        self.failed_reason = ""
        if serial is None and not cfg.test_mode:
            self.failed_reason = "未安装 pyserial（pip install pyserial）"

    # ------------------------------ 生命周期 ------------------------------
    def start(self):
        if not self.cfg.enabled:
            self.log("[舵机] 已禁用（--servo off），不打开串口")
            return
        if not self.cfg.enable_motion:
            self.log("[舵机] enable_motion=false，只登记不发送")
            return
        if self.failed_reason:
            self.log(f"[舵机] 不可用：{self.failed_reason}")
            return
        if self.cfg.test_mode:
            # 试运行不走线程：由仿真循环调 tick() 驱动，这样即使 --speed 0
            # 全速跑，命令流也与仿真时间轴严格对齐。
            self.aligned = True
            self.log("[舵机] 试运行：只打印 MOVE，不打开串口")
            return
        self._thread = threading.Thread(target=self._run, name="servo-link", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def update(self, positions: dict):
        """仿真侧每个控制周期调用，喂入最新的关节角（弧度）。"""
        with self._lock:
            self._positions = dict(positions)
            self._state_at = time.monotonic()

    def tick(self, sim_time: float | None = None):
        """dry 模式的输出节拍，由仿真循环调用（live 模式由串口线程自己跑）。

        节拍按「仿真时间」算而不是墙钟：这样 --speed 0 全速自检时命令流也
        和实时播放完全一致，便于对比日志。
        """
        cfg = self.cfg
        if not cfg.enabled or not cfg.enable_motion or not cfg.test_mode:
            return
        now = time.monotonic() if sim_time is None else float(sim_time)
        if now - self._last_tick < 1.0 / cfg.max_rate_hz:
            return
        self._last_tick = now
        targets = self._targets()
        if targets is None:
            return
        parts = []
        for sid, degrees in targets.items():
            if sid not in self._last_sent or abs(degrees - self._last_sent[sid]) >= cfg.deadband_deg:
                parts.append(f"MOVE {sid} {degrees:.2f} {cfg.move_time_ms}")
                self._last_sent[sid] = degrees
                self.sent += 1
        if parts:
            self.log(f"[{_stamp()}] [DRY] " + " | ".join(parts))

    # ------------------------------ 内部 ----------------------------------
    def _targets(self):
        with self._lock:
            positions = dict(self._positions)
            stamp = self._state_at
        if time.monotonic() - stamp > self.cfg.state_timeout_sec:
            return None
        try:
            return self.mapping.targets(positions)
        except ValueError as exc:
            self.log(f"[舵机] {exc}")
            return None

    def _run(self):
        if not self.cfg.calibration_confirmed:
            self.log("[舵机] 实机运动被阻止：确认六轴方向与零点后把 "
                     "calibration_confirmed 设为 true（或加 --calibration-confirmed）")
            return
        while not self._stop.is_set():
            try:
                with serial.Serial(self.cfg.port, self.cfg.baudrate,
                                   timeout=0.1, write_timeout=1.0) as ser:
                    self.log(f"[舵机] 串口 {self.cfg.port} @ {self.cfg.baudrate} 已打开")
                    self._connect_and_mirror(ser)
            except (OSError, ValueError, ProtocolError) as exc:
                if not self._stop.is_set():
                    self.log(f"[舵机] 串口/舵机异常：{exc}；3 秒后重试")
            self._stop.wait(3.0)

    def _connect_and_mirror(self, ser):
        ser.reset_input_buffer()
        request(ser, f"RANGE {self.mapping.range_deg}",
                f"OK RANGE {self.mapping.range_deg}", timeout=8.0)
        for sid in self.mapping.ids:
            request(ser, f"PING {sid}", f"OK PING {sid}")
        self.log(f"[舵机] {len(self.mapping.ids)} 个 ID 均已应答，等待仿真状态对齐")
        period = 1.0 / self.cfg.max_rate_hz
        self.aligned = False
        while not self._stop.is_set():
            started = time.monotonic()
            targets = self._targets()
            if targets is None:
                self._stop.wait(min(period, 0.1))
                continue
            if not self.aligned:
                actual = {
                    sid: self.mapping.degrees_from_position(read_position(ser, sid))
                    for sid in self.mapping.ids
                }
                mismatch = [
                    f"ID{sid} 实机{actual[sid]:.1f}° / 仿真{targets[sid]:.1f}°"
                    for sid in self.mapping.ids
                    if abs(actual[sid] - targets[sid]) > self.cfg.start_tolerance_deg
                ]
                if mismatch:
                    self.log("[舵机] 起点未对齐，暂不运动：" + "，".join(mismatch))
                    self._stop.wait(1.0)
                    continue
                self.aligned = True
                self.log("[舵机] 起点已对齐，开始跟随仿真")
            for sid, degrees in targets.items():
                if self._stop.is_set():
                    return
                if sid in self._last_sent and abs(degrees - self._last_sent[sid]) < self.cfg.deadband_deg:
                    continue
                request(ser, f"MOVE {sid} {degrees:.2f} {self.cfg.move_time_ms}",
                        f"OK MOVE {sid}")
                self._last_sent[sid] = degrees
                self.sent += 1
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        self.log("[舵机] 串口线程退出")
