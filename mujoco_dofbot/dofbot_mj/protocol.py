#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文本协议与角度映射 —— 与 src/dofbot_servo_bridge/servo_protocol.py 逐条对齐。

固件端（GCWJ/User/main.c）支持的文本命令：
    MOVE  <id> <角度°> <时间ms>    应答 OK MOVE <id> <pos>
    MOVEP <id> <位置0~4095> <ms>   应答 OK MOVEP <id>
    READ  <id>                     应答 POS <id> <pos> / ERR READ <id>
    PING  <id>                     应答 OK PING <id> / ERR PING <id>
    RANGE <角度°>                  应答 OK RANGE <deg>
    SETID <id> / RESET <id> / DEMO / HELP
开机问候：READY STM32F103 SERVO HOST V1.0

注意两点（固件解析决定的）：
  * id 必须是 1~250 的整数；
  * 时间必须是不带小数点的整数（固件用 StrToU16 解析）；
  * 角度可以带小数（固件用 StrToFloat 解析）。
"""

from __future__ import annotations

import math
import time


class ProtocolError(RuntimeError):
    pass


class ServoMapping:
    """关节角(rad) ↔ 舵机角度(°)，越限直接报错而不是静默钳位。"""

    def __init__(self, joints, range_deg: int):
        if not joints:
            raise ValueError("至少需要一个舵机映射")
        if len({j.name for j in joints}) != len(joints):
            raise ValueError("关节名不得重复")
        if len({j.sid for j in joints}) != len(joints):
            raise ValueError("舵机 ID 不得重复")
        if not 1 <= range_deg <= 360:
            raise ValueError("range_deg 必须在 1~360")
        for joint in joints:
            if not 1 <= joint.sid <= 250:
                raise ValueError(f"{joint.name} 的舵机 ID 必须在 1~250：{joint.sid}")
            if joint.direction not in (-1.0, 1.0):
                raise ValueError(f"{joint.name} 的 direction 必须是 ±1：{joint.direction}")
            if joint.scale_deg_per_rad <= 0:
                raise ValueError(f"{joint.name} 的 scale_deg_per_rad 必须大于 0")
            values = (joint.offset_deg, joint.direction, joint.scale_deg_per_rad,
                      joint.min_degrees, joint.max_degrees)
            if not all(math.isfinite(v) for v in values):
                raise ValueError(f"{joint.name} 的标定参数必须是有限数")
            if not 0 <= joint.min_degrees < joint.max_degrees <= range_deg:
                raise ValueError(
                    f"{joint.name} 的角度限制必须在行程内且最小值小于最大值")
        self.joints = tuple(joints)
        self.names = tuple(j.name for j in joints)
        self.ids = tuple(j.sid for j in joints)
        self.range_deg = range_deg

    def targets(self, positions: dict) -> dict:
        """{关节名: 弧度} → {舵机 ID: 角度°}；越限抛 ValueError。"""
        result = {}
        for joint in self.joints:
            rad = positions.get(joint.name)
            if rad is None or not math.isfinite(rad):
                raise ValueError(f"{joint.name} 没有有效的关节角")
            degrees = joint.offset_deg + joint.direction * rad * joint.scale_deg_per_rad
            if not joint.min_degrees <= degrees <= joint.max_degrees:
                raise ValueError(
                    f"{joint.name} 的舵机目标 {degrees:.2f}° 超出 "
                    f"[{joint.min_degrees:.0f}, {joint.max_degrees:.0f}]°")
            result[joint.sid] = degrees
        return result

    def degrees_from_position(self, raw: int) -> float:
        if not 0 <= raw <= 4095:
            raise ValueError("舵机位置读数超出 0~4095")
        return raw * self.range_deg / 4095.0

    def positions_from_targets(self, targets: dict) -> dict:
        """{舵机 ID: 角度°} → {关节名: 弧度}，targets 的逆运算。"""
        result = {}
        for joint in self.joints:
            if joint.sid not in targets:
                continue
            result[joint.name] = (
                (targets[joint.sid] - joint.offset_deg)
                / (joint.direction * joint.scale_deg_per_rad)
            )
        return result


def _line_matches(line: str, expected: str) -> bool:
    """完整 token 边界匹配。

    固件不同版本对同一条命令的应答不一样：本机实测 `MOVE` 只回 `OK MOVE 1`
    （不带位置码），而 main.c 头注释写的是 `OK MOVE <id> <pos>`。所以这里
    允许"期望串后面还跟着别的字段"，但必须在 token 边界上：
      'OK MOVE 1'  能匹配 'OK MOVE 1 2105'
      'OK MOVE 1'  不能匹配 'OK MOVE 12'
    """
    if line == expected:
        return True
    if expected.endswith(" "):          # 老式带尾随空格的期望串
        return line.startswith(expected)
    if line.startswith(expected):
        return line[len(expected):len(expected) + 1] in (" ", "\t")
    return False


def request(ser, command: str, expected: str, timeout: float = 1.5) -> str:
    """发一条命令并等它对应的应答，避免单片机接收环形缓冲被冲爆。

    expected 只写确定的部分（如 `OK MOVE 1`），应答后面多出的字段不影响匹配。
    """
    ser.write((command + "\n").encode("ascii"))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        if _line_matches(line, expected):
            return line
        if line.startswith("ERR"):
            raise ProtocolError(f"{command}: {line}")
        if line.startswith("READY"):
            raise ProtocolError("STM32 已重新启动，需要重新检查舵机 ID")
    raise ProtocolError(f"{command}: 等待 {expected!r} 超时")


def read_position(ser, servo_id: int, retries: int = 3, retry_delay: float = 0.05) -> int:
    """读舵机位置码。半双工总线上偶发 ERR/超时，默认重试 3 次。"""
    last_error = None
    attempts = max(1, int(retries))
    for attempt in range(attempts):
        try:
            line = request(ser, f"READ {servo_id}", f"POS {servo_id}")
            parts = line.split()
            return int(parts[2])
        except ProtocolError as exc:
            last_error = exc
            if attempt + 1 < attempts:
                time.sleep(retry_delay)
        except (IndexError, ValueError) as exc:
            last_error = ProtocolError(f"无效的位置应答: {exc}")
            if attempt + 1 < attempts:
                time.sleep(retry_delay)
    raise last_error if last_error else ProtocolError("READ 失败")
