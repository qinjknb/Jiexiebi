"""Mapping and line protocol for the unmodified GCWJ USART2 firmware."""

import math
import time


JOINT_NAMES = (
    "Arm1_Joint", "Arm2_Joint", "Arm3_Joint",
    "Arm4_Joint", "Arm5_Joint", "grip_joint",
)


class ServoMapping:
    def __init__(self, names, ids, offsets, directions, scales, minimums, maximums, range_deg):
        arrays = (names, ids, offsets, directions, scales, minimums, maximums)
        if not names or any(len(a) != len(names) for a in arrays):
            raise ValueError("joint_names、servo_ids 和全部标定数组必须等长且非空")
        if len(set(names)) != len(names) or len(set(ids)) != len(ids):
            raise ValueError("关节名和舵机 ID 不得重复")
        if any(not 1 <= sid <= 250 for sid in ids):
            raise ValueError("舵机 ID 必须在 1～250")
        if not 1 <= range_deg <= 360:
            raise ValueError("range_deg 必须在 1～360")
        for offset, direction, scale, minimum, maximum in zip(
            offsets, directions, scales, minimums, maximums
        ):
            if not all(map(math.isfinite, (offset, direction, scale, minimum, maximum))):
                raise ValueError("标定参数必须是有限数")
            if direction not in (-1.0, 1.0) or scale <= 0:
                raise ValueError("direction 必须为 ±1，scale 必须大于 0")
            if not 0 <= minimum < maximum <= range_deg:
                raise ValueError("舵机角度限制必须在行程内且最小值小于最大值")
        self.names = tuple(names)
        self.ids = tuple(ids)
        self.rows = tuple(zip(ids, offsets, directions, scales, minimums, maximums))
        self.range_deg = range_deg

    def targets(self, positions):
        """Return id -> degrees. Reject invalid/out-of-range targets; never silently clamp."""
        result = {}
        for name, (sid, offset, direction, scale, minimum, maximum) in zip(
            self.names, self.rows
        ):
            rad = positions[name]
            if not math.isfinite(rad):
                raise ValueError(f"{name} 的关节角不是有限数")
            degrees = offset + direction * rad * scale
            if not minimum <= degrees <= maximum:
                raise ValueError(
                    f"{name} 的舵机目标 {degrees:.2f}° 超出 [{minimum}, {maximum}]°"
                )
            result[sid] = degrees
        return result

    def degrees_from_position(self, raw):
        if not 0 <= raw <= 4095:
            raise ValueError("舵机位置读数超出 0～4095")
        return raw * self.range_deg / 4095.0


class ProtocolError(RuntimeError):
    pass


def request(ser, command, expected_prefix, timeout=1.5):
    """Send one command and wait for its matching line, so MCU RX ring cannot overflow."""
    ser.write((command + "\n").encode("ascii"))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if (line.startswith(expected_prefix) if expected_prefix.endswith(" ")
                else line == expected_prefix):
            return line
        if line.startswith("ERR"):
            raise ProtocolError(f"{command}: {line}")
        if line.startswith("READY"):
            raise ProtocolError("STM32 已重新启动，需重新检查舵机 ID")
    raise ProtocolError(f"{command}: 等待 {expected_prefix!r} 超时")


def read_position(ser, servo_id):
    line = request(ser, f"READ {servo_id}", f"POS {servo_id} ")
    try:
        return int(line.split()[2])
    except (IndexError, ValueError) as exc:
        raise ProtocolError(f"无效的位置应答: {line}") from exc
