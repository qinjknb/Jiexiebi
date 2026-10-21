"""STM32 <-> ROS 2 通信协议（与固件 robot_link.h 一一对应）。

帧格式（8N1，默认 115200）：

    主机 -> STM32 : AA 55 | CMD     | SEQ | LEN | PAYLOAD[LEN] | CRC_H CRC_L
    STM32 -> 主机 : AA 55 | CMD|0x80 | SEQ | LEN | PAYLOAD[LEN] | CRC_H CRC_L

CRC = CRC16-CCITT（初值 0xFFFF，poly 0x1021），对 CMD/SEQ/LEN/PAYLOAD 计算，
高字节先发。多字节数据一律大端（高字节在前）。

本模块不依赖 ROS，也不依赖 pyserial，可单独跑：
    python3 protocol.py
"""

from __future__ import annotations

import struct

SOH0 = 0xAA
SOH1 = 0x55
REPLY_FLAG = 0x80

# ---- 主机 -> STM32 ----
CMD_PING = 0x01
CMD_SET_TARGETS = 0x02
CMD_SET_MODE = 0x03
CMD_HOME = 0x04
CMD_SET_LIMITS = 0x05
CMD_READ_POS = 0x06
CMD_SET_TELEM = 0x07
CMD_STOP = 0x08
CMD_GET_STATUS = 0x09
CMD_SET_BUSMODE = 0x0A

# ---- STM32 -> 主机 ----
RSP_PONG = 0x81
RSP_ACK = 0x82
RSP_STATE = 0x83
RSP_POS = 0x84

# ---- ACK 结果码 ----
RES_OK = 0
RES_UNKNOWN_CMD = 1
RES_BAD_LEN = 2
RES_BUSY = 3
RES_RANGE = 4

RES_TEXT = {
    RES_OK: "OK",
    RES_UNKNOWN_CMD: "未知指令",
    RES_BAD_LEN: "长度错误",
    RES_BUSY: "忙/正在跟随轨迹",
    RES_RANGE: "参数超范围",
}

# ---- 运行模式 ----
MODE_IDLE = 0
MODE_STREAM = 1
MODE_HOME = 2

MODE_TEXT = {MODE_IDLE: "IDLE", MODE_STREAM: "STREAM", MODE_HOME: "HOME"}

# ---- 状态标志（STATE 帧第 1 字节）----
FLAG_STREAM = 0x01
FLAG_TIMEOUT = 0x02
FLAG_LIMIT = 0x04
FLAG_READBACK_ERR = 0x08

UNKNOWN_TICK = 0xFFFF
TICK_MAX = 4095


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT（同固件 Link_Crc16Update 的初始值/更新方式）。"""
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, payload: bytes = b"", seq: int = 0) -> bytes:
    """组一帧：AA 55 CMD SEQ LEN PAYLOAD CRC_H CRC_L"""
    if len(payload) > 32:
        raise ValueError("payload too long (max 32)")
    body = bytes([cmd & 0xFF, seq & 0xFF, len(payload)]) + payload
    c = crc16(body)
    return bytes([SOH0, SOH1]) + body + bytes([(c >> 8) & 0xFF, c & 0xFF])


class FrameParser:
    """字节流 -> 帧列表。容忍开机文本、噪声、半帧。"""

    MAX_PAYLOAD = 32

    def __init__(self) -> None:
        self._buf = bytearray()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes):
        """喂入字节，逐个 yield (cmd, seq, payload)。"""
        self._buf.extend(data)
        buf = self._buf
        while True:
            # 找帧头
            start = -1
            for i in range(len(buf) - 1):
                if buf[i] == SOH0 and buf[i + 1] == SOH1:
                    start = i
                    break
            if start < 0:
                # 只保留可能是帧头的一半
                if buf and buf[-1] == SOH0:
                    del buf[:-1]
                else:
                    buf.clear()
                return
            if start > 0:
                del buf[:start]

            if len(buf) < 5:
                return
            cmd = buf[2]
            seq = buf[3]
            length = buf[4]
            if length > self.MAX_PAYLOAD:
                del buf[:2]
                continue
            total = 5 + length + 2
            if len(buf) < total:
                return

            body = bytes(buf[2:5 + length])
            crc_rx = (buf[5 + length] << 8) | buf[6 + length]
            if crc16(body) != crc_rx:
                del buf[:2]           # 校验错：跳过这个帧头重新同步
                continue

            payload = bytes(buf[5:5 + length])
            del buf[:total]
            yield cmd, seq, payload


# ---------------------------------------------------------------------------
# 负载编解码
# ---------------------------------------------------------------------------
def enc_targets(ticks) -> bytes:
    """6 个目标位置 -> 12 字节（大端 int16）。"""
    if len(ticks) != 6:
        raise ValueError("need 6 ticks")
    return struct.pack(">6H", *[max(0, min(TICK_MAX, int(t))) for t in ticks])


def dec_targets(payload: bytes):
    if len(payload) != 12:
        raise ValueError("bad STATE/POS payload length")
    return list(struct.unpack(">6H", payload))


def enc_u8(v: int) -> bytes:
    return bytes([v & 0xFF])


def enc_u16(v: int) -> bytes:
    return struct.pack(">H", int(v) & 0xFFFF)


def dec_pong(payload: bytes):
    """-> dict(ver_major, ver_minor, njoint, busmode)"""
    if len(payload) < 4:
        raise ValueError("bad PONG payload")
    return {
        "ver_major": payload[0],
        "ver_minor": payload[1],
        "njoint": payload[2],
        "busmode": payload[3],
    }


def dec_ack(payload: bytes):
    """-> (cmd, result)"""
    if len(payload) != 2:
        raise ValueError("bad ACK payload")
    return payload[0], payload[1]


def dec_state(payload: bytes):
    """-> dict(flags, mode, cmd[6], meas[6])"""
    if len(payload) != 26:
        raise ValueError("bad STATE payload")
    flags = payload[0]
    mode = payload[1]
    cmd = list(struct.unpack(">6H", payload[2:14]))
    meas = list(struct.unpack(">6H", payload[14:26]))
    return {"flags": flags, "mode": mode, "cmd": cmd, "meas": meas}


def flags_text(flags: int) -> str:
    out = []
    if flags & FLAG_STREAM:
        out.append("STREAM")
    if flags & FLAG_TIMEOUT:
        out.append("TIMEOUT")
    if flags & FLAG_LIMIT:
        out.append("LIMIT")
    if flags & FLAG_READBACK_ERR:
        out.append("READ_ERR")
    return ",".join(out) if out else "-"


# ---------------------------------------------------------------------------
# 自测：python3 protocol.py
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # 1) CRC 已知值
    assert crc16(b"123456789") == 0x29B1, hex(crc16(b"123456789"))

    # 2) 固件文档里的 SYNC WRITE 帧结构核对（位置 2000 / 时间 1000）
    assert struct.pack(">H", 2000) == b"\x07\xd0"
    assert struct.pack(">H", 1000) == b"\x03\xe8"

    # 3) 收发回环
    frames = [
        (CMD_PING, b""),
        (CMD_SET_TARGETS, enc_targets([2047, 1500, 3000, 4095, 0, 1234])),
        (CMD_SET_MODE, enc_u8(MODE_STREAM)),
        (CMD_HOME, enc_u16(2000)),
        (CMD_SET_TELEM, enc_u8(25)),
    ]
    blob = b"\r\nREADY STM32F103\r\n" + b"".join(build_frame(c, p, i) for i, (c, p) in enumerate(frames))
    got = list(FrameParser().feed(blob))
    assert len(got) == len(frames), got
    for (cmd, seq, payload), (ecmd, epayload) in zip(got, frames):
        assert cmd == ecmd and payload == epayload

    # 4) 坏帧不影响后续好帧
    bad = bytearray(build_frame(CMD_PING, b"", 7))
    bad[-1] ^= 0xFF
    got = list(FrameParser().feed(bytes(bad) + build_frame(CMD_PING, b"", 8)))
    assert len(got) == 1 and got[0][1] == 8, got

    # 5) 分包喂入
    p = FrameParser()
    data = build_frame(CMD_SET_TARGETS, enc_targets([1, 2, 3, 4, 5, 6]), 9)
    out = []
    for i in range(len(data)):
        out.extend(p.feed(data[i:i + 1]))
    assert len(out) == 1 and dec_targets(out[0][2]) == [1, 2, 3, 4, 5, 6]

    # 6) STATE 编解码
    st = bytes([FLAG_STREAM, MODE_STREAM]) + struct.pack(">6H", 100, 200, 300, 400, 500, 600) \
         + struct.pack(">6H", 101, 202, 303, 404, 505, 0xFFFF)
    d = dec_state(st)
    assert d["cmd"][5] == 600 and d["meas"][5] == UNKNOWN_TICK and d["mode"] == MODE_STREAM

    print("protocol.py 自测全部通过")
