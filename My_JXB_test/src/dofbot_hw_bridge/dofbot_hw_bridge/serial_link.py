"""串口链路层：帧收发、超时、后台读线程，以及一个"假 STM32"用于离线自测。

依赖 pyserial（只在真正开串口时才 import，"假 STM32"模式不需要）。
"""

from __future__ import annotations

import queue
import threading
import time

from . import protocol as P


class LinkError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# 传输层
# ---------------------------------------------------------------------------
class Transport:
    """只要求三个方法：write / read / close。"""

    def write(self, data: bytes) -> None:      # pragma: no cover - 抽象
        raise NotImplementedError

    def read(self, n: int, timeout: float) -> bytes:   # pragma: no cover - 抽象
        raise NotImplementedError

    def close(self) -> None:                   # pragma: no cover - 抽象
        pass


class PySerialTransport(Transport):
    """真实串口（USB-TTL -> STM32）。"""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # 延迟导入：没装 pyserial 也能跑自测
        self._ser = serial.serial_for_url(port, baudrate=baud, timeout=0.02,
                                          write_timeout=0.5)
        self.port = port
        self._ser.reset_input_buffer()

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def read(self, n: int, timeout: float) -> bytes:
        self._ser.timeout = max(0.001, timeout)
        return self._ser.read(n)

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


class FakeMcuTransport(Transport):
    """内置"假 STM32"：按固件逻辑回应帧，用于没有硬件时验证整条链路。

    用法：SerialLink(FakeMcuTransport(), ...) 即可，行为与真实固件一致：
      * 上电先发一行文本问候
      * PING -> PONG；SET_* -> ACK；READ_POS -> POS
      * 收到 SET_TARGETS 后按 telem 频率回 STATE
    """

    def __init__(self, n_joints: int = 6, boot_banner: bool = True):
        self._parser = P.FrameParser()
        self._rx = bytearray()
        self._lock = threading.Lock()
        self._n = n_joints
        self._mode = P.MODE_IDLE
        self._cmd = [2047] * n_joints
        self._meas = [2047] * n_joints
        self._telem_hz = 25
        self._telem_last = 0.0
        self._last_target = 0.0
        self.link_timeout = 0.4
        self._tx_seq = 0
        self._frames_rx = 0
        self._sync_writes = 0
        if boot_banner:
            with self._lock:
                self._rx.extend(b"\r\nREADY STM32F103 ARM BRIDGE V2.0 (FAKE)\r\n")

    # ---- Transport 接口 ----
    def write(self, data: bytes) -> None:
        for cmd, seq, payload in self._parser.feed(data):
            self._frames_rx += 1
            self._handle(cmd, seq, payload)

    def read(self, n: int, timeout: float) -> bytes:
        time.sleep(min(0.01, max(0.001, timeout / 4.0)))
        self._maybe_telemetry()
        with self._lock:
            if not self._rx:
                return b""
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def close(self) -> None:
        pass

    # ---- 内部 ----
    def _push(self, cmd: int, payload: bytes, seq: int = None) -> None:
        if seq is None:
            self._tx_seq = (self._tx_seq + 1) & 0xFF
            seq = self._tx_seq
        with self._lock:
            self._rx.extend(P.build_frame(cmd, payload, seq))

    def _ack(self, seq: int, cmd: int, result: int = P.RES_OK) -> None:
        self._push(P.RSP_ACK, bytes([cmd, result]), seq)

    def _maybe_telemetry(self) -> None:
        now = time.monotonic()
        # 与固件一致：跟随时超过 link_timeout 没有新目标 -> 自动回 IDLE 保持
        if (self._mode == P.MODE_STREAM
                and now - self._last_target > self.link_timeout):
            self._mode = P.MODE_IDLE
        if self._telem_hz <= 0:
            return
        if now - self._telem_last < 1.0 / self._telem_hz:
            return
        self._telem_last = now
        flags = P.FLAG_STREAM if self._mode == P.MODE_STREAM else 0
        payload = bytes([flags, self._mode]) + P.enc_targets(self._cmd) \
            + P.enc_targets(self._meas)
        self._push(P.RSP_STATE, payload)

    def _handle(self, cmd: int, seq: int, payload: bytes) -> None:
        if cmd == P.CMD_PING:
            self._push(P.RSP_PONG, bytes([2, 0, self._n, 0]), seq)
        elif cmd == P.CMD_SET_TARGETS:
            if len(payload) != 12:
                self._ack(seq, cmd, P.RES_BAD_LEN)
                return
            self._last_target = time.monotonic()
            if self._mode == P.MODE_STREAM:        # 与固件一致：只有跟随模式才动
                self._cmd = P.dec_targets(payload)
                self._meas = list(self._cmd)      # 假舵机：立刻到位
                self._sync_writes += 1
            self._ack(seq, cmd)
        elif cmd == P.CMD_SET_MODE:
            if len(payload) != 1:
                self._ack(seq, cmd, P.RES_BAD_LEN)
                return
            if payload[0] != self._mode:
                self._last_target = time.monotonic()   # 与固件一致：重新计时
            self._mode = payload[0]
            self._ack(seq, cmd)
        elif cmd == P.CMD_HOME:
            self._mode = P.MODE_HOME
            self._ack(seq, cmd)
        elif cmd == P.CMD_SET_LIMITS:
            self._ack(seq, cmd, P.RES_OK if len(payload) == 24 else P.RES_BAD_LEN)
        elif cmd == P.CMD_READ_POS:
            self._push(P.RSP_POS, P.enc_targets(self._meas))
        elif cmd == P.CMD_SET_TELEM:
            if len(payload) != 1:
                self._ack(seq, cmd, P.RES_BAD_LEN)
                return
            self._telem_hz = payload[0]
            self._ack(seq, cmd)
        elif cmd == P.CMD_STOP:
            self._mode = P.MODE_IDLE
            self._ack(seq, cmd)
        elif cmd == P.CMD_GET_STATUS:
            flags = P.FLAG_STREAM if self._mode == P.MODE_STREAM else 0
            self._push(P.RSP_STATE, bytes([flags, self._mode])
                       + P.enc_targets(self._cmd) + P.enc_targets(self._meas))
        elif cmd == P.CMD_SET_BUSMODE:
            self._ack(seq, cmd)
        else:
            self._ack(seq, cmd, P.RES_UNKNOWN_CMD)

    # ---- 自测用统计 ----
    @property
    def frames_rx(self) -> int:
        return self._frames_rx

    @property
    def sync_writes(self) -> int:
        return self._sync_writes

    @property
    def mode(self) -> int:
        return self._mode


# ---------------------------------------------------------------------------
# 链路
# ---------------------------------------------------------------------------
class SerialLink:
    """把 Transport 变成"发一帧 / 等一帧"的同步接口。"""

    def __init__(self, transport: Transport, logger=None):
        self._t = transport
        self._log = logger
        self._parser = P.FrameParser()
        self._pending = {}
        self._lock = threading.Lock()
        self.state_queue = queue.Queue(maxsize=200)
        self.latest_state = None
        self.latest_pos = None
        self._pos_event = threading.Event()
        self.rx_frames = 0
        self.error = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread = None

    # ---- 生命周期 ----
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="dofbot_link",
                                        daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._t.close()

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._t.read(256, 0.02)
            except Exception as exc:                      # 串口被拔等
                self.error = exc
                time.sleep(0.1)
                continue
            if not data:
                continue
            for cmd, seq, payload in self._parser.feed(data):
                self._dispatch(cmd, seq, payload)

    def _dispatch(self, cmd: int, seq: int, payload: bytes) -> None:
        self.rx_frames += 1
        if cmd == P.RSP_STATE:
            try:
                st = P.dec_state(payload)
            except ValueError:
                return
            self.latest_state = st
            if self.state_queue.full():
                try:
                    self.state_queue.get_nowait()
                except queue.Empty:
                    pass
            self.state_queue.put(st)
            return
        if cmd == P.RSP_POS:
            self.latest_pos = payload
            self._pos_event.set()
        with self._lock:
            slot = self._pending.get((cmd, seq))
        if slot is not None:
            ev, holder = slot
            holder["payload"] = payload
            ev.set()

    # ---- 收发 ----
    def next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def send(self, cmd: int, payload: bytes = b"", seq: int = None) -> int:
        if seq is None:
            seq = self.next_seq()
        self._t.write(P.build_frame(cmd, payload, seq))
        return seq

    def request(self, cmd: int, payload: bytes, rsp: int, timeout: float = 0.5,
                seq: int = None) -> bytes:
        if seq is None:
            seq = self.next_seq()
        ev = threading.Event()
        holder = {}
        with self._lock:
            self._pending[(rsp, seq)] = (ev, holder)
        try:
            self._t.write(P.build_frame(cmd, payload, seq))
            if not ev.wait(timeout):
                raise LinkError(f"等待应答超时 cmd=0x{cmd:02X} rsp=0x{rsp:02X} seq={seq}")
            return holder.get("payload", b"")
        finally:
            with self._lock:
                self._pending.pop((rsp, seq), None)

    # ---- 高层封装（全部是"发一帧等一帧"）----
    def ping(self, timeout: float = 0.5) -> dict:
        pl = self.request(P.CMD_PING, b"", P.RSP_PONG, timeout)
        return P.dec_pong(pl)

    def set_mode(self, mode: int, timeout: float = 0.5) -> None:
        self._check_ack(P.CMD_SET_MODE, P.enc_u8(mode), timeout)

    def set_targets(self, ticks, timeout: float = 0.5) -> None:
        self._check_ack(P.CMD_SET_TARGETS, P.enc_targets(ticks), timeout)

    def send_targets(self, ticks) -> None:
        """不等应答的快速下发（控制循环里用，省一次往返）。"""
        self.send(P.CMD_SET_TARGETS, P.enc_targets(ticks))

    def home(self, time_ms: int = 2000, timeout: float = 0.5) -> None:
        self._check_ack(P.CMD_HOME, P.enc_u16(time_ms), timeout)

    def stop_motion(self, timeout: float = 0.5) -> None:
        self._check_ack(P.CMD_STOP, b"", timeout)

    def set_limits(self, tick_min, tick_max, timeout: float = 0.5) -> None:
        payload = b"".join(P.enc_u16(v) for v in tick_min) + \
                  b"".join(P.enc_u16(v) for v in tick_max)
        self._check_ack(P.CMD_SET_LIMITS, payload, timeout)

    def set_telemetry(self, hz: int, timeout: float = 0.5) -> None:
        self._check_ack(P.CMD_SET_TELEM, P.enc_u8(hz), timeout)

    def set_busmode(self, mode: int, timeout: float = 0.5) -> None:
        self._check_ack(P.CMD_SET_BUSMODE, P.enc_u8(mode), timeout)

    def read_pos(self, timeout: float = 0.5):
        """读 6 个舵机的真实位置。

        固件的 POS/STATE 应答用固件自己的自增序号（不像 ACK 那样回显 seq），
        所以这里用事件等待而不是按 seq 匹配，避免收到上一条残留应答。
        """
        self._pos_event.clear()
        self.send(P.CMD_READ_POS, b"")
        if not self._pos_event.wait(timeout):
            raise LinkError("读取舵机位置超时（舵机上电了吗？总线接对了吗？）")
        return P.dec_targets(self.latest_pos)

    def _check_ack(self, cmd: int, payload: bytes, timeout: float) -> None:
        pl = self.request(cmd, payload, P.RSP_ACK, timeout)
        rcmd, result = P.dec_ack(pl)
        if rcmd != cmd:
            raise LinkError(f"ACK 指令不匹配：发送 0x{cmd:02X}，收到 0x{rcmd:02X}")
        if result != P.RES_OK:
            raise LinkError(f"指令 0x{cmd:02X} 被拒绝：{P.RES_TEXT.get(result, result)}")


def open_serial(port: str, baud: int, logger=None) -> SerialLink:
    """打开真实串口并启动读线程。"""
    link = SerialLink(PySerialTransport(port, baud), logger=logger)
    link.start()
    return link
