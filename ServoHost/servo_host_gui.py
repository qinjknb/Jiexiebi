#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 STM32 舵机控制上位机 (PyCharm 工程)
============================================================================
 功能：通过串口发送命令，控制下位机(STM32F103)驱动"德晟串口总线智能舵机
      协议 v4.01"的智能舵机（SCS 系列等）。

 两种工作模式：
   模式 A - STM32 上位模式（推荐）
      电脑 ──USB转TTL──> STM32 USART2 (PA2/PA3) ──PA9半双工──> 舵机
      命令为文本行：MOVE id angleDeg ms / READ id / PING id 等，
      与改版后的 main.c 配套（见 GCWJ/User/main.c）。
   模式 B - 直连舵机模式（不用改下位机，PC 直接驱动舵机总线）
      电脑 ──USB转TTL(TX、RX 并联)──> 舵机数据线
      程序内直接组"德晟 v4.01"协议帧（FF FF ID LEN CMD 参数 SUM）。

 依赖：pyserial   安装：pip install pyserial
 运行：python servo_host_gui.py （PyCharm 中直接 Run）
============================================================================
"""

import queue
import threading
import time
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    _root = tk.Tk()
    _root.withdraw()
    messagebox.showerror(
        "缺少依赖库",
        "未找到 pyserial 库。\n\n请在 PyCharm 终端中执行：\n"
        "    pip install pyserial\n\n"
        "或执行：\n"
        "    pip install -r requirements.txt",
    )
    _root.destroy()
    raise SystemExit(1)

# ============================ 协议常量（德晟 v4.01） ==========================
BROADCAST_ID = 254
CMD_PING = 0x01
CMD_READ = 0x02
CMD_WRITE = 0x03
CMD_RESET = 0x06
ADDR_TARGET_POS = 0x2A
ADDR_CUR_POS = 0x38

MODE_STM32 = "stm32"   # 经 STM32 转舵机
MODE_DIRECT = "direct"  # 直接驱动舵机总线


def build_frame(sid: int, cmd: int, params: bytes = b"") -> bytes:
    """组一帧德晟 v4.01 主机帧：FF FF ID LEN CMD 参数... SUM"""
    length = 2 + len(params)
    s = (sid + length + cmd + sum(params)) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, cmd]) + params + bytes([(~s) & 0xFF])


def frame_move(sid: int, pos: int, time_ms: int) -> bytes:
    """写目标位置 0x2A(2字节)+运行时间 0x2C(2字节)，大端"""
    pos = max(0, min(4095, int(pos)))
    time_ms = max(0, min(65535, int(time_ms)))
    params = bytes([ADDR_TARGET_POS, pos >> 8, pos & 0xFF,
                    time_ms >> 8, time_ms & 0xFF])
    return build_frame(sid, CMD_WRITE, params)


def frame_read_pos(sid: int) -> bytes:
    """读当前位置寄存器 0x38，2 字节"""
    return build_frame(sid, CMD_READ, bytes([ADDR_CUR_POS, 0x02]))


def frame_ping(sid: int) -> bytes:
    return build_frame(sid, CMD_PING)


def frame_set_id(sid: int) -> bytes:
    """广播写寄存器 0x05 = 舵机 ID（掉电保存）"""
    return build_frame(BROADCAST_ID, CMD_WRITE, bytes([0x05, sid]))


def frame_reset(sid: int) -> bytes:
    return build_frame(sid, CMD_RESET)


# ================================ 主窗口类 ====================================
class ServoHostApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("STM32 舵机控制上位机 v1.0")
        root.geometry("780x700")
        root.minsize(720, 620)

        self.ser = None            # serial.Serial 对象
        self.running = False       # 读线程运行标志
        self.worker = None         # 读线程
        self.q = queue.Queue()     # 线程 -> GUI 队列
        self._linebuf = ""         # 读线程拼行缓冲（仅读线程访问）
        self._echo = bytearray()   # 直连模式下待过滤的自发回显
        self._waiting = None       # 直连模式等待应答: dict(kind, deadline)
        self._resp_buf = bytearray()

        self._build_ui()
        self.refresh_ports()
        self._update_mode_hint()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(40, self._poll)

    # ------------------------------ 界面搭建 --------------------------------
    def _build_ui(self):
        pad = {"padx": 6, "pady": 3}

        # ---------- 1. 串口连接 ----------
        f1 = ttk.LabelFrame(self.root, text=" 1. 串口连接 ")
        f1.pack(fill="x", padx=8, pady=4)

        row1 = ttk.Frame(f1)
        row1.pack(fill="x", **pad)
        ttk.Label(row1, text="工作模式:").pack(side="left")
        self.mode_var = tk.StringVar(value=MODE_STM32)
        ttk.Radiobutton(row1, text="STM32 上位模式(经 STM32 转舵机)",
                        variable=self.mode_var, value=MODE_STM32,
                        command=self._on_mode_changed).pack(side="left", padx=4)
        ttk.Radiobutton(row1, text="直连舵机模式(直接驱动舵机)",
                        variable=self.mode_var, value=MODE_DIRECT,
                        command=self._on_mode_changed).pack(side="left", padx=4)

        row2 = ttk.Frame(f1)
        row2.pack(fill="x", **pad)
        ttk.Label(row2, text="串口:").pack(side="left")
        self.com_var = tk.StringVar()
        self.com_box = ttk.Combobox(row2, textvariable=self.com_var, width=38,
                                    state="readonly")
        self.com_box.pack(side="left", padx=4)
        ttk.Button(row2, text="刷新", width=6,
                   command=self.refresh_ports).pack(side="left", padx=2)
        ttk.Label(row2, text="波特率:").pack(side="left", padx=(14, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(row2, textvariable=self.baud_var, width=10,
                     state="readonly",
                     values=("115200", "1000000", "9600")).pack(side="left", padx=4)
        self.btn_conn = ttk.Button(row2, text="连接", width=10,
                                   command=self._on_connect)
        self.btn_conn.pack(side="left", padx=6)

        self.hint_var = tk.StringVar()
        ttk.Label(f1, textvariable=self.hint_var,
                  foreground="#0a6ebd").pack(anchor="w", **pad)

        # ---------- 2. 舵机控制 ----------
        f2 = ttk.LabelFrame(self.root, text=" 2. 舵机控制 ")
        f2.pack(fill="x", padx=8, pady=4)

        r1 = ttk.Frame(f2)
        r1.pack(fill="x", **pad)
        ttk.Label(r1, text="舵机ID:").pack(side="left")
        self.id_var = tk.StringVar(value="1")
        ttk.Spinbox(r1, from_=1, to=254, width=5,
                    textvariable=self.id_var).pack(side="left", padx=4)
        ttk.Label(r1, text="满行程角度:").pack(side="left", padx=(16, 0))
        self.range_var = tk.StringVar(value="300")
        ttk.Spinbox(r1, from_=180, to=360, increment=10, width=5,
                    textvariable=self.range_var,
                    command=self._on_range_changed).pack(side="left", padx=4)
        ttk.Label(r1, text="(180/270/300/360, 按舵机型号)").pack(side="left")
        ttk.Label(r1, text="运行时间(ms):").pack(side="left", padx=(16, 0))
        self.time_var = tk.StringVar(value="400")
        ttk.Entry(r1, textvariable=self.time_var, width=7).pack(side="left", padx=4)

        r2 = ttk.Frame(f2)
        r2.pack(fill="x", **pad)
        ttk.Label(r2, text="目标角度:").pack(side="left")
        self.angle_var = tk.DoubleVar(value=150.0)
        self.scale = ttk.Scale(r2, from_=0, to=300, variable=self.angle_var,
                               command=self._on_scale_changed)
        self.scale.pack(side="left", fill="x", expand=True, padx=4)
        self.angle_entry_var = tk.StringVar(value="150.0")
        ttk.Entry(r2, textvariable=self.angle_entry_var, width=7,
                  justify="right").pack(side="left", padx=4)
        ttk.Label(r2, text="°").pack(side="left")

        r3 = ttk.Frame(f2)
        r3.pack(fill="x", **pad)
        ttk.Button(r3, text="移动到目标角度", command=self._on_move).pack(
            side="left", padx=3)
        ttk.Button(r3, text="读取当前位置", command=self._on_read).pack(
            side="left", padx=3)
        ttk.Button(r3, text="PING 检测舵机", command=self._on_ping).pack(
            side="left", padx=3)
        ttk.Button(r3, text="设置舵机ID", command=self._on_set_id).pack(
            side="left", padx=3)
        ttk.Button(r3, text="复位舵机", command=self._on_reset).pack(
            side="left", padx=3)
        self.btn_demo = ttk.Button(r3, text="恢复演示(STM32)",
                                   command=self._on_demo)
        self.btn_demo.pack(side="left", padx=3)

        self.pos_var = tk.StringVar(value="当前位置: --")
        ttk.Label(f2, textvariable=self.pos_var,
                  foreground="#006400").pack(anchor="w", **pad)

        # ---------- 3. 自定义指令 ----------
        f3 = ttk.LabelFrame(self.root, text=" 3. 自定义指令 (仅 STM32 上位模式) ")
        f3.pack(fill="x", padx=8, pady=4)
        r3b = ttk.Frame(f3)
        r3b.pack(fill="x", **pad)
        self.cmd_var = tk.StringVar()
        self.entry_cmd = ttk.Entry(r3b, textvariable=self.cmd_var)
        self.entry_cmd.pack(side="left", fill="x", expand=True, padx=4)
        self.entry_cmd.bind("<Return>", lambda e: self._on_custom())
        ttk.Button(r3b, text="发送", width=8,
                   command=self._on_custom).pack(side="left", padx=4)

        # ---------- 4. 日志 ----------
        f4 = ttk.LabelFrame(self.root, text=" 4. 通信日志 ")
        f4.pack(fill="both", expand=True, padx=8, pady=4)
        self.log = ScrolledText(f4, height=12, state="disabled",
                                font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=4, pady=2)
        self.log.tag_config("tx", foreground="#0a6ebd")
        self.log.tag_config("rx", foreground="#333333")
        self.log.tag_config("ok", foreground="#006400")
        self.log.tag_config("warn", foreground="#b8860b")
        self.log.tag_config("err", foreground="#cc0000")
        ttk.Button(f4, text="清空日志", width=10,
                   command=self._clear_log).pack(anchor="e", padx=4, pady=2)

        # ---------- 状态栏 ----------
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(self.root, textvariable=self.status_var,
                  relief="sunken", anchor="w").pack(fill="x", side="bottom")

    # ------------------------------ 日志工具 --------------------------------
    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%H:%M:%S")

    def _log(self, text: str, tag: str = "rx"):
        self.log.configure(state="normal")
        self.log.insert("end", f"[{self._now()}] {text}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _log_tx_text(self, text: str):
        self._log(f"TX >>> {text}", "tx")

    def _log_tx_hex(self, data: bytes):
        self._log(f"TX >>> {data.hex(' ').upper()}", "tx")

    def _log_rx_hex(self, data: bytes):
        self._log(f"RX <<< {data.hex(' ').upper()}", "rx")

    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    # ------------------------------ 串口连接 --------------------------------
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        names = [f"{p.device}  ({p.description})" for p in sorted(ports)]
        self.com_box["values"] = names
        if names:
            if self.com_var.get() not in names:
                self.com_var.set(names[0])
        else:
            self.com_var.set("")
        self._log(f"刷新串口列表，发现 {len(names)} 个串口", "ok")

    def _on_connect(self):
        if self.ser is not None:
            self._disconnect()
            return
        if not self.com_var.get():
            messagebox.showwarning("提示", "没有可用串口，请先点击\"刷新\"。")
            return
        port = self.com_var.get().split(" ")[0]
        baud = int(self.baud_var.get())
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except (serial.SerialException, OSError) as e:
            messagebox.showerror("连接失败", f"无法打开 {port}：\n{e}")
            self._log(f"连接失败: {e}", "err")
            return

        self.running = True
        self.worker = threading.Thread(target=self._reader_loop, daemon=True)
        self.worker.start()

        self.btn_conn.configure(text="断开")
        self.status_var.set(f"已连接 {port} @ {baud}")
        self._log(f"已连接串口 {port} @ {baud}", "ok")

        if self.mode_var.get() == MODE_STM32:
            # 同步满行程角度到固件
            self._send_text(f"RANGE {self._read_range()}")
            self._log("等待下位机开机问候 READY ...（若长时间无回应请检查接线）",
                      "warn")
        else:
            self._log("直连舵机模式：请确认 USB-TTL 的 TX、RX 已并联接到舵机数据线",
                      "warn")

    def _disconnect(self):
        self.running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.btn_conn.configure(text="连接")
        self.status_var.set("未连接")
        self._waiting = None
        self._resp_buf = bytearray()
        self._log("已断开连接", "ok")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    # ------------------------------ 读线程 ----------------------------------
    def _reader_loop(self):
        """读串口线程：原始字节 -> 队列；同时按行切分文本"""
        ser = self.ser
        while self.running and ser is not None:
            try:
                n = ser.in_waiting
                data = ser.read(n if n > 0 else 1)
            except (serial.SerialException, OSError, AttributeError) as e:
                if self.running:
                    self.q.put(("err", f"串口读取错误: {e}"))
                return
            if data:
                self.q.put(("raw", bytes(data)))
                try:
                    text = data.decode("ascii", errors="replace")
                except Exception:
                    text = ""
                if text:
                    self._linebuf += text
                    while "\n" in self._linebuf:
                        line, self._linebuf = self._linebuf.split("\n", 1)
                        line = line.strip("\r")
                        if line:
                            self.q.put(("line", line))
        self.q.put(("closed", None))

    # ------------------------------ GUI 轮询 ---------------------------------
    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "line":
                    if self.mode_var.get() == MODE_STM32:
                        self._handle_line(payload)
                elif kind == "raw":
                    if self.mode_var.get() == MODE_DIRECT:
                        self._handle_raw(payload)
                elif kind == "err":
                    self._log(payload, "err")
                    self._disconnect()
                elif kind == "closed":
                    if self.running:
                        self._log("串口连接已断开", "warn")
                        self._disconnect()
        except queue.Empty:
            pass

        # 直连模式应答超时检测
        if self._waiting is not None and time.time() > self._waiting["deadline"]:
            self._waiting = None
            self._resp_buf = bytearray()
            self._log("应答超时：舵机无回应（检查接线/供电/ID）", "err")

        self.root.after(40, self._poll)

    # ------------------------------ 接收处理 ---------------------------------
    def _handle_line(self, line: str):
        """STM32 上位模式的文本应答"""
        self._log(line, "rx")
        upper = line.upper()
        parts = line.split()
        if parts and parts[0].upper() == "POS" and len(parts) >= 3:
            try:
                sid = int(parts[1])
                pos = int(parts[2])
                self._update_pos(pos)
                self._log(f"舵机 ID={sid} 当前位置 {pos}", "ok")
            except ValueError:
                pass
        elif upper.startswith("OK PING"):
            self._log("舵机在线（PING 应答）", "ok")
        elif upper.startswith("ERR PING") or upper.startswith("ERR READ"):
            self._log("舵机无应答：检查供电、共地、数据线接 PA9、ID 是否正确",
                      "warn")
        elif upper.startswith("READY"):
            self._log("下位机已就绪，可发送命令", "ok")

    def _handle_raw(self, data: bytes):
        """直连模式：过滤自发回显，剩余字节进入应答扫描"""
        if self._echo:
            k = min(len(self._echo), len(data))
            if data[:k] == bytes(self._echo[:k]):
                del self._echo[:k]
                data = data[k:]
                if not data:
                    return
        self._log_rx_hex(data)
        self._resp_buf.extend(data)
        self._scan_direct_reply()

    def _scan_direct_reply(self):
        w = self._waiting
        if w is None:
            return
        buf = self._resp_buf
        need = 6 if w["kind"] == "read" else 4
        while True:
            idx = buf.find(b"\xff\xf5")
            if idx < 0:
                break
            body = buf[idx + 2:]
            if len(body) < need:
                break                       # 帧未收全，等下一次
            buf = body[need:]
            cand = body[:need]
            sid = cand[0]
            if w["kind"] == "read" and cand[1] == 0x04:
                s = (cand[0] + cand[1] + cand[2] + cand[3] + cand[4]) & 0xFF
                if cand[5] == ((~s) & 0xFF):
                    pos = (cand[3] << 8) | cand[4]
                    self._waiting = None
                    self._resp_buf = buf
                    self._update_pos(pos)
                    self._log(f"舵机 ID={sid} 应答：位置 {pos}", "ok")
                    return
                continue                    # 校验错，找下一帧头
            if w["kind"] == "ping" and cand[1] == 0x02:
                s = (cand[0] + cand[1] + cand[2]) & 0xFF
                if cand[3] == ((~s) & 0xFF):
                    self._waiting = None
                    self._resp_buf = buf
                    self._log(f"舵机 ID={sid} 在线（PING 应答）", "ok")
                    return
                continue
        self._resp_buf = buf

    # ------------------------------ 发送工具 ---------------------------------
    def _check_connected(self) -> bool:
        if self.ser is None or not self.ser.is_open:
            messagebox.showwarning("提示", "请先连接串口。")
            return False
        return True

    def _send_text(self, text: str):
        if not self._check_connected():
            return
        try:
            self.ser.write((text + "\r\n").encode("ascii"))
            self._log_tx_text(text)
        except (serial.SerialException, OSError) as e:
            self._log(f"发送失败: {e}", "err")

    def _send_frame(self, data: bytes):
        if not self._check_connected():
            return
        try:
            self.ser.write(data)
            self._echo = bytearray(data)   # 单线总线自发回显，稍后过滤
            self._log_tx_hex(data)
        except (serial.SerialException, OSError) as e:
            self._log(f"发送失败: {e}", "err")

    # ------------------------------ 参数读取 ---------------------------------
    def _read_sid(self):
        try:
            sid = int(self.id_var.get())
        except ValueError:
            sid = 0
        if not 1 <= sid <= 254:
            messagebox.showwarning("提示", "舵机 ID 应为 1~254 的整数。")
            return None
        return sid

    def _read_range(self) -> int:
        try:
            r = int(float(self.range_var.get()))
        except ValueError:
            r = 300
        return max(1, min(360, r))

    def _read_angle(self):
        try:
            angle = float(self.angle_entry_var.get())
        except ValueError:
            messagebox.showwarning("提示", "目标角度填写有误（支持小数，如 150.5）。")
            return None
        return angle

    def _read_time(self) -> int:
        try:
            t = int(self.time_var.get())
        except ValueError:
            messagebox.showwarning("提示", "运行时间应为整数（毫秒）。")
            return 0
        return max(0, min(65535, t))

    def _update_pos(self, pos: int):
        rng = self._read_range()
        angle = pos * rng / 4095.0
        self.pos_var.set(f"当前位置: {pos}  (≈ {angle:.1f}°)")

    # ------------------------------ 界面回调 ---------------------------------
    def _on_scale_changed(self, value):
        try:
            self.angle_entry_var.set(f"{float(value):.1f}")
        except (ValueError, tk.TclError):
            pass

    def _on_range_changed(self):
        """满行程角度变化：改滑条量程；STM32 模式同时下发给固件"""
        rng = self._read_range()
        self.scale.configure(to=rng)
        if (self.mode_var.get() == MODE_STM32 and
                self.ser is not None and self.ser.is_open):
            self._send_text(f"RANGE {rng}")

    def _on_mode_changed(self):
        self._update_mode_hint()
        self._waiting = None
        self._resp_buf = bytearray()
        if (self.mode_var.get() == MODE_STM32 and
                self.ser is not None and self.ser.is_open):
            self._send_text(f"RANGE {self._read_range()}")

    def _update_mode_hint(self):
        if self.mode_var.get() == MODE_STM32:
            self.hint_var.set(
                "接线：USB-TTL  TX→PA3(RX)  RX→PA2(TX)  GND→GND；舵机数据线接 PA9")
        else:
            self.hint_var.set(
                "接线：USB-TTL 的 TX 与 RX 并联后接舵机数据线；GND 与舵机电源共地；"
                "舵机用外部电源供电")

    def _on_move(self):
        sid = self._read_sid()
        if sid is None:
            return
        angle = self._read_angle()
        if angle is None:
            return
        tms = self._read_time()
        rng = self._read_range()
        angle = max(0.0, min(float(rng), angle))
        if self.mode_var.get() == MODE_STM32:
            self._send_text(f"MOVE {sid} {angle} {tms}")
        else:
            pos = int(round(angle * 4095.0 / rng))
            self._send_frame(frame_move(sid, pos, tms))

    def _on_read(self):
        sid = self._read_sid()
        if sid is None:
            return
        if self.mode_var.get() == MODE_STM32:
            self._send_text(f"READ {sid}")
        else:
            self._send_frame(frame_read_pos(sid))
            self._waiting = {"kind": "read",
                             "deadline": time.time() + 0.8}

    def _on_ping(self):
        sid = self._read_sid()
        if sid is None:
            return
        if self.mode_var.get() == MODE_STM32:
            self._send_text(f"PING {sid}")
        else:
            self._send_frame(frame_ping(sid))
            self._waiting = {"kind": "ping",
                             "deadline": time.time() + 0.8}

    def _on_set_id(self):
        sid = self._read_sid()
        if sid is None:
            return
        if self.mode_var.get() == MODE_STM32:
            self._send_text(f"SETID {sid}")
        else:
            if not messagebox.askyesno(
                    "确认",
                    f"将把总线上所有舵机的 ID 设为 {sid}（掉电保存）。\n"
                    "多舵机总线上请勿执行此操作！继续？"):
                return
            self._send_frame(frame_set_id(sid))

    def _on_reset(self):
        sid = self._read_sid()
        if sid is None:
            return
        if self.mode_var.get() == MODE_STM32:
            self._send_text(f"RESET {sid}")
        else:
            self._send_frame(frame_reset(sid))

    def _on_demo(self):
        if self.mode_var.get() != MODE_STM32:
            messagebox.showinfo("提示", "\"恢复演示\"仅适用于 STM32 上位模式。")
            return
        self._send_text("DEMO")

    def _on_custom(self):
        if self.mode_var.get() != MODE_STM32:
            messagebox.showinfo("提示", "自定义文本指令仅适用于 STM32 上位模式。")
            return
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        self._send_text(cmd)
        self.cmd_var.set("")


# ================================= 入口 =======================================
def main():
    root = tk.Tk()
    ServoHostApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
