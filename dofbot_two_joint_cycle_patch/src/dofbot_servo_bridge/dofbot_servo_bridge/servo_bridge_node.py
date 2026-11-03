#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
 dofbot_servo_bridge —— ROS2 仿真 → 德晟智能舵机 同步桥（经 STM32 上位模式）
============================================================================
 链路：
   /joint_states (Arm1_Joint / Arm2_Joint, 单位 rad)
       → 每个关节按可配置参数换算为舵机角度(°)
       → 按最大频率节流 + 角度死区过滤后生成文本命令 "MOVE <id> <角度°> <时间ms>"
       → USB-TTL (115200-8-N-1)
       → STM32 USART2 (PA2=TX / PA3=RX)
       → STM32 PA9 单线半双工总线
       → 德晟串口总线智能舵机协议 v4.01 智能舵机

 与 GCWJ/User/main.c（STM32 上位模式）配套，角度换算与 ServoHost 一致：
   位置 = 角度 × 4095 ÷ 满行程角度（默认 300°）

 角度映射（每个关节可独立配置）：
   舵机角度(°) = offset_deg + direction × 关节角(rad) × scale_deg_per_rad
   结果超出 [0, range_deg] 时自动钳位。
   例：offset=150、direction=1、scale=57.2958（180/π）、关节角 -0.5 rad
       → 150 - 28.65 = 121.35°

 无硬件自检：test_mode:=true 时使用虚拟串口，只把将发送的命令打印到日志，
 不打开任何真实串口（此时也无需安装 pyserial）。

 依赖：pyserial（真实串口模式）  安装：pip3 install pyserial
============================================================================
"""

import math
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:  # 未安装 pyserial：真实串口模式不可用，test_mode 不受影响
    serial = None


class _FakeSerial:
    """test_mode 使用的虚拟串口：只把写出的内容交给日志回调。"""

    def __init__(self, log):
        self._log = log
        self.is_open = True

    @property
    def in_waiting(self):
        return 0

    def read(self, size=1):
        return b""

    def write(self, data):
        if isinstance(data, bytes):
            data = data.decode("ascii", errors="replace")
        self._log(data.rstrip("\r\n"))

    def close(self):
        self.is_open = False


class ServoBridgeNode(Node):
    """订阅 /joint_states，把关节角换算成舵机命令经串口发给 STM32。"""

    def __init__(self):
        super().__init__("dofbot_servo_bridge")

        # ------------------------------ 参数 ------------------------------
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("range_deg", 300)
        self.declare_parameter("move_time_ms", 100)
        self.declare_parameter("max_rate_hz", 20.0)
        self.declare_parameter("deadband_deg", 0.2)
        self.declare_parameter("test_mode", False)
        self.declare_parameter("joint_names", ["Arm1_Joint"])
        self.declare_parameter("servo_ids", [1])
        self.declare_parameter("offsets_deg", [150.0])
        self.declare_parameter("scales_deg_per_rad", [57.295779513])
        self.declare_parameter("directions", [1.0])

        self._port = str(self.get_parameter("serial_port").value)
        self._baud = int(self.get_parameter("baudrate").value)
        self._range = int(self.get_parameter("range_deg").value)
        self._move_ms = int(self.get_parameter("move_time_ms").value)
        self._rate = float(self.get_parameter("max_rate_hz").value)
        self._deadband = float(self.get_parameter("deadband_deg").value)
        self._test_mode = bool(self.get_parameter("test_mode").value)

        if not 0 < self._range <= 360:
            raise ValueError(f"range_deg 必须在 1~360: {self._range}")
        if self._rate <= 0:
            raise ValueError(f"max_rate_hz 必须为正数: {self._rate}")

        # 关节映射表：并行数组参数，长度必须一致
        names = [str(s) for s in self.get_parameter("joint_names").value]
        ids = [int(v) for v in self.get_parameter("servo_ids").value]
        offsets = [float(v) for v in self.get_parameter("offsets_deg").value]
        scales = [float(v) for v in self.get_parameter("scales_deg_per_rad").value]
        dirs = [float(v) for v in self.get_parameter("directions").value]
        if not names or not (len(names) == len(ids) == len(offsets) == len(scales) == len(dirs)):
            raise ValueError(
                "joint_names / servo_ids / offsets_deg / scales_deg_per_rad / "
                "directions 必须数量一致且非空")

        self._joints = {}  # joint name -> (servo_id, offset_deg, scale, direction)
        for name, sid, off, sc, d in zip(names, ids, offsets, scales, dirs):
            if not 1 <= sid <= 250:
                raise ValueError(f"servo_id={sid} 超出德晟协议有效范围 1~250")
            if sc <= 0:
                raise ValueError(f"scales_deg_per_rad 必须为正数: {sc}")
            self._joints[name] = (sid, off, sc, d)

        # ------------------------------ 运行状态 ----------------------------
        self._lock = threading.Lock()
        self._latest = {}        # joint name -> rad（只由订阅回调写）
        self._last_sent = {}     # servo_id -> 上次已发送的舵机角度(°)
        self._saw_states = False
        self._out = queue.Queue(maxsize=64)
        self._shutdown = False

        # 订阅仿真关节状态；定时器按 max_rate_hz 节流生成命令
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)
        period = max(0.01, 1.0 / self._rate)
        self._timer = self.create_timer(period, self._publish_servo_targets)

        # 串口在独立线程里写，避免阻塞 executor
        self._serial_thread = threading.Thread(
            target=self._serial_loop, name="servo-serial", daemon=True)
        self._serial_thread.start()

        self.get_logger().info(
            f"联动桥已启动: 串口 {self._port} @ {self._baud} | "
            f"满行程 {self._range}° | 频率 ≤ {self._rate} Hz | "
            f"死区 {self._deadband}° | 运行时间 {self._move_ms} ms | "
            f"test_mode={self._test_mode}")
        for name, (sid, off, sc, d) in self._joints.items():
            self.get_logger().info(
                f"  关节映射: {name} → 舵机ID={sid} | "
                f"offset={off}° scale={sc}°/rad direction={d:+.0f}")

    # ---------------------------- 接收 /joint_states -------------------------
    def _on_joint_states(self, msg: JointState):
        first = not self._latest
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self._joints and i < len(msg.position):
                    self._latest[name] = float(msg.position[i])
        if not self._saw_states:
            self._saw_states = True
            present = sorted(set(msg.name) & set(self._joints))
            if not present:
                self.get_logger().warning(
                    f"/joint_states 中没有配置的关节 {sorted(self._joints)}，"
                    f"实际发布的是 {sorted(msg.name)}；请检查 bridge.yaml",
                    throttle_duration_sec=10.0)
            else:
                self.get_logger().info(
                    f"已收到 /joint_states，开始同步关节: {present}")

    # ---------------------------- 定时生成命令 -------------------------------
    def _publish_servo_targets(self):
        with self._lock:
            latest = dict(self._latest)
        if not latest:
            return
        lines = []
        for name, (sid, off, sc, d) in self._joints.items():
            rad = latest.get(name)
            if rad is None or not math.isfinite(rad):
                continue
            deg = off + d * rad * sc
            deg = max(0.0, min(float(self._range), deg))
            last = self._last_sent.get(sid)
            if last is not None and abs(deg - last) < self._deadband:
                continue
            self._last_sent[sid] = deg
            lines.append(f"MOVE {sid} {deg:.2f} {self._move_ms}")
        for line in lines:
            try:
                self._out.put_nowait(line)
            except queue.Full:
                self.get_logger().warning(
                    "发送队列已满，丢弃命令（串口侧可能已断开）",
                    throttle_duration_sec=5.0)
        if lines:
            self.get_logger().debug(f"生成 {len(lines)} 条舵机命令")

    # ------------------------------ 串口线程 --------------------------------
    def _serial_loop(self):
        if self._test_mode:
            self._fake_loop()
            return
        if serial is None:
            self.get_logger().error(
                "未安装 pyserial，真实串口模式不可用。请执行: "
                "pip3 install pyserial（或用 test_mode:=true 做无硬件自检）",
                throttle_duration_sec=10.0)
            return

        while not self._shutdown:
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.05)
            except Exception as exc:
                self.get_logger().error(
                    f"打开串口 {self._port} 失败: {exc}，3 秒后重试",
                    throttle_duration_sec=10.0)
                self._sleep(3.0)
                continue

            self.get_logger().info(
                f"串口 {self._port} @ {self._baud} 已打开，同步满行程 RANGE {self._range}")
            try:
                # 连接后先同步满行程角度（STM32 上电问候 READY 到达与否都会处理）
                ser.write(f"RANGE {self._range}\r\n".encode("ascii"))
                while not self._shutdown:
                    self._drain_replies(ser)
                    try:
                        line = self._out.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    if line is None:
                        break
                    ser.write((line + "\r\n").encode("ascii"))
                    self.get_logger().debug(f"TX >>> {line}")
            except Exception as exc:
                if not self._shutdown:
                    self.get_logger().warning(f"串口 {self._port} 通信异常: {exc}，3 秒后重连")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            if not self._shutdown:
                self.get_logger().info("串口已断开，等待重连")
                self._sleep(3.0)
        self.get_logger().info("串口线程退出")

    def _drain_replies(self, ser):
        """读取 STM32 应答并记录日志（读不到数据立即返回，不阻塞）。"""
        try:
            if not ser.in_waiting:
                return
            data = ser.read(ser.in_waiting)
        except Exception:
            return
        for line in data.decode("ascii", errors="replace").splitlines():
            line = line.strip()
            if not line:
                continue
            if "READY" in line.upper():
                self.get_logger().info(f"RX <<< {line}（下位机就绪）")
            else:
                self.get_logger().debug(f"RX <<< {line}")

    def _fake_loop(self):
        fake = _FakeSerial(lambda s: self.get_logger().info(f"[TEST] TX >>> {s}"))
        self.get_logger().info("test_mode：使用虚拟串口，命令只打印不发送")
        fake.write(f"RANGE {self._range}")
        while not self._shutdown:
            try:
                line = self._out.get(timeout=0.1)
            except queue.Empty:
                continue
            if line is None:
                break
            fake.write(line)
        fake.close()
        self.get_logger().info("虚拟串口线程退出")

    def _sleep(self, seconds):
        """可被打断的休眠（退出时不用等满时长）。"""
        deadline = time.monotonic() + seconds
        while not self._shutdown and time.monotonic() < deadline:
            time.sleep(0.05)

    # ------------------------------ 退出清理 --------------------------------
    def _shutdown_serial(self):
        self._shutdown = True
        try:
            self._out.put_nowait(None)
        except queue.Full:
            pass
        if self._serial_thread is not None:
            self._serial_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ServoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
