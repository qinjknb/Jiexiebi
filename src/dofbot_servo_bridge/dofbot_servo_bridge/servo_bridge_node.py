"""Mirror MoveIt's executed /joint_states to six GCWJ bus servos."""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .servo_protocol import JOINT_NAMES, ProtocolError, ServoMapping, read_position, request

try:
    import serial
except ImportError:
    serial = None


class ServoBridgeNode(Node):
    def __init__(self):
        super().__init__("dofbot_servo_bridge")
        defaults = {
            "serial_port": "/dev/ttyUSB0", "baudrate": 115200,
            "test_mode": True, "enable_motion": False,
            "calibration_confirmed": False, "range_deg": 300,
            "max_rate_hz": 10.0, "move_time_ms": 120,
            "deadband_deg": 0.2, "state_timeout_sec": 0.5,
            "start_tolerance_deg": 10.0,
            "joint_names": list(JOINT_NAMES), "servo_ids": [1, 2, 3, 4, 5, 6],
            "offsets_deg": [150.0] * 6,
            "directions": [1.0] * 6,
            "scales_deg_per_rad": [180.0 / math.pi] * 6,
            "min_degrees": [0.0] * 6, "max_degrees": [300.0] * 6,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self._port = str(p("serial_port"))
        self._baud = int(p("baudrate"))
        self._test_mode = bool(p("test_mode"))
        self._enabled = bool(p("enable_motion"))
        self._calibrated = bool(p("calibration_confirmed"))
        self._rate = float(p("max_rate_hz"))
        self._move_ms = int(p("move_time_ms"))
        self._deadband = float(p("deadband_deg"))
        self._state_timeout = float(p("state_timeout_sec"))
        self._start_tolerance = float(p("start_tolerance_deg"))
        self._mapping = ServoMapping(
            list(p("joint_names")), list(p("servo_ids")),
            list(p("offsets_deg")), list(p("directions")),
            list(p("scales_deg_per_rad")), list(p("min_degrees")),
            list(p("max_degrees")), int(p("range_deg")),
        )
        if self._rate <= 0 or not 1 <= self._move_ms <= 65535:
            raise ValueError("max_rate_hz 必须为正数，move_time_ms 必须在 1～65535")
        if self._deadband < 0 or self._state_timeout <= 0 or self._start_tolerance < 0:
            raise ValueError("死区、状态超时和起点容差参数无效")

        self._lock = threading.Lock()
        self._positions = {}
        self._state_times = {}
        self._stop = threading.Event()
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"六舵机镜像桥: {self._port} @ {self._baud}, "
            f"test_mode={self._test_mode}, enable_motion={self._enabled}"
        )
        if not self._enabled:
            self.get_logger().warning("运动未启用；启动时需显式设置 enable_motion:=true")

    def _on_state(self, msg):
        values = {
            name: float(msg.position[i])
            for i, name in enumerate(msg.name)
            if name in self._mapping.names and i < len(msg.position)
        }
        with self._lock:
            self._positions.update(values)
            now = time.monotonic()
            self._state_times.update({name: now for name in values})

    def _targets(self):
        with self._lock:
            positions = dict(self._positions)
            times = dict(self._state_times)
        now = time.monotonic()
        if any(
            n not in positions or now - times.get(n, 0.0) > self._state_timeout
            for n in self._mapping.names
        ):
            return None
        try:
            return self._mapping.targets(positions)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return None

    def _run(self):
        if not self._enabled:
            return
        if self._test_mode:
            self._dry_run()
            return
        if not self._calibrated:
            self.get_logger().error(
                "实机运动被阻止：请完成六轴标定后设置 calibration_confirmed:=true"
            )
            return
        if serial is None:
            self.get_logger().error("缺少 pyserial，请安装 python3-serial")
            return
        while not self._stop.is_set():
            try:
                with serial.Serial(self._port, self._baud, timeout=0.1, write_timeout=1.0) as ser:
                    self._connect_and_mirror(ser)
            except (OSError, ValueError, ProtocolError, serial.SerialException) as exc:
                self.get_logger().error(f"串口或舵机检查失败: {exc}；3 秒后重试")
            self._stop.wait(3.0)

    def _connect_and_mirror(self, ser):
        ser.reset_input_buffer()
        request(ser, f"RANGE {self._mapping.range_deg}",
                f"OK RANGE {self._mapping.range_deg}", timeout=8.0)
        for sid in self._mapping.ids:
            request(ser, f"PING {sid}", f"OK PING {sid}")
        self.get_logger().info("六个舵机 ID 均已应答；等待有效且与实机对齐的关节状态")
        last_sent = {}
        aligned = False
        period = 1.0 / self._rate
        while not self._stop.is_set():
            started = time.monotonic()
            targets = self._targets()
            if targets is None:
                self._stop.wait(min(period, 0.1))
                continue
            if not aligned:
                actual = {
                    sid: self._mapping.degrees_from_position(read_position(ser, sid))
                    for sid in self._mapping.ids
                }
                mismatch = [
                    f"ID{sid}: 实机{actual[sid]:.1f}°/仿真{targets[sid]:.1f}°"
                    for sid in self._mapping.ids
                    if abs(actual[sid] - targets[sid]) > self._start_tolerance
                ]
                if mismatch:
                    self.get_logger().warning(
                        "起点未对齐，拒绝运动: " + ", ".join(mismatch),
                        throttle_duration_sec=5.0,
                    )
                    self._stop.wait(1.0)
                    continue
                aligned = True
                self.get_logger().info("实机与仿真起点已对齐，开始跟随路径")
            for sid, deg in targets.items():
                if self._stop.is_set():
                    return
                if sid in last_sent and abs(deg - last_sent[sid]) < self._deadband:
                    continue
                command = f"MOVE {sid} {deg:.2f} {self._move_ms}"
                request(ser, command, f"OK MOVE {sid} ")
                last_sent[sid] = deg
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def _dry_run(self):
        last_sent = {}
        period = 1.0 / self._rate
        self.get_logger().info("试运行模式：只输出指令，不打开串口")
        while not self._stop.wait(period):
            targets = self._targets()
            if targets is None:
                continue
            for sid, deg in targets.items():
                if sid not in last_sent or abs(deg - last_sent[sid]) >= self._deadband:
                    self.get_logger().info(f"[DRY RUN] MOVE {sid} {deg:.2f} {self._move_ms}")
                    last_sent[sid] = deg

    def close(self):
        self._stop.set()
        self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ServoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
