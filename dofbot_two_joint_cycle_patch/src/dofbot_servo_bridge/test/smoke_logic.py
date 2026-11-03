#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
servo_bridge_node 的无 ROS 逻辑自检（smoke test）。

用假的 rclpy / sensor_msgs 模块加载真实的 servo_bridge_node.py，跑通：
  - 参数装载与关节映射表构建
  - /joint_states 回调 → 弧度换算舵机角度
  - 死区过滤、超程钳位、MOVE 文本格式
  - test_mode 虚拟串口输出
  - 退出清理

运行（任意装有 Python 3 的机器，无需 ROS / pyserial）：
    cd dofbot_servo_bridge 包根目录
    python test/smoke_logic.py
全部通过打印 ALL TESTS PASSED 并返回 0。
"""

import importlib.util
import os
import sys
import time
import types

# ------------------------------ 伪造依赖模块 --------------------------------
class FakeLogger:
    def __init__(self):
        self.lines = []

    def _log(self, msg, *args, **kwargs):
        self.lines.append(str(msg))

    info = _log
    warning = _log
    warn = _log
    error = _log
    debug = _log


class FakeNode:
    def __init__(self, *args, **kwargs):
        self._params = {}
        self._logger = FakeLogger()

    def declare_parameter(self, name, default):
        if name == "test_mode":
            default = True  # 自检固定走虚拟串口路径
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        class Value:
            pass

        value = Value()
        value.value = self._params[name]
        return value

    def get_logger(self):
        return self._logger

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, period, callback):
        self._timer_cb = callback
        return None

    def destroy_node(self):
        pass


rclpy_mod = types.ModuleType("rclpy")
rclpy_mod.__path__ = []  # 让 from rclpy.node import ... 当作包处理
rclpy_mod.init = lambda *a, **k: None
rclpy_mod.spin = lambda *a, **k: None
rclpy_mod.ok = lambda: True
rclpy_mod.shutdown = lambda *a, **k: None
rclpy_node_mod = types.ModuleType("rclpy.node")
rclpy_node_mod.Node = FakeNode
rclpy_mod.node = rclpy_node_mod

sensor_mod = types.ModuleType("sensor_msgs")
sensor_mod.__path__ = []
msg_mod = types.ModuleType("sensor_msgs.msg")
msg_mod.JointState = type("JointState", (), {})
sensor_mod.msg = msg_mod

sys.modules["rclpy"] = rclpy_mod
sys.modules["rclpy.node"] = rclpy_node_mod
sys.modules["sensor_msgs"] = sensor_mod
sys.modules["sensor_msgs.msg"] = msg_mod

# ------------------------------ 加载真实节点代码 ------------------------------
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # 包根目录
spec = importlib.util.spec_from_file_location(
    "servo_bridge_node",
    os.path.join(BASE, "dofbot_servo_bridge", "servo_bridge_node.py"))
node_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node_mod)

# ------------------------------ 断言工具 -------------------------------------
failures = []


def check(name, condition, detail=""):
    print(f"[{'PASS' if condition else 'FAIL'}] {name} {detail}")
    if not condition:
        failures.append(name)


def wait_for_log(lines, needle, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if any(needle in line for line in lines):
            return True
        time.sleep(0.05)
    return False


def make_state(*pairs):
    msg = msg_mod.JointState()
    msg.name = [n for n, _ in pairs]
    msg.position = [p for _, p in pairs]
    return msg


# ------------------------------ 测试流程 -------------------------------------
node = node_mod.ServoBridgeNode()
logger = node.get_logger()

check("参数装载: 串口", node._port == "/dev/ttyUSB0", f"port={node._port}")
check("参数装载: 映射表", node._joints.get("Arm1_Joint") == (1, 150.0, 57.295779513, 1.0),
      str(node._joints))
check("test_mode 生效", node._test_mode is True)
check("虚拟串口已发 RANGE", wait_for_log(logger.lines, "RANGE 300"))

# 关节角 -0.5 rad → 150 - 0.5*57.2958 = 121.3521 → "121.35"
node._on_joint_states(make_state(("Arm1_Joint", -0.5)))
node._publish_servo_targets()
check("弧度→舵机角度换算", wait_for_log(logger.lines, "MOVE 1 121.35 100"))

# 同一角度再发：死区 0.2° 内，不应重复发送
before = len(logger.lines)
node._publish_servo_targets()
time.sleep(0.3)
duplicates = [l for l in logger.lines[before:] if "MOVE 1 121.35 100" in l]
check("死区过滤", not duplicates, f"重复发送 {len(duplicates)} 条")

# -0.6 rad → 150 - 34.3775 = 115.6225 → "115.62"
node._on_joint_states(make_state(("Arm1_Joint", -0.6)))
node._publish_servo_targets()
check("角度变化后发送新命令", wait_for_log(logger.lines, "MOVE 1 115.62 100"))

# 5.0 rad → 436.5° → 钳位到满行程 300.00°
node._on_joint_states(make_state(("Arm1_Joint", 5.0)))
node._publish_servo_targets()
check("超程钳位到 300.00°", wait_for_log(logger.lines, "MOVE 1 300.00 100"))

# 非有限值应被跳过（不产生 NaN 命令）
node._on_joint_states(make_state(("Arm1_Joint", float("nan"))))
node._publish_servo_targets()
time.sleep(0.3)
check("非有限值跳过", not any("nan" in l for l in logger.lines))

# 退出清理：线程应在 2 秒内结束
node._shutdown_serial()
time.sleep(0.2)
check("退出清理: 串口线程结束", not node._serial_thread.is_alive())

print("-" * 60)
if failures:
    print(f"FAILED: {len(failures)} 项 -> {failures}")
    sys.exit(1)
print("ALL TESTS PASSED")
sys.exit(0)
