"""Exercise the real bridge loop without ROS or hardware."""

import math
import sys
import threading
import time
import types
import unittest


if "rclpy" not in sys.modules:
    rclpy = types.ModuleType("rclpy")
    rclpy.__path__ = []
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
if "sensor_msgs" not in sys.modules:
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.__path__ = []
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = type("JointState", (), {})
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

from dofbot_servo_bridge.servo_bridge_node import ServoBridgeNode
from dofbot_servo_bridge.servo_protocol import JOINT_NAMES, ProtocolError, ServoMapping


class FakeLogger:
    def info(self, *args, **kwargs):
        pass

    warning = info
    error = info


class FakeBoard:
    def __init__(self, stop, missing_id=None, actual_raw=2048, stop_after_read=False):
        self.stop = stop
        self.missing_id = missing_id
        self.actual_raw = actual_raw
        self.stop_after_read = stop_after_read
        self.lines = []
        self.reply = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        line = data.decode("ascii").strip()
        self.lines.append(line)
        fields = line.split()
        if fields[0] == "RANGE":
            reply = f"OK RANGE {fields[1]}"
        elif fields[0] == "PING":
            reply = (f"ERR PING {fields[1]}" if int(fields[1]) == self.missing_id
                     else f"OK PING {fields[1]}")
        elif fields[0] == "READ":
            reply = f"POS {fields[1]} {self.actual_raw}"
            if fields[1] == "6" and self.stop_after_read:
                self.stop.set()
        elif fields[0] == "MOVE":
            reply = f"OK MOVE {fields[1]} 2048"
            if fields[1] == "6":
                self.stop.set()
        else:
            raise AssertionError(line)
        self.reply.append((reply + "\r\n").encode("ascii"))

    def readline(self):
        return self.reply.pop(0) if self.reply else b""


def make_bridge():
    node = object.__new__(ServoBridgeNode)
    node._mapping = ServoMapping(
        list(JOINT_NAMES), [1, 2, 3, 4, 5, 6], [150.0] * 6,
        [1.0] * 6, [180.0 / math.pi] * 6, [0.0] * 6, [300.0] * 6, 300,
    )
    node._lock = threading.Lock()
    node._positions = dict.fromkeys(JOINT_NAMES, 0.0)
    node._state_times = dict.fromkeys(JOINT_NAMES, time.monotonic())
    node._state_timeout = 3.0
    node._start_tolerance = 10.0
    node._deadband = 0.2
    node._move_ms = 120
    node._rate = 100.0
    node._stop = threading.Event()
    node.get_logger = lambda: FakeLogger()
    return node


class BridgeFlowTests(unittest.TestCase):
    def test_six_axes_are_checked_before_motion(self):
        node = make_bridge()
        board = FakeBoard(node._stop)
        node._connect_and_mirror(board)
        self.assertEqual(board.lines[0], "RANGE 300")
        self.assertEqual(board.lines[1:7], [f"PING {i}" for i in range(1, 7)])
        self.assertEqual(board.lines[7:13], [f"READ {i}" for i in range(1, 7)])
        self.assertEqual(board.lines[13:], [f"MOVE {i} 150.00 120" for i in range(1, 7)])

    def test_missing_sixth_servo_prevents_all_motion(self):
        node = make_bridge()
        board = FakeBoard(node._stop, missing_id=6)
        with self.assertRaises(ProtocolError):
            node._connect_and_mirror(board)
        self.assertFalse(any(line.startswith("MOVE") for line in board.lines))

    def test_misaligned_start_prevents_motion(self):
        node = make_bridge()
        board = FakeBoard(node._stop, actual_raw=2600, stop_after_read=True)
        node._connect_and_mirror(board)
        self.assertFalse(any(line.startswith("MOVE") for line in board.lines))

    def test_stale_joint_blocks_motion(self):
        node = make_bridge()
        node._state_times["Arm3_Joint"] -= 10.0
        self.assertIsNone(node._targets())


if __name__ == "__main__":
    unittest.main()
