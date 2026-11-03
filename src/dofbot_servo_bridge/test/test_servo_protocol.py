import math
import unittest

from dofbot_servo_bridge.servo_protocol import (
    JOINT_NAMES, ProtocolError, ServoMapping, read_position, request,
)


class FakeSerial:
    def __init__(self, replies):
        self.replies = list(replies)
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def readline(self):
        return self.replies.pop(0) if self.replies else b""


class ServoProtocolTests(unittest.TestCase):
    def setUp(self):
        self.mapping = ServoMapping(
            list(JOINT_NAMES), [1, 2, 3, 4, 5, 6],
            [150.0] * 6, [1.0] * 6, [180.0 / math.pi] * 6,
            [0.0] * 6, [300.0] * 6, 300,
        )

    def test_six_joint_mapping(self):
        positions = dict.fromkeys(JOINT_NAMES, 0.0)
        positions["Arm2_Joint"] = -0.5
        positions["grip_joint"] = -1.2
        targets = self.mapping.targets(positions)
        self.assertEqual(len(targets), 6)
        self.assertAlmostEqual(targets[2], 121.35211, places=4)
        self.assertAlmostEqual(targets[6], 81.24506, places=4)

    def test_reject_out_of_range_and_nonfinite(self):
        positions = dict.fromkeys(JOINT_NAMES, 0.0)
        positions["Arm1_Joint"] = 4.0
        with self.assertRaises(ValueError):
            self.mapping.targets(positions)
        positions["Arm1_Joint"] = float("nan")
        with self.assertRaises(ValueError):
            self.mapping.targets(positions)

    def test_duplicate_servo_ids_rejected(self):
        with self.assertRaises(ValueError):
            ServoMapping(["a", "b"], [1, 1], [150] * 2, [1] * 2,
                         [57] * 2, [0] * 2, [300] * 2, 300)

    def test_ask_for_matching_ack(self):
        ser = FakeSerial([b"MODE DEMO\r\n", b"OK MOVE 4 2048\r\n"])
        self.assertEqual(request(ser, "MOVE 4 150.00 120", "OK MOVE 4 "),
                         "OK MOVE 4 2048")
        self.assertEqual(ser.writes, [b"MOVE 4 150.00 120\n"])

    def test_missing_id_and_board_reset_stop_motion(self):
        with self.assertRaises(ProtocolError):
            request(FakeSerial([b"ERR PING 6\r\n"]), "PING 6", "OK PING 6")
        with self.assertRaises(ProtocolError):
            request(FakeSerial([b"READY STM32F103\r\n"]), "MOVE 1 150 120", "OK MOVE 1 ")

    def test_read_position(self):
        ser = FakeSerial([b"POS 3 2048\r\n"])
        self.assertEqual(read_position(ser, 3), 2048)
        self.assertEqual(ser.writes, [b"READ 3\n"])
        self.assertAlmostEqual(self.mapping.degrees_from_position(2048), 150.03663, places=4)


if __name__ == "__main__":
    unittest.main()
