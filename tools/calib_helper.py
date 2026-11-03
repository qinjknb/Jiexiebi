#!/usr/bin/env python3
"""关节标定助手（Windows 双击运行，直接连 CH340 控制真机）。

比在上位机里手打 MOVEP 命令快得多：输入 `2 +100` 就把 ID2 往正方向挪 100 tick。

用法：
    1) 先在上位机里点【断开】释放串口
    2) 双击 calib_helper.bat（或 python calib_helper.py [COM口]）

命令一览（每行输完回车）：
    r            读 6 个舵机当前位置（tick）
    2 +100       ID2 往正方向挪 100 tick
    2 -100       ID2 往负方向挪 100 tick
    3 2047       ID3 直接到 2047 tick
    step 50      把默认步长改成 50 tick（此时直接输 `2 +` 就用这个步长）
    y           打印可直接粘进 arm_calib.yaml 的模板（center 用刚读到的值）
    p           重新 PING 一遍 6 个舵机（查掉线）
    q           退出

安全提示：每次只动一个关节、步长别超过 200 tick；手放在电源插头旁边。
"""

from __future__ import annotations

import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial：pip install pyserial")
    input("\n按回车退出 ...")
    raise SystemExit(2)

BAUD = 115200
JOINTS = ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint",
          "Arm4_Joint", "Arm5_Joint", "grip_joint"]
DEFAULT_CURRENT = [2047] * 6


def pick_port(argv) -> str:
    if len(argv) > 1 and argv[1]:
        return argv[1]
    for p in list_ports.comports():
        d = (p.description or "").upper()
        if any(k in d for k in ("CH340", "CH343", "CP210", "USB-SERIAL", "FT232")):
            return p.device
    return None       # 没找到 USB 串口，不要乱选蓝牙串口


class Mcu:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, BAUD, timeout=0.2)
        self.port = port
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass

    def cmd(self, line: str, wait: float = 0.7) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(line.encode() + b"\n")
        self.ser.flush()
        time.sleep(wait)
        return self.ser.read(2048).decode("ascii", "replace").strip()

    def positions(self):
        rep = self.cmd("JOINTS", 1.0)
        if not rep.startswith("JOINTS"):
            return None
        vals = []
        for tok in rep.split()[1:7]:
            vals.append(None if tok == "?" else int(tok))
        return vals if len(vals) == 6 else None

    def move(self, sid: int, tick: int, ms: int = 800) -> str:
        tick = max(0, min(4095, int(tick)))
        return self.cmd("MOVEP %d %d %d" % (sid, tick, ms), 0.25)

    def read_one(self, sid: int):
        rep = self.cmd("READ %d" % sid, 0.3)
        m = re.match(r"POS\s+\d+\s+(\d+)", rep)
        return int(m.group(1)) if m else None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def show(cur, mark=None):
    print("    当前 tick: ", end="")
    for i, v in enumerate(cur):
        tag = ("*%d:%s" % (i + 1, v if v is not None else "?"))
        print(tag.ljust(10), end="")
    print()


def yaml_block(cur):
    print("\n# ---- 复制到 dofbot_hw_bridge/config/arm_calib.yaml 的 joints: 下 ----")
    for i, name in enumerate(JOINTS):
        c = cur[i] if cur[i] is not None else 2047
        print("  %s:\n"
              "    servo_id: %d\n"
              "    center: %d          # ← 实测\n"
              "    direction: 1         # ← 第2步判定，反了就改 -1\n"
              "    tick_min: %d\n"
              "    tick_max: %d\n"
              "    range_deg: 300" % (name, i + 1, c, c - 1228, c + 1228))
    print("# ----------------------------------------------------------------\n")


def main() -> int:
    port = pick_port(sys.argv)
    if not port:
        print("\n没找到 USB 串口（CH340/CP210x）！")
        print("可能原因：1) USB-TTL 没插  2) 被 VMware 虚拟机占用了")
        print("  -> VMware 菜单：VM -> Removable Devices -> USB-SERIAL CH340 -> Disconnect(Connect to Host)")
        print("  -> 也可以手动指定：  check 时用  python calib_helper.py COM7")
        pause()
        return 1
    print("=" * 66)
    print(" DOFBOT Pro 关节标定助手")
    print("=" * 66)
    print("串口设备：")
    for p in list_ports.comports():
        print("   %-8s %s" % (p.device, p.description))
    print("\n使用 %s @ %d（若打不开，先在上位机点【断开】）" % (port, BAUD))

    try:
        mcu = Mcu(port)
    except Exception as exc:
        print("\n打不开 %s：%s" % (port, exc))
        pause()
        return 1

    try:
        cur = mcu.positions()
        if cur is None:
            print("\n读不到位置。请先确认：STM32 已复位、PING 1 能回 OK。")
            print("（本助手默认固件总线接法 WIRING 2；若你换过接法，先在上位机发 WIRING 2）")
            cur = list(DEFAULT_CURRENT)
        else:
            print("\n读到了 6 个位置 ✔")
        show(cur)
        print("""
输入命令（回车执行，q 退出）：
   r          重读位置
   2 +100     ID2 往正方向挪 100 tick
   2 -100     ID2 往负方向挪 100 tick
   3 2047     ID3 直接到 2047
   step 50    改默认步长（之后可以只输  2 + ）
   y          生成可粘贴的 arm_calib.yaml 片段
   p          重新 PING 6 个舵机
""")

        step = 100
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            low = line.lower()
            if low in ("q", "quit", "exit"):
                break
            if low == "r":
                cur = mcu.positions() or cur
                show(cur)
                continue
            if low == "y":
                yaml_block(cur)
                continue
            if low == "p":
                online = []
                for sid in range(1, 7):
                    rep = mcu.cmd("PING %d" % sid, 0.35)
                    if rep.startswith("OK PING"):
                        online.append(sid)
                print("    在线:", online if online else "无")
                continue
            if low.startswith("step"):
                try:
                    step = int(low.split()[1])
                    print("    默认步长 =", step)
                except (IndexError, ValueError):
                    print("    用法：step 50")
                continue

            # 形如  "2 +100" / "2 -100" / "2 +" / "3 2047"
            m = re.match(r"^\s*(\d)\s*([+-]?)\s*(\d*)\s*$", line)
            if not m:
                print("    看不懂。示例： 2 +100  /  3 2047  /  r  /  y")
                continue
            sid = int(m.group(1))
            sign = m.group(2)
            num = m.group(3)
            if not 1 <= sid <= 6:
                print("    ID 必须是 1~6")
                continue

            now = mcu.read_one(sid)
            if now is None:
                print("    ID%d 读不到位置（在线吗？先试 p）" % sid)
                continue

            if sign in ("+", "-"):
                delta = int(num) if num else step
                target = now + delta
            else:
                target = int(num)

            if abs(target - now) > 800:
                print("    ⚠ 一次要动 %d tick，幅度偏大。确认机械臂有空间吗？" % abs(target - now))
                if input("    继续请按 y 回车，其它键取消: ").strip().lower() != "y":
                    print("    已取消")
                    continue

            mcu.move(sid, target)
            time.sleep(1.2)
            newpos = mcu.read_one(sid)
            print("    ID%d: %d -> %s" % (sid, now, newpos if newpos is not None else "?"))
            cur[sid - 1] = newpos
    finally:
        mcu.close()

    print("\n已退出。别忘了把 center/direction/tick_min/tick_max 填进 arm_calib.yaml")
    pause()
    return 0


def pause(msg="\n按回车退出 ..."):
    try:
        input(msg)
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    raise SystemExit(main())
