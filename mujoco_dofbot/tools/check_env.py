#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""环境自检：依赖、模型、SRDF、串口，一条一条给结论。

用法：
    python tools/check_env.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OK, BAD, WARN = "[OK]  ", "[缺失]", "[注意]"


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError):
        pass

    problems = 0
    print(f"Python {sys.version.split()[0]}  ({sys.executable})")
    if sys.version_info < (3, 9):
        print(f"{BAD} 需要 Python 3.9+")
        problems += 1

    for name in ("numpy", "mujoco", "yaml"):
        try:
            module = __import__(name)
            print(f"{OK} {name} {getattr(module, '__version__', '')}")
        except ImportError:
            print(f"{BAD} {name} 未安装（pip install -r requirements.txt）")
            problems += 1

    try:
        import serial
        import serial.tools.list_ports

        print(f"{OK} pyserial {serial.__version__}")
        ports = list(serial.tools.list_ports.comports())
        ch340 = [p.device for p in ports if "CH340" in (p.description or "").upper()]
        print(f"       串口 {len(ports)} 个"
              + (f"，CH340: {', '.join(ch340)}" if ch340 else "，未发现 CH340"))
    except ImportError:
        print(f"{WARN} pyserial 未安装：只能试运行，不能驱动实物")

    from dofbot_mj.config import load_config

    settings = load_config(ROOT / "config" / "default.yaml")
    print(f"{OK} 配置 {settings.root / 'config' / 'default.yaml'}")

    model_path = settings.model.mjcf
    if not model_path.is_file():
        print(f"{BAD} 模型不存在：{model_path}")
        print("       生成：python -m dofbot_mj.urdf2mjcf "
              "../src/dofbot_pro_description/urdf/DOFBOT_Pro-V24.urdf -o models/dofbot_pro.xml")
        problems += 1
    else:
        import time

        import mujoco

        started = time.time()
        model = mujoco.MjModel.from_xml_path(str(model_path))
        cost = time.time() - started
        triangles = int(model.mesh_facenum.sum()) if model.mesh_facenum.size else 0
        print(f"{OK} 模型 {model_path.name}：{model.njnt} 关节 / {model.nu} 执行器 / "
              f"{triangles:,} 三角形，编译 {cost:.2f}s")

    if settings.model.srdf.is_file():
        from dofbot_mj.poses import parse_srdf

        states = parse_srdf(settings.model.srdf)["states"]
        print(f"{OK} SRDF 姿态：{', '.join(sorted(states))}")
    else:
        print(f"{WARN} 找不到 SRDF：{settings.model.srdf}（命名姿态将不可用）")

    servo = settings.servo
    print(f"       舵机：port={servo.port or '自动'} test_mode={servo.test_mode} "
          f"calibration_confirmed={servo.calibration_confirmed}")
    if not servo.test_mode and not servo.calibration_confirmed:
        print(f"{WARN} 实机模式但未确认真机标定，运行时会被拒绝")

    print()
    if problems:
        print(f"有 {problems} 项未就绪，按上面的提示处理后重跑。")
        return 1
    print("环境就绪。先跑：python run.py --headless --speed 0 --repeat 1 --servo dry")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
