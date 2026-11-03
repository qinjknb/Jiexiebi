#!/usr/bin/env python3
"""Align the conservative collision proxy with the photo-matched visual model."""

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
source = ROOT / "models" / "dofbot_pro_collision.xml"
dest = ROOT / "models" / "dofbot_pro_collision_visual.xml"
xml = source.read_text(encoding="utf-8")
old_base = 'name="c_base_link" type="capsule"'
old_camera = 'name="DaBai_DCW2_Link" pos="-0.0002 0.0689 0.0506"'
old_floor = '    <geom name="floor" type="plane" size="1.5 1.5 0.05" pos="0 0 0" contype="1" conaffinity="1"/>'
old_body = '    <body name="base_link">'
if any(xml.count(item) != 1 for item in (old_base, old_camera, old_floor, old_body)):
    raise RuntimeError("Collision model layout changed; inspect before regenerating")
xml = xml.replace(old_floor,
                  '    <geom name="tabletop" type="box" pos="0 0 -0.020" '
                  'size="0.40 0.50 0.025" contype="1" conaffinity="1" '
                  'rgba="0.35 0.32 0.27 1"/>')
xml = xml.replace(old_body, '    <body name="base_link" pos="0 0 0.015">')
xml = xml.replace(old_base, old_base + ' quat="0 0 0 1"')
xml = xml.replace(old_camera,
                  'name="DaBai_DCW2_Link" pos="-0.0002 0.0689 -0.02"')
dest.write_text(xml, encoding="utf-8")
print(dest)
