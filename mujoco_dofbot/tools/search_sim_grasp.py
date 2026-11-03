#!/usr/bin/env python3
"""Simulation-only local search for finger alignment around one fixed fruit."""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
scene = ROOT / "out/current_pose_strawberry_pointcloud/scene.json"
cloud = ROOT / "out/current_pose_strawberry_pointcloud/pointcloud.npz"
base = json.loads((ROOT / "out/current_pose_strawberry_moved_sim/plan.json").read_text())
target = base["target_base_xyz_m"]
angles = (159.78, 89.74, 255.02, 192.97, 103.44, 199.41)
results = []
for dx in (-0.006, 0, 0.006):
    for dy in (-0.006, 0, 0.006):
        name = f"dx{dx*1000:+.0f}_dy{dy*1000:+.0f}"
        plan_dir = ROOT / "out/grasp_alignment_search" / name / "plan"
        sim_dir = ROOT / "out/grasp_alignment_search" / name / "sim"
        p = subprocess.run([sys.executable, str(ROOT / "tools/plan_strawberry_pick.py"),
                            "--world-target", str(target[0]+dx), str(target[1]+dy), str(target[2]),
                            "--servo-deg", *map(str, angles),
                            "--grasp-height-offset-m", "0.005", "--out", str(plan_dir)],
                           capture_output=True, text=True)
        if p.returncode:
            print(name, "plan failed", p.stderr.splitlines()[-1], flush=True)
            continue
        s = subprocess.run([sys.executable, str(ROOT / "tools/simulate_strawberry_grasp.py"),
                            "--plan", str(plan_dir / "plan.json"), "--scene", str(scene),
                            "--cloud", str(cloud), "--out", str(sim_dir), "--no-video"],
                           capture_output=True, text=True)
        if s.returncode:
            print(name, "sim failed", s.stderr.splitlines()[-1], flush=True)
            continue
        data = json.loads((sim_dir / "physics_result.json").read_text())
        row = (name, data["fruit_lifted"], data["both_pad_contact_substeps"],
               data["fruit_contact_substeps"])
        results.append(row)
        print(row, flush=True)
print("best", sorted(results, key=lambda x:(x[1],x[2],x[3]), reverse=True)[:3])
