#!/usr/bin/env python3
"""Read-only diagnostic: fit old board views and check a newer board frame."""
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from tools.handeye_calib import aruco_board, base_from_tool, robot_model, _pose_of


def observations(path, board):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    ids = [int(x) for x in board.getIds().flatten()]
    index = {mid: n for n, mid in enumerate(ids)}
    out = []
    for sample in data["samples"]:
        entries = [(board.getObjPoints()[index[int(e["id"])]].astype(np.float32),
                    np.asarray(e["corners"], dtype=np.float32))
                   for e in sample["markers"] if int(e["id"]) in index]
        if entries:
            obj = np.vstack([x[0] for x in entries])
            img = np.vstack([x[1] for x in entries])
            out.append((sample, obj, img))
    return out


def main():
    board_spec = yaml.safe_load((ROOT / "calib/board.yaml").read_text(encoding="utf-8"))
    board, _ = aruco_board(board_spec)
    old = observations(ROOT / "calib/handeye_samples.yaml", board)
    current = observations(ROOT / "calib/handeye_samples_live_20261101.yaml", board)
    flags = (cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
             cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    rms, K, dist, _, _ = cv2.calibrateCamera(
        [x[1] for x in old], [x[2] for x in old], (640, 480), None, None, flags=flags)
    print(f"old views={len(old)}, camera fx={K[0,0]:.1f}, reprojection RMS={rms:.2f}px")
    _, robot = robot_model(ROOT / "config/visual_match.yaml")

    def solve_pose(view):
        sample, obj, img = view
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist)
        if not ok:
            raise RuntimeError(sample["name"])
        R, _ = cv2.Rodrigues(rvec)
        return base_from_tool(robot, sample["joint_rad"]), _pose_of(R, tvec.reshape(3))

    old_poses = [solve_pose(x) for x in old]
    current_pose = solve_pose(current[-1])
    manual_board = current_pose[0] @ current_pose[1]
    print(f"manual identity: new board origin_z_mm={manual_board[2,3]*1000:.1f}, "
          f"normal={np.round(manual_board[:3,2],3).tolist()}")
    for name, method in (("PARK", cv2.CALIB_HAND_EYE_PARK),
                         ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
                         ("TSAI", cv2.CALIB_HAND_EYE_TSAI)):
        try:
            R, t = cv2.calibrateHandEye(
                [x[0][:3,:3] for x in old_poses], [x[0][:3,3] for x in old_poses],
                [x[1][:3,:3] for x in old_poses], [x[1][:3,3] for x in old_poses],
                method=method)
        except cv2.error as exc:
            print(name, "failed", str(exc).splitlines()[0])
            continue
        X = _pose_of(R, t.reshape(3))
        poses = [a @ X @ b for a, b in old_poses]
        xyz = np.array([x[:3,3] for x in poses])
        new_board = current_pose[0] @ X @ current_pose[1]
        normal = new_board[:3,2]
        print(f"{name}: tool_cam_mm={np.round(X[:3,3]*1000,1).tolist()}, "
              f"old board spread_std_mm={np.round(np.std(xyz,axis=0)*1000,1).tolist()}, "
              f"new board origin_z_mm={new_board[2,3]*1000:.1f}, "
              f"new board normal={np.round(normal,3).tolist()}")


if __name__ == "__main__":
    main()
