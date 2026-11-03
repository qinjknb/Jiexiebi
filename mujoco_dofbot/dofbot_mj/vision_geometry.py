"""RGB-D camera registration to the MuJoCo base frame.

The table plane corrects camera pitch, roll and height from one depth frame.
X/Y placement and yaw still come from the approximate visual robot model.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

from .model import RobotModel


CAM_MOUNT = np.array([[-1., 0., 0.],
                      [0., 0., 1.],
                      [0., 1., 0.]])


def camera_pose(robot: RobotModel, table_plane: dict | None = None, handeye: dict | None = None):
    """Return optical-frame origin and rotation in robot-base coordinates."""
    camera_id = robot.model.body("DaBai_DCW2_Link").id
    pos = robot.data.xpos[camera_id].copy()
    rot = robot.data.xmat[camera_id].reshape(3, 3) @ CAM_MOUNT
    details = {"mode": "approximate_visual_mount"}
    if handeye is not None:
        X = np.asarray(handeye['T_tool_cam'], float)
        link_rot = robot.data.xmat[camera_id].reshape(3, 3)
        pos = pos + link_rot @ X[:3, 3]
        rot = link_rot @ X[:3, :3]
        details = {'mode': 'calibrated_handeye'}
    if table_plane is None:
        return pos, rot, details

    normal = np.array(table_plane["normal_camera"], float)
    height = float(table_plane["camera_height_above_table_m"])
    fraction = float(table_plane["inlier_fraction"])
    if (not np.isfinite(normal).all() or
            abs(np.linalg.norm(normal) - 1) > 0.01 or
            not 0.1 < height < 0.5 or fraction < 0.7):
        raise RuntimeError("Table plane failed geometric quality checks")
    predicted_world_normal = rot @ normal
    correction, _ = Rotation.align_vectors([[0, 0, 1]], [predicted_world_normal])
    angle = float(correction.magnitude())
    if angle > math.radians(25):
        raise RuntimeError("Table plane disagrees with camera mount by over 25 degrees")
    rot = correction.as_matrix() @ rot
    pos[2] = height
    details = {
        "mode": "table_plane_corrected",
        "camera_height_m": height,
        "orientation_correction_deg": math.degrees(angle),
        "table_inlier_fraction": fraction,
        "table_median_residual_m": float(table_plane["median_residual_m"]),
        "remaining_uncertainty": "camera X/Y and yaw from visual model",
    }
    return pos, rot, details
