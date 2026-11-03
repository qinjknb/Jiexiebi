# Python strawberry pick preview

This pipeline uses the workspace YOLO model, aligned Orbbec RGB-D, SciPy IK,
and MuJoCo. It does not use MATLAB or command the physical arm.

## Run

The Orbbec camera is exclusive. Close `strawberry_demo.py` before the one-frame
capture, then reopen it if a live view is needed.

```powershell
cd D:\jixiebi_pro\3\yolo_pkg
.\.venv\python.exe capture_targets.py --conf 0.35 --out out\targets.json
.\.venv\python.exe estimate_table_plane.py out\targets.depth.npy

cd D:\jixiebi_pro\3\mujoco_dofbot
.\.venv\Scripts\python.exe tools\plan_strawberry_pick.py `
  --targets ..\yolo_pkg\out\targets.json `
  --table-plane ..\yolo_pkg\out\targets.depth.table.json `
  --target-id 1 --read-servo --out out\strawberry_pick
.\.venv\Scripts\python.exe tools\render_pick_video.py `
  out\strawberry_pick\plan.json
```

The capture saves the YOLO overlay, raw RGB image, aligned depth array, and
target JSON. The plane estimator fits the cutting mat in the same depth frame.
The planner produces five stages: open, pregrasp, grasp, close, lift. It saves
305 interpolated joint states, three MuJoCo renders, and a JSON plan. The video
renderer illustrates an idealized fruit attachment after closing.

## YOLO + point-cloud MuJoCo scene

After producing the capture, table plane, and pick plan above, run:

```powershell
cd D:\jixiebi_pro\3\mujoco_dofbot
.\.venv\Scripts\python.exe tools\model_strawberry_pointcloud.py `
  --targets ..\yolo_pkg\out\targets.json `
  --rgb ..\yolo_pkg\out\targets.rgb.png `
  --depth ..\yolo_pkg\out\targets.depth.npy `
  --table-plane ..\yolo_pkg\out\targets.depth.table.json `
  --plan out\strawberry_pick\plan.json `
  --out out\strawberry_pointcloud_scene --view
```

The script filters the YOLO box by red color, valid depth, and height above the
fitted mat. It places the measured strawberry surface points and a sparse mat
cloud beside the robot in MuJoCo. It writes a reusable static MuJoCo scene XML,
colored PLY, compressed NumPy cloud, fruit mask preview, scene metadata, and a
rendered PNG. `--view` opens an interactive window for orbiting the scene.

## Record and replay the grasp simulation

```powershell
cd D:\jixiebi_pro\3\mujoco_dofbot
.\.venv\Scripts\python.exe tools\simulate_strawberry_grasp.py `
  --plan out\strawberry_pick\plan.json `
  --scene out\strawberry_pointcloud_scene\scene.json `
  --cloud out\strawberry_pointcloud_scene\pointcloud.npz `
  --out out\strawberry_physics_pick

.\.venv\Scripts\python.exe tools\replay_strawberry_grasp.py `
  --plan out\strawberry_pick\plan.json `
  --result out\strawberry_physics_pick\physics_result.json `
  --scene out\strawberry_pointcloud_scene\scene.json `
  --cloud out\strawberry_pointcloud_scene\pointcloud.npz
```

The first command writes `grasp_pointcloud.mp4`, key frames, and physics
measurements. The replay command loops the recorded MuJoCo states in an
interactive window. Two simplified finger-pad capsules and a dynamic fruit
ellipsoid produce contact during closing. A weld constraint is enabled only
after both pads contact the fruit for three physics steps, which keeps it in
the gripper during lift. `--no-latch --no-video` runs a pure contact diagnostic;
in the captured scene that version slid the fruit instead of lifting it.
The contact-triggered weld is an assisted grasp, not a validated force-closure
model or a physical-arm command.

## Interpretation

The 2026-10-31 center-strawberry capture detected a ripe fruit at pixel
`(371, 251)`, depth `279 mm`, confidence `0.725`, with full valid-depth coverage.
The measured table plane put its center about `22 mm` above the mat. The
simulation target was approximately `(-0.029, -0.304, 0.022) m` in the current
model base frame and was reachable by the numerical IK solver.

This is a simulation plan, not a verified hardware grasp. The visual model's
camera X/Y placement and yaw are estimated, the servo-to-joint mapping is
photo-based, and the model has no active robot collision geometry. The planner
checks joint limits and a 12 mm tool-point floor margin only. The rendered
fruit follows the gripper after closing by an idealized attachment; MuJoCo
contact forces do not establish a grasp. Calibrate hand-eye geometry and servo
motion, then add collision and contact checks before sending the plan to the
physical arm.
