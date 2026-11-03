"""Offline holdout validation; never sends robot commands."""
from pathlib import Path
import sys, json
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from tools.handeye_calib import aruco_board, robot_model, base_from_tool, _pose_of

result_path = ROOT / 'config/handeye_20261103.yaml'
r = yaml.safe_load(result_path.read_text(encoding='utf-8'))
d = yaml.safe_load((ROOT/'calib/handeye_validation_20261103.yaml').read_text(encoding='utf-8'))
board, _ = aruco_board(d['board'])
index = {int(mid):i for i,mid in enumerate(board.getIds().flatten())}
k = r['camera_matrix']
K = np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
_, robot = robot_model(ROOT/'config/visual_match.yaml')
for s in d['samples']:
    entries = [e for e in s['markers'] if int(e['id']) in index]
    obj = np.vstack([board.getObjPoints()[index[int(e['id'])]] for e in entries]).astype(np.float32)
    img = np.vstack([e['corners'] for e in entries]).astype(np.float32)
    ok, rv, tv = cv2.solvePnP(obj,img,K,np.array(k['dist']))
    if not ok: raise RuntimeError('PnP failed')
    R,_ = cv2.Rodrigues(rv)
    T = base_from_tool(robot,s['joint_rad']) @ np.array(r['T_tool_cam']) @ _pose_of(R,tv.ravel())
    error = float(np.linalg.norm(T[:3,3]-np.array(r['marker_in_base_m'])))
    tilt = float(np.degrees(np.arccos(np.clip(abs(T[2,2]),0,1))))
    report = {'sample':s['name'],'position_base_m':T[:3,3].tolist(),
              'position_error_mm':error*1000,'board_plane_tilt_deg':tilt,
              'board_height_mm':float(T[2,3]*1000),
              'passed':error<0.005 and abs(T[2,3])<0.005 and tilt<5}
    print(json.dumps(report,indent=2))
    (ROOT/'calib/handeye_validation_report_20261103.json').write_text(json.dumps(report,indent=2))
