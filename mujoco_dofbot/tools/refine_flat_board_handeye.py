"""Offline full-pose handeye refinement with the board flat on the table."""
from pathlib import Path
import sys
import cv2, yaml
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from tools.handeye_calib import aruco_board,robot_model,base_from_tool,_pose_of
p=ROOT/'config/handeye_20261103.yaml'
r=yaml.safe_load(p.read_text(encoding='utf-8'))
d=yaml.safe_load((ROOT/'calib/handeye_samples_20261103.yaml').read_text(encoding='utf-8'))
board,_=aruco_board(d['board']); ids={int(v):i for i,v in enumerate(board.getIds().ravel())}
k=r['camera_matrix']; K=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1.]])
_,robot=robot_model(ROOT/'config/visual_match.yaml')
poses=[]
for s in d['samples']:
    entries=s['markers']
    obj=np.vstack([board.getObjPoints()[ids[int(e['id'])]] for e in entries]).astype(np.float32)
    img=np.vstack([e['corners'] for e in entries]).astype(np.float32)
    ok,rv,tv=cv2.solvePnP(obj,img,K,np.array(k['dist']))
    if not ok: raise RuntimeError('PnP failed')
    poses.append((base_from_tool(robot,s['joint_rad']),_pose_of(cv2.Rodrigues(rv)[0],tv.ravel())))
X0=np.array(r['T_tool_cam']); W0=poses[0][0]@X0@poses[0][1]
flip=np.diag([1.,-1.,-1.]) if W0[2,2]<0 else np.eye(3)
yaw=np.arctan2(W0[1,0],W0[0,0])
x0=np.r_[Rotation.from_matrix(X0[:3,:3]).as_rotvec(),X0[:3,3],W0[:2,3],yaw]
def residual(v):
    X=_pose_of(Rotation.from_rotvec(v[:3]).as_matrix(),v[3:6])
    W=_pose_of(Rotation.from_euler('z',v[8]).as_matrix()@flip,np.r_[v[6:8],0.])
    out=[]
    for A,B in poses:
        T=A@X@B
        out.extend((T[:3,3]-W[:3,3])/.003)
        out.extend(Rotation.from_matrix(W[:3,:3].T@T[:3,:3]).as_rotvec()/.03)
    return out
fit=least_squares(residual,x0,loss='soft_l1',max_nfev=2000)
X=_pose_of(Rotation.from_rotvec(fit.x[:3]).as_matrix(),fit.x[3:6])
positions=np.array([(A@X@B)[:3,3] for A,B in poses])
print('camera_offset_mm',np.round(X[:3,3]*1000,2),'spread_mm',np.round(np.std(positions,axis=0)*1000,2))
r['T_tool_cam']=X.tolist();r['marker_in_base_m']=np.r_[fit.x[6:8],0.].tolist()
r['method']='FULL_POSE_FLAT_BOARD';r['spread_std_mm']=float(np.mean(np.std(positions,axis=0))*1000)
r['validation_status']='pending';r['flat_board_constraint']=True
(ROOT/'config/handeye_20261103_before_refinement.yaml').write_text(p.read_text(encoding='utf-8'),encoding='utf-8')
p.write_text(yaml.safe_dump(r,allow_unicode=True,sort_keys=False),encoding='utf-8')
