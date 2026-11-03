"""Locally checked, slow hardware approach. Stops at pregrasp for visual review."""
import argparse,json,math,sys,time,itertools
from pathlib import Path
import serial
from contextlib import contextmanager
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.protocol import ServoMapping,read_position,request
from dofbot_mj.servo_link import detect_port
from dofbot_mj.collision import CollisionChecker
p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
plan=json.loads(a.plan.read_text());cfg=load_config(ROOT/'config/visual_match.yaml')
v=json.loads((ROOT/'calib/handeye_validation_report_20261103.json').read_text())
if not v['passed']:raise RuntimeError('Handeye holdout failed')
m=ServoMapping(cfg.servo.joints,cfg.servo.range_deg);check=CollisionChecker(ROOT/'models/dofbot_pro_collision_visual.xml')
frames=[f for f in plan['frames'] if f['stage'] in ('open','pregrasp')]
for f in frames:
    servo=m.targets(f['q_rad'])
    servo[6]=max(140.,servo[6])  # Current photo confirms sufficient aperture here.
    f['q_rad']=m.positions_from_targets(servo)
for f in frames:
    m.targets(f['q_rad'])
    for signs in itertools.product((-1,1),repeat=4):
        q=dict(f['q_rad'])
        for j,sign in enumerate(signs,1):q[f'Arm{j}_Joint']+=math.radians(sign*(2 if j==4 else 1.5))
        if check.check(q):raise RuntimeError('Approach joint-error envelope collision')
print('Checked approach frames:',len(frames),flush=True)
if not a.execute:sys.exit(0)
log=[]
@contextmanager
def guarded_serial(ser):
    try:
        yield ser
    except BaseException:
        for sid in range(1,7):
            try:
                value=read_position(ser,sid)*300/4095
                request(ser,f'MOVE {sid} {value:.2f} 700',f'OK MOVE {sid}',timeout=3)
            except Exception:pass
        raise
    finally:ser.close()
with guarded_serial(serial.Serial(cfg.servo.port or detect_port(),cfg.servo.baudrate,timeout=.25,write_timeout=1)) as s:
    time.sleep(1.5);s.reset_input_buffer();request(s,'RANGE 300','OK RANGE 300',timeout=8)
    def read_all():
        d={}
        for sid in range(1,7):
            for trial in range(3):
                try:d[sid]=read_position(s,sid)*300/4095;break
                except Exception:
                    if trial==2:raise
        return d
    actual=read_all();initial=m.targets(plan['frames'][0]['q_rad'])
    if max(abs(actual[i]-initial[i]) for i in actual)>2:
        distances=[max(abs(actual[i]-m.targets(f['q_rad'])[i]) for i in actual) for f in frames]
        nearest=min(range(len(frames)),key=lambda i:distances[i])
        if distances[nearest]>1.5:raise RuntimeError('Pose is not on checked approach path')
        print('Resuming checked path at frame',nearest,flush=True)
        frames=frames[nearest:]
    for n,f in enumerate(frames):
        desired=m.targets(f['q_rad'])
        tolerances={i:(2.0 if i==4 else 1.5) for i in actual}
        def outside():return any(abs(desired[i]-actual[i])>tolerances[i] for i in actual)
        if not outside():continue
        attempts=0
        while outside():
            attempts+=1
            if attempts>4:raise RuntimeError('Target did not settle within four checked commands')
            before=dict(actual);delta=max(abs(desired[i]-actual[i]) for i in actual)
            target={i:actual[i]+max(-(3. if i==4 else 2.),min(3. if i==4 else 2.,desired[i]-actual[i])) for i in actual}
            for t in (0,.25,.5,.75,1):
                blend={i:actual[i]+t*(target[i]-actual[i]) for i in actual}
                for signs in itertools.product((-1,1),repeat=4):
                    uncertain=dict(blend)
                    for j,sign in enumerate(signs,1):uncertain[j]+=sign*(2. if j==4 else 1.5)
                    if check.check(m.positions_from_targets(uncertain)):raise RuntimeError('Live ID4 uncertainty envelope collision')
            for i in target:
                if abs(desired[i]-actual[i])>tolerances[i]:
                    request(s,f'MOVE {i} {target[i]:.2f} 700',f'OK MOVE {i}',timeout=3)
            time.sleep(1.1);actual=read_all()
            log.append({'frame':n,'target':target,'actual':actual})
            (ROOT/'out/real_approach_20261103_log.json').write_text(json.dumps(log,indent=2))
            err=max(abs(target[i]-actual[i]) for i in actual)
            movement=max(abs(actual[i]-before[i]) for i in actual)
            if any(abs(target[i]-actual[i])>(2.0 if i==4 else 1.5) for i in actual):raise RuntimeError(f'Position tracking failed: error={err:.2f}, movement={movement:.2f}')
        if n%15==0:print('approach',n,'/',len(frames),'readback',actual,flush=True)
    print('PREGRASP REACHED',actual,flush=True)
