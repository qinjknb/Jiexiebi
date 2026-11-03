"""Measure ID4 response in a checked 3-degree envelope, without compensation."""
import argparse,json,sys,time
from pathlib import Path
import serial
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from dofbot_mj.config import load_config
from dofbot_mj.protocol import ServoMapping,read_position,request
from dofbot_mj.servo_link import detect_port
from dofbot_mj.collision import CollisionChecker
p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');a=p.parse_args()
c=load_config(ROOT/'config/visual_match.yaml');m=ServoMapping(c.servo.joints,300)
k=CollisionChecker(ROOT/'models/dofbot_pro_collision_visual.xml')
out=ROOT/'out/id4_tracking_diagnostic_20261103.json'
with serial.Serial(c.servo.port or detect_port(),c.servo.baudrate,timeout=.25,write_timeout=1) as s:
    time.sleep(1.5);s.reset_input_buffer();request(s,'RANGE 300','OK RANGE 300',timeout=8)
    pose={i:read_position(s,i)*300/4095 for i in range(1,7)};base=pose[4]
    for n in range(31):
        state=dict(pose);state[4]=base-3+n*.1
        q=m.positions_from_targets(state)
        for name,value in q.items():
            joint=k.model.joint(name);lo,hi=joint.range
            if not lo<=value<=hi:raise RuntimeError('Joint limit')
        if k.check(q):raise RuntimeError('Collision envelope rejected')
    report={'baseline_deg':base,'initial_pose':pose,'trials':[]}
    print('baseline',base,'checked range',base-3,base,flush=True)
    if not a.execute:sys.exit(0)
    try:
        for delta in (-1,-2,-3,-2,-1,0):
            target=base+delta
            ack=request(s,f'MOVE 4 {target:.2f} 700','OK MOVE 4',timeout=3)
            trial={'target':target,'ack':ack,'samples':[]};report['trials'].append(trial)
            start=time.monotonic()
            while time.monotonic()-start<3:
                measured=read_position(s,4)*300/4095
                trial['samples'].append([time.monotonic()-start,measured])
                if not base-3.5<=measured<=base+.5:raise RuntimeError('Envelope exceeded')
                time.sleep(.2)
            trial['final']=trial['samples'][-1][1]
            trial['error']=trial['final']-target
            out.write_text(json.dumps(report,indent=2))
            print('target',round(target,2),'actual',round(trial['final'],2),'error',round(trial['error'],2),flush=True)
    finally:
        actual=read_position(s,4)*300/4095
        request(s,f'MOVE 4 {actual:.2f} 700','OK MOVE 4',timeout=3)
        report['held_deg']=actual;out.write_text(json.dumps(report,indent=2))
    print('saved',out,flush=True)
