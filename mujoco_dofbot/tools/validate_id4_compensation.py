"""Hardware validation of bounded ID4 compensation, in one serial session."""
import argparse,json,sys,time
from pathlib import Path
import serial
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
parser=argparse.ArgumentParser()
parser.add_argument('--execute',action='store_true')
args=parser.parse_args()
if not args.execute:
    print('Dry run: no serial connection or motion. Supply/model verification required before --execute.')
    sys.exit(0)
import yaml
power=yaml.safe_load((ROOT/'config/servo_power.yaml').read_text(encoding='utf-8'))
if not power.get('power_verified'):
    raise RuntimeError('Servo model and loaded supply voltage must be verified before compensation testing')
from dofbot_mj.config import load_config
from dofbot_mj.protocol import ServoMapping,request,read_position
from dofbot_mj.servo_link import detect_port
from dofbot_mj.collision import CollisionChecker
from dofbot_mj.id4_tracking import move_id4_checked
c=load_config(ROOT/'config/visual_match.yaml');m=ServoMapping(c.servo.joints,300)
k=CollisionChecker(ROOT/'models/dofbot_pro_collision_visual.xml')
report={'events':[],'results':[],'passed':False};out=ROOT/'out/id4_compensation_validation_20261103.json'
def log(row):
    report['events'].append(row);out.write_text(json.dumps(report,indent=2))
with serial.Serial(c.servo.port or detect_port(),115200,timeout=.25,write_timeout=1) as s:
    time.sleep(1.5);s.reset_input_buffer();request(s,'RANGE 300','OK RANGE 300',timeout=8)
    pose={i:read_position(s,i)*300/4095 for i in range(1,7)};base=pose[4]
    def check(value):
        if not base-3<=value<=base+3:raise RuntimeError('Global test envelope exceeded')
        measured=read_position(s,4)*300/4095
        for j in range(11):
            deg=dict(pose);deg[4]=measured+(value-measured)*j/10
            q=m.positions_from_targets(deg)
            m.targets(q)
            for name,v in q.items():
                lo,hi=k.model.joint(name).range
                if not lo<=v<=hi:raise RuntimeError('ID4 joint limit')
            if k.check(q):raise RuntimeError('Compensation collision')
    try:
        for delta in (-1,0,-1,0):
            goal=base+delta
            actual=move_id4_checked(s,goal,check,log)
            row={'goal':goal,'actual':actual,'error_deg':actual-goal}
            report['results'].append(row);print(row,flush=True)
        report['passed']=all(abs(r['error_deg'])<=.6 for r in report['results'])
    finally:
        actual=read_position(s,4)*300/4095
        request(s,f'MOVE 4 {actual:.2f} 700','OK MOVE 4',timeout=3)
        report['held_deg']=actual;out.write_text(json.dumps(report,indent=2))
print('PASS',report['passed'],flush=True)
