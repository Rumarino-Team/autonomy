"""Replay and OS-thread scheduling must not affect a case's scene or outcome."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

exe, plugin, backend, config = map(lambda p: str(Path(p).resolve()), sys.argv[1:5])
env = {k:v for k,v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
env['AUTONOMY_SIM_BACKEND'] = backend
if len(sys.argv)>5:
    env['LD_PRELOAD'] = sys.argv[5] + (':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
base = [exe,plugin,'prequalify_robosub',config,'--fuzz']

def read(path): return json.loads(Path(path).read_text())
def records(path): return [read(p) for p in sorted(Path(path).glob('case-*.json'))]
def run(path,*flags,extra=None):
    result=subprocess.run(base+list(flags)+['--results',str(path)],env=env | (extra or {}),capture_output=True,text=True,timeout=360 if len(sys.argv)>5 else 180)
    assert result.returncode in (0,1), result.stderr[-5000:]
    assert 'panic' not in result.stderr.lower(), result.stderr[-5000:]
    cases=records(path)
    assert cases and len(cases)==len((Path(path)/'summary.jsonl').read_text().splitlines())
    totals=read(Path(path)/'totals.json')
    assert sum(totals.values())==len(cases)
    assert (result.returncode==0)==(totals['failed']==totals['cancelled']==0)
    return cases

with tempfile.TemporaryDirectory(prefix='sim-zig-fuzz-') as temporary:
    root=Path(temporary)
    serial=run(root/'serial','--cases','8','--workers','1')
    parallel=run(root/'parallel','--cases','8','--workers','8')
    assert serial==parallel, [(a['seed'],a['hash'],b['hash']) for a,b in zip(serial,parallel)]
    for case in serial:
        assert case['profile_version']==1 and case['backend']=='mujoco'
        assert case['resolved'] is not None and len(case['resolved']['faults'])==2
        assert .0<=sum(v*v for v in case['resolved']['current'])<=.150001**2
        assert all(o['body']['mass']>0 and o['body']['dry_mass']>0 for o in case['resolved']['objects'] if o['body']['motion_type']==1)
        assert all(abs(sum(q*q for q in o['body']['initial_pose']['quat']['buf'])-1)<1e-5 for o in case['resolved']['objects'])
        objects={o['body']['id']:o['body'] for o in case['resolved']['objects']}
        # Rigid gate rotation preserves the left/right post separation.
        p=objects[10]['initial_pose']['pos']['buf']; q=objects[11]['initial_pose']['pos']['buf']
        assert abs(sum((a-b)**2 for a,b in zip(p,q))**.5-3.3)<1e-5
        assert objects[100]['initial_pose']['pos']['buf']==[0,-10,5]
        sensor_ids={s['id'] for s in case['resolved']['sensors']}
        sensor_fault=case['resolved']['faults'][1]
        assert sensor_fault['sensor_id'] in sensor_ids and (sensor_fault['start'],sensor_fault['end'])==(360,720)
    replay=run(root/'replay','--replay-seed',str(serial[3]['seed']))[0]
    assert {k:v for k,v in replay.items() if k!='index'}=={k:v for k,v in serial[3].items() if k!='index'}
    nominal=run(root/'nominal','--cases','2','--workers','8','--profile','nominal')
    assert all(c['outcome']=='passed' and c['tick']==13488 for c in nominal), nominal
    budget=run(root/'budget','--cases','3','--workers','2',extra={'AUTONOMY_SIM_MAX_TICKS':'6'})
    assert all(c['outcome']=='failed' and c['error_name']=='SimulationTickBudget' and c['tick']==6 for c in budget)
    invalid=run(root/'invalid','--cases','3','--workers','2',extra={'AUTONOMY_SIM_ROBOT':'missing'})
    assert all(c['outcome']=='failed' and c['error_name']=='UnknownRobot' for c in invalid)
    # Cancellation joins all workers and records unassigned cases too.
    path=root/'cancel'
    process=subprocess.Popen(base+['--cases','100','--workers','2','--results',str(path)],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
    try:
        deadline=time.monotonic()+(60 if len(sys.argv)>5 else 20)
        while not (path/'case-000000.json').exists():
            assert process.poll() is None
            assert time.monotonic()<deadline
            time.sleep(.01)
        process.send_signal(signal.SIGINT)
        _,stderr=process.communicate(timeout=60)
        assert process.returncode!=0 and 'FuzzInterrupted' in stderr, stderr[-4000:]
        totals=read(path/'totals.json')
        assert totals['cancelled']>0 and sum(totals.values())==100
    finally:
        if process.poll() is None: process.kill(); process.wait()
print('MuJoCo fuzz serial/8-thread/replay/nominal/failure/cancellation checks passed')
