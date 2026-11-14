"""Binary cases, seed replay and OS-thread scheduling preserve physical results."""
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from app_fixture import Application,read
_,plugin,backend,_=sys.argv[1:5]
preload=sys.argv[5] if len(sys.argv)>5 else None

def fuzz_settings(path,cases=8,workers=1,nominal=False):
    return '.{ .results="'+str(path)+'", .cases='+str(cases)+', .workers='+str(workers)+', .nominal='+str(nominal).lower()+' }'
def records(path):return [read(p,kind='case') for p in sorted(path.glob('case-*.bin'))]
def comparable(case):
    return {k:v for k,v in case.items() if k not in ('index','configuration','provenance')}
def run(app,path,*args,cases=8,workers=1,nominal=False):
    app.configure(fuzz=fuzz_settings(path,cases,workers,nominal));app.build()
    p=app.run(*(args or ('fuzz',)))
    assert p.returncode in (0,1),p.stderr[-5000:]
    assert 'panic' not in p.stderr.lower(),p.stderr[-5000:]
    rs=records(path);assert len(rs)==len(read(path/'summary.bin',kind='summary'))
    totals=read(path/'totals.bin',kind='totals');assert sum(totals.values())==len(rs)
    assert (p.returncode==0)==(totals['failed']==totals['cancelled']==0),p.stderr[-5000:]
    return rs
with tempfile.TemporaryDirectory(prefix='sim-binary-fuzz-') as tmp:
    root=Path(tmp);app=Application(root/'app',plugin,backend,preload)
    serial=run(app,root/'serial',cases=2 if preload else 8)
    parallel=run(app,root/'parallel',cases=2 if preload else 8,workers=8)
    assert [comparable(c) for c in serial]==[comparable(c) for c in parallel]
    separations=[]
    for case in serial:
        assert case['profile_version']==1 and case['backend']=='mujoco'
        assert case['profile']['rotation']==[1.05,1.05,1.15]
        assert case['resolved'] is not None and len(case['resolved']['faults'])==2
        assert sum(v*v for v in case['resolved']['current'])<=.150001**2
        assert all(o['body']['mass']>0 and o['body']['dry_mass']>0 for o in case['resolved']['objects'] if o['body']['motion_type']==1)
        assert all(abs(sum(q*q for q in o['body']['initial_pose']['quat']['buf'])-1)<1e-5 for o in case['resolved']['objects'])
        objects={o['body']['id']:o['body'] for o in case['resolved']['objects']}
        a=objects[10]['initial_pose']['pos']['buf'];b=objects[11]['initial_pose']['pos']['buf']
        separations.append(sum((x-y)**2 for x,y in zip(a,b))**.5)
        assert abs(objects[104]['initial_pose']['pos']['buf'][2]-3.38)<1e-5
        assert objects[104]['initial_pose']['pos']['buf'][:2]==[0,10]
        fault=case['resolved']['faults'][1]
        assert fault['sensor_id'] in {s['id'] for s in case['resolved']['sensors']}
        assert (fault['start'],fault['end'])==(360,720)
    assert max(separations)-min(separations)<1e-5,separations
    replay=run(app,root/'replay','replay',str(serial[1]['seed']))[0]
    assert comparable(replay)==comparable(serial[1])
    # File replay restores recorded settings even when today's profile differs.
    replay_file=run(app,root/'file-replay','replay',str(root/'serial/case-000001.bin'),nominal=True)[0]
    assert comparable(replay_file)==comparable(serial[1])
    nominal=run(app,root/'nominal',cases=2,workers=8,nominal=True)
    assert all(c['outcome']=='passed' and c['tick']==14988 for c in nominal),nominal
    base={o['body']['id']:o['body'] for o in nominal[0]['resolved']['objects']}
    a=base[10]['initial_pose']['pos']['buf'];b=base[11]['initial_pose']['pos']['buf']
    expected=sum((x-y)**2 for x,y in zip(a,b))**.5
    assert all(abs(d-expected)<1e-5 for d in separations),(separations,expected)
    app.configure(max_ticks=6)
    budget=run(app,root/'budget',cases=3,workers=2)
    assert all(c['outcome']=='failed' and c['error_name']=='SimulationTickBudget' and c['tick']==6 for c in budget)
    # Initialization failures are recorded per case without aborting the batch.
    app.configure(robot='missing',max_ticks=21600)
    invalid=run(app,root/'invalid',cases=3,workers=2)
    assert all(c['outcome']=='failed' and c['error_name']=='UnknownRobot' for c in invalid)
    app.configure(robot='hydrus',fuzz=fuzz_settings(root/'cancel',cases=100,workers=2));app.build()
    p=subprocess.Popen([str(app.exe),'fuzz'],cwd=app.root,env=app.env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
    try:
        deadline=time.monotonic()+90
        while not (root/'cancel/case-000000.bin').exists():
            assert p.poll() is None
            assert time.monotonic()<deadline
            time.sleep(.02)
        p.send_signal(signal.SIGINT);_,err=p.communicate(timeout=90)
        assert p.returncode!=0 and 'FuzzInterrupted' in err,err[-5000:]
        totals=read(root/'cancel/totals.bin',kind='totals')
        assert totals['cancelled']>0 and sum(totals.values())==100
    finally:
        if p.poll() is None:p.kill();p.wait()
print('Binary serial/8-worker/seed and file replay/nominal/failure/cancellation checks passed.')
