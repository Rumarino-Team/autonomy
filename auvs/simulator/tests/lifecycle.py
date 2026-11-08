"""Live application replacement, rollback, fresh restart, SIGINT and legacy AUV."""
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from app_fixture import Application
_,plugin,backend,_,stationary,_=sys.argv[1:7]
preload=sys.argv[7] if len(sys.argv)>7 else None

def wait(process,path,needle,count=1):
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        text=path.read_text()
        if text.count(needle)>=count:return text
        assert process.poll() is None,text[-8000:]
        time.sleep(.05)
    raise AssertionError(path.read_text()[-8000:])

with tempfile.TemporaryDirectory(prefix='sim-app-reload-') as tmp:
    root=Path(tmp)
    app=Application(root/'reload',plugin,backend,preload,mission='.teleop',scene='robot_only',realtime=True,
                    reload='.{ .watch=true, .preserve_state=true }')
    outpath,errpath=root/'out',root/'err'
    with outpath.open('w') as out,errpath.open('w') as err:
        p=subprocess.Popen([str(app.exe),'run'],cwd=app.root,env=app.env,stdout=out,stderr=err)
        try:
            wait(p,errpath,'application ready')
            source=app.root/'src/MissionContext.zig'
            source.write_text(source.read_text()+'\n// rebuilt MissionContext code\n')
            wait(p,errpath,'state preserved')
            # Invalid source must not replace/destroy the running generation.
            cfg=app.root/'config/application.zig';valid=cfg.read_text()
            cfg.write_text(valid+'this is not valid Zig;\n')
            wait(p,errpath,'build failed; kept previous application')
            cfg.write_text(valid)
            wait(p,errpath,'state preserved',2)
            app.configure(scene='robosub_course')
            wait(p,errpath,'StructuralChangeRequiresRestart')
            app.configure(scene='robot_only',reload='.{ .watch=true, .preserve_state=false }')
            wait(p,errpath,'fresh world')
            p.send_signal(signal.SIGINT)
            assert p.wait(timeout=15)==0,errpath.read_text()[-5000:]
        finally:
            if p.poll() is None:p.kill();p.wait()
    summaries=[line for line in outpath.read_text().splitlines() if line.startswith('simulation ')]
    assert len(summaries)==2,summaries
    log=errpath.read_text()
    pauses=re.findall(r'paused tick=(\d+) hash=(\d+)',log)
    resumes=re.findall(r'resumed tick=(\d+) hash=(\d+)',log)
    assert len(pauses)==len(resumes)>=3,(pauses,resumes,log[-5000:])
    # Fresh world is deliberately excluded from preserve/rollback assertions.
    assert pauses[:-1]==resumes[:-1],(pauses,resumes)
    assert int(resumes[-1][0])==0,resumes
    # Stationary hardware-compatible plugin is retained across application reload.
    legacy=Application(root/'legacy',stationary,backend,preload,mission='.teleop',controller='@import("stationary.zig").controller',reload='.{ .watch=false, .preserve_state=true }')
    with errpath.open('w') as err:
        p=subprocess.Popen([str(legacy.exe),'run'],cwd=legacy.root,env=legacy.env,stdout=subprocess.DEVNULL,stderr=err)
        try:
            wait(p,errpath,'application ready');time.sleep(.1);p.send_signal(signal.SIGINT)
            assert p.wait(timeout=15)==0,errpath.read_text()[-4000:]
        finally:
            if p.poll() is None:p.kill();p.wait()
print('Application code reload, preserved physics, rollback, fresh restart and stationary plugin passed.')
