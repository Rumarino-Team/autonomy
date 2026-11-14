"""Closed-loop equivalence through the reloadable application library."""
import sys
import tempfile
from app_fixture import Application,outcome
_, plugin, backend, config = sys.argv[1:5]
preload=sys.argv[5] if len(sys.argv)>5 else None
with tempfile.TemporaryDirectory(prefix='sim-app-mission-') as tmp:
    app=Application(tmp,plugin,backend,preload)
    runs=[app.run('run') for _ in range(2)]
    for p in runs:
        assert p.returncode==0,p.stderr[-5000:]
        assert 'completed mission' in p.stderr
        o=outcome(p)
        assert o['status']==0 and 720<o['steps']<21600,o
        assert 1.5<=o['min_depth']<=o['max_depth']<=2.6,o
    assert outcome(runs[0])==outcome(runs[1])
    # Closed-loop tick count for the saved course with the current compiler.
    assert outcome(runs[0])['steps']==14988,outcome(runs[0])
    app.configure(max_ticks=720); app.build()
    p=app.run('run')
    assert p.returncode!=0 and 'SimulationTickBudget' in p.stderr,p.stderr[-3000:]
    assert outcome(p)['steps']==720
    app.configure(backend='/no/such/backend.so');app.build()
    p=app.run('run');assert p.returncode!=0,p.stderr
print('Repeated mission results:',outcome(runs[0]))
