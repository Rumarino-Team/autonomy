"""Partial worker startup must join running workers before releasing libraries."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
exe,plugin,backend,config=map(lambda p:str(Path(p).resolve()),sys.argv[1:5])
env={k:v for k,v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
env['AUTONOMY_SIM_BACKEND']=backend
if len(sys.argv)>5: env['LD_PRELOAD']=sys.argv[5]
with tempfile.TemporaryDirectory(prefix='fuzz-startup-') as tmp:
    path=Path(tmp)/'results'
    result=subprocess.run([exe,plugin,'prequalify_robosub',config,'--fuzz','--cases','8','--workers','4','--results',str(path)],env=env,capture_output=True,text=True,timeout=45)
    assert result.returncode==0,result.stderr
    totals=json.loads((path/'totals.json').read_text())
    assert totals['cancelled']>0 and sum(totals.values())==8,totals
    assert len(list(path.glob('case-*.json')))==8
print('injected second-worker startup failure joined and cleaned the first worker')
