"""Partial worker startup joins running workers before releasing DLL resources."""
from pathlib import Path
import subprocess
import sys
import tempfile
from app_fixture import Application,read
_,plugin,backend,_=sys.argv[1:5]
preload=sys.argv[5] if len(sys.argv)>5 else None
with tempfile.TemporaryDirectory(prefix='binary-fuzz-startup-') as tmp:
    root=Path(tmp);results=root/'results'
    app=Application(root/'app',plugin,backend,preload,fuzz='.{ .cases=8, .workers=4, .results="'+str(results)+'" }')
    app.build('fuzz-startup-test')
    p=subprocess.run([str(app.root/'package/bin/fuzz_startup_test'),'fuzz'],cwd=app.root,env=app.env,capture_output=True,text=True,timeout=60)
    assert p.returncode==0,p.stderr
    totals=read(results/'totals.bin',kind='totals')
    assert totals['cancelled']>0 and sum(totals.values())==8,totals
    assert len(list(results.glob('case-*.bin')))==8
print('Injected second-worker failure joined and cleaned the first worker.')
