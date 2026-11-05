"""Direct .blend loading, binary compatibility, failure handling and replay."""
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import shutil
import sys
import tempfile

repo=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('scene_wire',Path(__file__).parent/'autonomy_sim/wire.py')
wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
simctl=Path(sys.argv[1]).resolve()
backends=[Path(p).resolve() for p in sys.argv[2:]]
environment={k:v for k,v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
environment['AUTONOMY_SIM_PROFILES']='/unavailable-profile-sidecars'


def run(*args,success=True,**kwargs):
    result=subprocess.run([str(simctl),*map(str,args)],env=kwargs.pop('env',environment),
                          capture_output=True,timeout=120,**kwargs)
    assert (result.returncode==0)==success, result.stderr.decode(errors='replace')[-5000:]
    return result


scene={'version':1,'gravity':[0,0,0],'bodies':[{'id':1,'motion':1,'mass':2,'inertia':[1,1,1],
        'pose':{'position':[0,0,2]},'shapes':[{'kind':0,'size':[.1,.2,.3]}]}]}
assert wire.decode(wire.encode(scene))['bodies'][0]['pose']['position']==[0,0,2]
blob=wire.encode(scene)
for bad in (b'',blob[:-1],b'wrongver'+blob[8:],wire.HEADER.pack(wire.MAGIC,wire.MAX_BYTES+1)):
    try: wire.decode(bad)
    except ValueError: pass
    else: raise AssertionError('malformed wire accepted')
scene['gravity']=[float('nan'),0,0]
try: wire.encode(scene)
except ValueError: pass
else: raise AssertionError('nonfinite wire accepted')

with tempfile.TemporaryDirectory(prefix='sim-direct-blender-') as temporary:
    folder=Path(temporary)
    source=repo/'auvs/stonefish_sim/data/pool_scene.blend'
    frozen=folder/'scene.asim'
    run('snapshot',source,frozen)
    payload=frozen.read_bytes()
    snapshot=wire.decode(payload)
    expected=wire.decode((repo/'sim/scenarios/pool.asim').read_bytes())
    assert snapshot['source']['content_hash']==expected['source']['content_hash'], 'direct loader differs from evaluated authoring snapshot'
    assert len(snapshot['bodies'])==len(expected['bodies'])
    relocated=folder/'self contained pool.blend'
    shutil.copy2(source,relocated)
    relocated_data=json.loads(run('validate',relocated).stdout)
    assert relocated_data['source']['content_hash']==snapshot['source']['content_hash'], 'relocated .blend required external calibration'
    a=json.loads(run('validate',frozen).stdout)
    b=json.loads(run('validate','-',input=wire.encode(snapshot)).stdout)
    assert a==b and a['bodies']==8
    # Native and Python encoders preserve every field, including uint64 values.
    snapshot['case_seed']=2**64-1
    b=json.loads(run('validate','-',input=wire.encode(snapshot)).stdout)
    assert b['status']=='valid'
    for malformed in (payload[:8],b'BADVERS!'+payload[8:],wire.HEADER.pack(wire.MAGIC,wire.MAX_BYTES+1),payload[:-1]):
        run('validate','-',input=malformed,success=False)
    trailing=folder/'trailing.asim';trailing.write_bytes(payload+b'extra')
    run('validate',trailing,success=False)
    wrong=environment.copy();wrong['AUTONOMY_SIM_SOURCE_SCENE']='Missing source'
    result=run('validate',source,env=wrong,success=False)
    assert b'authoring scene not found' in result.stderr
    wrong=environment.copy();wrong['AUTONOMY_SIM_BLENDER_EXECUTABLE']='/missing-blender-binary'
    run('validate',source,env=wrong,success=False)
    # Subsequent runs work after the Blender executable disappears.
    for backend in backends:
        first=json.loads(run('run',backend,frozen,36,7,env=wrong).stdout)
        second=json.loads(run('run',backend,frozen,36,7,env=wrong).stdout)
        assert first['status']=='passed' and first['hash']==second['hash'] and first['steps']==second['steps']==36
    # Freeze the blend once, then later runs use that snapshot.
    counter=folder/'evaluations.log'
    proxy=folder/'blender-proxy'
    blender=shutil.which('blender')
    proxy.write_text('#!'+sys.executable+'\nimport os,sys\nwith open('+repr(str(counter))+',"a") as log: log.write("evaluation\\n")\nos.execv('+repr(blender)+',['+repr(blender)+',*sys.argv[1:]])\n')
    proxy.chmod(0o755)
    batch_env=environment.copy();batch_env['AUTONOMY_SIM_BLENDER_EXECUTABLE']=str(proxy)
    frozen_batch=folder/'batch-scene.asim'
    run('snapshot',source,frozen_batch,env=batch_env)
    assert counter.read_text().splitlines()==['evaluation'], 'snapshot did not evaluate Blender once'
    for seed in (1, 2):
        run('run',backends[0],frozen_batch,12,seed,env=wrong)
    assert counter.read_text().splitlines()==['evaluation'], 'later runs reopened Blender'
    assert not list(folder.glob('*.json')), 'direct launch produced an intermediate JSON scene'
print('DIRECT_BLEND_BINARY_AND_REPLAY_PASSED')
