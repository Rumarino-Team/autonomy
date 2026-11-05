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


values={'finite':[-1.25,0.0,1.5],'integers':[-2**63,0,2**64-1],
        'bool':[True,False,None],'text':'Blender → simulator','nested':{'list':[]}}
assert wire.decode(wire.encode(values))==values
for bad in (b'',wire.encode(values)[:-1],b'wrongver'+wire.encode(values)[8:]):
    try: wire.decode(bad)
    except ValueError: pass
    else: raise AssertionError('malformed wire accepted')
try: wire.encode({'bad':float('nan')})
except ValueError: pass
else: raise AssertionError('nonfinite wire accepted')

with tempfile.TemporaryDirectory(prefix='sim-direct-blender-') as temporary:
    folder=Path(temporary)
    source=repo/'auvs/stonefish_sim/data/pool_scene.blend'
    frozen=folder/'scene.asim'
    run('snapshot',source,frozen)
    payload=frozen.read_bytes()
    snapshot=wire.decode(payload)
    expected=json.loads((repo/'sim/scenarios/pool.json').read_text())
    assert snapshot==expected, 'direct loader differs from evaluated authoring snapshot'
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
    # Subsequent worlds/replays work after the Blender executable disappears.
    for index,backend in enumerate(backends):
        trace=folder/f'trace-{index}.jsonl';replay=folder/f'replay-{index}.jsonl'
        run('run',backend,frozen,trace,36,7,env=wrong)
        run('replay',backend,trace,replay,env=wrong)
        assert trace.exists() and replay.exists()
    # A batch evaluates Blender exactly once, then all parallel cases use the
    # frozen binary scene. Preserved cases remain runnable after its tempdir ends.
    counter=folder/'evaluations.log'
    proxy=folder/'blender-proxy'
    blender=shutil.which('blender')
    proxy.write_text('#!'+sys.executable+'\nimport os,sys\nwith open('+repr(str(counter))+',"a") as log: log.write("evaluation\\n")\nos.execv('+repr(blender)+',['+repr(blender)+',*sys.argv[1:]])\n')
    proxy.chmod(0o755)
    batch_env=environment.copy();batch_env['AUTONOMY_SIM_BLENDER_EXECUTABLE']=str(proxy)
    batch=folder/'batch'
    result=subprocess.run([sys.executable,str(repo/'sim/fuzz.py'),'--simctl',str(simctl),
        '--backend',str(backends[0]),'--scenario',str(source),'--cases','2','--workers','2',
        '--ticks','12','--verify-replay','--keep-passed','--output',str(batch)],
        env=batch_env,capture_output=True,timeout=120)
    assert result.returncode==0,result.stderr.decode(errors='replace')
    assert counter.read_text().splitlines()==['evaluation'], 'parallel workers reopened Blender'
    for manifest_path in batch.glob('case-*/failure.json'):
        manifest=json.loads(manifest_path.read_text())
        assert Path(manifest['source_scenario']).suffix=='.asim'
        subprocess.run(manifest['command'],env=wrong,check=True,capture_output=True,timeout=120)
    assert not list(folder.glob('*.json')), 'direct launch produced an intermediate JSON scene'
print('DIRECT_BLEND_BINARY_AND_REPLAY_PASSED')
