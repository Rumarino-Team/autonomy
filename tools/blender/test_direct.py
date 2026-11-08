"""Direct .blend loading, binary compatibility, failure handling and replay."""
import importlib.util
import sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
import data_binary as binary
import os
from pathlib import Path
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
environment.update(AUTONOMY_SIM_PROFILES='/unavailable-profile-sidecars', AUTONOMY_SIM_BLENDER_EXECUTABLE='/missing-blender', AUTONOMY_SIM_BLENDER_SCRIPT='/missing-exporter')


def run(*args,success=True,**kwargs):
    result=subprocess.run([str(simctl),*map(str,args)],env=kwargs.pop('env',environment),
                          capture_output=True,timeout=180,**kwargs)
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
# The v3 mesh table stores one allocation for two identical cube geometries.
faces=wire.surface(0,(1,1,1))
points=[list(p) for face in faces for p in face]
triangles=[[i,i+1,i+2] for i in range(0,len(points),3)]
mesh_shape={'kind':3,'vertices':points,'triangles':triangles}
mesh_scene={'bodies':[{'id':i,'shapes':[mesh_shape], 'pose':{'position':[3*(i-1),0,0]}} for i in (1,2)]}
mesh_blob=wire.encode(mesh_scene)
assert wire._SECTION.unpack_from(mesh_blob,16+8+11*wire._SECTION.size)[1]==1, 'mesh table duplicated shared geometry'
assert run('resolve','-',input=mesh_blob).stdout==mesh_blob, 'C++/Python mesh encoding differs'
old=bytearray(mesh_blob);old[7]=ord('2')
assert b'ASIMSC03' in run('validate','-',input=old,success=False).stderr

scene['gravity']=[float('nan'),0,0]
try: wire.encode(scene)
except ValueError: pass
else: raise AssertionError('nonfinite wire accepted')

with tempfile.TemporaryDirectory(prefix='sim-direct-blender-') as temporary:
    folder=Path(temporary)
    source=repo/'auvs/simulator/data/pool_scene.blend'
    original=binary.decode(run('validate',source).stdout)
    assert original['bodies']==8 and original['source']['content_hash'] and original['source']['kind']=='blender_native'
    relocated=folder/'self contained pool.blend'
    shutil.copy2(source,relocated)
    relocated_data=binary.decode(run('validate',relocated).stdout)
    assert relocated_data['source']['content_hash']==original['source']['content_hash'], 'relocated .blend required external calibration'
    payload=run('resolve',source,7).stdout
    snapshot=wire.decode(payload)
    native=binary.decode(run('validate','-',input=payload).stdout)
    roundtrip=binary.decode(run('validate','-',input=wire.encode(snapshot)).stdout)
    assert native['fingerprint']==roundtrip['fingerprint'] and native['status']=='valid'
    snapshot['case_seed']=2**64-1
    seeded=binary.decode(run('validate','-',input=wire.encode(snapshot)).stdout)
    assert seeded['status']=='valid'
    for malformed in (payload[:8],b'BADVERS!'+payload[8:],wire.HEADER.pack(wire.MAGIC,wire.MAX_BYTES+1),payload[:-1]):
        run('validate','-',input=malformed,success=False)
    wrong=environment.copy();wrong['AUTONOMY_SIM_SOURCE_SCENE']='Missing source'
    result=run('validate',source,env=wrong,success=False)
    assert b'authoring scene not found' in result.stderr
    wrong=environment.copy();wrong['AUTONOMY_SIM_BLENDER_EXECUTABLE']='/missing-blender-binary'
    run('validate',source,env=wrong)
    def result_line(stdout):
        start=stdout.rfind(b'AUVDATA1')
        if start<0: raise AssertionError(stdout[-500:])
        return binary.decode(stdout[start:])
    for backend in backends:
        first=result_line(run('run',backend,source,36,7).stdout)
        second=result_line(run('run',backend,source,36,7).stdout)
        assert first['status']=='passed' and first['hash']==second['hash'] and first['steps']==second['steps']==36
    counter=folder/'evaluations.log'
    proxy=folder/'blender-proxy'
    blender=shutil.which('blender')
    proxy.write_text('#!'+sys.executable+'\nimport os,sys\nwith open('+repr(str(counter))+',"a") as log: log.write("evaluation\\n")\nos.execv('+repr(blender)+',['+repr(blender)+',*sys.argv[1:]])\n')
    proxy.chmod(0o755)
    batch_env=environment.copy();batch_env['AUTONOMY_SIM_BLENDER_EXECUTABLE']=str(proxy)
    run('validate',source,'--twice',env=batch_env)
    assert not counter.exists(), 'native loading unexpectedly invoked Blender'
    assert not list(folder.glob('*.bin')), 'direct launch produced an intermediate scene file'
    assert not list(folder.glob('*.asim')), 'direct launch wrote a scene file'
print('DIRECT_BLEND_BINARY_AND_REPLAY_PASSED')
