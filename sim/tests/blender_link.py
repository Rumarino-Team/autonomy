"""Check both AMJB presentation output and fixed-step determinism under pacing."""
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile

executable, plugin, backend, scenario, controller = map(lambda p: Path(p).resolve(), sys.argv[1:])
if not executable.exists():
    raise SystemExit(77)
header, pose = struct.Struct('<4sHHd'), struct.Struct('<32s3d4d')
with tempfile.TemporaryDirectory(prefix='autonomy-blender-test-') as folder, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
    listener.bind(('127.0.0.1', 0))
    listener.setblocking(False)
    port = listener.getsockname()[1]
    hashes = []
    records = {}
    for paced in (False, True):
        path = Path(folder)/f'{paced}.jsonl'
        env = {k:v for k,v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
        env.update(AUTONOMY_SIM_BACKEND=str(backend), AUTONOMY_SIM_SCENARIO=str(scenario),
                   AUTONOMY_SIM_TRACE=str(path), AUTONOMY_SIM_MAX_TICKS='144',
                   AUTONOMY_SIM_BLENDER=str(int(paced)), AUTONOMY_SIM_REALTIME=str(int(paced)),
                   AUTONOMY_SIM_BLENDER_PORT=str(port))
        result = subprocess.run([str(executable),str(plugin),'prequalify_robosub',str(controller)], env=env, capture_output=True, text=True, timeout=60)
        assert 'SimulationTickBudget' in result.stderr, result.stderr[-2000:]
        with path.open() as source:
            for line in source:
                r = json.loads(line)
                if r['t']=='sample':
                    records[r['timestamp_ns']] = r
        hashes.append([r['hash'] for r in records.values()])
    assert hashes[0] == hashes[1], 'presentation changed simulation states'
    packets = 0
    while True:
        try:
            packet = listener.recv(65535)
        except BlockingIOError:
            break
        magic, version, count, time = header.unpack_from(packet)
        layout=struct.Struct('<I32s3d4d') if version==2 else pose
        assert magic==b'AMJB' and version in (1,2) and len(packet)==header.size+count*layout.size
        sample = records[round(time*1e9)]
        found = {}
        for index in range(count):
            fields=layout.unpack_from(packet, header.size+index*layout.size)
            name,*values=fields[1:] if version==2 else fields
            if version==2:
                assert fields[0]=={'hydrus':1,'gate':2,'marker':3,'camera':4294967295}[name.rstrip(b'\0').decode()]
            found[name.rstrip(b'\0').decode()] = values
        assert set(found) == {'hydrus','gate','marker','camera'}, found.keys()
        vehicle=next(b for b in sample['bodies'] if b['id']==1)
        for name, state in [('hydrus',vehicle['pose']),('camera',sample['camera'])]:
            x,y,z = state['position']; qx,qy,qz,qw = state['orientation']
            assert all(abs(a-b)<1e-10 for a,b in zip(found[name],[-x,y,-z,qw,-qx,qy,-qz]))
        packets += 1
    assert 10 <= packets <= 14, packets
print(f'{packets} Blender packets: named poses, camera transform, NED conversion and paced/unpaced states match')
