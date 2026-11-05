"""Check both AMJB presentation output and fixed-step determinism under pacing."""
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys

def result_line(stdout):
    for line in reversed(stdout.splitlines()):
        text = line.strip()
        if text.startswith("{") and '"hash"' in text:
            return json.loads(text)
    raise AssertionError(stdout[-2000:])

executable, plugin, backend, scenario, controller = map(lambda p: Path(p).resolve(), sys.argv[1:])
if not executable.exists():
    raise SystemExit(77)
header = struct.Struct('<4sHHd')
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
    listener.bind(('127.0.0.1', 0))
    listener.setblocking(False)
    port = listener.getsockname()[1]
    hashes = []
    for paced in (False, True):
        env = {k: v for k, v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
        env.update(AUTONOMY_SIM_BACKEND=str(backend), AUTONOMY_SIM_SCENARIO=str(scenario),
                   AUTONOMY_SIM_MAX_TICKS='144',
                   AUTONOMY_SIM_BLENDER=str(int(paced)), AUTONOMY_SIM_REALTIME=str(int(paced)),
                   AUTONOMY_SIM_BLENDER_PORT=str(port))
        result = subprocess.run([str(executable), str(plugin), 'prequalify_robosub', str(controller)], env=env, capture_output=True, text=True, timeout=60)
        assert 'SimulationTickBudget' in result.stderr, result.stderr[-2000:]
        hashes.append(result_line(result.stdout)['hash'])
    assert hashes[0] == hashes[1], 'presentation changed simulation states'
    packets = 0
    while True:
        try:
            packet = listener.recv(65535)
        except BlockingIOError:
            break
        magic, version, count, time = header.unpack_from(packet)
        layout = struct.Struct('<I32s3d4d') if version == 2 else struct.Struct('<32s3d4d')
        assert magic == b'AMJB' and version in (1, 2) and len(packet) == header.size + count * layout.size
        found = {}
        for index in range(count):
            fields = layout.unpack_from(packet, header.size + index * layout.size)
            name = fields[1] if version == 2 else fields[0]
            label = name.rstrip(b'\0').decode()
            if version == 2:
                assert fields[0] == {'hydrus': 1, 'gate': 2, 'marker': 3, 'camera': 4294967295}[label]
            found[label] = fields
        assert set(found) == {'hydrus', 'gate', 'marker', 'camera'}, found.keys()
        packets += 1
    assert 10 <= packets <= 14, packets
print(f'{packets} Blender packets: named poses, camera id, and paced/unpaced hashes match')
