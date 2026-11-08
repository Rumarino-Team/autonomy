"""Check fixed-robot registration, isolation, relocation and checksum failures."""
import hashlib
import sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
import data_binary as binary
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

simctl, assets, blender = map(Path, sys.argv[1:])
environment = {k: v for k, v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
environment['AUTONOMY_SIM_PROFILES'] = '/no-external-robot-calibration'


def call(*args):
    return subprocess.run([str(simctl), *map(str, args)], env=environment,
                          capture_output=True)


definitions = binary.decode(call('robots').stdout)
expected = {'hydrus': 8, 'proteus': 6, 'bluerov': 8, 'girona500': 5}
assert {d['name']: d['thrusters'] for d in definitions} == expected
manifest = binary.read(assets / 'manifest.bin', kind='metadata')
with tempfile.TemporaryDirectory(prefix='fixed-robot-check-') as folder:
    folder = Path(folder)
    for definition in definitions:
        name = definition['name']
        path = assets / (name + '.blend')
        record = next(r for r in manifest['robots'] if r['name'] == name)
        for key in ('asset_sha256', 'configuration_sha256'):
            assert record[key] == definition[key], (name, key)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == definition['asset_sha256']
        validation = call('validate', path, '--twice')
        assert validation.returncode == 0, validation.stderr
        values = call('inspect', path)
        assert values.returncode == 0, values.stderr
        model = binary.decode(values.stdout)
        assert len(model['bodies']) == 1 and model['bodies'][0]['name'] == name
        assert len(model['thrusters']) == expected[name]
        for actual, reference in zip(model['thrusters'], definition['mounts']):
            for key in ('position', 'direction'):
                assert max(abs(a-b) for a, b in zip(actual[key], reference[key])) < 1e-4, (name, key)
        body = model['bodies'][0]
        for key in ('mass', 'dry_mass', 'inertia', 'volume', 'center_of_mass'):
            assert body[key] == definition['body'][key], (name, key)
        relocated = folder / (name + '-relocated.blend')
        shutil.copyfile(path, relocated)
        again = call('validate', relocated)
        assert again.returncode == 0, again.stderr
        assert binary.decode(again.stdout)['fingerprint'] == binary.decode(validation.stdout)['fingerprint']
    # Saving even an unrelated property changes the exact asset checksum. The
    # changed file must not quietly load with the old compiled configuration.
    changed = folder / 'changed.blend'
    code = ('import bpy; bpy.context.scene["checksum_test"]=1; '
            'bpy.ops.wm.save_as_mainfile(filepath=' + repr(str(changed)) + ')')
    process = subprocess.run([str(blender), '--background', '--factory-startup',
                              '--disable-autoexec', str(assets / 'hydrus.blend'),
                              '--python-expr', code], capture_output=True)
    assert process.returncode == 0, process.stderr
    rejected = call('validate', changed)
    assert rejected.returncode != 0 and b'SHA-256 mismatch' in rejected.stderr, rejected.stderr
    # The existing pool also resolves Hydrus from code, even if its old profile
    # Text reference is removed and no external profile directory is available.
    pool_copy = folder / 'pool-without-embedded-profile.blend'
    code = ('import bpy; del bpy.context.scene["sim_profile_texts"]; '
            'bpy.ops.wm.save_as_mainfile(filepath=' + repr(str(pool_copy)) + ')')
    process = subprocess.run([str(blender), '--background', '--factory-startup',
                              '--disable-autoexec', str(assets.parent / 'pool_scene.blend'),
                              '--python-expr', code], capture_output=True)
    assert process.returncode == 0, process.stderr
    loaded = call('inspect', pool_copy)
    assert loaded.returncode == 0, loaded.stderr
    robot = next(b for b in binary.decode(loaded.stdout)['bodies'] if b['name'] == 'hydrus')
    assert robot['mass'] == next(d for d in definitions if d['name'] == 'hydrus')['body']['mass']
print('Four fixed robots: compiled calibration, exact asset hashes, independent files, relocation and mismatch rejection passed')
