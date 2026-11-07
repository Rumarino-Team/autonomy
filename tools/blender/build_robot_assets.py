"""Refresh fixed robot asset metadata from typed C++ definitions.

Run with Blender --background --factory-startup --python this_file. Each robot
owns its existing .blend; this tool validates geometry and mounts, stamps the
compiled configuration hash and updates the separate file-checksum manifest.
It never generates robot C++ definitions or reads a robot profile JSON file.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import bpy
from mathutils import Matrix, Vector

REPO = Path(__file__).resolve().parents[2]
NAMES = ('hydrus', 'proteus', 'bluerov', 'girona500')


def ned(v):
    return (-float(v.x), float(v.y), -float(v.z))


def near(actual, expected, label):
    if max(abs(a-b) for a, b in zip(actual, expected)) > 1e-4:
        raise ValueError(label + ': saved asset differs from the fixed C++ definition')


def verify(scene, definition):
    collection = bpy.data.collections[scene['sim_collection']]
    objects = list(collection.all_objects)
    roots = [o for o in objects if o.get('sim_role') == 'body']
    if len(roots) != 1:
        raise ValueError('a fixed asset must contain exactly one robot body')
    root = roots[0]
    if root.get('sim_profile') != definition['name']:
        raise ValueError('robot/profile mismatch')
    colliders = {o['sim_id']: o for o in objects if o.get('sim_role') == 'collider'}
    if len(colliders) != definition['shapes']:
        raise ValueError('fixed collider count changed')
    for reference in definition['geometry_reference']:
        obj = colliders[reference['id']]
        local = root.matrix_world.inverted() @ obj.matrix_world
        points = [ned(local @ v.co) for v in obj.data.vertices]
        expected = reference['points']
        def matches(left, right):
            return all(any(sum((a-b)**2 for a, b in zip(p, q)) <= 1e-8 for q in right) for p in left)
        if not matches(points, expected) or not matches(expected, points):
            raise ValueError(reference['id'] + ': update the C++ collision reference first')
    mounts = sorted((o for o in objects if o.get('sim_role') == 'thruster'), key=lambda o: o['sim_index'])
    if len(mounts) != definition['thrusters']:
        raise ValueError('fixed thruster count changed')
    for obj, expected in zip(mounts, definition['mounts']):
        local = root.matrix_world.inverted() @ obj.matrix_world
        near(ned(local.translation), expected['position'], obj.name)
        near(ned(local.to_quaternion() @ Vector((-1, 0, 0))), expected['direction'], obj.name)
    camera = scene.camera
    local = root.matrix_world.inverted() @ camera.matrix_world @ Matrix.Rotation(math.pi/2, 4, 'Z')
    position, rotation, _ = local.decompose()
    m = Matrix.Diagonal((-1., 1., -1.))
    q = (m @ rotation.to_matrix() @ m).to_quaternion()
    reference = definition['camera']
    near(ned(position), reference['pose']['position'], 'camera')
    a = (float(q.x), float(q.y), float(q.z), float(q.w))
    b = reference['pose']['orientation']
    if sum(x*y for x, y in zip(a, b)) < 0:
        a = tuple(-x for x in a)
    near(a, b, 'camera orientation')
    fov = math.degrees(2 * math.atan(camera.data.sensor_width / (2 * camera.data.lens)))
    near((fov,), (reference['horizontal_fov'],), 'camera FOV')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simctl', type=Path, default=REPO / 'build/portable/sim/simctl')
    parser.add_argument('--output', type=Path, default=REPO / 'auvs/stonefish_sim/data/robots')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    records = []
    for name in NAMES:
        # JSON here is a read-only authoring report from typed C++ values. A
        # normal simulator launch never runs this script or parses that report.
        definition = json.loads(subprocess.check_output([str(args.simctl.resolve()), 'robot-config', name]))
        path = args.output.resolve() / (name + '.blend')
        bpy.ops.wm.open_mainfile(filepath=str(path))
        scene = bpy.data.scenes['AutonomySource']
        verify(scene, definition)
        scene['sim_robot'] = name
        scene['sim_robot_config_sha256'] = definition['configuration_sha256']
        bpy.ops.wm.save_as_mainfile(filepath=str(path), compress=True, check_existing=False)
        records.append({k: definition[k] for k in ('name', 'asset', 'configuration_sha256', 'thrusters', 'description')})
        records[-1]['asset_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    lines = ['// Saved-asset checksums; updated by tools/blender/build_robot_assets.py.',
             'constexpr AssetHash asset_hashes[] = {']
    lines += ['  {"'+r['name']+'", "'+r['asset_sha256']+'"},' for r in records]
    lines += ['};', '']
    (REPO / 'sim/robots/asset_hashes.inc').write_text('\n'.join(lines))
    (args.output / 'manifest.json').write_text(json.dumps({'version': 2, 'robots': records}, indent=2)+'\n')
    print('FIXED_ROBOT_CHECKSUMS_UPDATED', json.dumps(records))


if __name__ == '__main__':
    main()
