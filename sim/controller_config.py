#!/usr/bin/env python3
"""Generate the Hydrus SI wrench allocator from the owned thruster geometry.

The controller outputs N and N m; TAM outputs fractions of bollard thrust.
The quadratic_rpm curve converts those fractions to normalized shaft speed.
No external numerical libraries are required.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path

def load_scene(path):
    path = Path(path)
    if path.suffix == '.json':
        return json.loads(path.read_text())
    spec = importlib.util.spec_from_file_location(
        "scene_wire", Path(__file__).resolve().parents[1]/"tools/blender/autonomy_sim/wire.py")
    wire = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wire)
    return wire.decode(path.read_bytes())


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def inverse(matrix):
    n = len(matrix)
    rows = [row[:] + [float(i == j) for j in range(n)] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda i: abs(rows[i][col]))
        if abs(rows[pivot][col]) < 1e-12:
            raise ValueError("thruster layout does not have six independent wrench axes")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        rows[col] = [v/scale for v in rows[col]]
        for i in range(n):
            if i != col:
                scale = rows[i][col]
                rows[i] = [a-scale*b for a, b in zip(rows[i], rows[col])]
    return [row[n:] for row in rows]


def wrench_columns(scene):
    bodies = {b['id']: b for b in scene['bodies']}
    columns = []
    for thruster in scene['thrusters']:
        spec = thruster['spec']
        diameter = spec['diameter']
        maximum = scene['water_density'] * diameter**4 * spec['thrust_coeff'] * (spec['max_rpm']/60)**2
        force = [v * spec.get('command_sign', -1) * maximum for v in thruster['direction']]
        com = bodies[thruster['body']]['center_of_mass']['position']
        arm = [p-c for p, c in zip(thruster['position'], com)]
        moment = [m-f*diameter*spec['torque_coeff']/spec['thrust_coeff'] for m, f in zip(cross(arm, force), force)]
        columns.append(force + moment)
    return columns


def allocation(scene):
    columns = wrench_columns(scene)
    gram = [[sum(c[i]*c[j] for c in columns) for j in range(6)] for i in range(6)]
    inv = inverse(gram)
    return [[sum(c[k]*inv[k][j] for k in range(6)) for j in range(6)] for c in columns]


def configuration(scene):
    body = next(b for b in scene['bodies'] if b['id'] == scene['camera']['body'])
    weight = (body['dry_mass'] - scene['water_density']*body['volume'])
    return {
        'teleop_port': 8080,
        'camera_forward': [0, 1, 0],
        'rotation_limit': [0.3, 0, math.pi],
        'tam': allocation(scene),
        'thruster_output_scale': 1,
        'thruster_curve': 'quadratic_rpm',
        'odometry': {
            'rotation_gate': 0.5,
            'kp': [100, 100, 100, 25, 30, 60],
            'ki': [0, 0, 1, 0, 0, 0],
            'kd': [110, 110, 140, 12, 16, 50],
            'derivative_on_measurement': True,
            'translation_frame': 'world',
            'integral_limit': [0, 0, 15, 0, 0, 0],
            'output_limit': [180, 400, 150, 20, 20, 70],
            'feedforward_world_force': [-weight*g for g in scene['gravity']],
        },
        'ibvs': {
            'center_deadband': 0.05, 'target_height': 0.9,
            'interaction': [[0, 0, 0], [0, 0, 1], [0, 0, 0], [0, 0, 0], [0, 0, 0], [-1, 0, 0]],
        },
    }


if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',type=Path,default=root/'scenarios/pool.asim')
    parser.add_argument('--output',type=Path,default=root/'scenarios/controller.json')
    args=parser.parse_args()
    scene = load_scene(args.scenario)
    args.output.write_text(json.dumps(configuration(scene), indent=2)+'\n')
