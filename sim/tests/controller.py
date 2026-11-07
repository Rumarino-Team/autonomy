#!/usr/bin/env python3
"""Verify the authored allocator against forces, arms and propeller torques."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller_config import configuration, load_scene, wrench_columns

root = Path(__file__).resolve().parents[1]

def close(actual, expected, tolerance):
    if isinstance(actual, dict) and isinstance(expected, dict):
        return actual.keys() == expected.keys() and all(close(actual[key], expected[key], tolerance) for key in actual)
    if isinstance(actual, list) and isinstance(expected, list):
        return len(actual) == len(expected) and all(close(a, b, tolerance) for a, b in zip(actual, expected))
    if isinstance(actual, bool) or isinstance(expected, bool):
        return actual == expected
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        return abs(float(actual) - float(expected)) <= tolerance
    return actual == expected

def verify(scene, config_path=None):
    config = json.loads((config_path or root/'scenarios/controller.json').read_text())
    fresh = configuration(scene)
    # Re-evaluating the blend moves thruster mounts by a few nanometres.
    assert close(fresh, config, 1e-6), 'regenerate the controller after changing the vehicle or tuning'
    columns = wrench_columns(scene)
    for axis in range(6):
        wrench = [sum(c[row]*a[axis] for c, a in zip(columns, fresh['tam'])) for row in range(6)]
        assert all(abs(v - float(row == axis)) < 1e-10 for row, v in enumerate(wrench)), (axis, wrench)
    print('all six commanded wrench axes allocate independently, including reaction torque')

if __name__ == '__main__':
    verify(load_scene(root.parent/'auvs/stonefish_sim/data/pool_scene.blend'))
