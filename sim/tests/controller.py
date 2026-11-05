#!/usr/bin/env python3
"""Verify the authored allocator against forces, arms and propeller torques."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller_config import configuration, wrench_columns

root = Path(__file__).resolve().parents[1]
scene = json.loads((root/'scenarios/pool.json').read_text())
config = json.loads((root/'scenarios/controller.json').read_text())
assert config == configuration(scene), 'regenerate the controller after changing the vehicle or tuning'
columns = wrench_columns(scene)
for axis in range(6):
    wrench = [sum(c[row]*a[axis] for c, a in zip(columns, config['tam'])) for row in range(6)]
    assert all(abs(v - float(row == axis)) < 1e-10 for row, v in enumerate(wrench)), (axis, wrench)
print('all six commanded wrench axes allocate independently, including reaction torque')
