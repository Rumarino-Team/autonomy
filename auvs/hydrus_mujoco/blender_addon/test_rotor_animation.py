"""Run: blender --background --factory-startup pool_scene.blend --python this_file."""

import math
import json
import struct
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import blender_addon as link
from blender_addon import rotor_animation as rotors

scene = bpy.context.scene
vehicle_names = rotors.VEHICLES
bpy.context.view_layer.update()
original = {ob.name: ob.matrix_world.copy() for name in vehicle_names
            for ob in bpy.data.objects[name].children if ob.type == "MESH"}
link.register()
bpy.context.view_layer.update()
assert sum(len(rotors.rotors(scene.objects[name])) for name in vehicle_names) == 27
for name, matrix in original.items():
    error = max(abs(matrix[r][c] - bpy.data.objects[name].matrix_world[r][c])
                for r in range(4) for c in range(4))
    assert error < 2e-5, (name, error, list(matrix.translation), list(bpy.data.objects[name].matrix_world.translation))
counts = {name: len(rotors.rig_vehicle(scene.objects[name])) for name in vehicle_names}
assert counts == {"hydrus": 8, "proteus": 6, "bluerov": 8, "girona500": 5}
assert len([ob for ob in scene.objects if ob.get("auv_rotor_mount")]) == 27

# Exact motor integration is independent of timer subdivision.
phase, speed = rotors.advance(0, 0, 15, 1)
p = w = 0
for _ in range(100):
    p, w = rotors.advance(p, w, 15, .01)
assert abs(phase - p) < 1e-10 and abs(speed - w) < 1e-10
assert rotors.angular_speed(-.5) == -rotors.angular_speed(.5)
assert rotors.angular_speed(float("nan")) == 0
assert rotors.advance(1, 2, 3, 0) == (1, 2)

# Live packets: actual translated root poses drive blades, then spin down at rest.
root = scene.objects['hydrus']
rotor = rotors.rotors(root)[0]
hull = next(ob for ob in root.children if ob.type == 'MESH')
original_hull_local = hull.matrix_basis.copy()
start = root.matrix_world.copy()
clock = [10.0]
rotors.time = SimpleNamespace(monotonic=lambda: clock[0])
def packet(matrix, timestamp):
    q = matrix.to_quaternion()
    payload = struct.pack('<4sHHd', link.MAGIC, link.VERSION, 1, timestamp)
    payload += struct.pack('<32s3d4d', b'hydrus', *matrix.translation, q.w, q.x, q.y, q.z)
    applied = link._apply_packet(payload)
    bpy.context.view_layer.update()
    rotors.observe_live(scene, *applied)
    rotors.tick_live(scene)
packet(start, 0)
clock[0] += .04
moved = start.copy()
moved.translation += Vector((.02, .02, 0))
packet(moved, .04)
assert abs(rotor.rotation_euler.x) > .001
initial_speed = abs(rotors._live['hydrus']['motors'][rotor.name][1])
assert initial_speed > 0
for i in range(1, 81):
    clock[0] += .04
    packet(moved, .04 * (i + 1))
assert abs(rotors._live['hydrus']['motors'][rotor.name][1]) < 1e-4
assert all(abs(hull.matrix_basis[r][c] - original_hull_local[r][c]) < 1e-7
           for r in range(4) for c in range(4))
rotors.reset_live()

# Turning alone engages correctly positioned shafts on all four robot types.
for name in vehicle_names:
    robot = scene.objects[name]
    before = robot.matrix_world.copy()
    from mathutils import Matrix
    after = before @ Matrix.Rotation(.1, 4, 'Z')
    targets = rotors._targets(robot, before, after, .1)
    assert any(abs(speed) > 0 for speed in targets.values()), name

# Native rotor actions preserve stops/reversals and arbitrary scrubbing order.
frames = [1, 25, 49, 73, 97, 121, 145]
samples = [{'thr': [v] + [0] * 7} for v in (0, 1, 1, 0, -1, -1, 0)]
rotors.bake_take(root, 'rotor_test', frames, samples, 24)
curve = rotor.animation_data.action.fcurves[0]
assert curve.evaluate(49) > curve.evaluate(25) > curve.evaluate(1)
assert curve.evaluate(121) < curve.evaluate(97)
values = [curve.evaluate(f) for f in (49.5, 1, 120.25, 49.5)]
assert values[0] == values[-1]
inactive = rotors.rotors(root)[1]
assert all(abs(key.co.y) < 1e-8 for key in inactive.animation_data.action.fcurves[0].keyframe_points)
rotors.detach_takes(scene)
assert all(ob.animation_data.action is None for ob in rotors.rotors(root))
assert rotors.activate_take(scene, root, 'rotor_test')

# Exercise the real recording import, including selection and take activation.
take = scene.hydrus_link.takes.add()
take.name = 'rotor_integration_test'
folder = tempfile.TemporaryDirectory()
recording_path = Path(folder.name) / 'rotor_recording.jsonl'
records = [{'t': 'header', 'version': 1, 'vehicle': 'hydrus', 'objects': ['hydrus'],
            'fps': 24, 'timestep': .01, 'every': 50, 'thrusters': 8}]
for i in range(9):
    records.append({'t': 'sample', 'step': i * 50, 'time': i * .5,
                    'poses': {'hydrus': [0, i * .2, -2, 1, 0, 0, 0]},
                    'thr': [0 if i in (0, 8) else .5] * 8})
recording_path.write_text(''.join(json.dumps(record) + '\n' for record in records))
take.recording = str(recording_path)
take.frame_offset = 1
link._import_take(scene, take, 0)
assert all(ob.animation_data.action.name.startswith(take.name + ':rotor:')
           for ob in rotors.rotors(root))
link.activate_take(scene, take)
scene.frame_set(30)
angle_a = rotor.rotation_euler.x
scene.frame_set(100)
scene.frame_set(30)
assert abs(rotor.rotation_euler.x - angle_a) < 1e-6
link.detach_takes(scene)
assert all(ob.animation_data.action is None for ob in rotors.rotors(root))
link.unregister()
folder.cleanup()
assert rotors._on_frame not in bpy.app.handlers.frame_change_post
print('ROTOR_TESTS_PASSED', counts)
