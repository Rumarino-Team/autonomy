"""Run: blender --background --factory-startup pool_scene.blend --python this_file."""

import math
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import autonomy_sim as link
from autonomy_sim import rotor_animation as rotors

scene = bpy.context.scene
source=scene
vehicle_names = rotors.VEHICLES
bpy.context.view_layer.update()
original = {ob.name: ob.matrix_world.copy() for name in vehicle_names
            for ob in bpy.data.objects[name].children if ob.type == "MESH"}
link.register()
snapshot=link.exporter.export_scene(source,Path(__file__).resolve().parents[3]/'sim/profiles')
scene=link.preview.ensure(source,snapshot)
vehicle_names=rotors.VEHICLES
# Preview datablock names can differ; the viewer resolves display names/IDs.
original={}
rotors.rig_scene(scene)
bpy.context.view_layer.update()
assert sum(len(rotors.rotors(link._object_for(name))) for name in vehicle_names) == 27
for name, matrix in original.items():
    error = max(abs(matrix[r][c] - bpy.data.objects[name].matrix_world[r][c])
                for r in range(4) for c in range(4))
    assert error < 2e-5, (name, error, list(matrix.translation), list(bpy.data.objects[name].matrix_world.translation))
counts = {name: len(rotors.rig_vehicle(link._object_for(name))) for name in vehicle_names}
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
root = link._object_for('hydrus')
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
initial_speed = abs(rotors._live[root.name]['motors'][rotor.name][1])
assert initial_speed > 0
for i in range(1, 81):
    clock[0] += .04
    packet(moved, .04 * (i + 1))
assert abs(rotors._live[root.name]['motors'][rotor.name][1]) < 1e-4
assert all(abs(hull.matrix_basis[r][c] - original_hull_local[r][c]) < 1e-7
           for r in range(4) for c in range(4))
rotors.reset_live()

# Turning alone engages correctly positioned shafts on all four robot types.
for name in vehicle_names:
    robot = link._object_for(name)
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
link.unregister()
assert rotors._on_frame not in bpy.app.handlers.frame_change_post
print('ROTOR_TESTS_PASSED', counts)
