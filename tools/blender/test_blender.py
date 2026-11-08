"""Exercise the real add-on in an isolated background Blender session.

blender --background --factory-startup pool_scene.blend --python-exit-code 1 \
  --python tools/blender/test_blender.py -- mujoco
"""
import sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
import data_binary as binary
from pathlib import Path
import socket
import sys
import time
import tempfile

import bpy

repo = Path(__file__).resolve().parents[2]
if not (repo/'zig-out/bin/src_3').is_file():
    print('Build the controller with zig build first')
    raise SystemExit(77)
sys.path.insert(0, str(Path(__file__).parent))
import autonomy_sim as link

engine = sys.argv[sys.argv.index('--')+1] if '--' in sys.argv else 'mujoco'
link.register()
link.build_scene()
scene_count = len(bpy.data.scenes)
link.build_scene()
assert len(bpy.data.scenes) == scene_count, 'loading the pool twice duplicated it'
scene = bpy.context.scene
link.rotor_animation.rig_scene(scene)
settings = scene.autonomy_sim
settings.backend = engine
settings.max_ticks = 21600
settings.randomize = False
source=scene
source_hash=link.exporter.export_scene(source,repo/'sim/profiles')['source']['content_hash']
# Runtime launch must never call the Python scene exporter.
original_export=link.exporter.export_scene
def forbidden_export(*args, **kwargs):
    raise AssertionError('runtime used the Python exporter snapshot')
link.exporter.export_scene=forbidden_export
temporary=tempfile.TemporaryDirectory(prefix='native-blender-addon-')
# Configure/build an isolated application; the test never edits user config.
sys.path.insert(0,str(repo/'auvs/simulator/tests'))
from app_fixture import Application
application=Application(Path(temporary.name)/'app',repo/'build/simulator/auvs/simulator/libauv_simulator.so',repo/'build/simulator/auvs/simulator/libsimulator_backend_mujoco.so')
def isolated_course_tools(_repo):
    return dict(executable=application.exe,plugin=repo/'build/simulator/auvs/simulator/libauv_simulator.so',
                config=application.root/'config/application.zig',simctl=None,backend=None,repo=application.root,profiles=None,course=True)
link._course_tools=isolated_course_tools
saved=Path(temporary.name)/'source.blend'
bpy.ops.wm.save_as_mainfile(filepath=str(saved))
scene_files_before=set(link._takes_dir().glob('scene_*.bin'))

# Test the same Record run operator used by the sidebar, including its subprocess.
assert bpy.ops.autonomy_sim.record_take() == {'FINISHED'}
scene=bpy.context.scene
settings=scene.autonomy_sim
link._job['proc'].wait(timeout=120)
take = link.finish_job()
assert set(link._takes_dir().glob('scene_*.bin'))==scene_files_before, 'record run wrote an intermediate scene file'
assert take.status == 'ok' and take.reason == 'complete', link._job['message']
log = Path(take.log).read_text(encoding='utf-8', errors='replace')
result = link._result_line(log)
root = link._object_for('hydrus')
assert 720 < result['steps'] < settings.max_ticks and result['status'] == 0, result
assert 'completed mission' in log, log[-2000:]
assert len(link.rotor_animation.rotors(root)) == 8
settings.onboard_camera = True
assert scene.camera == link._object_for('hydrus_camera')
settings.onboard_camera = False

# The live receiving path: a real paced mission emits datagrams, the same drain
# callback used by the UI applies them, and meshes/rotors follow named roots.
extra=bpy.data.objects.new('Renamed authored body',None)
scene.collection.objects.link(extra);extra['sim_role']='body';extra['sim_id']='121212'
link._assign_action(extra,bpy.data.actions.new(take.name+':arbitrary-body'))
link.detach_takes(scene)
assert extra.animation_data.action is None, 'newly authored bodies retained replay actions'
bpy.data.objects.remove(extra,do_unlink=True)
settings.take_index = -1
settings.max_ticks = 720
listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listener.bind(('127.0.0.1', 0))
listener.setblocking(False)
settings.port = listener.getsockname()[1]
application.configure(max_ticks=720,realtime=True,blender='.{ .enabled=true, .port='+str(settings.port)+' }')
application.build()
link._link.update(socket=listener, running=True, packets=0, time=0.0)
bpy.ops.wm.save_as_mainfile(filepath=str(saved))
live_take = link.launch_take(scene, 0, live=True)
link.preview.select(source)  # Editing source while live updates arrive is safe.
seen = []
deadline = time.monotonic()+60
while link._job['proc'].poll() is None:
    assert time.monotonic() < deadline, 'live run exceeded watchdog'
    link._drain()
    if link._link['packets']:
        seen.append(tuple(root.matrix_world.translation))
    time.sleep(.01)
link._drain()
link.preview.select(scene)
assert link._link['packets'] >= 30, link._link
assert link._link['time'] > 1.9, link._link
assert len(set(seen)) > 15, len(set(seen))
assert abs(link.rotor_animation.rotors(root)[0].rotation_euler.x) > .001
updates = link._link['packets']
link._link.update(running=False, socket=None)
listener.close()
assert link.finish_job().status == 'failed', 'bounded run must report tick exhaustion'
settings.takes.remove(len(settings.takes)-1)
settings.max_ticks = 21600
settings.port = 9760
settings.take_index = 0
scene.frame_set(1)
assert original_export(source,repo/'sim/profiles')['source']['content_hash']==source_hash, 'live/take playback changed the authoring scene'
for image in bpy.data.images:
    if image.source == 'FILE' and not image.packed_file:
        assert Path(bpy.path.abspath(image.filepath)).exists(), image.filepath
for name in ('path_controller_bad', 'path_controller_good', 'path_take_004'):
    obj = bpy.data.objects.get(name)
    if obj:
        obj.hide_viewport = obj.hide_render = True
artifact = repo/f'build/portable/blender_{engine}_replay.blend'
bpy.ops.wm.save_as_mainfile(filepath=str(artifact))
report = {'backend': engine, 'ticks': result['steps'], 'hash': result['hash'],
          'live_updates': updates, 'rotors': 8,
          'log': take.log, 'artifact': str(artifact), 'status': 'passed'}
report_path = repo/f'build/portable/blender-port/{engine}-test.bin'
report_path.parent.mkdir(parents=True, exist_ok=True)
binary.write(report_path, report, 'run')
link.unregister()
print('BLENDER_INTEGRATION_PASSED', report)
