"""Authoring mutations must survive evaluation; no world preset may restore them."""
from pathlib import Path
import importlib.util
import os
import tempfile
import shutil
import subprocess
import sys
import bpy
from mathutils import Vector

repo=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).parent))
import autonomy_sim as link
from autonomy_sim import exporter, wire

arguments=sys.argv[sys.argv.index('--')+1:]
checker, backends = Path(arguments[0]), [Path(path) for path in arguments[1:]]
scene=bpy.context.scene
profiles=repo/'sim/profiles'
collection=bpy.data.collections[scene['sim_collection']]
cases={}
temporary=tempfile.TemporaryDirectory(prefix='native-blend-source-')
folder=Path(temporary.name)
simctl=checker.parent/'simctl'
env={k:v for k,v in os.environ.items() if not k.startswith('AUTONOMY_SIM_')}
env.update(AUTONOMY_SIM_PROFILES='/unavailable-profile-sidecars', AUTONOMY_SIM_BLENDER_EXECUTABLE='/missing-blender', AUTONOMY_SIM_BLENDER_SCRIPT='/missing-exporter')


def evaluate(name=None):
    path=folder/((name or 'check')+'.blend')
    bpy.ops.wm.save_as_mainfile(filepath=str(path))
    result=subprocess.run([str(simctl),'resolve',str(path)],env=env,capture_output=True)
    if result.returncode: raise ValueError(result.stderr.decode(errors='replace'))
    document=wire.decode(result.stdout)
    if name: cases[name]=path
    return document


def expect_failure(fn, text):
    try: fn()
    except ValueError as error:
        assert text in str(error),(text,str(error))
    else: raise AssertionError('evaluation unexpectedly accepted '+text)


baseline=evaluate('baseline')
scene['sim_hz']=-1
expect_failure(lambda:evaluate(),'sim_hz must be')
scene['sim_hz']=360
assert baseline['source']['kind']=='blender_native', 'native reader did not load the saved file'
assert baseline['source']['content_hash']==evaluate()['source']['content_hash']
allocation=importlib.util.spec_from_file_location('allocation', repo/'sim/tests/controller.py')
allocation_check=importlib.util.module_from_spec(allocation)
allocation.loader.exec_module(allocation_check)
allocation_check.verify(baseline)
gate=scene.objects['Gate']; marker=scene.objects['Marker']; robot=scene.objects['hydrus']
gate.location.x-=.75;gate.rotation_euler.z+=.2
moved=evaluate('moved')
assert moved['source']['content_hash']!=baseline['source']['content_hash']
assert abs(next(b for b in moved['bodies'] if b['id']==2)['pose']['position'][0]-1.75)<1e-5
original=robot.matrix_world.copy()
robot.location.x+=.3
robot.rotation_euler.z+=.2
evaluate('robot_moved')
robot.matrix_world=original
mount=next(o for o in collection.objects if o.get('sim_id')=='hydrus/thruster/0')
mount.location.x+=.1
camera=scene.objects['camera'];camera.location.z+=.15
attached=evaluate('mounts')
assert abs(attached['thrusters'][0]['position'][0]-baseline['thrusters'][0]['position'][0])>.09
assert abs(attached['camera']['pose']['position'][2]-baseline['camera']['pose']['position'][2])>.14

# A newly authored collider blocks the vehicle. The very same object with
# collisions disabled remains in observations/geometry without contact forces.
bpy.ops.mesh.primitive_cube_add(size=2,location=robot.matrix_world.translation+Vector((0,0,-.2)))
box=bpy.context.object;box.name='Authored obstacle';box.scale=(1,1,.1)
if box.name not in collection.objects:
    collection.objects.link(box)
for key,value in {'sim_id':'200','sim_role':'body','sim_shape':'box','sim_collision':True,
                  'sim_class':42,'sim_tracked':True}.items():box[key]=value
evaluate('added')
box['sim_collision']=False
disabled=evaluate('disabled')
bpy.data.objects.remove(marker,do_unlink=True)
deleted=evaluate('deleted')
assert all(b['id']!=3 for b in deleted['bodies'])

# Saved parent transforms are read natively; modifiers must be applied first.
box['sim_shape']='convex'
modifier=box.modifiers.new('Evaluated bevel','BEVEL');modifier.width=.1;modifier.segments=2
parent=bpy.data.objects.new('Authoring parent',None);scene.collection.objects.link(parent)
box.parent=parent;parent.location.x=2
expect_failure(lambda:evaluate(),'apply modifiers')
bpy.context.view_layer.objects.active=box
bpy.ops.object.modifier_apply(modifier=modifier.name)
evaluated=evaluate()
shape=next(b for b in evaluated['bodies'] if b['id']==200)['shapes'][0]
assert len(shape['vertices'])>8
before=next(b for b in disabled['bodies'] if b['id']==200)['pose']['position'][0]
after=next(b for b in evaluated['bodies'] if b['id']==200)['pose']['position'][0]
assert abs(after-before+2)<1e-5

# Collection instancing produces independent stable physical IDs/transforms.
prototype=bpy.data.collections.new('Prototype')
mesh=box.data.copy(); obj=bpy.data.objects.new('Instanced box',mesh);prototype.objects.link(obj)
for key,value in {'sim_id':'prototype','sim_role':'body','sim_shape':'convex','sim_collision':True}.items():obj[key]=value
instance=bpy.data.objects.new('Box instance',None);collection.objects.link(instance)
instance.instance_type='COLLECTION';instance.instance_collection=prototype;instance.location=(3,4,-2)
instance['sim_role']='instance';instance['sim_id']='instances/one'
instances=evaluate('instances')
assert len(instances['bodies'])>len(deleted['bodies'])

old=box['sim_id'];box['sim_id']='2'
expect_failure(lambda:evaluate(),'duplicate sim_id')
box['sim_id']=old
robot['sim_profile']='missing'
expect_failure(lambda:evaluate(),'missing profile')
robot['sim_profile']='hydrus'
box['sim_shape']='mesh';box['sim_collision']=True
expect_failure(lambda:evaluate(),'convex proxy')
box['sim_shape']='convex'
vertex=scene.objects['box'].data.vertices[0]; old_vertex=vertex.co.copy(); vertex.co.x+=.2
expect_failure(lambda:evaluate(),'recalibrate')
vertex.co=old_vertex

order=('baseline','moved','deleted','robot_moved','mounts','instances','added','disabled')
for backend in backends:
    result=subprocess.run([str(checker),str(backend),*(str(cases[name]) for name in order)],env=env,capture_output=True)
    assert result.returncode==0, result.stderr.decode(errors='replace')[-4000:]

# The authoring button sets all required fields and registers the object.
link.register()
assert bpy.ops.autonomy_sim.add_obstacle()=={'FINISHED'}
authored=bpy.context.object
assert authored.get('sim_role')=='body' and authored.get('sim_collision')
assert authored.name in collection.objects
link.unregister()

temporary.cleanup()
print('NATIVE_BLENDER_SOURCE_OWNERSHIP_PASSED')
