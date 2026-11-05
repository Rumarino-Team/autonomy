"""Authoring mutations must survive export; no world preset may restore them."""
import json
from pathlib import Path
import sys
import bpy
from mathutils import Vector

repo=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).parent))
import autonomy_sim as link
from autonomy_sim import exporter

folder=Path(sys.argv[sys.argv.index('--')+1]); folder.mkdir(parents=True,exist_ok=True)
scene=bpy.context.scene
profiles=repo/'sim/profiles'
collection=bpy.data.collections[scene['sim_collection']]


def save(name):
    return exporter.export_scene(scene,profiles,output=folder/(name+'.json'))


def expect_failure(fn, text):
    try: fn()
    except ValueError as error:
        assert text in str(error),(text,str(error))
    else: raise AssertionError('export unexpectedly accepted '+text)


baseline=save('baseline')
assert exporter.export_scene(scene,Path('/unavailable-profile-sidecars'))['source']['content_hash']==baseline['source']['content_hash'], 'calibration is not embedded in .blend'
published=json.loads((repo/'sim/scenarios/pool.json').read_text())
assert baseline['source']['content_hash']==published['source']['content_hash'], 'run zig build sim-scene and commit the refreshed snapshot'
assert baseline['source']['content_hash']==save('repeat')['source']['content_hash']
gate=scene.objects['Gate']; marker=scene.objects['Marker']; robot=scene.objects['hydrus']
gate.location.x-=.75;gate.rotation_euler.z+=.2
moved=save('moved')
assert moved['source']['content_hash']!=baseline['source']['content_hash']
assert abs(next(b for b in moved['bodies'] if b['id']==2)['pose']['position'][0]-1.75)<1e-5
original=robot.matrix_world.copy()
robot.location.x+=.3
robot.rotation_euler.z+=.2
save('robot_moved')
robot.matrix_world=original
mount=next(o for o in collection.objects if o.get('sim_id')=='hydrus/thruster/0')
mount.location.x+=.1
camera=scene.objects['camera'];camera.location.z+=.15
attached=save('mounts')
assert abs(attached['thrusters'][0]['position'][0]-baseline['thrusters'][0]['position'][0])>.09
assert abs(attached['camera']['pose']['position'][2]-baseline['camera']['pose']['position'][2])>.14

# A newly authored collider blocks the vehicle. The very same object with
# collisions disabled remains in observations/geometry without contact forces.
bpy.ops.mesh.primitive_cube_add(size=2,location=robot.matrix_world.translation+Vector((0,0,-.2)))
box=bpy.context.object;box.name='Authored obstacle';box.scale=(1,1,.1)
collection.objects.link(box)
for key,value in {'sim_id':'200','sim_role':'body','sim_shape':'box','sim_collision':True,
                  'sim_class':42,'sim_tracked':True}.items():box[key]=value
save('added')
box['sim_collision']=False
disabled=save('disabled')
bpy.data.objects.remove(marker,do_unlink=True)
deleted=save('deleted')
assert all(b['id']!=3 for b in deleted['bodies'])

# Evaluate modifiers and parenting rather than reading the stored mesh alone.
box['sim_shape']='convex'
modifier=box.modifiers.new('Evaluated bevel','BEVEL');modifier.width=.1;modifier.segments=2
parent=bpy.data.objects.new('Authoring parent',None);scene.collection.objects.link(parent)
box.parent=parent;parent.location.x=2
evaluated=save('evaluated')
shape=next(b for b in evaluated['bodies'] if b['id']==200)['shapes'][0]
assert len(shape['vertices'])>8
before=next(b for b in disabled['bodies'] if b['id']==200)['pose']['position'][0]
after=next(b for b in evaluated['bodies'] if b['id']==200)['pose']['position'][0]
assert abs(after-before+2)<1e-5

# Collection instancing produces independent stable physical IDs/transforms.
prototype=bpy.data.collections.new('Prototype')
mesh=box.data.copy(); obj=bpy.data.objects.new('Instanced box',mesh);prototype.objects.link(obj)
for key,value in {'sim_id':'prototype','sim_role':'body','sim_shape':'box','sim_collision':True}.items():obj[key]=value
instance=bpy.data.objects.new('Box instance',None);collection.objects.link(instance)
instance.instance_type='COLLECTION';instance.instance_collection=prototype;instance.location=(3,4,-2)
instance['sim_role']='instance';instance['sim_id']='instances/one'
instances=save('instances')
assert any(b['sim_id'].startswith('instances/one/prototype/') for b in instances['bodies'])

# Failures leave the previous valid snapshot untouched, never launch a fallback.
path=folder/'instances.json';saved=path.read_bytes()
old=box['sim_id'];box['sim_id']='2'
expect_failure(lambda:exporter.export_scene(scene,profiles,output=path),'duplicate sim_id')
assert path.read_bytes()==saved
box['sim_id']=old
robot['sim_profile']='missing'
expect_failure(lambda:save('bad'),'missing profile')
robot['sim_profile']='hydrus'
box['sim_shape']='mesh';box['sim_collision']=True
expect_failure(lambda:save('bad'),'concave mesh')
box['sim_shape']='convex'
vertex=scene.objects['box'].data.vertices[0]; old_vertex=vertex.co.copy(); vertex.co.x+=.2
expect_failure(lambda:save('bad'),'recalibrate')
vertex.co=old_vertex

# The authoring button sets all required fields and registers the object.
link.register()
assert bpy.ops.autonomy_sim.add_obstacle()=={'FINISHED'}
authored=bpy.context.object
assert authored.get('sim_role')=='body' and authored.get('sim_collision')
assert authored.name in collection.objects
link.unregister()

print('BLENDER_SOURCE_OWNERSHIP_PASSED',folder)
