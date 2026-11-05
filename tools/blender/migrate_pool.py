"""One-time migration of the experimental pool to the simulation contract."""
import copy
import json
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Quaternion, Vector

repo = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).parent))
from autonomy_sim import exporter

scene = bpy.context.scene
if scene.get('sim_schema'):
    raise RuntimeError('scene is already migrated; migration is intentionally one-time')
legacy = json.loads((repo/'sim/scenarios/hydrus.json').read_text())
vehicle = next(b for b in legacy['bodies'] if b['name']=='hydrus')
folder = repo/'sim/profiles'; folder.mkdir(exist_ok=True)
profile = {'version':1,'description':'Measured Hydrus mass, added inertia, buoyancy, fluid coefficients and motor calibration.',
           'body':{k:copy.deepcopy(vehicle[k]) for k in ('mass','dry_mass','inertia','center_of_mass','volume','buoyancy_center')},
           'hull':[{k:copy.deepcopy(v) for k,v in h.items() if k!='body'} for h in legacy['hull']],
           'thruster_specs':[copy.deepcopy(t['spec']) for t in legacy['thrusters']]}
exporter.atomic_json(folder/'hydrus.json',profile)
scene.name = 'AutonomySource'
scene['sim_schema'] = 1
scene['sim_collection'] = 'Simulation'
scene['sim_hz'] = 360; scene['sim_hydro_period'] = 7
scene['sim_water_density'] = 1025.; scene['sim_gravity'] = 9.81
scene.unit_settings.system = 'METRIC'; scene.unit_settings.scale_length = 1
collection = bpy.data.collections.new('Simulation'); scene.collection.children.link(collection)


def tag(obj, key, role, **properties):
    if obj.name not in collection.objects: collection.objects.link(obj)
    obj['sim_id'] = str(key); obj['sim_role'] = role
    for k,v in properties.items(): obj['sim_'+k] = v
    return obj


def from_pose(p):
    x,y,z,w = p['orientation']
    return exporter.M @ Matrix.LocRotScale(Vector(p['position']),Quaternion((w,x,y,z)),Vector((1,1,1))) @ exporter.M


def box(name,key,center,half,owner=None,parent=None):
    bpy.ops.mesh.primitive_cube_add(size=2, location=center)
    obj=bpy.context.object; obj.name=name
    obj.scale=half
    if parent:
        obj.parent=parent; obj.matrix_parent_inverse=Matrix.Identity(4)
        obj.location=center; obj.scale=half
    tag(obj,key,'collider' if owner else 'body',**({'body':str(owner)} if owner else {}),shape='box',collision=True)
    obj.display_type='WIRE'; obj.hide_render=True
    return obj


root=scene.objects['hydrus']; root.animation_data_clear()
tag(root,'1','body',name='hydrus',motion='dynamic',profile='hydrus')
for index,name in enumerate(('cabin','box','legs_left','legs_right','dvl')):
    tag(scene.objects[name],f'hydrus/hull/{index}','collider',body='1',shape='convex',collision=True)

for number,name in ((2,'Gate'),(3,'Marker')):
    obj=scene.objects[name]; obj.animation_data_clear()
    tag(obj,str(number),'body',name=name.lower(),motion='kinematic',tracked=True,
        **{'class':2 if number==2 else 1},shape='mesh',collision=False,fuzz=True)
    if number==2:
        lo=[min(v.co[i] for v in obj.data.vertices) for i in range(3)]
        hi=[max(v.co[i] for v in obj.data.vertices) for i in range(3)]
        width=.035
        # Three separate bars leave the opening empty.
        for index,x in enumerate((lo[0]+width,hi[0]-width)):
            box(f'Gate collision upright {index}',f'gate/bar/{index}',
                (x,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2),(width,max((hi[1]-lo[1])/2,.04),(hi[2]-lo[2])/2),'2',obj)
        box('Gate collision top','gate/bar/2',((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,hi[2]-width),
            ((hi[0]-lo[0])/2,max((hi[1]-lo[1])/2,.04),width),'2',obj)

# Explicit pool collision proxies, authored in Blender rather than synthesized
# from a render mesh at runtime. Same inner bounds as the previous course.
box('Pool west wall','100',(-6.45,10,-1.64),(.15,14.3,1.64))
box('Pool east wall','101',(6.45,10,-1.64),(.15,14.3,1.64))
box('Pool south wall','102',(0,-4.45,-1.64),(6.6,.15,1.64))
box('Pool north wall','103',(0,24.45,-1.64),(6.6,.15,1.64))
box('Pool floor','104',(0,10,-3.38),(6.6,14.6,.1))

for index,t in enumerate(legacy['thrusters']):
    mount=bpy.data.objects.new(f'Hydrus thrust mount {index}',None); collection.objects.link(mount)
    q=Vector((1,0,0)).rotation_difference(Vector(t['direction']))
    p={'position':t['position'],'orientation':[q.x,q.y,q.z,q.w]}
    mount.parent=root; mount.matrix_parent_inverse=Matrix.Identity(4); mount.matrix_basis=from_pose(p)
    mount.empty_display_type='ARROWS'; mount.empty_display_size=.08
    tag(mount,f'hydrus/thruster/{index}','thruster',body='1',index=index)
    visual=scene.objects.get(f'hydrus | rotor mount {index}')
    if visual:
        bpy.context.view_layer.update(); world=visual.matrix_world.copy()
        mount.matrix_basis=exporter.rigid(root.matrix_world.inverted() @ world)
        bpy.context.view_layer.update()
        visual.parent=mount; visual.matrix_parent_inverse=Matrix.Identity(4)
        visual.matrix_basis=mount.matrix_world.inverted() @ world
    # The visual rotor's +X shaft points opposite the legacy shaft convention.
    # Convert command polarity with the reference axis so the calibrated
    # command-to-world wrench remains unchanged after migration.
    local_axis=(exporter.M @ mount.matrix_basis @ exporter.M).to_3x3() @ Vector((1,0,0))
    if local_axis.normalized().dot(Vector(t['direction'])) < 0:
        spec=profile['thruster_specs'][index]
        spec['command_sign']=-spec.get('command_sign',-1)

camera=scene.objects['camera']; camera_world=camera.matrix_world.copy();camera.animation_data_clear(); camera.parent=root
camera.matrix_parent_inverse=Matrix.Identity(4); camera.matrix_basis=root.matrix_world.inverted() @ camera_world
tag(camera,'sensor/camera','sensor',body='1',width=800,height=600)
scene.frame_set(1); bpy.context.view_layer.update()
exporter.atomic_json(folder/'hydrus.json',profile)
snapshot=exporter.export_scene(scene,folder,check_calibration=False)
profile['geometry_hash']=next(b['geometry_hash'] for b in snapshot['bodies'] if b['id']==1)
profile['geometry_reference']=exporter.geometry_reference(next(b['shapes'] for b in snapshot['bodies'] if b['id']==1))
exporter.atomic_json(folder/'hydrus.json',profile)
exporter.embed_profiles(scene,folder)
snapshot=exporter.export_scene(scene,folder,output=repo/'sim/scenarios/pool.json')
scene['sim_migration']='experimental ad1c601 -> evaluated authoring contract v1'
bpy.ops.wm.save_as_mainfile(filepath=str(repo/'auvs/stonefish_sim/data/pool_scene.blend'))
print('MIGRATED_POOL',len(snapshot['bodies']),snapshot['source']['content_hash'])
