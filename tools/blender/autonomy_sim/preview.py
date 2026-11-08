"""Disposable presentation objects; authoring objects never receive state updates."""
import json
import math
import bpy
from mathutils import Matrix, Quaternion, Vector
from . import exporter

CAMERA_ID = 4294967295


def snapshot_key(snapshot):
    # Include resolved seed/faults/profile values, not only the authored base hash.
    return exporter.digest({k:v for k,v in snapshot.items() if k!='source'})


def source_scene(scene):
    if scene.get('sim_preview'):
        source = bpy.data.scenes.get(scene.get('sim_source_scene',''))
        if source is None: raise ValueError('authoring scene is unavailable; open the source .blend to start a new run')
        return source
    return scene


def select(scene):
    if bpy.context.window:
        bpy.context.window.scene = scene


def matrix(p):
    x,y,z,w = p['orientation']
    return exporter.M @ Matrix.LocRotScale(Vector(p['position']),Quaternion((w,x,y,z)),Vector((1,1,1))) @ exporter.M


def ensure(source, snapshot=None, *, identity=None):
    # Native launches clone the saved authoring objects for display. Only the
    # physical identity comes from simctl; no Python scene snapshot feeds physics.
    key = identity if identity is not None else snapshot_key(snapshot)
    hash_property = 'sim_source_hash' if identity is not None else 'sim_snapshot_hash'
    previous = next((s for s in bpy.data.scenes if s.get('sim_preview') and s.get(hash_property)==key),None)
    if previous:
        select(previous); return previous
    scene = bpy.data.scenes.new('AutonomyPreview')
    scene['sim_preview'] = True
    scene[hash_property] = key
    if identity is None:
        scene['sim_snapshot'] = exporter.canonical(snapshot)
    if source is not None:
        scene['sim_source_scene'] = source.name
        scene.world = source.world
        scene.render.engine = source.render.engine
        scene.render.fps = source.render.fps
        scene.render.fps_base = source.render.fps_base
        copies = {}
        for obj in source.objects:
            clone = obj.copy(); clone.animation_data_clear()
            clone['sim_display_name'] = str(obj.get('sim_name',obj.name))
            scene.collection.objects.link(clone); copies[obj] = clone
        for obj,clone in copies.items():
            clone.parent = copies.get(obj.parent)
            clone.matrix_parent_inverse = obj.matrix_parent_inverse.copy()
            if obj.get('sim_role') in ('body','sensor'):
                clone.parent = None
                clone.matrix_world = obj.matrix_world.copy()
            if obj.type=='CAMERA' and obj.parent and obj.parent.get('sim_role')=='sensor':
                clone.matrix_parent_inverse=Matrix.Identity(4)
                clone.matrix_basis=Matrix.Rotation(-math.pi/2,4,'Z')
        if source.camera: scene.camera = copies.get(source.camera)
        for field in ('backend','port','max_ticks','randomize','seed','source_frame'):
            setattr(scene.autonomy_sim,field,getattr(source.autonomy_sim,field))
    else:
        # An imported trace contains all physical geometry and calibration; no
        # source file, profile files or engine is needed for timeline playback.
        for body in snapshot['bodies']:
            root = bpy.data.objects.new(body['name'],None)
            root['sim_id'] = str(body['id']); root['sim_role'] = 'body'
            root['sim_display_name'] = body['name']; root['sim_name'] = body['name']
            scene.collection.objects.link(root); root.matrix_world = matrix(body['pose'])
            for index,shape in enumerate(body['shapes']):
                if shape['kind'] == 3:
                    vertices = [list(exporter.M @ Vector(v)) for v in shape['vertices']]
                    faces = shape['triangles']
                elif shape['kind'] == 0:
                    size=shape['size']
                    vertices=[list(exporter.M @ Vector([size[i] if k&(1<<i) else -size[i] for i in range(3)])) for k in range(8)]
                    faces=[(0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)]
                else:
                    import bmesh
                    bm=bmesh.new()
                    if shape['kind']==1:
                        bmesh.ops.create_uvsphere(bm,u_segments=16,v_segments=8,radius=shape['size'][0])
                    else:
                        bmesh.ops.create_cone(bm,cap_ends=True,segments=24,radius1=shape['size'][0],radius2=shape['size'][0],depth=2*shape['size'][2])
                    bm.verts.ensure_lookup_table()
                    vertices=[list(v.co) for v in bm.verts]; faces=[[v.index for v in f.verts] for f in bm.faces]; bm.free()
                mesh=bpy.data.meshes.new(body['name']+f' shape {index}'); mesh.from_pydata(vertices,[],faces)
                obj=bpy.data.objects.new(mesh.name,mesh); scene.collection.objects.link(obj)
                obj.parent=root; obj.matrix_basis=matrix(shape.get('pose',{'position':[0,0,0],'orientation':[0,0,0,1]}))
        sensor=bpy.data.objects.new('camera',None); scene.collection.objects.link(sensor)
        sensor['sim_role']='sensor'; sensor['sim_id']='sensor/camera'; sensor['sim_display_name']='camera'
        camera=bpy.data.cameras.new('hydrus_camera'); camera.angle=math.radians(snapshot['camera']['horizontal_fov'])
        obj=bpy.data.objects.new('hydrus_camera',camera);scene.collection.objects.link(obj);obj.parent=sensor
        obj.matrix_basis=Matrix.Rotation(-math.pi/2,4,'Z')
        obj['sim_display_name']='hydrus_camera';scene.camera=obj
    select(scene)
    return scene


def for_recording(scene, recording):
    snapshot = recording['header'].get('scenario')
    if snapshot is None:
        # Legacy recordings lack geometry; clone the current authoring asset.
        source=source_scene(scene)
        snapshot={'source':{'content_hash':'legacy/'+source.name},'bodies':[]}
        return ensure(source,snapshot)
    snapshot = dict(snapshot)
    if 'content_hash' not in snapshot.get('source',{}):
        snapshot['source'] = dict(snapshot.get('source',{}), content_hash=exporter.digest(snapshot))
    if scene.get('sim_preview') and scene.get('sim_snapshot_hash')==snapshot_key(snapshot):
        return scene
    return ensure(None,snapshot)


def object_for(scene,name,number=None):
    if number is not None:
        role='sensor' if number==CAMERA_ID else 'body'
        result=next((o for o in scene.objects if o.get('sim_role')==role and
                     (role=='sensor' or exporter.body_id(str(o.get('sim_id',''))) == number)),None)
        if result: return result
    return next((o for o in scene.objects if str(o.get('sim_display_name',o.get('sim_name',o.name))).casefold()==name.casefold()),None)
