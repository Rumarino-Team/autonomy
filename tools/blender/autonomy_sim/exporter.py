"""Evaluated Blender -> owned portable scene. No preset world-object merging."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import base64

import bpy
import bmesh
from mathutils import Matrix, Vector
from . import wire

VERSION = 1
M = Matrix.Diagonal((-1., 1., -1., 1.))
ROLES = {'body', 'collider', 'thruster', 'sensor', 'visual', 'instance'}


def embed_profiles(scene, profiles_dir):
    """Save measured calibration inside .blend; no sidecar files needed at run time."""
    collection=bpy.data.collections.get(scene.get('sim_collection',''))
    if collection is None: raise ValueError('missing simulation source collection')
    names=sorted({str(o.get('sim_profile')) for o in collection.all_objects if o.get('sim_profile')})
    mapping=dict(scene.get('sim_profile_texts',{}))
    for name in names:
        path=Path(profiles_dir)/(name+'.json')
        if not path.is_file(): raise ValueError('missing profile '+name)
        profile=json.loads(path.read_text())
        previous=mapping.get(name)
        text_name=previous.name if isinstance(previous,bpy.types.Text) else previous or f'.autonomy/{scene.name}/profile/{name}'
        text=bpy.data.texts.get(text_name) or bpy.data.texts.new(text_name)
        text.use_fake_user=True
        text.clear();text.write('ASIM_PROFILE_1\n'+base64.b64encode(wire.pack(profile)).decode('ascii'))
        # A real datablock reference follows scene append/link and name remaps.
        mapping[name]=text
    scene['sim_profile_texts']=mapping
    return names


def read_profile(scene, name, profiles_dir):
    reference=scene.get('sim_profile_texts',{}).get(name)
    if reference:
        text=reference if isinstance(reference,bpy.types.Text) else bpy.data.texts.get(reference)
        if text is None: raise ValueError('missing embedded profile '+name)
        header,separator,data=text.as_string().partition('\n')
        if header!='ASIM_PROFILE_1' or not separator: raise ValueError('unsupported embedded profile '+name)
        return wire.unpack(base64.b64decode(data,validate=True))
    path=profiles_dir/(name+'.json')
    if not path.is_file(): raise ValueError('missing profile '+name)
    return json.loads(path.read_text())


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as output:
            output.write(canonical(value)+'\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def fail(obj, message):
    raise ValueError(f'{obj.name}: {message}')


def identifier(obj):
    value = str(obj.get('sim_id', '')).strip()
    if not value:
        fail(obj, 'missing sim_id')
    return value


def body_id(value):
    result = int(value) if value.isdecimal() else int(hashlib.sha256(value.encode()).hexdigest()[:8], 16)
    if not 0 <= result < 4294967295:
        raise ValueError(f'invalid or reserved body ID: {value}')
    return result


def rigid(matrix):
    p, q, _ = matrix.decompose()
    return Matrix.LocRotScale(p, q.normalized(), Vector((1, 1, 1)))


def pose(matrix):
    converted = M @ rigid(matrix) @ M
    p, q, _ = converted.decompose()
    return {'position': list(p), 'orientation': [q.x, q.y, q.z, q.w]}


def shape(obj, world, body_world, depsgraph, calibrated=False):
    if obj.type != 'MESH':
        fail(obj, 'collision/observation geometry must be a mesh')
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        if not mesh.vertices or not mesh.polygons:
            fail(obj, 'empty evaluated mesh')
        local = rigid(body_world).inverted() @ world
        part = rigid(local)
        residual = part.inverted() @ local
        vertices = [list((M @ residual) @ v.co) for v in mesh.vertices]
        calibration_points=copy.deepcopy(vertices)
        if not all(math.isfinite(v) for point in vertices for v in point):
            fail(obj,'non-finite evaluated vertex')
        mesh.calc_loop_triangles()
        faces = [list(t.vertices) for t in mesh.loop_triangles]
        if residual.to_3x3().determinant() < 0:
            faces = [list(reversed(t)) for t in faces]
        kind = str(obj.get('sim_shape', 'mesh'))
        collision = bool(obj.get('sim_collision', False))
        if collision and kind not in ('box', 'sphere', 'cylinder', 'convex'):
            fail(obj, 'collisions require an explicit primitive or convex proxy; concave mesh is unsupported')
        if kind == 'convex':
            bm = bmesh.new()
            try:
                for v in vertices:
                    bm.verts.new(v)
                bmesh.ops.convex_hull(bm, input=list(bm.verts), use_existing_faces=False)
                if not bm.faces:
                    fail(obj, 'degenerate convex hull')
                bmesh.ops.triangulate(bm, faces=list(bm.faces))
                used = sorted({v for f in bm.faces for v in f.verts}, key=lambda v: tuple(v.co))
                indices = {v:i for i,v in enumerate(used)}
                vertices = [list(v.co) for v in used]
                faces = sorted([indices[v] for v in f.verts] for f in bm.faces)
            finally:
                bm.free()
        result = {'kind': 3, 'pose': pose(part), 'vertices': vertices, 'triangles': faces,
                  'collisions': collision, 'friction': float(obj.get('sim_friction', 1)),
                  'restitution': float(obj.get('sim_restitution', 0)), 'source_id': identifier(obj)}
        if calibrated:result['calibration_points']=calibration_points
        if kind in ('box', 'sphere', 'cylinder'):
            lo = [min(v[i] for v in vertices) for i in range(3)]
            hi = [max(v[i] for v in vertices) for i in range(3)]
            half = [(b-a)/2 for a,b in zip(lo, hi)]
            center = Vector([(a+b)/2 for a,b in zip(lo, hi)])
            if min(half) <= 1e-7:
                fail(obj, 'degenerate primitive')
            if kind == 'box' and any(min(abs(v[i]-lo[i]), abs(v[i]-hi[i])) > 1e-5 for v in vertices for i in range(3)):
                fail(obj, 'evaluated geometry is not a box; use a convex proxy')
            if kind == 'sphere' and max(half)-min(half) > 1e-5:
                fail(obj, 'nonuniform sphere scale')
            if kind == 'cylinder' and abs(half[0]-half[1]) > 1e-5:
                fail(obj, 'elliptical cylinder; use a convex proxy')
            transform = M @ part @ M
            transform.translation += transform.to_3x3() @ center
            # transform is already NED; pose() expects Blender coordinates.
            result.update(kind={'box':0,'sphere':1,'cylinder':2}[kind], pose=pose(M @ transform @ M), size=half)
            result.pop('vertices')
            result.pop('triangles')
        return result
    finally:
        evaluated.to_mesh_clear()


def geometry_reference(shapes):
    # Hash physical points, not triangulation/order. A root rotation can reorder
    # equal-coordinate hull vertices through f32 roundoff without changing shape.
    parts=[]
    for s in shapes:
        x,y,z,w=s['pose']['orientation']
        from mathutils import Quaternion
        rotation=Quaternion((w,x,y,z)).to_matrix(); position=Vector(s['pose']['position'])
        points=s.get('calibration_points',s.get('vertices'))
        if points is None:
            size=s['size'];points=[[size[i] if k&(1<<i) else -size[i] for i in range(3)] for k in range(8)]
        points=sorted(set(tuple(v for v in position+rotation@Vector(p)) for p in points))
        parts.append({'id':s['source_id'],'kind':s['kind'],'collision':s['collisions'],'points':points})
    return parts


def geometry_hash(shapes):
    parts=geometry_reference(shapes)
    for part in parts:part['points']=[tuple(round(v,4) for v in point) for point in part['points']]
    return digest(parts)


def calibration_matches(shapes,reference,tolerance=1e-4):
    actual=geometry_reference(shapes)
    if len(actual)!=len(reference):return False
    for a,b in zip(actual,reference):
        if any(a[k]!=b[k] for k in ('id','kind','collision')) or len(a['points'])!=len(b['points']):return False
        for p in a['points']:
            if min(sum((x-y)**2 for x,y in zip(p,q)) for q in b['points'])>tolerance*tolerance:return False
        for p in b['points']:
            if min(sum((x-y)**2 for x,y in zip(p,q)) for q in a['points'])>tolerance*tolerance:return False
    return True


def export_scene(scene, profiles_dir, frame=1, output=None, check_calibration=True):
    if scene.get('sim_preview'):
        raise ValueError('preview scenes cannot be simulation sources')
    if scene.get('sim_schema') != VERSION:
        raise ValueError('scene has no simulation contract; migrate/author it first')
    collection = bpy.data.collections.get(scene.get('sim_collection', ''))
    if collection is None or collection.name not in {c.name for c in scene.collection.children_recursive}:
        raise ValueError('missing simulation source collection')
    if abs(scene.unit_settings.scale_length-1) > 1e-8:
        raise ValueError('simulation source must use metre units (scale_length=1)')
    old_frame, old_subframe = scene.frame_current, scene.frame_subframe
    with bpy.context.temp_override(scene=scene):
        for obj in collection.all_objects:
            if obj.mode=='EDIT': obj.update_from_editmode()
        scene.frame_set(frame)
        depsgraph = bpy.context.evaluated_depsgraph_get()
        try:
            return _export(scene, collection, depsgraph, Path(profiles_dir), frame, output, check_calibration)
        finally:
            scene.frame_set(old_frame, subframe=old_subframe)


def _export(scene, collection, dg, profiles_dir, frame, output, check_calibration):
    objects = sorted(collection.all_objects, key=lambda o: str(o.get('sim_id', o.name)))
    seen = set()
    for obj in objects:
        role = obj.get('sim_role')
        if role not in ROLES:
            fail(obj, 'unclassified object in simulation collection')
        if role == 'visual': continue
        key = identifier(obj)
        if key in seen: fail(obj, f'duplicate sim_id {key}')
        seen.add(key)
        matrix = obj.evaluated_get(dg).matrix_world
        if not all(math.isfinite(v) for row in matrix for v in row) or abs(matrix.to_3x3().determinant()) < 1e-12:
            fail(obj, 'invalid or singular transform')
        if role == 'body' and (obj.constraints or obj.animation_data):
            fail(obj, 'authoring bodies must have static initial transforms; animate preview objects instead')
    entries = [(obj, identifier(obj), obj.evaluated_get(dg).matrix_world.copy()) for obj in objects if obj.get('sim_role') == 'body']
    for instance in dg.object_instances:
        if not instance.is_instance or instance.parent is None: continue
        parent = instance.parent.original
        if parent not in objects or parent.get('sim_role') == 'visual': continue
        if parent.get('sim_role') != 'instance':
            fail(parent,'unrealized instances require an explicit instance container; realize modifier instances before export')
        obj = instance.object.original
        if obj.get('sim_role') == 'visual': continue
        if obj.get('sim_role') != 'body' or obj.type != 'MESH' or obj.get('sim_profile') or obj.get('sim_motion', 'static') != 'static':
            fail(parent, 'instances require standalone static mesh bodies with explicit roles')
        key = identifier(parent)+'/'+identifier(obj)+'/'+'.'.join(map(str,instance.persistent_id))
        entries.append((obj,key,instance.matrix_world.copy()))
    entries.sort(key=lambda x:x[1])
    bodies, hull, thrusters, profiles, origins = [], [], [], {}, []
    by_id, numeric = {}, set()
    for obj,key,world in entries:
        number = body_id(key)
        if number in numeric: fail(obj, 'numeric body ID collision')
        numeric.add(number)
        profile_name = str(obj.get('sim_profile', ''))
        profile = {}
        if profile_name:
            try: profile = read_profile(scene,profile_name,profiles_dir)
            except ValueError as error: fail(obj,str(error))
            if profile.get('version') != 1 or 'shapes' in profile.get('body', {}): fail(obj, 'invalid calibration profile')
            profiles[profile_name] = digest(profile)
        motion = str(obj.get('sim_motion','static'))
        if motion not in ('static','dynamic','kinematic'): fail(obj, 'invalid sim_motion')
        if motion == 'dynamic' and not profile: fail(obj, 'dynamic body requires a calibrated profile')
        name = str(obj.get('sim_name', obj.name))
        b = copy.deepcopy(profile.get('body', {}))
        b.update(id=number, sim_id=key, name=name, motion={'static':0,'dynamic':1,'kinematic':2}[motion],
                 pose=pose(world), shapes=[], tracked=bool(obj.get('sim_tracked',False)),
                 **{'class':int(obj.get('sim_class',0)), 'fuzz':bool(obj.get('sim_fuzz',False))})
        if not 0 <= b['class'] <= 255: fail(obj, 'class must fit uint8')
        parts = [p for p in objects if p.get('sim_role') == 'collider' and str(p.get('sim_body','')) == identifier(obj)] if key == identifier(obj) else []
        if obj.type == 'MESH':
            b['shapes'].append(shape(obj,world,world,dg,bool(profile)))
        for part in parts:
            b['shapes'].append(shape(part,part.evaluated_get(dg).matrix_world,world,dg,bool(profile)))
        if not b['shapes']: fail(obj, 'body has no geometry')
        gh = geometry_hash(b['shapes'])
        if profile and check_calibration and gh != profile.get('geometry_hash') and not calibration_matches(b['shapes'],profile.get('geometry_reference',[])):
            fail(obj, 'vehicle geometry changed; recalibrate the physics profile before running')
        b['geometry_hash'] = gh
        if profile_name: b['profile'] = profile_name
        bodies.append(b)
        by_id[key] = (obj,world,b,profile)
        origins.append({'id':number,'sim_id':key,'object':obj.name,'role':'body'})
        for h in profile.get('hull',[]):
            h = copy.deepcopy(h); h['body'] = number; hull.append(h)
    if not bodies: raise ValueError('source scene has no simulation bodies')
    cameras = []
    for obj in objects:
        role = obj.get('sim_role')
        if role not in ('collider','sensor','thruster'): continue
        owner = str(obj.get('sim_body',''))
        if owner not in by_id: fail(obj, f'missing owning body {owner}')
        body,world,b,profile = by_id[owner]
        if role == 'collider': continue
        transform = pose(rigid(world).inverted() @ obj.evaluated_get(dg).matrix_world)
        if role == 'thruster':
            index = int(obj.get('sim_index',-1))
            specs = profile.get('thruster_specs',[])
            if not 0 <= index < len(specs): fail(obj, 'missing thruster calibration/index')
            q = (M @ rigid(rigid(world).inverted() @ obj.evaluated_get(dg).matrix_world) @ M).to_3x3()
            direction = q @ Vector((1,0,0))
            thrusters.append({'body':b['id'],'position':transform['position'],'direction':list(direction.normalized()),
                              'spec':copy.deepcopy(specs[index]),'source_id':identifier(obj),'index':index})
        else:
            camera = obj if obj.type == 'CAMERA' else next((c for c in obj.children if c.type == 'CAMERA'), None)
            if camera is None: fail(obj, 'sensor requires a camera datablock or child camera')
            # Blender cameras look along -Z with +Y up. Preserve the existing
            # pool optical convention while including edits to the camera child.
            optical = rigid(world).inverted() @ rigid(camera.evaluated_get(dg).matrix_world) @ Matrix.Rotation(math.pi/2,4,'Z')
            cameras.append({'body':b['id'],'pose':pose(optical),'width':int(obj.get('sim_width',800)),
                            'height':int(obj.get('sim_height',600)),'horizontal_fov':math.degrees(camera.data.angle),
                            'source_id':identifier(obj)})
    if len(cameras) != 1: raise ValueError('exactly one simulation camera is required')
    thrusters.sort(key=lambda t:(t['body'],t['index']))
    for key,(_,_,b,p) in by_id.items():
        indices = [t['index'] for t in thrusters if t['body']==b['id']]
        if indices != list(range(len(p.get('thruster_specs',[])))):
            raise ValueError(f'{key}: missing/duplicate thruster mounts')
    document = {'version':1,'dt':[1,int(scene.get('sim_hz',360))], 'gravity':[0,0,float(scene.get('sim_gravity',9.81))],
                'water_density':float(scene.get('sim_water_density',1025)), 'hydro_period_ticks':int(scene.get('sim_hydro_period',7)),
                'bodies':bodies,'hull':hull,'thrusters':thrusters,'camera':cameras[0],
                'vehicle':next((b['name'] for b in bodies if b['id']==cameras[0]['body']), '')}
    identity = digest({'scene':document,'profiles':profiles})
    document['source'] = {'kind':'blender_evaluated','schema':VERSION,'exporter_version':VERSION,
                          'blender_version':bpy.app.version_string,'file':bpy.data.filepath,'scene':scene.name,
                          'frame':frame,'content_hash':identity,'profiles':profiles,'objects':origins,
                          'excluded':sorted(o.name for o in scene.objects if o not in objects or o.get('sim_role')=='visual')}
    if output: atomic_json(output,document)
    return document
