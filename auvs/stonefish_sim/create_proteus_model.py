"""Rebuild the photo-based Proteus asset in Blender (live MCP or --background).

Run with Blender's Python. Geometry uses metres and the controller's body frame:
Stonefish +Y forward, +Z down; Blender coordinates are (-x, y, -z).
The existing pool is kept in its own scene and its source file is never overwritten.
"""
import bpy
import json
import math
from pathlib import Path
from mathutils import Vector, Quaternion

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
OUT = DATA / "models" / "proteus_photo"
OUT.mkdir(parents=True, exist_ok=True)
SCENE_NAME = "Proteus — photo reference"
if SCENE_NAME in bpy.data.scenes:
    bpy.data.scenes.remove(bpy.data.scenes[SCENE_NAME])
scene = bpy.data.scenes.new(SCENE_NAME)
bpy.context.window.scene = scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0
collection = bpy.data.collections.new("Proteus components")
scene.collection.children.link(collection)
groups = {}
inventory = []

PALETTE = {
    "frame": ((0.008, 0.075, 0.72, 1), .38, .0, "abs"),
    "endplates": ((.52, .095, .065, 1), .3, .65, "aluminium"),
    "thrusters": ((.009, .012, .017, 1), .26, .08, "pvc"),
    "metal": ((.62, .65, .67, 1), .3, .82, "aluminium"),
    "white": ((.78, .8, .75, 1), .5, .0, "abs"),
    "boards": ((.015, .14, .055, 1), .6, .0, "abs"),
    "cables": ((.008, .009, .012, 1), .55, .0, "pvc"),
    "red_wire": ((.65, .018, .012, 1), .5, .0, "pvc"),
    "connectors": ((.67, .035, .065, 1), .28, .6, "aluminium"),
    "labels": ((.85, .85, .8, 1), .65, .0, "abs"),
    "acrylic": ((.65, .79, .86, .18), .06, .0, "acrylic"),
    "lens": ((.02, .065, .095, 1), .06, .3, "acrylic"),
}
materials = {}
for key, (color, rough, metal, _) in PALETTE.items():
    mat = bpy.data.materials.new("Proteus_" + key)
    mat.diffuse_color = color
    mat.use_nodes = True
    bsdf = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Base Color'].default_value = color
    bsdf.inputs['Roughness'].default_value = rough
    bsdf.inputs['Metallic'].default_value = metal
    if key == 'acrylic':
        bsdf.inputs['Transmission Weight'].default_value = .9
        bsdf.inputs['IOR'].default_value = 1.49
        bsdf.inputs['Alpha'].default_value = .18
        mat.use_transparency_overlap = False
        try:
            mat.surface_render_method = 'DITHERED'
        except (AttributeError, TypeError):
            pass
    materials[key] = mat

def pos(p):
    return Vector((-p[0], p[1], -p[2]))

def register(obj, name, group, dimensions=None, source="photo estimate"):
    obj.name = name
    for old in list(obj.users_collection):
        old.objects.unlink(obj)
    collection.objects.link(obj)
    obj.data.materials.append(materials[group])
    obj['stonefish'] = False  # These are robot assets, never pool static obstacles.
    obj['component'] = name
    obj['source'] = source
    obj['role'] = 'appearance; mass and hydrodynamics use legacy body proxies'
    groups.setdefault(group, []).append(obj)
    inventory.append({"name": name, "group": group, "dimensions_m": dimensions,
                      "position_body_m": [-obj.location.x, obj.location.y, -obj.location.z],
                      "source": source, "physics": "visual only; no independent mass or displacement"})
    return obj

def box(name, p, size, group, bevel=.002):
    bpy.ops.mesh.primitive_cube_add(size=1, location=pos(p))
    obj = bpy.context.object
    obj.scale = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        mod = obj.modifiers.new("Machined edges", 'BEVEL')
        mod.width = bevel
        mod.segments = 2
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return register(obj, name, group, list(size))

def cylinder(name, p, radius, length, group, axis=(0, 0, 1), vertices=48):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=length, location=pos(p))
    obj = bpy.context.object
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(pos(axis))
    for polygon in obj.data.polygons:
        polygon.use_smooth = len(polygon.vertices) == 4
    return register(obj, name, group, {"radius": radius, "length": length})

def tube(name, p, radius, wall, length, group, axis=(0, 1, 0), count=64):
    verts, faces = [], []
    for r, z in [(radius, -length/2), (radius, length/2),
                 (radius-wall, -length/2), (radius-wall, length/2)]:
        verts.extend([(r*math.cos(2*math.pi*i/count), r*math.sin(2*math.pi*i/count), z)
                      for i in range(count)])
    for i in range(count):
        j = (i+1) % count
        faces.extend([(i, j, count+j, count+i),
                      (2*count+j, 2*count+i, 3*count+i, 3*count+j),
                      (j, i, 2*count+i, 2*count+j),
                      (count+i, count+j, 3*count+j, 3*count+i)])
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    collection.objects.link(obj)
    obj.location = pos(p)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(pos(axis))
    for polygon in mesh.polygons:
        polygon.use_smooth = True
    return register(obj, name, group, {"outer_radius": radius, "wall": wall, "length": length})

def cable(name, points, radius=.0025, group="cables"):
    curve = bpy.data.curves.new(name, 'CURVE')
    curve.dimensions = '3D'
    curve.resolution_u = 12
    curve.bevel_depth = radius
    curve.bevel_resolution = 3
    spline = curve.splines.new('BEZIER')
    spline.bezier_points.add(len(points)-1)
    for bp, p in zip(spline.bezier_points, points):
        bp.co = pos(p)
        bp.handle_left_type = bp.handle_right_type = 'AUTO'
    obj = bpy.data.objects.new(name, curve)
    collection.objects.link(obj)
    return register(obj, name, group, {"radius": radius, "path_body_m": points})

def label(name, body, p, size=.011, group="labels"):
    curve = bpy.data.curves.new(name, 'FONT')
    curve.body = body
    curve.size = size
    curve.align_x = 'CENTER'
    curve.extrude = .00012
    obj = bpy.data.objects.new(name, curve)
    collection.objects.link(obj)
    obj.location = pos(p)
    return register(obj, name, group)

# Two longitudinal pressure housings, scaled from the existing Stonefish model.
tube("Upper acrylic pressure tube", (0, 0, -.05), .1, .005, .3, 'acrylic')
tube("Lower acrylic pressure tube", (0, 0, .13), .05, .005, .31, 'acrylic')
for y in [-.155, .155]:
    cylinder("Upper red endplate " + str(y), (0, y, -.05), .103, .01, 'endplates', (0, 1, 0))
    cylinder("Upper black O-ring " + str(y), (0, y*.965, -.05), .1005, .003, 'thrusters', (0, 1, 0))
    cylinder("Lower black endcap " + str(y), (0, y*1.04, .13), .051, .009, 'thrusters', (0, 1, 0))

# Broad split blue side rails and annular saddles, with triangular braces.
for x in [-.17, .17]:
    for z in [-.05, -.008]:
        box("Blue split rail", (x, 0, z), (.045, .45, .039), 'frame')
    for y in [-.15, -.05, .05, .15]:
        box("Rail segment seam", (x, y, -.029), (.0455, .001, .079), 'thrusters', .0001)
    for y in [-.205, .205]:
        cylinder("Rail retaining bolt", (x, y, -.071), .004, .003, 'metal')
    box("Side aluminium mounting plate", (x*1.145, 0, .024), (.006, .095, .064), 'metal')
    for y in [-.034, .034]:
        for z in [.005, .043]:
            cylinder("Side plate fastener", (x*1.17, y, z), .004, .004, 'thrusters', (1, 0, 0), 16)

for y in [-.135, .135]:
    tube("Blue upper clamp saddle", (0, y, -.05), .114, .012, .023, 'frame')
    box("Clamp lifting tab", (0, y, -.17), (.042, .027, .016), 'frame')
    for x in [-.132, .132]:
        brace = box("Angled saddle brace", (x, y, -.068), (.061, .023, .028), 'frame')
        brace.rotation_euler.y = (-1 if x < 0 else 1) * .65

# Slotted aluminium lower cradle: slots are actual gaps, not black decals.
for x in [-.063, .063]:
    box("Cradle top rail", (x, 0, .18), (.006, .315, .014), 'metal')
    box("Cradle bottom rail", (x, 0, .252), (.006, .315, .014), 'metal')
    box("Cradle middle rail", (x, 0, .215), (.006, .315, .01), 'metal')
    for y in [-.15, -.1, -.05, 0, .05, .1, .15]:
        box("Cradle slot web", (x, y, .215), (.006, .014, .066), 'metal')
    box("Lower cabin hanger", (x, 0, .113), (.007, .105, .105), 'metal')
for y in [-.14, .14]:
    box("Cradle cross tie", (0, y, .253), (.132, .013, .009), 'metal')

# Visible internal trays, boards, heatsinks and wire runs. Identities are illustrative.
box("Upper electronics tray", (0, 0, -.005), (.135, .265, .006), 'white')
for x in [-.062, .062]:
    cylinder("Upper tray tie rod", (x, 0, -.045), .003, .278, 'metal', (0, 1, 0))
for y in [-.085, .0, .083]:
    box("Electronics PCB", (0, y, -.024), (.105, .068, .003), 'boards', .0005)
    for x in [-.032, .025]:
        box("PCB component", (x, y, -.032), (.024, .033, .012), 'thrusters', .0005)
for y in [-.105, .11]:
    box("Upper internal end support", (0, y, -.052), (.137, .012, .092), 'white')
for x in [-.036, -.018, 0, .018, .036]:
    box("Heatsink fin", (x, .008, -.09), (.006, .074, .026), 'metal', .0005)
box("Lower battery placeholder", (0, 0, .13), (.058, .2, .044), 'thrusters')
box("Battery red terminal block", (0, -.107, .13), (.056, .012, .026), 'connectors')
for x in [-.025, .025]:
    cable("Internal power wire", [(x, -.105, -.019), (x, -.06, -.078),
                                  (x, .05, -.064), (x, .105, -.036)], .0015, 'red_wire')
for x in [-.044, .038]:
    cable("Internal signal loom", [(x, -.1, -.01), (x, 0, -.06), (x, .1, -.01)], .0012)

# Red bulkhead penetrators: six positions seen on the photographed endplate.
ports = [(-.052, -.095), (.052, -.095), (-.052, -.021), (.052, -.021), (0, -.125), (0, .033)]
for i, (x, z) in enumerate(ports):
    cylinder("Bulkhead penetrator " + str(i), (x, -.17, z), .0085, .018, 'connectors', (0, 1, 0), 32)
    cylinder("Penetrator seal " + str(i), (x, -.182, z), .0045, .005, 'white', (0, 1, 0))
    for j in range(8):
        angle = 2*math.pi*j/8
        cylinder("Connector knurl", (x+.0085*math.cos(angle), -.17, z+.0085*math.sin(angle)),
                 .0007, .014, 'connectors', (0, 1, 0), 8)

# Four inherited vertical mounts; side pods moved below the rails from photos.
# The two horizontal mount positions are provisional, not measured extrinsics.
mounts = [(.205, 0, .13), (-.205, 0, .13), (.165, .265, 0),
          (-.165, .265, 0), (.165, -.265, 0), (-.165, -.265, 0)]
names = ['thruster_0_left', 'thruster_1_right', 'thruster_2_depth_front_left',
         'thruster_3_depth_front_right', 'thruster_4_depth_back_left', 'thruster_5_depth_back_right']
for i, (x, y, z) in enumerate(mounts):
    axis = (0, -1, 0) if i < 2 else (0, 0, 1)
    tube(names[i] + " black open duct", (x, y, z), .054, .006, .059, 'thrusters', axis)
    cylinder(names[i] + " motor hub", (x, y, z), .016, .085, 'thrusters', axis)
    # Rounded cones protrude at both faces, as on the photographs.
    for direction in [-1, 1]:
        p = Vector((x, y, z)) + Vector(axis)*(.046*direction)
        bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=1, location=pos(p))
        obj = bpy.context.object
        obj.scale = (.016, .016, .022)
        obj.rotation_mode = 'QUATERNION'
        obj.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(pos(axis))
        register(obj, names[i] + " streamlined spinner", 'thrusters')
    for angle in [0, 2*math.pi/3, 4*math.pi/3]:
        if i < 2:
            p = (x+.031*math.cos(angle), y+.023, z+.031*math.sin(angle))
            size = (.048, .005, .004)
            spoke = box(names[i] + " stator", p, size, 'thrusters', .0005)
            spoke.rotation_euler.y = angle
        else:
            p = (x+.031*math.cos(angle), y+.031*math.sin(angle), z+.023)
            spoke = box(names[i] + " stator", p, (.048, .004, .005), 'thrusters', .0005)
            spoke.rotation_euler.z = -angle
    box(names[i] + " blue mounting lug", (x, y, z+.045), (.036, .026, .02), 'frame')
    cylinder(names[i] + " fastening screw", (x+.047, y, z), .003, .004, 'metal', (1, 0, 0), 16)
    start = (ports[i][0], -.184, ports[i][1])
    side = 1 if x > 0 else -1
    cable(names[i] + " cable", [start, (side*.1, -.24, .10),
                                (side*.19, -.13, .17), (x, y, z+.04)])

# Front optical module under the upper hull.
cylinder("Lower optical retaining ring", (0, .169, .13), .043, .015, 'metal', (0, 1, 0))
cylinder("Lower optical window", (0, .179, .13), .036, .007, 'lens', (0, 1, 0))
cylinder("Camera lens proxy", (0, .184, .13), .013, .009, 'thrusters', (0, 1, 0))
label("Proteus identification", "PROTEUS", (0, .035, -.153), .014)
label("Reference scale label", "PHOTO MODEL", (0, -.045, -.153), .008)

# Local +X propeller, exported separately for Stonefish's rotating actuators.
prop_collection = bpy.data.collections.new("Animated propellers")
scene.collection.children.link(prop_collection)
verts, faces = [], []
for k in range(3):
    angle = k*2*math.pi/3
    # A pitched closed blade prism, with local +X as the shaft axis.
    base = [(0, .014, -.005), (.004, .043, -.010), (.004, .045, .004), (0, .018, .007)]
    offset = len(verts)
    for dx in [-.0015, .0015]:
        for px, py, pz in base:
            verts.append((px+dx, py*math.cos(angle)-pz*math.sin(angle), py*math.sin(angle)+pz*math.cos(angle)))
    faces.extend([tuple(offset+j for j in [3, 2, 1, 0]), tuple(offset+j for j in [4, 5, 6, 7])])
    for j in range(4):
        faces.append((offset+j, offset+(j+1)%4, offset+(j+1)%4+4, offset+j+4))
propmesh = bpy.data.meshes.new("Three blade propeller local +X")
propmesh.from_pydata(verts, [], faces)
propmesh.update()
propmesh.materials.append(materials['thrusters'])
for i, p in enumerate(mounts):
    obj = bpy.data.objects.new(names[i] + " rotor", propmesh)
    prop_collection.objects.link(obj)
    obj.location = pos(p)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = Vector((1, 0, 0)).rotation_difference(pos((0, -1, 0) if i < 2 else (0, 0, 1)))
    obj['stonefish'] = False
    obj['actuator_index'] = i

def export_mesh(objects, path, transform=True):
    """Deterministic triangle OBJ; no global Blender exporter settings or MTL dependency."""
    depsgraph = bpy.context.evaluated_depsgraph_get()
    lines = ['# Proteus photo model; metres; Stonefish body frame; triangles']
    offset = 1
    count = 0
    for obj in objects:
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        mesh.calc_loop_triangles()
        for v in mesh.vertices:
            p = obj.matrix_world @ v.co if transform else v.co
            if transform:
                p = Vector((-p.x, p.y, -p.z))
            lines.append('v %.9f %.9f %.9f' % tuple(p))
        for tri in mesh.loop_triangles:
            lines.append('f ' + ' '.join(str(offset+i) for i in tri.vertices))
        offset += len(mesh.vertices)
        count += len(mesh.loop_triangles)
        evaluated.to_mesh_clear()
    path.write_text('\n'.join(lines)+'\n')
    return count

bpy.context.view_layer.update()
asset_groups = []
for key, objects in groups.items():
    triangles = export_mesh(objects, OUT / (key+'.obj'))
    color, rough, metal, material = PALETTE[key]
    asset_groups.append({"name": key, "mesh": 'models/proteus_photo/'+key+'.obj',
                         "material": material, "look": 'proteus_'+key,
                         "rgb": list(color[:3]), "roughness": rough, "metalness": metal,
                         "triangles": triangles, "component_count": len(objects)})
rotor = next(o for o in prop_collection.objects if o.get('actuator_index') == 0)
export_mesh([rotor], OUT / 'propeller.obj', transform=False)
# A nondegenerate physical proxy for zero-mass internal appearance parts.
(OUT / 'appearance_proxy.obj').write_text(
    'v -.001 -.001 -.001\nv .001 -.001 -.001\nv .001 .001 -.001\nv -.001 .001 -.001\n'
    'v -.001 -.001 .001\nv .001 -.001 .001\nv .001 .001 .001\nv -.001 .001 .001\n'
    'f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n'
    'f 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n')

spec = {
    "model": "Proteus photo reference v1", "date": "2026-10-09",
    "coordinate_frame": {"units": "metres", "body_forward": "+Y", "body_down": "+Z",
                          "body_to_blender": "(-x, y, -z)"},
    "accuracy": "Photo-based proportions, anchored to existing simulator dimensions; not a measured CAD reconstruction.",
    "physics_policy": "Six original external body proxies supply mass, inertia, collision, drag and buoyancy. Appearance groups are zero-mass non-buoyant internal parts. No electronics mass or thruster-duct displacement is invented.",
    "dimensions": {"upper_cabin_radius_m": .1, "upper_cabin_length_m": .3,
                   "lower_cabin_radius_m": .05, "lower_cabin_length_m": .31,
                   "cabin_wall_m": .005, "rail_length_m": .45,
                   "photo_duct_outer_diameter_m": .108, "photo_propeller_diameter_m": .09},
    "dimension_sources": {"cabins_and_vertical_mounts": "inherited from robot.cpp and proteus_auv.scn",
                          "horizontal_mounts": "photo estimate: side pods below rails at body z=0.13 m, x=+/-0.205 m",
                          "ducts_frame_cradle_connectors_internal_layout": "estimated from six supplied photographs"},
    "thrusters": [{"index": i, "name": name, "position_body_m": list(p),
                   "rpy_rad": [0, 0, 4.7123] if i < 2 else [0, -1.571, 0],
                   "effective_diameter_m": .18, "visual_diameter_m": .09,
                   "max_rpm": 1000, "normalized": True, "inverted": True, "right_handed": True,
                   "time_constant_s": .2, "thrust_coeff_forward": .48,
                   "thrust_coeff_reverse": .48, "torque_coeff": .05,
                   "source": "inherited dynamics; photo-estimated visual mesh and horizontal mount positions"}
                  for i, (name, p) in enumerate(zip(names, mounts))],
    "sensors": {"Odometry": {"rate_hz": 30, "position_body_m": [0, 0, 0]},
                "ProteusCameraIMU": {"rate_hz": 200, "position_body_m": [0, 0, 0],
                    "angular_velocity_range_rad_s": [20, 20, 20], "acceleration_range_m_s2": 30,
                    "angle_noise_rad": [.00016968]*3, "angular_velocity_noise_rad_s": .00016968,
                    "yaw_drift_rad_s": .000019393, "acceleration_noise_m_s2": .002},
                "Camera": {"rate_hz": 21, "resolution": [800, 600], "horizontal_fov_deg": 60,
                    "position_body_m": [0, .3, 0], "rpy_rad": [1.5708, 0, 3.14],
                    "note": "legacy simulation camera retained; photographed lower optical housing is visual only"}},
    "appearance_groups": asset_groups, "components": inventory,
    "pending_measurements": ["overall dimensions", "dry mass", "mass distribution and CG", "sealed displaced volume",
                             "thruster identity, diameter, thrust curves and poses", "battery type and mass", "camera and IMU extrinsics"],
    "limitations": ["Stonefish standard looks are opaque; Blender supports transparent acrylic",
                    "Internal electronic identities and wiring topology are illustrative",
                    "Visual envelope differs from legacy collision proxies",
                    "Hardware controller file lists eight thrusters; the selected simulation and visible layout use six"]
}
spec["physics_proxies"] = json.loads("[{\"name\":\"TopCabin\",\"type\":\"cylinder\",\"radius_m\":0.1,\"height_m\":0.3,\"thickness_m\":0.005,\"material\":\"acrylic\",\"density_kg_m3\":1200,\"position_body_m\":[0,0,-0.05],\"rpy_rad\":[1.5708,0,0],\"buoyant\":true,\"mass_source\":\"Stonefish shell geometry and material\"},{\"name\":\"BottomCabin\",\"type\":\"cylinder\",\"radius_m\":0.05,\"height_m\":0.31,\"thickness_m\":0.005,\"material\":\"acrylic\",\"density_kg_m3\":1200,\"position_body_m\":[0,0,0.13],\"rpy_rad\":[1.5708,0,0],\"buoyant\":true,\"mass_source\":\"Stonefish shell geometry and material\"},{\"name\":\"LeftRail\",\"type\":\"box\",\"dimensions_m\":[0.04,0.45,0.04],\"thickness_m\":-1,\"material\":\"abs\",\"density_kg_m3\":1040,\"position_body_m\":[0.17,0,-0.03],\"rpy_rad\":[0,0,0],\"buoyant\":true,\"mass_source\":\"solid box in native C++ builder; XML reference differs\"},{\"name\":\"RightRail\",\"type\":\"box\",\"dimensions_m\":[0.04,0.45,0.04],\"thickness_m\":-1,\"material\":\"abs\",\"density_kg_m3\":1040,\"position_body_m\":[-0.17,0,-0.03],\"rpy_rad\":[0,0,0],\"buoyant\":true,\"mass_source\":\"solid box in native C++ builder; XML reference differs\"},{\"name\":\"ProtySupport1\",\"type\":\"mesh\",\"mesh\":\"models/protysupport.obj\",\"scale\":0.001,\"material\":\"abs\",\"density_kg_m3\":1040,\"position_body_m\":[0,0.135,-0.012],\"rpy_rad\":[1.5708,0,0],\"thickness_m\":-1,\"buoyant\":true,\"mass_source\":\"existing mesh volume and ABS density\"},{\"name\":\"ProtySupport2\",\"type\":\"mesh\",\"mesh\":\"models/protysupport.obj\",\"scale\":0.001,\"material\":\"abs\",\"density_kg_m3\":1040,\"position_body_m\":[0,-0.135,-0.012],\"rpy_rad\":[1.5708,0,0],\"thickness_m\":-1,\"buoyant\":true,\"mass_source\":\"existing mesh volume and ABS density\"}]")
spec["controller"] = json.loads("{\"path\":\"auvs/stonefish_sim/proteus.json\",\"tam\":[[0,1,0,0,0,1],[0,1,0,0,0,-1],[0,0,-1,1,1,0],[0,0,-1,1,-1,0],[0,0,-1,-1,1,0],[0,0,-1,-1,-1,0]],\"note\":\"Original sign-based TAM and PID gains retained; not a measured force/moment allocation matrix.\"}")
(OUT / 'specification.json').write_text(json.dumps(spec, indent=2)+'\n')
code = ['// Generated by create_proteus_model.py; do not hand-edit.',
        'bool AddProteusAppearance(RobotBuilder& world)', '{']
for group in asset_groups:
    c = group['rgb']
    code.append('    if(!AddLook(world, "%s", %.6f, %.6f, %.6f, %.6f, %.6f, 0.15, nullptr)) return false;' %
                (group['look'], *c, group['roughness'], group['metalness']))
    code.append('    if(!CallOk(world.addAppearance("Photo_%s", "%s", "%s", "%s"), "Proteus appearance")) return false;' %
                (group['name'], group['material'], group['look'], group['mesh']))
code.extend(['    world.showAppearance();', '    return true;', '}'])
(ROOT / 'proteus_appearance.inc').write_text('\n'.join(code)+'\n')

# Native file includes the preserved pool as a separate scene for easy switching.
scene['reference_photos'] = '/home/cesar/Downloads/WhatsApp Unknown 2026-10-09 at 5.28.48 PM'
scene['specification'] = str(OUT / 'specification.json')
scene['physics_policy'] = spec['physics_policy']
scene.world = bpy.data.worlds.new("Proteus neutral world")
scene.world.color = (.15, .15, .15)
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type == 'VIEW_3D':
            space = area.spaces.active
            space.shading.type = 'MATERIAL'
            space.region_3d.view_location = Vector((0, 0, -.02))
            space.region_3d.view_distance = 1.1
            space.region_3d.view_rotation = Quaternion((.8536, .3536, -.1464, -.3536))
            space.overlay.show_floor = False
            space.clip_start = .001
for obj in bpy.context.selected_objects:
    obj.select_set(False)
bpy.ops.wm.save_as_mainfile(filepath=str(DATA / 'proteus_photo.blend'))
print(json.dumps({"blend": str(DATA/'proteus_photo.blend'), "objects": len(collection.objects)+len(prop_collection.objects),
                  "appearance_groups": len(asset_groups), "triangles": sum(g['triangles'] for g in asset_groups),
                  "specification": str(OUT/'specification.json')}))
