"""Live Hydrus view, plus recorded takes. UDP poses from the MuJoCo plugin move named objects.

Install this folder as an add-on, or enable it from the repository. Build Hydrus
scene creates a starter layout from hydrus.xml. Start link binds 127.0.0.1 and
applies the newest datagram. The scene's render engine is left alone, so Eevee
and Cycles are whichever engine the file uses.

Takes are deterministic headless runs (src_3 record). Each one is a seed and its
recording, baked into keyframes so Render Animation reproduces it offline.
"""

import json
import math
import os
import re
import socket
import struct
import subprocess
from pathlib import Path

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import AddonPreferences, Operator, Panel, PropertyGroup, UIList
from mathutils import Matrix, Quaternion, Vector

if __package__:
    from . import rotor_animation
else:
    # Preserve the documented --python __init__.py starter-scene entry point.
    import importlib.util
    _rotor_spec = importlib.util.spec_from_file_location("hydrus_rotor_animation", Path(__file__).with_name("rotor_animation.py"))
    rotor_animation = importlib.util.module_from_spec(_rotor_spec)
    _rotor_spec.loader.exec_module(rotor_animation)

bl_info = {
    "name": "Hydrus MuJoCo Link",
    "author": "Rumarino",
    "version": (1, 2, 1),
    "blender": (4, 5, 0),
    "location": "View3D > Sidebar > Hydrus",
    "description": "Drive a Hydrus scene from the MuJoCo simulator, record takes, and branch them",
    "category": "Animation",
}

MAGIC = b"AMJB"
VERSION = 1
HEADER = struct.Struct("<4sHHd")
POSE = struct.Struct("<32s3d4d")
# pool_scene.blend frame. NED (north, east, down) <-> Blender (-north, east, -down).
S = Matrix(((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)))
ROOTS = ("hydrus", "gate", "marker", "camera", "water", "floor")

_link = {"running": False, "socket": None}


def _discover_xml():
    prefs = bpy.context.preferences.addons.get(__name__)
    if prefs is not None:
        chosen = prefs.preferences.model_dir.strip()
        if chosen:
            path = Path(bpy.path.abspath(chosen)) / "hydrus.xml"
            if path.is_file():
                return path
    here = Path(__file__).resolve()
    candidates = [
        here.parents[1] / "hydrus.xml",
        here.parents[2] / "hydrus_mujoco" / "hydrus.xml",
    ]
    for path in candidates:
        if path.is_file():
            return path
    return None


def _floats(text, count, fallback):
    if not text:
        return list(fallback)
    values = [float(part) for part in text.split()]
    if len(values) < count:
        values.extend(fallback[len(values) :])
    return values[:count]


def _euler_matrix(angles, degrees):
    if degrees:
        angles = [math.radians(angle) for angle in angles]
    x, y, z = angles
    # MuJoCo xyz is an intrinsic rotation: Rx, then Ry, then Rz.
    return Matrix.Rotation(z, 3, "Z") @ Matrix.Rotation(y, 3, "Y") @ Matrix.Rotation(x, 3, "X")


def _world_matrix(pos, rotation):
    blender = S @ rotation @ S
    return Matrix.LocRotScale(S @ Vector(pos), blender.to_quaternion(), Vector((1.0, 1.0, 1.0)))


def _child_matrix(pos, rotation, scale):
    # Mesh vertices stay in the file's coordinates. S * R maps the geom frame
    # into the parent empty, whose axes are already the converted body frame.
    return Matrix.LocRotScale(S @ Vector(pos), (S @ rotation).to_quaternion(), Vector(scale))


def _material(name, rgba):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = next(node for node in mat.node_tree.nodes if node.type == "BSDF_PRINCIPLED")
    color = (rgba[0], rgba[1], rgba[2], 1.0)
    bsdf.inputs["Base Color"].default_value = color
    if "Alpha" in bsdf.inputs:
        bsdf.inputs["Alpha"].default_value = rgba[3]
    if rgba[3] < 0.999:
        if hasattr(mat, "blend_method"):
            mat.blend_method = "BLEND"
        elif hasattr(mat, "surface_render_method"):
            mat.surface_render_method = "BLENDED"
    return mat


def _water_material():
    mat = _material("hydrus_water", (0.15, 0.40, 0.65, 0.25))
    bsdf = next(node for node in mat.node_tree.nodes if node.type == "BSDF_PRINCIPLED")
    if "Transmission Weight" in bsdf.inputs:
        bsdf.inputs["Transmission Weight"].default_value = 1.0
    if "Roughness" in bsdf.inputs:
        bsdf.inputs["Roughness"].default_value = 0.05
    return mat


def _remove_tree(name):
    root = bpy.data.objects.get(name)
    if root is None:
        return
    pending = [root]
    ordered = []
    while pending:
        obj = pending.pop()
        pending.extend(list(obj.children))
        ordered.append(obj)
    for obj in reversed(ordered):
        data = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        if data is not None and data.users == 0:
            if isinstance(data, bpy.types.Mesh):
                bpy.data.meshes.remove(data)
            elif isinstance(data, bpy.types.Camera):
                bpy.data.cameras.remove(data)


def _parent(obj, parent, local):
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.matrix_basis = local


def _import_mesh(path):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=str(path), up_axis="Z", forward_axis="Y")
    imported = [obj for obj in bpy.data.objects if obj not in before and obj.type == "MESH"]
    if not imported:
        raise RuntimeError(f"OBJ import produced no mesh: {path}")
    if len(imported) > 1:
        bpy.ops.object.select_all(action="DESELECT")
        for obj in imported:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = imported[0]
        bpy.ops.object.join()
        imported = [bpy.context.view_layer.objects.active]
    return imported[0]


def _parse_model(xml_path):
    import xml.etree.ElementTree as ET

    tree = ET.parse(xml_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    degrees = True
    meshdir = Path("meshes")
    if compiler is not None:
        degrees = compiler.get("angle", "degree") != "radian"
        meshdir = Path(compiler.get("meshdir", "meshes"))
    if not meshdir.is_absolute():
        meshdir = (xml_path.parent / meshdir).resolve()

    assets = {}
    asset_root = root.find("asset")
    if asset_root is not None:
        for mesh in asset_root.findall("mesh"):
            assets[mesh.get("name")] = {
                "file": meshdir / mesh.get("file"),
                "scale": _floats(mesh.get("scale"), 3, (1.0, 1.0, 1.0)),
            }

    camera_spec = (800.0, 600.0, 60.0)
    custom = root.find("custom")
    if custom is not None:
        for numeric in custom.findall("numeric"):
            if numeric.get("name") == "camera_spec":
                camera_spec = tuple(_floats(numeric.get("data"), 3, camera_spec))

    bodies = {}
    planes = {}
    world = root.find("worldbody")
    for child in list(world):
        if child.tag == "body":
            bodies[child.get("name")] = child
        elif child.tag == "geom" and child.get("type") == "plane":
            planes[child.get("name")] = child
    return {
        "degrees": degrees,
        "assets": assets,
        "bodies": bodies,
        "planes": planes,
        "camera_spec": camera_spec,
    }


def _body_transform(body, degrees):
    pos = _floats(body.get("pos"), 3, (0.0, 0.0, 0.0))
    euler = _floats(body.get("euler"), 3, (0.0, 0.0, 0.0))
    return pos, _euler_matrix(euler, degrees)


def _geom_transform(geom, degrees):
    pos = _floats(geom.get("pos"), 3, (0.0, 0.0, 0.0))
    euler = _floats(geom.get("euler"), 3, (0.0, 0.0, 0.0))
    return pos, _euler_matrix(euler, degrees)


def _assign_material(obj, mat):
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)


def _rgba(geom):
    return _floats(geom.get("rgba"), 4, (0.8, 0.8, 0.8, 1.0))


def build_scene():
    xml_path = _discover_xml()
    if xml_path is None:
        raise FileNotFoundError(
            "hydrus.xml was not found. Set the Hydrus directory in the add-on preferences "
            "to auvs/hydrus_mujoco."
        )
    model = _parse_model(xml_path)
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for name in ROOTS:
        _remove_tree(name)

    collection = bpy.context.collection
    materials = {}

    def shared_material(rgba):
        key = tuple(round(channel, 5) for channel in rgba)
        if key not in materials:
            materials[key] = _material(f"hydrus_{len(materials)}", rgba)
        return materials[key]

    mesh_objects = {}

    def mesh_object(asset_name, object_name):
        asset = model["assets"].get(asset_name)
        if asset is None:
            raise FileNotFoundError(f"hydrus.xml has no mesh '{asset_name}'")
        path = asset["file"]
        if not path.is_file():
            raise FileNotFoundError(path)
        cached = mesh_objects.get(str(path))
        if cached is None:
            imported = _import_mesh(path)
            mesh_objects[str(path)] = imported
            imported.name = object_name
            return imported, asset["scale"]
        obj = bpy.data.objects.new(object_name, cached.data)
        collection.objects.link(obj)
        return obj, asset["scale"]

    def place_body(body_name):
        body = model["bodies"].get(body_name)
        if body is None:
            raise FileNotFoundError(f"hydrus.xml has no body '{body_name}'")
        pos, rotation = _body_transform(body, model["degrees"])
        empty = bpy.data.objects.new(body_name, None)
        empty.empty_display_type = "ARROWS"
        empty.empty_display_size = 0.25
        collection.objects.link(empty)
        empty.matrix_world = _world_matrix(pos, rotation)
        for geom in body.findall("geom"):
            gpos, grot = _geom_transform(geom, model["degrees"])
            geom_name = geom.get("name") or geom.get("type")
            child_name = geom_name if geom_name != body_name else f"{geom_name}_mesh"
            kind = geom.get("type", "sphere")
            scale = (1.0, 1.0, 1.0)
            if kind == "mesh":
                obj, scale = mesh_object(geom.get("mesh"), child_name)
            elif kind == "cylinder":
                radius, half = _floats(geom.get("size"), 2, (0.1, 0.1))
                bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=2.0 * half, vertices=48)
                obj = bpy.context.active_object
                obj.name = child_name
            elif kind == "box":
                half = _floats(geom.get("size"), 3, (0.1, 0.1, 0.1))
                bpy.ops.mesh.primitive_cube_add(size=1.0)
                obj = bpy.context.active_object
                obj.name = child_name
                scale = (2.0 * half[0], 2.0 * half[1], 2.0 * half[2])
            else:
                continue
            _assign_material(obj, shared_material(_rgba(geom)))
            _parent(obj, empty, _child_matrix(gpos, grot, scale))
        return empty, pos, rotation

    place_body("hydrus")
    place_body("gate")
    place_body("marker")

    hydrus = model["bodies"]["hydrus"]
    site = None
    for candidate in hydrus.findall("site"):
        if candidate.get("name") == "camera":
            site = candidate
            break
    if site is not None:
        body_pos, body_rot = _body_transform(hydrus, model["degrees"])
        site_pos = _floats(site.get("pos"), 3, (0.0, 0.0, 0.0))
        site_euler = _floats(site.get("euler"), 3, (0.0, 0.0, 0.0))
        site_rot = _euler_matrix(site_euler, model["degrees"])
        world_pos = Vector(body_pos) + body_rot @ Vector(site_pos)
        world_rot = body_rot @ site_rot
        empty = bpy.data.objects.new("camera", None)
        empty.empty_display_type = "ARROWS"
        empty.empty_display_size = 0.15
        collection.objects.link(empty)
        empty.matrix_world = _world_matrix(world_pos, world_rot)
        width, height, fov = model["camera_spec"]
        camera = bpy.data.cameras.new("hydrus_camera")
        camera.sensor_fit = "HORIZONTAL"
        camera.angle = math.radians(fov)
        camera.clip_start = 0.1
        camera.clip_end = 50.0
        if height:
            camera.sensor_height = camera.sensor_width * (height / width)
        obj = bpy.data.objects.new("hydrus_camera", camera)
        collection.objects.link(obj)
        # Blender cameras look down -Z with +Y up. The converted site already
        # looks down -Z; this roll puts the site's up on the camera's +Y.
        _parent(obj, empty, Matrix.Rotation(-math.pi / 2.0, 4, "Z"))

    surface = model["planes"].get("surface")
    floor = model["planes"].get("floor")
    if surface is not None and floor is not None:
        spos = _floats(surface.get("pos"), 3, (0.0, 0.0, 0.0))
        fpos = _floats(floor.get("pos"), 3, (0.0, 0.0, 3.0))
        half = _floats(surface.get("size"), 2, (15.0, 20.0))
        depth = abs(fpos[2] - spos[2])
        center = ((spos[0] + fpos[0]) * 0.5, (spos[1] + fpos[1]) * 0.5, (spos[2] + fpos[2]) * 0.5)
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        water = bpy.context.active_object
        water.name = "water"
        _assign_material(water, _water_material())
        water.matrix_world = Matrix.LocRotScale(
            S @ Vector(center),
            Quaternion((1.0, 0.0, 0.0, 0.0)),
            Vector((2.0 * half[1], 2.0 * half[0], depth)),
        )
        bpy.ops.mesh.primitive_plane_add(size=1.0)
        floor_obj = bpy.context.active_object
        floor_obj.name = "floor"
        _assign_material(floor_obj, shared_material(_rgba(floor)))
        floor_obj.matrix_world = Matrix.LocRotScale(
            S @ Vector(fpos),
            Quaternion((1.0, 0.0, 0.0, 0.0)),
            Vector((2.0 * half[1], 2.0 * half[0], 1.0)),
        )

    rotor_animation.rig_scene(bpy.context.scene)
    bpy.context.view_layer.update()
    return xml_path


def _object_for(name):
    obj = bpy.data.objects.get(name)
    if obj is not None:
        return obj
    lowered = name.casefold()
    matches = [candidate for candidate in bpy.data.objects if candidate.name.casefold() == lowered]
    if len(matches) == 1:
        return matches[0]
    return None


def _ensure_driven_object(name):
    """The object the pose stream or a recording moves. Created when the scene has no match."""
    found = _object_for(name)
    if found is not None:
        return found
    collection = bpy.context.collection or bpy.context.scene.collection
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = "ARROWS"
    empty.empty_display_size = 0.15 if name == "camera" else 0.4
    collection.objects.link(empty)
    if name == "camera" and bpy.data.objects.get("hydrus_camera") is None:
        camera = bpy.data.cameras.new("hydrus_camera")
        camera.sensor_fit = "HORIZONTAL"
        camera.angle = math.radians(60.0)
        camera.clip_start = 0.1
        camera.clip_end = 50.0
        obj = bpy.data.objects.new("hydrus_camera", camera)
        collection.objects.link(obj)
        _parent(obj, empty, Matrix.Identity(4))
    return empty


def ensure_link_roots():
    """The camera empty is shared by every vehicle. Other bodies arrive by name."""
    _ensure_driven_object("camera")


def _apply_packet(payload):
    if len(payload) < HEADER.size:
        return
    magic, version, count, _sim_time = HEADER.unpack_from(payload, 0)
    if magic != MAGIC or version != VERSION:
        return
    need = HEADER.size + count * POSE.size
    if len(payload) < need:
        return
    moved = []
    for index in range(count):
        name, px, py, pz, qw, qx, qy, qz = POSE.unpack_from(payload, HEADER.size + index * POSE.size)
        object_name = name.split(b"\0", 1)[0].decode("utf-8", "replace")
        if not object_name:
            continue
        obj = _ensure_driven_object(object_name)
        rotation = Quaternion((qw, qx, qy, qz)).to_matrix().to_4x4()
        rotation.translation = (px, py, pz)
        obj.matrix_world = rotation
        moved.append(obj.name)
    return moved, _sim_time


def _drain():
    sock = _link["socket"]
    if sock is None:
        return
    latest = None
    while True:
        try:
            payload, _addr = sock.recvfrom(4096)
        except BlockingIOError:
            break
        latest = payload
    if latest is not None:
        applied = _apply_packet(latest)
        bpy.context.view_layer.update()
        if applied is not None:
            rotor_animation.observe_live(bpy.context.scene, *applied)
    rotor_animation.tick_live(bpy.context.scene)


# ---------------------------------------------------------------------------
# Takes: record and keyframe.

TAKE_DIR = "hydrus_takes"
TAKE_COLLECTION = "hydrus_takes"
MARKER_PREFIX = "hydrus: "
# Keyframe.interpolation enum value for LINEAR.
LINEAR = 1

_job = {"proc": None, "log": None, "scene": "", "take": "", "time": 0.0, "message": ""}
_recordings = {}


def _prefs():
    addon = bpy.context.preferences.addons.get(__name__)
    return addon.preferences if addon is not None else None


def _repo_dir():
    prefs = _prefs()
    if prefs is not None and prefs.repo_dir.strip():
        return Path(bpy.path.abspath(prefs.repo_dir.strip())).resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "build.zig").is_file():
            return parent
    return None


def _repo_path(repo, value):
    path = Path(bpy.path.abspath(value))
    return path if path.is_absolute() else (repo / path).resolve()


def _tool_paths():
    """Executable, plugin, config, and MJCF, or raise with what is missing."""
    repo = _repo_dir()
    if repo is None:
        raise FileNotFoundError("Set the repository folder in the Hydrus add-on preferences")
    prefs = _prefs()
    values = {
        "executable": prefs.executable if prefs else "zig-out/bin/src_3",
        "plugin": prefs.plugin if prefs else "zig-out/lib/libauv_mujoco.so",
        "config": prefs.config if prefs else "auvs/hydrus_mujoco/auv.json",
        "mjcf": prefs.mjcf if prefs else "auvs/hydrus_mujoco/hydrus.xml",
    }
    paths = {key: _repo_path(repo, value) for key, value in values.items()}
    for key, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f"{key} not found: {path}. Build with zig build -Dmujoco")
    paths["repo"] = repo
    return paths


def _takes_dir():
    if bpy.data.filepath:
        folder = Path(bpy.path.abspath("//" + TAKE_DIR))
    else:
        repo = _repo_dir()
        folder = (repo if repo is not None else Path(bpy.app.tempdir)) / TAKE_DIR
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def _scene_fps(scene):
    return scene.render.fps / scene.render.fps_base


def _read_recording(path):
    """Header, samples sorted by step, events, and the end line (None while running)."""
    path = Path(path)
    stat = path.stat()
    cached = _recordings.get(str(path))
    if cached is not None and cached[0] == (stat.st_mtime_ns, stat.st_size):
        return cached[1]
    header = None
    samples = []
    events = []
    end = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            kind = record.get("t")
            if kind == "sample":
                samples.append(record)
            elif kind == "event":
                events.append(record)
            elif kind == "header":
                header = record
            elif kind == "end":
                end = record
    if header is None:
        raise ValueError(f"{path} has no header")
    samples.sort(key=lambda sample: sample["step"])
    recording = {"header": header, "samples": samples, "events": events, "end": end}
    _recordings[str(path)] = ((stat.st_mtime_ns, stat.st_size), recording)
    return recording


def _take_frame(scene, take, time):
    return take.frame_offset + time * _scene_fps(scene)


def _action_name(take_name, object_name):
    return f"{take_name}:{object_name}"


def _fcurve(action, obj, data_path, index):
    if hasattr(action, "fcurve_ensure_for_datablock"):
        return action.fcurve_ensure_for_datablock(obj, data_path, index=index)
    return action.fcurves.new(data_path, index=index)


def _write_curve(curve, frames, values):
    count = len(frames)
    points = curve.keyframe_points
    points.clear()
    points.add(count)
    co = [0.0] * (2 * count)
    co[0::2] = frames
    co[1::2] = values
    points.foreach_set("co", co)
    points.foreach_set("interpolation", [LINEAR] * count)
    curve.update()


def _assign_action(obj, action):
    obj.animation_data_create()
    animation = obj.animation_data
    animation.action = action
    if hasattr(animation, "action_slot") and animation.action_slot is None and len(action.slots) > 0:
        animation.action_slot = action.slots[0]


def _key_object(obj, action_name, frames, poses):
    action = bpy.data.actions.get(action_name)
    if action is not None:
        bpy.data.actions.remove(action)
    action = bpy.data.actions.new(action_name)
    # Inactive takes keep their keyframes when the file is saved.
    action.use_fake_user = True
    _assign_action(obj, action)
    obj.rotation_mode = "QUATERNION"
    parent_inverse = obj.parent.matrix_world.inverted() if obj.parent is not None else None
    locations = ([], [], [])
    rotations = ([], [], [], [])
    previous = None
    for pose in poses:
        loc = Vector(pose[:3])
        quat = Quaternion(pose[3:7])
        if parent_inverse is not None:
            loc, quat, _scale = (parent_inverse @ Matrix.LocRotScale(loc, quat, Vector((1.0, 1.0, 1.0)))).decompose()
        # Same rotation, shortest interpolation between neighbouring keys.
        if previous is not None and previous.dot(quat) < 0:
            quat.negate()
        previous = quat
        for index in range(3):
            locations[index].append(loc[index])
        for index in range(4):
            rotations[index].append(quat[index])
    for index in range(3):
        _write_curve(_fcurve(action, obj, "location", index), frames, locations[index])
    for index in range(4):
        _write_curve(_fcurve(action, obj, "rotation_quaternion", index), frames, rotations[index])
    return action


def _take_collection(scene):
    collection = bpy.data.collections.get(TAKE_COLLECTION)
    if collection is None:
        collection = bpy.data.collections.new(TAKE_COLLECTION)
    # Respect an existing nested collection instead of also linking it at the root.
    pending = [scene.collection]
    visited = set()
    while pending:
        parent = pending.pop()
        if parent in visited:
            continue
        visited.add(parent)
        if collection.name in parent.children:
            return collection
        pending.extend(parent.children)
    parent = bpy.data.collections.get("07 | Animation")
    if parent not in visited:
        parent = scene.collection
    parent.children.link(collection)
    return collection


def _path_material(index):
    name = f"hydrus_path_{index % 8}"
    mat = bpy.data.materials.get(name)
    if mat is not None:
        return mat
    hue = (index * 0.618034) % 1.0
    rgb = _hsv(hue, 0.8, 1.0)
    mat = _material(name, (rgb[0], rgb[1], rgb[2], 1.0))
    mat.diffuse_color = (rgb[0], rgb[1], rgb[2], 1.0)
    return mat


def _hsv(h, s, v):
    i = int(h * 6.0) % 6
    f = h * 6.0 - int(h * 6.0)
    p, q, t = v * (1 - s), v * (1 - f * s), v * (1 - (1 - f) * s)
    return ((v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q))[i]


def _path_object(scene, take, index, positions):
    name = f"path_{take.name}"
    old = bpy.data.objects.get(name)
    if old is not None:
        data = old.data
        bpy.data.objects.remove(old, do_unlink=True)
        if data is not None and data.users == 0:
            bpy.data.curves.remove(data)
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = 0.01
    spline = curve.splines.new("POLY")
    spline.points.add(max(0, len(positions) - 1))
    flat = []
    for position in positions:
        flat.extend((position[0], position[1], position[2], 1.0))
    spline.points.foreach_set("co", flat)
    curve.materials.append(_path_material(index))
    obj = bpy.data.objects.new(name, curve)
    obj.hide_render = True
    _take_collection(scene).objects.link(obj)
    return obj


def _import_take(scene, take, index):
    """Bake the recording into one action per object, plus the vehicle path."""
    recording = _read_recording(take.recording)
    header = recording["header"]
    samples = recording["samples"]
    if not samples:
        raise ValueError(f"{take.recording} has no samples")
    ensure_link_roots()
    frames = [_take_frame(scene, take, sample["time"]) for sample in samples]
    for name in header["objects"]:
        obj = _ensure_driven_object(name)
        poses = [sample["poses"][name] for sample in samples]
        _key_object(obj, _action_name(take.name, name), frames, poses)
    vehicle = header.get("vehicle", "hydrus")
    root = _object_for(vehicle)
    if root is not None:
        rotor_animation.bake_take(root, take.name, frames, samples, _scene_fps(scene))
    _path_object(scene, take, index, [sample["poses"][vehicle][:3] for sample in samples])
    take.end_frame = int(math.ceil(frames[-1]))
    end = recording["end"]
    take.reason = end["reason"] if end is not None else "incomplete"


def _take_by_name(settings, name):
    for take in settings.takes:
        if take.name == name:
            return take
    return None


def _set_markers(scene, take, recording):
    for marker in list(scene.timeline_markers):
        if marker.name.startswith(MARKER_PREFIX):
            scene.timeline_markers.remove(marker)
    for event in recording["events"]:
        frame = int(round(_take_frame(scene, take, event["time"])))
        scene.timeline_markers.new(MARKER_PREFIX + event["text"], frame=frame)
    if take.branch_frame >= 0:
        scene.timeline_markers.new(MARKER_PREFIX + "branch", frame=take.branch_frame)
    end = recording["end"]
    if end is not None:
        frame = int(round(_take_frame(scene, take, end["time"])))
        scene.timeline_markers.new(MARKER_PREFIX + "end " + end["reason"], frame=frame)


def _show_paths(settings, active):
    visible = {active.name}
    if settings.show_parent_path and active.parent:
        visible.add(active.parent)
    for take in settings.takes:
        obj = bpy.data.objects.get(f"path_{take.name}")
        if obj is not None:
            obj.hide_viewport = take.name not in visible


def activate_take(scene, take):
    """Assign the take's actions, frame range, markers, and paths."""
    _link["running"] = False
    settings = scene.hydrus_link
    recording = _read_recording(take.recording)
    index = list(settings.takes).index(take)
    missing = False
    for name in recording["header"]["objects"]:
        obj = _object_for(name)
        if obj is None:
            continue
        action = bpy.data.actions.get(_action_name(take.name, name))
        if action is None:
            missing = True
            break
        _assign_action(obj, action)
    if missing or bpy.data.objects.get(f"path_{take.name}") is None:
        _import_take(scene, take, index)
    vehicle = _object_for(recording["header"].get("vehicle", "hydrus"))
    if vehicle is not None and not rotor_animation.activate_take(scene, vehicle, take.name):
        frames = [_take_frame(scene, take, sample["time"]) for sample in recording["samples"]]
        rotor_animation.bake_take(vehicle, take.name, frames, recording["samples"], _scene_fps(scene))
    scene.frame_start = take.frame_offset
    scene.frame_end = max(take.frame_offset + 1, take.end_frame)
    _set_markers(scene, take, recording)
    _show_paths(settings, take)
    if 0 <= take.branch_frame <= scene.frame_end:
        scene.frame_set(take.branch_frame)
    else:
        scene.frame_set(scene.frame_start)


def detach_takes(scene):
    """Drop take actions so the live link or manual poses are not overridden."""
    for name in (*rotor_animation.VEHICLES, "gate", "marker", "camera"):
        obj = _object_for(name)
        if obj is not None and obj.animation_data is not None:
            action = obj.animation_data.action
            if action is not None and ":" in action.name:
                obj.animation_data.action = None
    rotor_animation.detach_takes(scene)
    for marker in list(scene.timeline_markers):
        if marker.name.startswith(MARKER_PREFIX):
            scene.timeline_markers.remove(marker)
    for take in scene.hydrus_link.takes:
        obj = bpy.data.objects.get(f"path_{take.name}")
        if obj is not None:
            obj.hide_viewport = True


def _check_divergence(scene, take, parent):
    """First frame at or before the branch where the child's state differs from the parent's."""
    child = _read_recording(take.recording)
    base = _read_recording(parent.recording)
    parent_hash = {sample["step"]: sample["hash"] for sample in base["samples"]}
    for sample in child["samples"]:
        if sample["step"] > take.branch_step:
            break
        expected = parent_hash.get(sample["step"])
        if expected is not None and expected != sample["hash"]:
            return int(round(_take_frame(scene, take, sample["time"])))
    return None


def _next_take_name(settings):
    used = set()
    for take in settings.takes:
        match = re.fullmatch(r"take_(\d+)", take.name)
        if match:
            used.add(int(match.group(1)))
    folder = _takes_dir()
    for pattern in ("take_*.rec.jsonl", "take_*.run.json"):
        for path in folder.glob(pattern):
            match = re.fullmatch(r"take_(\d+)\.(?:rec\.jsonl|run\.json)", path.name)
            if match:
                used.add(int(match.group(1)))
    return f"take_{(max(used) + 1) if used else 0:03d}"


def _redraw():
    wm = bpy.context.window_manager
    if wm is None:
        return
    for window in wm.windows:
        for area in window.screen.areas:
            area.tag_redraw()


def launch_take(scene, seed):
    """Start src_3 record for this seed and add the take as running."""
    if _job["proc"] is not None:
        raise RuntimeError("A simulation is already running")
    tools = _tool_paths()
    settings = scene.hydrus_link
    name = _next_take_name(settings)
    folder = _takes_dir()
    recording_path = folder / f"{name}.rec.jsonl"
    log_path = folder / f"{name}.log"

    take = settings.takes.add()
    take.name = name
    take.seed = int(seed)
    take.recording = str(recording_path)
    take.frame_offset = scene.frame_start
    take.status = "running"

    log = log_path.open("w", encoding="utf-8")
    env = dict(os.environ, AUV_MJCF=str(tools["mjcf"]), HYDRUS_MUJOCO_HEADLESS="1")
    command = [
        str(tools["executable"]),
        str(tools["plugin"]),
        "record",
        str(tools["config"]),
        str(int(seed)),
        str(recording_path),
    ]
    try:
        proc = subprocess.Popen(command, cwd=str(tools["repo"]), env=env, stdout=log, stderr=subprocess.STDOUT)
    except OSError:
        log.close()
        take.status = "failed to start"
        raise
    _job.update(
        proc=proc,
        log=log,
        scene=scene.name,
        take=name,
        time=0.0,
        message=f"Simulating {name}",
    )
    if not bpy.app.background:
        bpy.app.timers.register(_poll_job, first_interval=0.5)
    return take


def _progress_time(path):
    try:
        with open(path, "rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - 8192))
            tail = handle.read().decode("utf-8", "replace")
    except OSError:
        return 0.0
    times = re.findall(r'"time":([0-9.eE+-]+)', tail)
    return float(times[-1]) if times else 0.0


def _poll_job():
    proc = _job["proc"]
    if proc is None:
        return None
    scene = bpy.data.scenes.get(_job["scene"])
    take = _take_by_name(scene.hydrus_link, _job["take"]) if scene is not None else None
    if proc.poll() is None:
        if take is not None:
            _job["time"] = _progress_time(take.recording)
        _redraw()
        return 0.5
    finish_job()
    _redraw()
    return None


def finish_job():
    """Import the finished take, activate it, and check that it matches its parent before the branch."""
    proc = _job["proc"]
    returncode = proc.wait()
    _job["log"].close()
    _job.update(proc=None, log=None)
    scene = bpy.data.scenes.get(_job["scene"])
    if scene is None:
        return None
    settings = scene.hydrus_link
    take = _take_by_name(settings, _job["take"])
    if take is None:
        return None
    log_path = Path(take.recording).with_suffix("").with_suffix(".log")
    try:
        recording = _read_recording(take.recording)
    except (OSError, ValueError):
        recording = None
    if returncode != 0 or recording is None or recording["end"] is None:
        take.status = "failed"
        _job["message"] = f"{take.name} failed (exit {returncode}). See {log_path}"
        return take
    index = list(settings.takes).index(take)
    try:
        _import_take(scene, take, index)
        take.status = "ok"
        parent = _take_by_name(settings, take.parent) if take.parent else None
        if parent is not None:
            diverged = _check_divergence(scene, take, parent)
            if diverged is not None:
                take.status = f"diverged at frame {diverged}"
    except (OSError, ValueError, KeyError) as exc:
        take.status = "failed"
        _job["message"] = f"{take.name} import failed: {exc}"
        return take
    settings.take_index = index
    _job["message"] = f"{take.name}: {take.reason}, {take.status}"
    return take


def _on_take_index(self, context):
    if 0 <= self.take_index < len(self.takes):
        take = self.takes[self.take_index]
        if take.status != "running" and Path(take.recording).is_file():
            activate_take(context.scene, take)


def _on_onboard_camera(self, context):
    scene = context.scene
    camera = bpy.data.objects.get("hydrus_camera")
    if self.onboard_camera:
        if camera is None:
            return
        if scene.camera is not None and scene.camera != camera:
            self.previous_camera = scene.camera.name
        scene.camera = camera
    else:
        previous = bpy.data.objects.get(self.previous_camera)
        if previous is not None and previous.type == "CAMERA":
            scene.camera = previous


def _on_show_parent_path(self, context):
    if 0 <= self.take_index < len(self.takes):
        _show_paths(self, self.takes[self.take_index])


class HydrusPreferences(AddonPreferences):
    bl_idname = __name__

    model_dir: StringProperty(
        name="Hydrus directory",
        description="Folder that contains hydrus.xml. Leave empty when this add-on still sits in the repository",
        subtype="DIR_PATH",
        default="",
    )

    repo_dir: StringProperty(
        name="Repository",
        description="Autonomy checkout. Leave empty when this add-on still sits in the repository",
        subtype="DIR_PATH",
        default="",
    )
    executable: StringProperty(name="src_3", description="Relative to the repository", default="zig-out/bin/src_3")
    plugin: StringProperty(
        name="Plugin",
        description="Relative to the repository",
        default="zig-out/lib/libauv_mujoco.so",
    )
    mjcf: StringProperty(name="MJCF", description="AUV_MJCF, relative to the repository", default="auvs/hydrus_mujoco/hydrus.xml")
    config: StringProperty(name="Config", description="Relative to the repository", default="auvs/hydrus_mujoco/auv.json")

    def draw(self, _context):
        layout = self.layout
        layout.prop(self, "model_dir")
        layout.prop(self, "repo_dir")
        layout.prop(self, "executable")
        layout.prop(self, "plugin")
        layout.prop(self, "mjcf")
        layout.prop(self, "config")


class HydrusTake(PropertyGroup):
    # name is the take id, such as take_003.
    parent: StringProperty(name="Parent")
    seed: IntProperty(name="Seed", default=0, min=0)
    recording: StringProperty(name="Recording", subtype="FILE_PATH")
    frame_offset: IntProperty(name="First frame", default=1)
    end_frame: IntProperty(name="Last frame", default=1)
    branch_frame: IntProperty(name="Branch frame", default=-1)
    branch_step: IntProperty(name="Branch step", default=-1)
    status: StringProperty(name="Status")
    reason: StringProperty(name="End reason")


class HydrusLinkSettings(PropertyGroup):
    port: IntProperty(name="Port", default=9760, min=1, max=65535)

    takes: CollectionProperty(type=HydrusTake)
    take_index: IntProperty(name="Take", default=-1, update=_on_take_index)
    seed: IntProperty(
        name="Seed",
        description="The run. The plugin draws the start pose, object placement, and faults from this integer",
        default=1,
        min=0,
    )
    onboard_camera: BoolProperty(
        name="Onboard camera",
        description="Render through hydrus_camera",
        default=False,
        update=_on_onboard_camera,
    )
    previous_camera: StringProperty(default="")
    show_parent_path: BoolProperty(
        name="Show parent path",
        description="Show the parent take's path next to the active one",
        default=True,
        update=_on_show_parent_path,
    )



class HYDRUS_UL_takes(UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_propname, _index):
        row = layout.row(align=True)
        row.label(text=item.name, icon="SEQUENCE")
        if item.parent:
            row.label(text=f"from {item.parent} @ {item.branch_frame}")
        else:
            row.label(text=f"seed {item.seed}")
        row.label(text=item.status if item.status != "ok" else item.reason)


class HYDRUS_OT_record_take(Operator):
    bl_idname = "hydrus.record_take"
    bl_label = "Record run"
    bl_description = "Run the mission headless from the start and import it as a new take"

    def execute(self, context):
        scene = context.scene
        settings = scene.hydrus_link
        try:
            take = launch_take(scene, settings.seed)
        except (OSError, RuntimeError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Recording {take.name}")
        return {"FINISHED"}


class HYDRUS_OT_cancel_take(Operator):
    bl_idname = "hydrus.cancel_take"
    bl_label = "Cancel"
    bl_description = "Stop the running simulation"

    def execute(self, _context):
        proc = _job["proc"]
        if proc is not None:
            proc.terminate()
        return {"FINISHED"}


class HYDRUS_OT_detach_take(Operator):
    bl_idname = "hydrus.detach_take"
    bl_label = "Detach"
    bl_description = "Remove take keyframes from the scene objects. Takes stay in the list"

    def execute(self, context):
        detach_takes(context.scene)
        context.scene.hydrus_link.take_index = -1
        return {"FINISHED"}


class HYDRUS_OT_remove_take(Operator):
    bl_idname = "hydrus.remove_take"
    bl_label = "Remove take"
    bl_description = "Remove the active take from the list and delete its actions and path. Files stay on disk"

    @classmethod
    def poll(cls, context):
        settings = context.scene.hydrus_link
        return 0 <= settings.take_index < len(settings.takes) and settings.takes[settings.take_index].status != "running"

    def execute(self, context):
        scene = context.scene
        settings = scene.hydrus_link
        index = settings.take_index
        take = settings.takes[index]
        detach_takes(scene)
        for action in list(bpy.data.actions):
            if action.name.startswith(take.name + ":"):
                bpy.data.actions.remove(action)
        path = bpy.data.objects.get(f"path_{take.name}")
        if path is not None:
            data = path.data
            bpy.data.objects.remove(path, do_unlink=True)
            if data is not None and data.users == 0:
                bpy.data.curves.remove(data)
        settings.takes.remove(index)
        settings.take_index = min(index, len(settings.takes) - 1)
        return {"FINISHED"}


class HYDRUS_OT_import_take(Operator):
    bl_idname = "hydrus.import_take"
    bl_label = "Import recording"
    bl_description = "Add a recording made by src_3 record as a take"

    filepath: StringProperty(subtype="FILE_PATH")
    filter_glob: StringProperty(default="*.rec.jsonl", options={"HIDDEN"})

    def invoke(self, context, _event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        scene = context.scene
        settings = scene.hydrus_link
        recording = Path(self.filepath)
        if not recording.is_file():
            self.report({"ERROR"}, f"Need {recording.name}")
            return {"CANCELLED"}
        name = recording.name.replace(".rec.jsonl", "")
        if _take_by_name(settings, name) is not None:
            self.report({"ERROR"}, f"{name} is already in the take list")
            return {"CANCELLED"}
        try:
            header = _read_recording(recording)["header"]
        except (OSError, ValueError, KeyError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        take = settings.takes.add()
        take.name = name
        take.seed = int(header.get("seed", 0))
        take.recording = str(recording)
        take.frame_offset = scene.frame_start
        try:
            index = len(settings.takes) - 1
            _import_take(scene, take, index)
        except (OSError, ValueError, KeyError) as exc:
            settings.takes.remove(len(settings.takes) - 1)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        take.status = "ok"
        settings.take_index = index
        return {"FINISHED"}


class HYDRUS_OT_build_scene(Operator):
    bl_idname = "hydrus.build_scene"
    bl_label = "Build Hydrus scene"
    bl_description = "Create the Hydrus starter layout from hydrus.xml"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        try:
            xml_path = build_scene()
        except (OSError, RuntimeError, ValueError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Built Hydrus scene from {xml_path.name}")
        for window in context.window_manager.windows:
            screen = window.screen
            for area in screen.areas:
                if area.type != "VIEW_3D":
                    continue
                region = next((item for item in area.regions if item.type == "WINDOW"), None)
                if region is None:
                    continue
                try:
                    with context.temp_override(window=window, area=area, region=region):
                        bpy.ops.view3d.view_all(center=True)
                except RuntimeError:
                    pass
                return {"FINISHED"}
        return {"FINISHED"}


class HYDRUS_OT_start_link(Operator):
    bl_idname = "hydrus.start_link"
    bl_label = "Start link"
    bl_description = "Listen for MuJoCo poses and move matching objects"

    def modal(self, context, event):
        if not _link["running"]:
            self.cancel(context)
            return {"CANCELLED"}
        if event.type == "TIMER":
            _drain()
        return {"PASS_THROUGH"}

    def execute(self, context):
        if _link["running"]:
            self.report({"WARNING"}, "Hydrus link is already running")
            return {"CANCELLED"}
        # Take keyframes would override the streamed poses on every frame change.
        detach_takes(context.scene)
        context.scene.hydrus_link.take_index = -1
        ensure_link_roots()
        port = context.scene.hydrus_link.port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            sock.close()
            self.report({"ERROR"}, f"Cannot bind UDP port {port}: {exc}")
            return {"CANCELLED"}
        sock.setblocking(False)
        _link["socket"] = sock
        _link["running"] = True
        self._timer = context.window_manager.event_timer_add(1.0 / 30.0, window=context.window)
        context.window_manager.modal_handler_add(self)
        self.report({"INFO"}, f"Hydrus link listening on 127.0.0.1:{port}")
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        _link["running"] = False
        sock = _link["socket"]
        _link["socket"] = None
        if sock is not None:
            sock.close()
        timer = getattr(self, "_timer", None)
        if timer is not None:
            context.window_manager.event_timer_remove(timer)
            self._timer = None


class HYDRUS_OT_stop_link(Operator):
    bl_idname = "hydrus.stop_link"
    bl_label = "Stop link"
    bl_description = "Stop listening for MuJoCo poses"

    def execute(self, _context):
        _link["running"] = False
        return {"FINISHED"}


class HYDRUS_PT_link(Panel):
    bl_label = "Hydrus"
    bl_idname = "HYDRUS_PT_link"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Hydrus"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.hydrus_link
        layout.prop(settings, "port")
        layout.operator("hydrus.build_scene", icon="MESH_CUBE")
        row = layout.row(align=True)
        row.operator("hydrus.start_link", icon="PLAY")
        row.operator("hydrus.stop_link", icon="PAUSE")
        layout.label(text="Listening" if _link["running"] else "Idle")


class HYDRUS_PT_takes(Panel):
    bl_label = "Takes"
    bl_idname = "HYDRUS_PT_takes"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Hydrus"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.hydrus_link
        layout.prop(settings, "seed")
        row = layout.row(align=True)
        row.operator("hydrus.record_take", icon="REC")
        row.operator("hydrus.import_take", icon="IMPORT", text="")
        layout.template_list("HYDRUS_UL_takes", "", settings, "takes", settings, "take_index", rows=4)
        row = layout.row(align=True)
        row.operator("hydrus.detach_take", icon="UNLINKED")
        row.operator("hydrus.remove_take", icon="X", text="")
        layout.prop(settings, "show_parent_path")
        layout.prop(settings, "onboard_camera", icon="CAMERA_DATA")
        if _job["proc"] is not None:
            box = layout.box()
            box.label(text=f"{_job['take']}: {_job['time']:.1f} s")
            box.operator("hydrus.cancel_take", icon="CANCEL")
        elif _job["message"]:
            layout.label(text=_job["message"])


CLASSES = (
    HydrusPreferences,
    HydrusTake,
    HydrusLinkSettings,
    HYDRUS_UL_takes,
    HYDRUS_OT_build_scene,
    HYDRUS_OT_start_link,
    HYDRUS_OT_stop_link,
    HYDRUS_OT_record_take,
    HYDRUS_OT_cancel_take,
    HYDRUS_OT_detach_take,
    HYDRUS_OT_remove_take,
    HYDRUS_OT_import_take,
    HYDRUS_PT_link,
    HYDRUS_PT_takes,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.hydrus_link = PointerProperty(type=HydrusLinkSettings)
    rotor_animation.register()


def unregister():
    rotor_animation.unregister()
    _link["running"] = False
    proc = _job["proc"]
    if proc is not None:
        proc.terminate()
        proc.wait()
        _job["log"].close()
        _job.update(proc=None, log=None)
    if bpy.app.timers.is_registered(_poll_job):
        bpy.app.timers.unregister(_poll_job)
    sock = _link["socket"]
    _link["socket"] = None
    if sock is not None:
        sock.close()
    del bpy.types.Scene.hydrus_link
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
    path = build_scene()
    hydrus = bpy.data.objects["hydrus"]
    gate = bpy.data.objects["gate"]
    marker = bpy.data.objects["marker"]
    camera = bpy.data.objects["camera"]
    print(
        "HYDRUS_BUILD",
        path,
        tuple(round(v, 4) for v in hydrus.matrix_world.translation),
        tuple(round(v, 4) for v in gate.matrix_world.translation),
        tuple(round(v, 4) for v in marker.matrix_world.translation),
        tuple(round(v, 4) for v in camera.matrix_world.translation),
    )
    legs = bpy.data.objects["legs_left"]
    print("LEGS_LOCAL", tuple(round(v, 4) for v in legs.matrix_local.translation))
    print("LEGS_SCALE", tuple(round(v, 4) for v in legs.matrix_local.to_scale()))
