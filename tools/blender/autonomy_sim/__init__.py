"""Live simulation poses and recorded takes, ported from experimental.

The pool asset, take/action management, paths and rotor animation are retained.
Live AMJB packets and portable JSONL traces work with any simulation backend.
"""

import json
import math
import os
import re
import socket
import struct
import subprocess
import uuid
from pathlib import Path

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    IntProperty,
    EnumProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import AddonPreferences, Operator, Panel, PropertyGroup, UIList
from mathutils import Matrix, Quaternion, Vector

from . import rotor_animation, trace, exporter, preview, wire

bl_info = {
    "name": "Autonomy Simulation",
    "author": "Rumarino",
    "version": (3, 1, 0),
    "blender": (4, 5, 0),
    "location": "View3D > Sidebar > Autonomy",
    "description": "Live poses, recorded takes and propeller animation for portable simulation backends",
    "category": "Animation",
}

MAGIC = b"AMJB"
VERSION = 1
HEADER = struct.Struct("<4sHHd")
POSE = struct.Struct("<32s3d4d")
POSE_V2 = struct.Struct("<I32s3d4d")
# pool_scene.blend frame. NED (north, east, down) <-> Blender (-north, east, -down).
S = Matrix(((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)))
ROOTS = ("hydrus", "gate", "marker", "camera", "water", "floor")

_link = {"running": False, "socket": None, "packets": 0, "time": 0.0, "scene": ""}


def _parent(obj, parent, local):
    obj.parent = parent
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj.matrix_basis = local


def build_scene():
    """Append the experimental pool scene without discarding the current file."""
    repo = _repo_dir()
    if repo is None:
        raise FileNotFoundError("Set the repository folder in the add-on preferences")
    path = repo / "auvs/stonefish_sim/data/pool_scene.blend"
    if not path.is_file():
        raise FileNotFoundError(path)
    scene = next((s for s in bpy.data.scenes if s.get("autonomy_pool_source") == str(path)
                  or (bpy.data.filepath == str(path) and "hydrus" in s.objects)), None)
    if scene is None:
        with bpy.data.libraries.load(str(path), link=False) as (source, target):
            target.scenes = source.scenes[:1]
        scene = target.scenes[0]
    if bpy.context.window:
        bpy.context.window.scene = scene
    scene["autonomy_pool_source"] = str(path)
    rotor_animation.rig_scene(scene)
    return path


def _object_for(name, number=None):
    return preview.object_for(bpy.context.scene,name,number)


def _ensure_driven_object(name, number=None):
    """The object the pose stream or a recording moves. Created when the scene has no match."""
    if not bpy.context.scene.get('sim_preview'):
        raise ValueError('state updates may only drive preview objects')
    found = _object_for(name,number)
    if found is not None:
        return found
    collection = bpy.context.collection or bpy.context.scene.collection
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = "ARROWS"
    empty.empty_display_size = 0.15 if name == "camera" else 0.4
    collection.objects.link(empty)
    empty['sim_display_name'] = name
    if number is not None:
        empty['sim_id'] = str(number)
        empty['sim_role'] = 'sensor' if number==preview.CAMERA_ID else 'body'
    if name == "camera" and bpy.data.objects.get("hydrus_camera") is None:
        camera = bpy.data.cameras.new("hydrus_camera")
        camera.sensor_fit = "HORIZONTAL"
        camera.angle = math.radians(60.0)
        camera.clip_start = 0.1
        camera.clip_end = 50.0
        obj = bpy.data.objects.new("hydrus_camera", camera)
        collection.objects.link(obj)
        _parent(obj, empty, Matrix.Rotation(-math.pi/2,4,'Z'))
    return empty


def ensure_link_roots():
    """The camera empty is shared by every vehicle. Other bodies arrive by name."""
    _ensure_driven_object("camera")


def _apply_packet(payload):
    if len(payload) < HEADER.size:
        return
    magic, version, count, _sim_time = HEADER.unpack_from(payload, 0)
    if magic != MAGIC or version not in (1,2) or count > 64 or not math.isfinite(_sim_time):
        return
    layout = POSE_V2 if version==2 else POSE
    need = HEADER.size + count * layout.size
    if len(payload) != need:
        return
    moved = []
    for index in range(count):
        values = layout.unpack_from(payload, HEADER.size + index * layout.size)
        number = values[0] if version==2 else None
        name, px, py, pz, qw, qx, qy, qz = values[1:] if version==2 else values
        object_name = name.split(b"\0", 1)[0].decode("utf-8", "replace")
        if not object_name:
            continue
        if not all(math.isfinite(v) for v in (px, py, pz, qw, qx, qy, qz)) or qw*qw+qx*qx+qy*qy+qz*qz < 1e-12:
            return
        obj = _ensure_driven_object(object_name,number)
        rotation = Quaternion((qw, qx, qy, qz)).to_matrix().to_4x4()
        rotation.translation = (px, py, pz)
        obj.matrix_world = rotation
        moved.append(obj.name)
    _link["packets"] += 1
    _link["time"] = _sim_time
    return moved, _sim_time


def _drain():
    sock = _link["socket"]
    if sock is None:
        return
    latest = None
    while True:
        try:
            payload, _addr = sock.recvfrom(65535)
        except BlockingIOError:
            break
        latest = payload
    scene=bpy.data.scenes.get(_link.get('scene','')) or bpy.context.scene
    with bpy.context.temp_override(scene=scene,view_layer=scene.view_layers[0]):
        if latest is not None:
            applied = _apply_packet(latest)
            scene.view_layers[0].update()
            if applied is not None:
                rotor_animation.observe_live(scene, *applied)
        rotor_animation.tick_live(scene)


# ---------------------------------------------------------------------------
# Takes: record and keyframe.

TAKE_DIR = "autonomy_takes"
TAKE_COLLECTION = "autonomy_takes"
MARKER_PREFIX = "hydrus: "
# Keyframe.interpolation enum value for LINEAR.
LINEAR = 1

_job = {"proc": None, "log": None, "scene": "", "take": "", "time": 0.0, "message": "", "cancelled": False}
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


def _tool_paths(scene):
    """Portable runtime paths, or raise with what is missing."""
    repo = _repo_dir()
    if repo is None:
        raise FileNotFoundError("Set the repository folder in the Autonomy add-on preferences")
    prefs = _prefs()
    values = {
        "executable": prefs.executable if prefs else "zig-out/bin/src_3",
        "plugin": prefs.plugin if prefs else "build/portable/sim/libauv_sim.so",
        "config": prefs.config if prefs else "sim/scenarios/controller.json",
        "simctl": "build/portable/sim/simctl",
        "backend": (prefs.backend if prefs else "build/portable/sim/libsim_backend_{backend}.so").format(backend=scene.autonomy_sim.backend),
    }
    paths = {key: _repo_path(repo, value) for key, value in values.items()}
    for key, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(f"{key} not found: {path}. Build with zig build sim and zig build")
    paths["repo"] = repo
    paths["profiles"] = repo/'sim/profiles'
    return paths


def _takes_dir():
    repo = _repo_dir()
    folder = (repo / "build/portable" if repo is not None else Path(bpy.app.tempdir)) / TAKE_DIR
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def _scene_fps(scene):
    return scene.render.fps / scene.render.fps_base


def _read_recording(path):
    """Header, samples sorted by step, events, and the end line (None while running)."""
    path = Path(path)
    stat = path.stat()
    cached = _recordings.get(str(path))
    identity = (stat.st_mtime_ns, stat.st_size, _scene_fps(bpy.context.scene))
    if cached is not None and cached[0] == identity:
        return cached[1]
    recording = trace.read_recording(path, _scene_fps(bpy.context.scene))
    _recordings[str(path)] = (identity, recording)
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
    if header.get('scenario'):
        scene['sim_snapshot']=exporter.canonical(header['scenario'])
        scene['sim_snapshot_hash']=preview.snapshot_key(header['scenario'])
    samples = recording["samples"]
    if not samples:
        raise ValueError(f"{take.recording} has no samples")
    ensure_link_roots()
    frames = [_take_frame(scene, take, sample["time"]) for sample in samples]
    for name in header["objects"]:
        obj = _ensure_driven_object(name,header.get('object_ids',{}).get(name))
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
    settings = scene.autonomy_sim
    recording = _read_recording(take.recording)
    index = list(settings.takes).index(take)
    missing = False
    for name in recording["header"]["objects"]:
        obj = _object_for(name,recording['header'].get('object_ids',{}).get(name))
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
    if not scene.get('sim_preview'): return
    legacy_names = {*rotor_animation.VEHICLES, "gate", "marker", "camera"}
    for obj in scene.objects:
        name=str(obj.get('sim_display_name',obj.name)).casefold()
        if obj.get('sim_role') not in ('body','sensor') and name not in legacy_names:
            continue
        if obj.animation_data is not None:
            action = obj.animation_data.action
            if action is not None and ":" in action.name:
                obj.animation_data.action = None
    rotor_animation.detach_takes(scene)
    for marker in list(scene.timeline_markers):
        if marker.name.startswith(MARKER_PREFIX):
            scene.timeline_markers.remove(marker)
    for take in scene.autonomy_sim.takes:
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


def launch_take(scene, seed, live=False):
    """Run the current mission through the portable plugin and record its trace."""
    if _job["proc"] is not None:
        raise RuntimeError("A simulation is already running")
    tools = _tool_paths(scene)
    requested={key:getattr(scene.autonomy_sim,key) for key in ('backend','port','max_ticks','randomize','seed','source_frame')}
    source = preview.source_scene(scene)
    frame = scene.autonomy_sim.source_frame
    snapshot = exporter.export_scene(source,tools['profiles'],frame)
    payload=wire.encode(snapshot)
    validation = subprocess.run([str(tools['simctl']),'validate','-'],input=payload,capture_output=True)
    if validation.returncode:
        raise ValueError('Evaluated scene rejected: '+validation.stderr.decode('utf-8','replace').strip())
    scene = preview.ensure(source,snapshot)
    for key,value in requested.items():setattr(scene.autonomy_sim,key,value)
    if live:_link['scene']=scene.name
    if live and not _link['running']:
        if bpy.ops.autonomy_sim.start_link() != {'RUNNING_MODAL'}:
            raise RuntimeError('could not start Blender pose receiver')
    settings = scene.autonomy_sim
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
    env = {key: value for key, value in os.environ.items() if not key.startswith("AUTONOMY_SIM_")}
    sender,receiver=socket.socketpair()
    sender.settimeout(120)
    env.update(AUTONOMY_SIM_BACKEND=str(tools["backend"]), AUTONOMY_SIM_SCENARIO=f'fd:{receiver.fileno()}',
               AUTONOMY_SIM_MAX_TICKS=str(settings.max_ticks), AUTONOMY_SIM_TRACE=str(recording_path),
               AUTONOMY_SIM_BLENDER=str(int(live)), AUTONOMY_SIM_REALTIME=str(int(live)),
               AUTONOMY_SIM_BLENDER_PORT=str(settings.port))
    if settings.randomize:
        env["AUTONOMY_SIM_SEED"] = str(int(seed))
    command = [
        str(tools["executable"]),
        str(tools["plugin"]),
        "prequalify_robosub",
        str(tools["config"]),
    ]
    try:
        proc = subprocess.Popen(command, cwd=str(tools["repo"]), env=env, stdout=log, stderr=subprocess.STDOUT,
                                pass_fds=(receiver.fileno(),))
        receiver.close()
        sender.sendall(payload)
    except OSError:
        if 'proc' in locals():
            proc.terminate();proc.wait(timeout=15)
        log.close()
        take.status = "failed to start"
        raise
    finally:
        sender.close();receiver.close()
    _job.update(
        proc=proc,
        log=log,
        scene=scene.name,
        take=name,
        time=0.0,
        message=f"Simulating {name}",
        cancelled=False,
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
    times = re.findall(r'"timestamp_ns":\s*(\d+)', tail)
    return int(times[-1]) * 1e-9 if times else 0.0


def _poll_job():
    proc = _job["proc"]
    if proc is None:
        return None
    scene = bpy.data.scenes.get(_job["scene"])
    take = _take_by_name(scene.autonomy_sim, _job["take"]) if scene is not None else None
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
    preview.select(scene)
    settings = scene.autonomy_sim
    take = _take_by_name(settings, _job["take"])
    if take is None:
        return None
    log_path = Path(take.recording).with_suffix("").with_suffix(".log")
    try:
        recording = _read_recording(take.recording)
    except (OSError, ValueError):
        recording = None
    if returncode != 0 or recording is None or recording["end"] is None:
        take.status = "cancelled" if _job["cancelled"] else "failed"
        if recording is not None and recording["end"] is not None:
            # Failed missions are still useful recordings for diagnosis.
            index = list(settings.takes).index(take)
            _import_take(scene, take, index)
            settings.take_index = index
        _job["message"] = f"{take.name} failed (exit {returncode}). See {log_path}"
        return take
    index = list(settings.takes).index(take)
    try:
        _import_take(scene, take, index)
        take.status = "cancelled" if _job["cancelled"] else "ok"
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
    camera = _object_for("hydrus_camera")
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


class AutonomyPreferences(AddonPreferences):
    bl_idname = __name__

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
        default="build/portable/sim/libauv_sim.so",
    )
    backend: StringProperty(name="Backend library template", default="build/portable/sim/libsim_backend_{backend}.so")
    config: StringProperty(name="Config", description="Relative to the repository", default="sim/scenarios/controller.json")

    def draw(self, _context):
        layout = self.layout
        layout.prop(self, "repo_dir")
        layout.prop(self, "executable")
        layout.prop(self, "plugin")
        layout.prop(self, "backend")
        layout.prop(self, "config")


class AutonomyTake(PropertyGroup):
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


class AutonomyLinkSettings(PropertyGroup):
    port: IntProperty(name="Port", default=9760, min=1, max=65535)
    source_frame: IntProperty(name="Initial condition frame", default=1, min=0)
    backend: EnumProperty(name="Backend", items=(("mujoco", "MuJoCo", ""), ("stonefish", "Stonefish", "")), default="mujoco")
    max_ticks: IntProperty(name="Tick budget", default=21600, min=1)
    randomize: BoolProperty(name="Randomize scenario", default=False)

    takes: CollectionProperty(type=AutonomyTake)
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



class AUTONOMY_UL_takes(UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_propname, _index):
        row = layout.row(align=True)
        row.label(text=item.name, icon="SEQUENCE")
        if item.parent:
            row.label(text=f"from {item.parent} @ {item.branch_frame}")
        else:
            row.label(text=f"seed {item.seed}")
        row.label(text=item.status if item.status != "ok" else item.reason)


class AUTONOMY_OT_record_take(Operator):
    bl_idname = "autonomy_sim.record_take"
    bl_label = "Record run"
    bl_description = "Run the mission headless from the start and import it as a new take"

    def execute(self, context):
        scene = context.scene
        settings = scene.autonomy_sim
        try:
            take = launch_take(scene, settings.seed)
        except (OSError, RuntimeError, ValueError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Recording {take.name}")
        return {"FINISHED"}


class AUTONOMY_OT_run_live(Operator):
    bl_idname = "autonomy_sim.run_live"
    bl_label = "Run live"
    bl_description = "Run the current mission at real time, stream poses and save a trace"

    def execute(self, context):
        if _job["proc"] is not None:
            self.report({"ERROR"}, "A simulation is already running")
            return {"CANCELLED"}
        try:
            launch_take(context.scene, context.scene.autonomy_sim.seed, live=True)
        except (OSError, RuntimeError, ValueError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        return {"FINISHED"}


class AUTONOMY_OT_cancel_take(Operator):
    bl_idname = "autonomy_sim.cancel_take"
    bl_label = "Cancel"
    bl_description = "Stop the running simulation"

    def execute(self, _context):
        proc = _job["proc"]
        if proc is not None:
            _job["cancelled"] = True
            proc.terminate()
        return {"FINISHED"}


class AUTONOMY_OT_detach_take(Operator):
    bl_idname = "autonomy_sim.detach_take"
    bl_label = "Detach"
    bl_description = "Remove take keyframes from the scene objects. Takes stay in the list"

    def execute(self, context):
        detach_takes(context.scene)
        context.scene.autonomy_sim.take_index = -1
        return {"FINISHED"}


class AUTONOMY_OT_remove_take(Operator):
    bl_idname = "autonomy_sim.remove_take"
    bl_label = "Remove take"
    bl_description = "Remove the active take from the list and delete its actions and path. Files stay on disk"

    @classmethod
    def poll(cls, context):
        settings = context.scene.autonomy_sim
        return 0 <= settings.take_index < len(settings.takes) and settings.takes[settings.take_index].status != "running"

    def execute(self, context):
        scene = context.scene
        settings = scene.autonomy_sim
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


class AUTONOMY_OT_import_take(Operator):
    bl_idname = "autonomy_sim.import_take"
    bl_label = "Import recording"
    bl_description = "Import a portable simulation trace or an experimental recording"

    filepath: StringProperty(subtype="FILE_PATH")
    filter_glob: StringProperty(default="*.jsonl", options={"HIDDEN"})

    def invoke(self, context, _event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        scene = context.scene
        settings = scene.autonomy_sim
        recording = Path(self.filepath)
        if not recording.is_file():
            self.report({"ERROR"}, f"Need {recording.name}")
            return {"CANCELLED"}
        name = recording.name.removesuffix(".rec.jsonl").removesuffix(".jsonl")
        if _take_by_name(settings, name) is not None:
            self.report({"ERROR"}, f"{name} is already in the take list")
            return {"CANCELLED"}
        try:
            data = _read_recording(recording)
            scene = preview.for_recording(scene,data)
            settings = scene.autonomy_sim
            header = data['header']
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
        end = _read_recording(recording)["end"]
        take.status = "incomplete" if end is None else "failed" if end.get("status", 0) else "ok"
        settings.take_index = index
        return {"FINISHED"}


class AUTONOMY_OT_build_scene(Operator):
    bl_idname = "autonomy_sim.build_scene"
    bl_label = "Load pool scene"
    bl_description = "Append the original pool scene from the repository"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        try:
            xml_path = build_scene()
        except (OSError, RuntimeError, ValueError) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Built Autonomy scene from {xml_path.name}")
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


class AUTONOMY_OT_start_link(Operator):
    bl_idname = "autonomy_sim.start_link"
    bl_label = "Start link"
    bl_description = "Listen for simulation poses and move matching objects"

    def modal(self, context, event):
        if not _link["running"]:
            self.cancel(context)
            return {"CANCELLED"}
        if event.type == "TIMER":
            _drain()
        return {"PASS_THROUGH"}

    def execute(self, context):
        if _link["running"]:
            self.report({"WARNING"}, "Autonomy link is already running")
            return {"CANCELLED"}
        scene = context.scene
        if not scene.get('sim_preview'):
            source = preview.source_scene(scene)
            snapshot = exporter.export_scene(source,_repo_dir()/'sim/profiles',scene.autonomy_sim.source_frame)
            scene = preview.ensure(source,snapshot)
        # Take keyframes would override the streamed poses on every frame change.
        detach_takes(scene)
        scene.autonomy_sim.take_index = -1
        ensure_link_roots()
        port = scene.autonomy_sim.port
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
        _link['scene']=scene.name
        _link.update(packets=0, time=0.0)
        self._timer = context.window_manager.event_timer_add(1.0 / 30.0, window=context.window)
        context.window_manager.modal_handler_add(self)
        self.report({"INFO"}, f"Autonomy link listening on 127.0.0.1:{port}")
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


class AUTONOMY_OT_stop_link(Operator):
    bl_idname = "autonomy_sim.stop_link"
    bl_label = "Stop link"
    bl_description = "Stop listening for simulation poses"

    def execute(self, _context):
        _link["running"] = False
        return {"FINISHED"}


class AUTONOMY_OT_edit_source(Operator):
    bl_idname = 'autonomy_sim.edit_source'
    bl_label = 'Edit source scene'

    def execute(self, context):
        try:
            preview.select(preview.source_scene(context.scene))
        except ValueError as exc:
            self.report({'ERROR'},str(exc));return {'CANCELLED'}
        return {'FINISHED'}


class AUTONOMY_OT_add_obstacle(Operator):
    bl_idname = 'autonomy_sim.add_obstacle'
    bl_label = 'Add simulation obstacle'
    bl_description = 'Add a collidable box to the authored simulation collection'
    bl_options = {'REGISTER','UNDO'}

    def execute(self,context):
        try:
            source=preview.source_scene(context.scene)
            collection=bpy.data.collections.get(source.get('sim_collection',''))
            if collection is None:raise ValueError('load an authored simulation scene first')
            preview.select(source)
            bpy.ops.mesh.primitive_cube_add(size=1,location=source.cursor.location)
            obj=bpy.context.object;obj.name='Simulation obstacle'
            if obj.name not in collection.objects:collection.objects.link(obj)
            for key,value in {'sim_id':str(uuid.uuid4()),'sim_role':'body','sim_shape':'box','sim_motion':'static',
                              'sim_collision':True,'sim_class':0,'sim_tracked':False}.items():obj[key]=value
        except ValueError as exc:
            self.report({'ERROR'},str(exc));return {'CANCELLED'}
        return {'FINISHED'}


class AUTONOMY_OT_embed_profiles(Operator):
    bl_idname = 'autonomy_sim.embed_profiles'
    bl_label = 'Embed calibration profiles'
    bl_description = 'Refresh calibration from repository profiles and store it inside this Blender file'
    bl_options = {'REGISTER','UNDO'}

    def execute(self,context):
        try:
            source=preview.source_scene(context.scene)
            repo=_repo_dir()
            if repo is None: raise ValueError('set the repository preference first')
            names=exporter.embed_profiles(source,repo/'sim/profiles')
            self.report({'INFO'},'Embedded '+', '.join(names)+'. Save the Blender file to keep calibration.')
        except (OSError,ValueError) as error:
            self.report({'ERROR'},str(error));return {'CANCELLED'}
        return {'FINISHED'}


class AUTONOMY_PT_link(Panel):
    bl_label = "Autonomy"
    bl_idname = "AUTONOMY_PT_link"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Autonomy"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.autonomy_sim
        layout.label(text='Source: '+str(context.scene.get('sim_source_scene',context.scene.name)))
        layout.prop(settings,'source_frame')
        if context.scene.get('sim_snapshot_hash'):
            layout.label(text='Snapshot: '+context.scene['sim_snapshot_hash'][:12])
        layout.prop(settings, "backend")
        layout.prop(settings, "port")
        layout.operator("autonomy_sim.build_scene", icon="MESH_CUBE")
        layout.operator("autonomy_sim.run_live", icon="PLAY")
        layout.operator('autonomy_sim.edit_source')
        layout.operator('autonomy_sim.add_obstacle',icon='MESH_CUBE')
        layout.operator('autonomy_sim.embed_profiles')
        row = layout.row(align=True)
        row.operator("autonomy_sim.start_link", icon="PLAY")
        row.operator("autonomy_sim.stop_link", icon="PAUSE")
        layout.label(text="Listening" if _link["running"] else "Idle")
        if _link["packets"]:
            layout.label(text=f"{_link['time']:.2f} s · {_link['packets']} updates")


class AUTONOMY_PT_takes(Panel):
    bl_label = "Takes"
    bl_idname = "AUTONOMY_PT_takes"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Autonomy"

    def draw(self, context):
        layout = self.layout
        settings = context.scene.autonomy_sim
        layout.prop(settings, "randomize")
        if settings.randomize:
            layout.prop(settings, "seed")
        layout.prop(settings, "max_ticks")
        row = layout.row(align=True)
        row.operator("autonomy_sim.record_take", icon="REC")
        row.operator("autonomy_sim.import_take", icon="IMPORT", text="")
        layout.template_list("AUTONOMY_UL_takes", "", settings, "takes", settings, "take_index", rows=4)
        row = layout.row(align=True)
        row.operator("autonomy_sim.detach_take", icon="UNLINKED")
        row.operator("autonomy_sim.remove_take", icon="X", text="")
        layout.prop(settings, "show_parent_path")
        layout.prop(settings, "onboard_camera", icon="CAMERA_DATA")
        if _job["proc"] is not None:
            box = layout.box()
            box.label(text=f"{_job['take']}: {_job['time']:.1f} s")
            box.operator("autonomy_sim.cancel_take", icon="CANCEL")
        elif _job["message"]:
            layout.label(text=_job["message"])


CLASSES = (
    AutonomyPreferences,
    AutonomyTake,
    AutonomyLinkSettings,
    AUTONOMY_UL_takes,
    AUTONOMY_OT_build_scene,
    AUTONOMY_OT_start_link,
    AUTONOMY_OT_stop_link,
    AUTONOMY_OT_edit_source,
    AUTONOMY_OT_add_obstacle,
    AUTONOMY_OT_embed_profiles,
    AUTONOMY_OT_record_take,
    AUTONOMY_OT_run_live,
    AUTONOMY_OT_cancel_take,
    AUTONOMY_OT_detach_take,
    AUTONOMY_OT_remove_take,
    AUTONOMY_OT_import_take,
    AUTONOMY_PT_link,
    AUTONOMY_PT_takes,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.autonomy_sim = PointerProperty(type=AutonomyLinkSettings)
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
    del bpy.types.Scene.autonomy_sim
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
