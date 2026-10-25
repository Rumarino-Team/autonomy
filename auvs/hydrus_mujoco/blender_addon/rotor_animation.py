"""Visual propeller animation for live poses, manual animation and recorded commands.

The presentation RPM is deliberately lower than motor RPM so individual blades
remain readable at 24 fps. This is an appearance effect, never a physics input.
"""

import math
import re
import time

VEHICLES = ("hydrus", "proteus", "bluerov", "girona500")
_live = {}
_frames = {}
_manual = {}
_busy = False
_last_tick = None


def angular_speed(command, rpm=210.0):
    """Signed presentation speed; normalized commands retain reverse thrust."""
    if not math.isfinite(command) or abs(command) < 0.002:
        return 0.0
    return math.copysign(math.sqrt(min(abs(command), 1.0)), command) * rpm * math.tau / 60.0


def advance(phase, speed, target, dt, tau=0.18):
    """Exact first-order motor response, including its integrated shaft angle."""
    if dt <= 0.0:
        return phase, speed
    decay = math.exp(-dt / tau)
    phase += target * dt + (speed - target) * tau * (1.0 - decay)
    speed = target + (speed - target) * decay
    if target == 0.0 and abs(speed) < 1e-4:
        speed = 0.0
    return phase, speed


def rotor_index(name, vehicle):
    tail = name.split(" | ")[-1]
    match = re.fullmatch(r"prop_(\d+)|propeller_(\d+)", tail)
    if match:
        return int(match.group(1) or match.group(2))
    if vehicle == "girona500":
        order = ("ThrusterSurgePort", "ThrusterSurgeStarboard", "ThrusterSway",
                 "ThrusterHeaveStern", "ThrusterHeaveBow")
        if tail in order:
            return order.index(tail)
    return None


def rotors(root):
    return sorted((ob for ob in root.children_recursive
                   if ob.type == "MESH" and "auv_rotor_index" in ob),
                  key=lambda ob: ob["auv_rotor_index"])


def rig_vehicle(root):
    """Put each mesh beneath an immutable mount; only its local shaft X rotates."""
    import bpy
    from mathutils import Matrix

    if root.name not in VEHICLES:
        return []
    candidates = [ob for ob in root.children if ob.type == "MESH"
                  and rotor_index(ob.name, root.name) is not None]
    for ob in candidates:
        index = rotor_index(ob.name, root.name)
        mount = bpy.data.objects.new(root.name + " | rotor mount " + str(index), None)
        (ob.users_collection[0] if ob.users_collection else bpy.context.scene.collection).objects.link(mount)
        mount.parent = root
        mount.matrix_parent_inverse = Matrix.Identity(4)
        mount.rotation_mode = "QUATERNION"
        mount.matrix_basis = ob.matrix_local.copy()
        mount.empty_display_size = 0.025
        mount["stonefish"] = "0"
        mount["auv_rotor_mount"] = True
        ob.parent = mount
        ob.matrix_parent_inverse = Matrix.Identity(4)
        ob.matrix_basis = Matrix.Identity(4)
        ob.rotation_mode = "XYZ"
        ob["auv_rotor_index"] = index
        ob["auv_rotor_vehicle"] = root.name
        # BlueROV uses separate clockwise/counterclockwise blade meshes.
        ob["auv_rotor_direction"] = -1.0 if root.name == "bluerov" and index in (0, 1, 5, 6) else 1.0
        ob["stonefish"] = "0"
    if "rotor_visual_rpm" not in root:
        root["rotor_visual_rpm"] = 210.0
        root.id_properties_ui("rotor_visual_rpm").update(min=0.0, max=1000.0,
            description="Presentation RPM at full command; reduced for readable blades at 24 fps")
    if "rotor_animation" not in root:
        root["rotor_animation"] = "Recorded thruster commands; pose-derived estimate for live/manual motion. Visual effect only."
    return rotors(root)


def rig_scene(scene):
    return {name: rig_vehicle(scene.objects[name]) for name in VEHICLES if name in scene.objects}


def _targets(root, previous, current, dt):
    """Estimate effort from velocity along each shaft, including turn-induced motion."""
    from mathutils import Vector

    inverse = current.to_quaternion().inverted()
    velocity = inverse @ ((current.translation - previous.translation) / dt)
    delta = previous.to_quaternion().rotation_difference(current.to_quaternion())
    if delta.w < 0:
        delta.negate()
    axis, angle = delta.to_axis_angle()
    angular = axis * angle / dt
    targets = {}
    for rotor in rotors(root):
        mount = rotor.parent.matrix_basis
        direction = (mount.to_3x3() @ Vector((1, 0, 0))).normalized()
        point_velocity = velocity + angular.cross(mount.translation)
        effort = point_velocity.dot(direction) / 0.6
        if abs(effort) < 0.008:
            effort = 0.0
        targets[rotor.name] = angular_speed(effort, root.get("rotor_visual_rpm", 210.0)) * rotor.get("auv_rotor_direction", 1.0)
    return targets


def observe_live(scene, names, sim_time):
    """Called after applying a complete UDP packet; uses simulation time for velocity."""
    now = time.monotonic()
    for name in names:
        root = scene.objects.get(name)
        if root is None or name not in VEHICLES:
            continue
        rig_vehicle(root)
        matrix = root.matrix_world.copy()
        state = _live.get(name)
        if state is None or sim_time <= state["time"] or sim_time - state["time"] > 1.0:
            _live[name] = {"matrix": matrix, "time": sim_time, "seen": now,
                           "targets": {}, "motors": {}}
            continue
        state["targets"] = _targets(root, state["matrix"], matrix, sim_time - state["time"])
        state.update(matrix=matrix, time=sim_time, seen=now)


def tick_live(scene):
    """The existing 30 Hz link timer advances and decelerates visible rotors."""
    global _last_tick
    now = time.monotonic()
    dt = min(now - _last_tick, 0.1) if _last_tick is not None else 0.0
    _last_tick = now
    for name, state in list(_live.items()):
        root = scene.objects.get(name)
        if root is None:
            continue
        for ob in rotors(root):
            if _is_baked(ob):
                continue
            phase, speed = state["motors"].get(ob.name, (ob.rotation_euler.x, 0.0))
            target = state["targets"].get(ob.name, 0.0) if now - state["seen"] < 0.25 else 0.0
            phase, speed = advance(phase, speed, target, dt)
            state["motors"][ob.name] = (phase, speed)
            ob.rotation_euler.x = phase % math.tau


def reset_live():
    global _last_tick
    _live.clear()
    _frames.clear()
    _manual.clear()
    _last_tick = None


def _is_baked(ob):
    return (ob.animation_data is not None and ob.animation_data.action is not None
            and ":rotor:" in ob.animation_data.action.name)


def _assign(ob, action):
    ob.animation_data_create()
    ob.animation_data.action = action
    if hasattr(ob.animation_data, "action_slot") and ob.animation_data.action_slot is None and len(action.slots):
        ob.animation_data.action_slot = action.slots[0]


def bake_take(root, take_name, frames, samples, fps):
    """Native F-curves render and scrub deterministically without Python handlers."""
    import bpy

    result = []
    for ob in rig_vehicle(root):
        index = ob["auv_rotor_index"]
        phase = speed = previous_target = 0.0
        values = []
        for i, sample in enumerate(samples):
            commands = sample.get("thr", ())
            command = float(commands[index]) if index < len(commands) else 0.0
            target = angular_speed(command, root.get("rotor_visual_rpm", 210.0)) * ob.get("auv_rotor_direction", 1.0)
            if i:
                # Commands apply over the preceding interval; integrate the mean
                # endpoint target, avoiding a whole-frame lead at starts/stops.
                dt = max(0.0, (frames[i] - frames[i-1]) / fps)
                phase, speed = advance(phase, speed, (previous_target + target) * 0.5, dt)
            values.append(phase)
            previous_target = target
        name = take_name + ":rotor:" + ob.name
        old = bpy.data.actions.get(name)
        if old is not None:
            bpy.data.actions.remove(old)
        action = bpy.data.actions.new(name)
        action.use_fake_user = True
        _assign(ob, action)
        if hasattr(action, "fcurve_ensure_for_datablock"):
            curve = action.fcurve_ensure_for_datablock(ob, "rotation_euler", index=0)
        else:
            curve = action.fcurves.new("rotation_euler", index=0)
        curve.keyframe_points.add(len(frames))
        curve.keyframe_points.foreach_set("co", [v for pair in zip(frames, values) for v in pair])
        for key in curve.keyframe_points:
            key.interpolation = "LINEAR"
        curve.update()
        result.append(name)
    return result


def activate_take(scene, root, take_name):
    import bpy

    detach_takes(scene)
    complete = True
    for ob in rig_vehicle(root):
        action = bpy.data.actions.get(take_name + ":rotor:" + ob.name)
        if action is None:
            complete = False
        else:
            _assign(ob, action)
    return complete


def detach_takes(scene):
    for name in VEHICLES:
        root = scene.objects.get(name)
        if root is not None:
            for ob in rotors(root):
                if _is_baked(ob):
                    ob.animation_data.action = None
    reset_live()


def _on_frame(scene, _depsgraph=None):
    """Motion estimate for ordinary user-keyed vehicles; recorded actions win."""
    global _busy
    if _busy:
        return
    _busy = True
    try:
        fps = scene.render.fps / scene.render.fps_base
        frame = scene.frame_current + scene.frame_subframe
        for name in VEHICLES:
            root = scene.objects.get(name)
            if root is None or name in _live or root.animation_data is None or root.animation_data.action is None:
                continue
            matrix = root.matrix_world.copy()
            previous = _frames.get(name)
            state = {"frame": frame, "matrix": matrix, "motors": {}}
            if previous and 0 < frame - previous["frame"] <= 4:
                dt = (frame - previous["frame"]) / fps
                targets = _targets(root, previous["matrix"], matrix, dt)
                for ob in rotors(root):
                    if _is_baked(ob):
                        continue
                    phase, speed = previous["motors"].get(ob.name, (ob.rotation_euler.x, 0.0))
                    phase, speed = advance(phase, speed, targets.get(ob.name, 0.0), dt)
                    state["motors"][ob.name] = (phase, speed)
                    ob.rotation_euler.x = phase % math.tau
            _frames[name] = state
    finally:
        _busy = False


def _on_load(_unused):
    import bpy
    reset_live()
    for scene in bpy.data.scenes:
        rig_scene(scene)


def _initialize_scene():
    # Add-on registration runs inside Blender's restricted context. Initialize
    # scene data once registration has finished and the normal context returns.
    _on_load(None)
    return None


def _on_depsgraph(scene, depsgraph):
    """Also react to manual root transforms while the timeline is paused."""
    global _busy
    if _busy:
        return
    _busy = True
    try:
        changed = {update.id.name for update in depsgraph.updates if update.is_updated_transform}
        now = time.monotonic()
        for name in VEHICLES:
            if name not in changed or name in _live:
                continue
            root = scene.objects.get(name)
            if root is None or (root.animation_data and root.animation_data.action):
                continue
            matrix = root.matrix_world.copy()
            previous = _manual.get(name)
            state = {"matrix": matrix, "time": now, "motors": {}}
            if previous and any(abs(matrix[r][c] - previous["matrix"][r][c]) > 1e-7
                                for r in range(4) for c in range(4)):
                dt = min(max(now - previous["time"], 1 / 240), 0.1)
                targets = _targets(root, previous["matrix"], matrix, dt)
                for ob in rotors(root):
                    if _is_baked(ob):
                        continue
                    phase, speed = previous["motors"].get(ob.name, (ob.rotation_euler.x, 0.0))
                    phase, speed = advance(phase, speed, targets.get(ob.name, 0.0), dt)
                    state["motors"][ob.name] = (phase, speed)
                    ob.rotation_euler.x = phase % math.tau
            elif previous:
                state["motors"] = previous["motors"]
            _manual[name] = state
    finally:
        _busy = False


def register():
    import bpy
    from bpy.app.handlers import persistent
    for handler, collection in ((_on_frame, bpy.app.handlers.frame_change_post),
                                (_on_load, bpy.app.handlers.load_post),
                                (_on_depsgraph, bpy.app.handlers.depsgraph_update_post)):
        persistent(handler)
        if handler not in collection:
            collection.append(handler)
    if not bpy.app.timers.is_registered(_initialize_scene):
        bpy.app.timers.register(_initialize_scene, first_interval=0.0)


def unregister():
    import bpy
    if bpy.app.timers.is_registered(_initialize_scene):
        bpy.app.timers.unregister(_initialize_scene)
    for handler, collection in ((_on_frame, bpy.app.handlers.frame_change_post),
                                (_on_load, bpy.app.handlers.load_post),
                                (_on_depsgraph, bpy.app.handlers.depsgraph_update_post)):
        if handler in collection:
            collection.remove(handler)
    reset_live()
