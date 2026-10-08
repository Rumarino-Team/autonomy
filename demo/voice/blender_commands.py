"""Scene commands that run inside Blender.

The simulator reads this .blend file and reloads it when the file is saved.
Positions are Blender world meters. Stonefish then maps (x, y, z) to
(-x, y, -z). Objects must stay unparented, which is what the pool scene uses.

Import this module from a `blender --python` script or from the command
server. It does not import bpy until a command runs.
"""

import math

SENTINEL = "VOICE_RESULT "

ID_KEYS = (
    "stonefish",
    "stonefish_name",
    "material",
    "look",
    "cls",
    "physics_mesh",
    "visual_mesh",
    "convex",
)

MATERIALS = {
    "acrylic",
    "hdpe",
    "abs",
    "aluminium",
    "steel",
    "pvc",
    "ceramic",
    "concrete",
    "stone",
    "glass",
}

CLASSES = {"none", "scenery", "gate", "cube", "rect"}

MUTATING = {"move_object", "nudge_object", "rotate_object", "set_stonefish"}


def _bpy():
    import bpy

    return bpy


def _sim_pose(x: float, y: float, z: float) -> dict:
    return {"x": -x, "y": y, "z": -z}


def _round(value: float, digits: int) -> float:
    number = round(float(value), digits)
    return 0.0 if number == 0 else number


def _degrees(radians: float) -> float:
    return _round(math.degrees(radians), 3)


def _pose(obj) -> dict:
    x, y, z = (_round(component, 4) for component in obj.location)
    roll, pitch, yaw = (_degrees(component) for component in obj.rotation_euler)
    return {
        "name": obj.name,
        "blender": {"x": x, "y": y, "z": z},
        "sim": _sim_pose(x, y, z),
        "rotation_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "stonefish": str(obj.get("stonefish", "")),
        "stonefish_name": str(obj.get("stonefish_name", "")),
        "material": str(obj.get("material", "")),
        "look": str(obj.get("look", "")),
        "cls": str(obj.get("cls", "")),
    }


def find_object(name: str):
    if not isinstance(name, str) or not name.strip():
        raise ValueError("name is required")
    bpy = _bpy()
    exact = bpy.data.objects.get(name)
    if exact is not None and exact.type == "MESH":
        return exact
    lowered = name.strip().lower()
    meshes = [obj for obj in bpy.data.objects if obj.type == "MESH"]
    matches = [obj for obj in meshes if obj.name.lower() == lowered]
    if len(matches) == 1:
        return matches[0]
    partial = [obj for obj in meshes if lowered in obj.name.lower()]
    if len(partial) == 1:
        return partial[0]
    known = ", ".join(obj.name for obj in meshes)
    raise KeyError(f"no mesh named {name!r}. Meshes: {known}")


def list_objects() -> dict:
    bpy = _bpy()
    objects = [_pose(obj) for obj in bpy.data.objects if obj.type == "MESH"]
    objects.sort(key=lambda item: item["name"])
    return {"ok": True, "objects": objects}


def get_object(name: str) -> dict:
    return {"ok": True, **_pose(find_object(name))}


def _number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def move_object(name: str, x, y, z) -> dict:
    obj = find_object(name)
    obj.location = (_number(x, "x"), _number(y, "y"), _number(z, "z"))
    obj.rotation_mode = "XYZ"
    return {"ok": True, **_pose(obj)}


def nudge_object(name: str, dx=0, dy=0, dz=0) -> dict:
    obj = find_object(name)
    x, y, z = obj.location
    obj.location = (
        float(x) + _number(dx, "dx"),
        float(y) + _number(dy, "dy"),
        float(z) + _number(dz, "dz"),
    )
    obj.rotation_mode = "XYZ"
    return {"ok": True, **_pose(obj)}


def rotate_object(name: str, roll_deg=None, pitch_deg=None, yaw_deg=None) -> dict:
    obj = find_object(name)
    obj.rotation_mode = "XYZ"
    roll, pitch, yaw = obj.rotation_euler
    if roll_deg is not None:
        roll = math.radians(_number(roll_deg, "roll_deg"))
    if pitch_deg is not None:
        pitch = math.radians(_number(pitch_deg, "pitch_deg"))
    if yaw_deg is not None:
        yaw = math.radians(_number(yaw_deg, "yaw_deg"))
    if roll_deg is None and pitch_deg is None and yaw_deg is None:
        raise ValueError("pass roll_deg, pitch_deg, or yaw_deg")
    obj.rotation_euler = (roll, pitch, yaw)
    return {"ok": True, **_pose(obj)}


def set_stonefish(
    name: str,
    enabled=True,
    stonefish_name=None,
    material=None,
    look=None,
    cls=None,
) -> dict:
    obj = find_object(name)
    if isinstance(enabled, str):
        enabled = enabled.strip().lower() in ("1", "true", "yes")
    if not enabled:
        for key in ID_KEYS:
            if key in obj:
                del obj[key]
        return {"ok": True, "name": obj.name, "enabled": False}
    obj["stonefish"] = "1"
    obj["stonefish_name"] = stonefish_name or str(obj.get("stonefish_name", obj.name))
    if material is not None:
        if material not in MATERIALS:
            raise ValueError(f"material must be one of: {', '.join(sorted(MATERIALS))}")
        obj["material"] = material
    if look is not None:
        if not isinstance(look, str) or not look.strip():
            raise ValueError("look must be a name from config.yaml")
        obj["look"] = look
    if cls is not None:
        if cls not in CLASSES:
            raise ValueError(f"cls must be one of: {', '.join(sorted(CLASSES))}")
        if cls == "none":
            if "cls" in obj:
                del obj["cls"]
        else:
            obj["cls"] = cls
    return {"ok": True, "enabled": True, **_pose(obj)}


def _save() -> None:
    bpy = _bpy()
    if bpy.context.view_layer is not None:
        bpy.context.view_layer.update()
    bpy.ops.wm.save_mainfile()


OPS = {
    "list_objects": list_objects,
    "get_object": get_object,
    "move_object": move_object,
    "nudge_object": nudge_object,
    "rotate_object": rotate_object,
    "set_stonefish": set_stonefish,
}


def dispatch(command: dict) -> dict:
    if not isinstance(command, dict):
        return {"ok": False, "error": "command must be an object"}
    op = command.get("op")
    fn = OPS.get(op)
    if fn is None:
        known = ", ".join(sorted(OPS))
        return {"ok": False, "error": f"unknown blender command {op!r}; use one of: {known}"}
    kwargs = {key: value for key, value in command.items() if key != "op"}
    try:
        result = fn(**kwargs)
        if op in MUTATING:
            _save()
            result["saved"] = True
        return result
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
