"""Handlers the voice model calls.

JSON and ocean edits hit the files the simulator already watches. Scene edits
go through the Blender command server, or a one-shot Blender process, which
saves the .blend file and triggers the same reload.
"""

import threading

import blender_client
import json_edit

_lock = threading.Lock()


def _blender(command: dict) -> dict:
    with _lock:
        return blender_client.run(command)


def describe_controls() -> dict:
    return json_edit.describe_controls()


def get_number(file: str, path: str) -> dict:
    value = json_edit.get_value(file, path)
    return {"ok": True, "file": file, "path": path, "value": value}


def set_number(file: str, path: str, value) -> dict:
    with _lock:
        return json_edit.set_number(file, path, value)


def set_robot(robot: str) -> dict:
    with _lock:
        return json_edit.set_robot(robot)


def list_scene_objects() -> dict:
    return _blender({"op": "list_objects"})


def get_object(name: str) -> dict:
    return _blender({"op": "get_object", "name": name})


def move_object(name: str, x, y, z) -> dict:
    return _blender({"op": "move_object", "name": name, "x": x, "y": y, "z": z})


def nudge_object(name: str, dx=0, dy=0, dz=0) -> dict:
    return _blender({"op": "nudge_object", "name": name, "dx": dx, "dy": dy, "dz": dz})


def rotate_object(name: str, roll_deg=None, pitch_deg=None, yaw_deg=None) -> dict:
    command = {"op": "rotate_object", "name": name}
    if roll_deg is not None:
        command["roll_deg"] = roll_deg
    if pitch_deg is not None:
        command["pitch_deg"] = pitch_deg
    if yaw_deg is not None:
        command["yaw_deg"] = yaw_deg
    return _blender(command)


def set_stonefish(
    name: str,
    enabled=True,
    stonefish_name=None,
    material=None,
    look=None,
    cls=None,
) -> dict:
    command = {"op": "set_stonefish", "name": name, "enabled": enabled}
    if stonefish_name is not None:
        command["stonefish_name"] = stonefish_name
    if material is not None:
        command["material"] = material
    if look is not None:
        command["look"] = look
    if cls is not None:
        command["cls"] = cls
    return _blender(command)


HANDLERS = {
    "describe_controls": describe_controls,
    "get_number": get_number,
    "set_number": set_number,
    "set_robot": set_robot,
    "list_scene_objects": list_scene_objects,
    "get_object": get_object,
    "move_object": move_object,
    "nudge_object": nudge_object,
    "rotate_object": rotate_object,
    "set_stonefish": set_stonefish,
}

_NUMBER = {"type": "number"}
_NAME = {"type": "string", "description": "Blender object name, for example Gate, Bin, Board, Marker, or Pole1."}
_FILE = {
    "type": "string",
    "enum": list(json_edit.FILES),
    "description": "platform, a vehicle controller (hydrus, proteus, bluerov2, girona500), or ocean for the scene YAML.",
}

TOOLS = [
    {
        "type": "function",
        "name": "describe_controls",
        "description": "List editable files, the current platform settings, and example field paths. Use this before guessing a name.",
        "parameters": {"type": "object", "properties": {}, "required": []},
    },
    {
        "type": "function",
        "name": "get_number",
        "description": "Read one number from a watched simulation file.",
        "parameters": {
            "type": "object",
            "properties": {"file": _FILE, "path": {"type": "string"}},
            "required": ["file", "path"],
        },
    },
    {
        "type": "function",
        "name": "set_number",
        "description": "Change one number in a watched file. The running simulator reloads platform.json, the vehicle JSON, and the ocean YAML on its own. Examples: file hydrus path non_odometry.target_height; file platform path steps_per_second; file ocean path environment.ocean.wave_height.",
        "parameters": {
            "type": "object",
            "properties": {"file": _FILE, "path": {"type": "string"}, "value": _NUMBER},
            "required": ["file", "path", "value"],
        },
    },
    {
        "type": "function",
        "name": "set_robot",
        "description": "Switch the simulated vehicle. Allowed: hydrus, proteus, bluerov2, girona500. The simulator rebuilds the scene.",
        "parameters": {
            "type": "object",
            "properties": {"robot": {"type": "string", "enum": list(json_edit.ROBOTS)}},
            "required": ["robot"],
        },
    },
    {
        "type": "function",
        "name": "list_scene_objects",
        "description": "List mesh objects in the pool blend file with Blender and simulator coordinates. Saving is not required. If Blender is not already listening, this starts Blender and can take a while.",
        "parameters": {"type": "object", "properties": {}, "required": []},
    },
    {
        "type": "function",
        "name": "get_object",
        "description": "Read one mesh object's position and rotation.",
        "parameters": {
            "type": "object",
            "properties": {"name": _NAME},
            "required": ["name"],
        },
    },
    {
        "type": "function",
        "name": "move_object",
        "description": "Move a mesh to an absolute Blender location in meters, save the blend file, and let the simulator reload it. Simulator coordinates are sim_x=-x, sim_y=y, sim_z=-z.",
        "parameters": {
            "type": "object",
            "properties": {"name": _NAME, "x": _NUMBER, "y": _NUMBER, "z": _NUMBER},
            "required": ["name", "x", "y", "z"],
        },
    },
    {
        "type": "function",
        "name": "nudge_object",
        "description": "Move a mesh by a Blender-meter offset and save the blend file.",
        "parameters": {
            "type": "object",
            "properties": {"name": _NAME, "dx": _NUMBER, "dy": _NUMBER, "dz": _NUMBER},
            "required": ["name"],
        },
    },
    {
        "type": "function",
        "name": "rotate_object",
        "description": "Set Blender XYZ Euler rotation in degrees. Omit an angle to leave it unchanged. Saves the blend file.",
        "parameters": {
            "type": "object",
            "properties": {
                "name": _NAME,
                "roll_deg": _NUMBER,
                "pitch_deg": _NUMBER,
                "yaw_deg": _NUMBER,
            },
            "required": ["name"],
        },
    },
    {
        "type": "function",
        "name": "set_stonefish",
        "description": "Mark a mesh for the simulator or change its material, look, or class, then save. Classes: none, scenery, gate, cube, rect.",
        "parameters": {
            "type": "object",
            "properties": {
                "name": _NAME,
                "enabled": {"type": "boolean"},
                "stonefish_name": {"type": "string"},
                "material": {"type": "string"},
                "look": {"type": "string"},
                "cls": {"type": "string"},
            },
            "required": ["name"],
        },
    },
]


def call(name: str, arguments: dict) -> dict:
    fn = HANDLERS.get(name)
    if fn is None:
        return {"ok": False, "error": f"unknown tool {name}"}
    if not isinstance(arguments, dict):
        return {"ok": False, "error": "arguments must be an object"}
    try:
        return fn(**arguments)
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
