"""Export pool_scene.blend to OpenUSD and a NED MuJoCo world.

Run from the repository:

    blender auvs/stonefish_sim/data/pool_scene.blend --background \\
        --python auvs/hydrus_mujoco/export_pool_usd.py

The USD stage stays in the blend's Z-up metres. pool_world.xml is read back
from that stage. Blender (x, y, z) becomes NED (-x, y, -z). Tile meshes are
visual. A floor and four walls take collision. A mesh with a non-empty fuzz
property becomes a mocap body at its own pose. The property lists the axes
the fuzzer may change, for example "pos yaw", and the class token is added
from cls when that class is gate, cube, or rect.
"""

import math
import re
from pathlib import Path

import bpy
from mathutils import Matrix, Vector
from pxr import Gf, Sdf, Usd, UsdGeom

REPO = Path(__file__).resolve().parents[2]
USD_PATH = REPO / "auvs/stonefish_sim/data/pool_scene.usd"
XML_PATH = REPO / "auvs/hydrus_mujoco/pool_world.xml"
MESH_DIR = REPO / "auvs/stonefish_sim/data/models/pool_usd"
# M sends Blender (x, y, z) to NED (-x, y, -z) and is its own inverse.
M = Matrix(((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0)))


def included(obj):
    if obj.type != "MESH":
        return False
    # hydrus.xml owns the vehicle. The blend keeps a copy parented to "hydrus"
    # so the pool can be posed around it; those meshes are not pool scenery.
    ancestor = obj.parent
    while ancestor is not None:
        if ancestor.name.casefold() == "hydrus":
            return False
        ancestor = ancestor.parent
    flag = str(obj.get("stonefish", "")).strip().casefold()
    return flag not in {"0", "false"}


def classify(obj):
    name = str(obj.get("stonefish_name", "")).strip() or obj.name
    cls = str(obj.get("cls", "")).strip().casefold()
    if not cls and name.casefold() == "marker":
        cls = "cube"
    if not cls and name.casefold() == "gate":
        cls = "gate"
    if not cls:
        cls = "scenery"
    convex = str(obj.get("convex", "")).strip().casefold() in {"1", "true"}
    return name, cls, convex


def stamp(obj):
    name, cls, _convex = classify(obj)
    obj["stonefish_name"] = name
    obj["cls"] = cls


def usd_to_matrix(transform):
    # USD multiplies row vectors and stores translation in the last row.
    return Matrix(tuple(tuple(transform[row][col] for col in range(4)) for row in range(4))).transposed()


def xml_name(text):
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_")
    if not cleaned or cleaned[0].isdigit():
        cleaned = "m_" + cleaned
    return cleaned


def write_obj(path, vertices, faces):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for vertex in vertices:
            handle.write(f"v {vertex[0]:.8f} {vertex[1]:.8f} {vertex[2]:.8f}\n")
        for face in faces:
            handle.write("f " + " ".join(str(index + 1) for index in face) + "\n")


def triangulate(counts, indices):
    faces = []
    cursor = 0
    for count in counts:
        polygon = list(indices[cursor : cursor + count])
        cursor += count
        if count < 3:
            continue
        for offset in range(1, count - 1):
            faces.append((polygon[0], polygon[offset], polygon[offset + 1]))
    return faces


def quat_from_matrix(rotation):
    quat = rotation.to_quaternion()
    return (quat.w, quat.x, quat.y, quat.z)


def fmt(values):
    return " ".join(f"{value:.8g}" for value in values)


def attr_string(prim, key):
    current = prim
    while current and current.IsValid() and current.GetPath() != "/":
        for name in (f"userProperties:{key}", key):
            attr = current.GetAttribute(name)
            if attr and attr.HasValue():
                return str(attr.Get())
        custom = current.GetCustomData()
        if key in custom:
            return str(custom[key])
        nested = custom.get("userProperties")
        if isinstance(nested, dict) and key in nested:
            return str(nested[key])
        current = current.GetParent()
    return ""


def export_stage():
    for obj in bpy.data.objects:
        if included(obj):
            stamp(obj)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in bpy.data.objects:
        if included(obj):
            obj.select_set(True)
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    USD_PATH.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.usd_export(
        filepath=str(USD_PATH),
        selected_objects_only=True,
        visible_objects_only=False,
        export_animation=False,
        export_meshes=True,
        export_materials=False,
        export_lights=False,
        export_cameras=False,
        triangulate_meshes=True,
        convert_orientation=False,
        convert_scene_units="METERS",
        export_custom_properties=True,
        custom_properties_namespace="userProperties",
        author_blender_name=True,
        evaluation_mode="VIEWPORT",
        xform_op_mode="MAT",
    )


def stamp_stage(stage):
    by_name = {}
    for obj in bpy.data.objects:
        if included(obj):
            name, cls, convex = classify(obj)
            by_name[obj.name] = (name, cls, "true" if convex else "false")
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        blender_name = prim.GetName()
        custom = prim.GetCustomData().get("blender_name") or prim.GetCustomData().get("object_name")
        if custom:
            blender_name = str(custom)
        info = by_name.get(blender_name) or by_name.get(prim.GetName())
        if info is None:
            continue
        stonefish_name, cls, convex = info
        fuzz = ""
        source = bpy.data.objects.get(blender_name)
        if source is not None:
            fuzz = str(source.get("fuzz", "")).strip()
        for key, value in (
            ("stonefish_name", stonefish_name),
            ("cls", cls),
            ("convex", convex),
            ("fuzz", fuzz),
        ):
            attr = prim.CreateAttribute(f"userProperties:{key}", Sdf.ValueTypeNames.String, custom=True)
            attr.Set(value)


def read_records(stage):
    cache = UsdGeom.XformCache()
    records = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not points or not counts or not indices:
            continue
        transform = usd_to_matrix(cache.GetLocalToWorldTransform(prim))
        location, rotation, _scale = transform.decompose()
        rotation_m = rotation.to_matrix()
        world = [transform @ Vector(point) for point in points]
        ned_world = [M @ point for point in world]
        origin = M @ Vector(location)
        rotation_ned = M @ rotation_m @ M
        local = [rotation_ned.transposed() @ (point - origin) for point in ned_world]
        stonefish_name = attr_string(prim, "stonefish_name") or prim.GetName()
        cls = attr_string(prim, "cls").casefold() or "scenery"
        convex = attr_string(prim, "convex").casefold() in {"1", "true"}
        fuzz = " ".join(attr_string(prim, "fuzz").casefold().split())
        records.append(
            {
                "prim": prim.GetPath().pathString,
                "name": stonefish_name,
                "cls": cls,
                "convex": convex,
                "fuzz": fuzz,
                "origin": origin,
                "quat": quat_from_matrix(rotation_ned),
                "local": local,
                "world": ned_world,
                "faces": triangulate(counts, indices),
            }
        )
    return records


def bounds(records, predicate):
    chosen = [record for record in records if predicate(record)]
    if not chosen:
        chosen = records
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for record in chosen:
        for point in record["world"]:
            for axis in range(3):
                minimum[axis] = min(minimum[axis], point[axis])
                maximum[axis] = max(maximum[axis], point[axis])
    return minimum, maximum


def write_xml(records):
    used = {}

    def unique(prefix):
        base = xml_name(prefix)
        name = base
        index = 2
        while name in used:
            name = f"{base}_{index}"
            index += 1
        used[name] = True
        return name

    mocap = []
    scenery = []
    for record in records:
        if record["fuzz"]:
            record["body"] = unique(record["name"].casefold())
            record["mesh"] = record["body"]
            mocap.append(record)
        else:
            record["mesh"] = unique(record["name"])
            scenery.append(record)
    mocap.sort(key=lambda record: record["body"])

    for record in records:
        write_obj(
            MESH_DIR / f"{record['mesh']}.obj",
            record["local"] if "body" in record else record["world"],
            record["faces"],
        )

    minimum, maximum = bounds(records, lambda record: record["name"].casefold() == "pool_tile")
    center = [(minimum[axis] + maximum[axis]) * 0.5 for axis in range(3)]
    half = [(maximum[axis] - minimum[axis]) * 0.5 for axis in range(3)]
    bottom = maximum[2]
    thickness = 0.3
    lines = [
        "<mujocoinclude>",
        "  <!-- Generated by export_pool_usd.py from pool_scene.usd. Do not edit by hand. -->",
        "  <asset>",
    ]
    for record in records:
        lines.append(f'    <mesh name="{record["mesh"]}" file="pool_usd/{record["mesh"]}.obj"/>')
    lines += [
        "  </asset>",
        "  <worldbody>",
        "    <!-- Basin collision. The tile meshes stay visual. -->",
        f'    <geom name="pool_floor" type="plane" pos="{fmt((center[0], center[1], bottom))}" '
        f'euler="3.141592653589793 0 0" size="{fmt((half[0], half[1], 0.1))}" rgba="0.45 0.45 0.45 1"/>',
    ]
    walls = (
        ("pool_wall_nx", (minimum[0] - thickness * 0.5, center[1], bottom * 0.5), (thickness * 0.5, half[1], bottom * 0.5)),
        ("pool_wall_px", (maximum[0] + thickness * 0.5, center[1], bottom * 0.5), (thickness * 0.5, half[1], bottom * 0.5)),
        ("pool_wall_ny", (center[0], minimum[1] - thickness * 0.5, bottom * 0.5), (half[0], thickness * 0.5, bottom * 0.5)),
        ("pool_wall_py", (center[0], maximum[1] + thickness * 0.5, bottom * 0.5), (half[0], thickness * 0.5, bottom * 0.5)),
    )
    for name, pos, size in walls:
        lines.append(
            f'    <geom name="{name}" type="box" pos="{fmt(pos)}" size="{fmt(size)}" rgba="0.55 0.55 0.55 1"/>'
        )
    for record in scenery:
        collide = ' contype="1" conaffinity="1"' if record["convex"] else ' contype="0" conaffinity="0"'
        lines.append(
            f'    <geom name="{record["mesh"]}" type="mesh" mesh="{record["mesh"]}"{collide} rgba="0.75 0.75 0.75 1"/>'
        )
    for record in mocap:
        body_name = record["body"]
        lines.append(
            f'    <body name="{body_name}" pos="{fmt(record["origin"])}" quat="{fmt(record["quat"])}" mocap="true">'
        )
        lines.append(
            f'      <geom name="{body_name}" type="mesh" mesh="{record["mesh"]}" contype="0" conaffinity="0" '
            'rgba="0.75 0.75 0.75 1"/>'
        )
        lines.append("    </body>")
    lines += ["  </worldbody>"]
    if mocap:
        lines += ["  <custom>"]
        for record in mocap:
            cls = record["cls"] if record["cls"] in {"cube", "rect", "gate"} else ""
            data = (record["fuzz"] + " " + cls).strip()
            lines.append(f'    <text name="fuzz:{record["body"]}" data="{data}"/>')
        lines.append("  </custom>")
    lines += ["</mujocoinclude>", ""]
    XML_PATH.write_text("\n".join(lines), encoding="utf-8")
    return mocap


def main():
    export_stage()
    stage = Usd.Stage.Open(str(USD_PATH))
    if stage is None:
        raise RuntimeError(f"could not open {USD_PATH}")
    stamp_stage(stage)
    stage.GetRootLayer().Save()
    stage = Usd.Stage.Open(str(USD_PATH))
    records = read_records(stage)
    if not records:
        raise RuntimeError("USD stage has no meshes")
    gate = next(record for record in records if record["cls"] == "gate")
    blender_gate = bpy.data.objects["Gate"].matrix_world.translation
    usd_origin = M @ gate["origin"]
    delta = (Vector(blender_gate) - usd_origin).length
    print(f"GATE_USD_DELTA {delta:.6f}")
    if delta > 1e-3:
        raise RuntimeError(f"USD gate origin disagrees with the blend by {delta} m")
    mocap = write_xml(records)
    print(f"WROTE {USD_PATH}")
    print(f"WROTE {XML_PATH} meshes={len(records)}")
    for record in mocap:
        origin = tuple(round(value, 4) for value in record["origin"])
        print(f"MOCAP {record['body']} {record['fuzz']} {origin}")


if __name__ == "__main__":
    main()
