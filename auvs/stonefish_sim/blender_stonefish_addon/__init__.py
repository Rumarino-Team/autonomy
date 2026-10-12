"""Blender panel that marks objects as Stonefish statics.

Values are written as ID properties (obj["stonefish"] = "1"). The simulator
reads those from the .blend file. Addon RNA on its own is invisible to that reader.
"""

import bpy
from pathlib import Path

bl_info = {
    "name": "Stonefish Objects",
    "author": "Rumarino",
    "version": (1, 0, 0),
    "blender": (4, 5, 0),
    "location": "Properties > Object > Stonefish",
    "description": "Mark Blender objects as Stonefish statics for the autonomy simulator",
    "category": "Object",
}

# Fallback metadata for pool meshes not stamped with custom properties.
# (stonefish name, material, look, class, physics obj, visual obj, convex)
POOL_MESHES = {
    "pool_tile": ("PoolShell", "ceramic", "pool_tile", "scenery", "models/pool_tile.obj", "models/pool_tile.obj", False),
    "pool_deck": ("PoolDeck", "concrete", "deck", "scenery", "models/pool_deck.obj", "models/pool_deck.obj", False),
    "pool_stainless": ("PoolFittings", "steel", "stainless", "scenery", "models/pool_stainless.obj", "models/pool_stainless.obj", False),
    "pool_band": ("PoolBand", "ceramic", "pool_band", "scenery", "models/pool_buried.obj", "models/pool_band.obj", False),
    "pool_coping": ("PoolCoping", "stone", "coping", "scenery", "models/pool_buried.obj", "models/pool_coping.obj", False),
    "pool_light": ("PoolLights", "glass", "light", "scenery", "models/pool_buried.obj", "models/pool_light.obj", False),
    "pool_glass": ("PoolWindow", "glass", "glass", "scenery", "models/pool_buried.obj", "models/pool_glass.obj", False),
    "pool_hall": ("PoolHall", "concrete", "hall", "scenery", "models/pool_buried.obj", "models/pool_hall.obj", False),
}

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

_syncing = set()
_catalog = None


def _config_path():
    return Path(__file__).resolve().parent.parent / "stonefish_config.yaml"


def _catalog_lists():
    global _catalog
    if _catalog is not None:
        return _catalog
    materials = []
    looks = []
    section = None
    path = _config_path()
    if path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.startswith("materials:"):
                section = "materials"
                continue
            if line.startswith("looks:"):
                section = "looks"
                continue
            if line and not line.startswith((" ", "#")):
                section = None
                continue
            stripped = line.strip()
            if section and stripped.endswith(":") and not stripped.startswith(("-", "#")):
                name = stripped[:-1].strip()
                if section == "materials":
                    materials.append(name)
                else:
                    looks.append(name)
    if not materials:
        materials = ["steel"]
    if not looks:
        looks = ["gray"]
    _catalog = (materials, looks)
    return _catalog


def _enum_items(names):
    return [(name, name, "") for name in names]


def _material_items(self, context):
    return _enum_items(_catalog_lists()[0])


def _look_items(self, context):
    return _enum_items(_catalog_lists()[1])


def _class_items(self, context):
    return [
        ("none", "None", "Not a tracked detection"),
        ("scenery", "Scenery", "Static scenery, not tracked"),
        ("gate", "Gate", "Tracked gate"),
        ("cube", "Cube", "Tracked cube"),
        ("rect", "Rectangle", "Tracked rectangle"),
    ]


def _write_id_properties(self, context):
    obj = self.id_data
    if obj is None or not isinstance(obj, bpy.types.Object):
        return
    if obj.as_pointer() in _syncing:
        return
    if not self.enabled:
        for key in ID_KEYS:
            if key in obj:
                del obj[key]
        return
    obj["stonefish"] = "1"
    obj["stonefish_name"] = self.stonefish_name or obj.name
    obj["material"] = self.material
    obj["look"] = self.look
    if self.cls_name == "none":
        if "cls" in obj:
            del obj["cls"]
    else:
        obj["cls"] = self.cls_name
    if self.physics_mesh:
        obj["physics_mesh"] = self.physics_mesh
        obj["visual_mesh"] = self.visual_mesh
        obj["convex"] = "true" if self.convex else "false"
    else:
        for key in ("physics_mesh", "visual_mesh", "convex"):
            if key in obj:
                del obj[key]


def _pull_from_id_properties(obj):
    settings = obj.stonefish_settings
    if settings.pulled:
        return
    materials, looks = _catalog_lists()
    _syncing.add(obj.as_pointer())
    try:
        flag = str(obj.get("stonefish", "")).strip().lower()
        settings.enabled = flag in ("1", "true")
        settings.stonefish_name = str(obj.get("stonefish_name", obj.name))
        material = str(obj.get("material", materials[0]))
        settings.material = material if material in materials else materials[0]
        look = str(obj.get("look", looks[0]))
        settings.look = look if look in looks else looks[0]
        cls_name = str(obj.get("cls", "none")).strip().lower()
        settings.cls_name = cls_name if cls_name in ("scenery", "gate", "cube", "rect") else "none"
        settings.physics_mesh = str(obj.get("physics_mesh", ""))
        settings.visual_mesh = str(obj.get("visual_mesh", ""))
        settings.convex = str(obj.get("convex", "")).strip().lower() in ("1", "true")
        settings.pulled = True
    finally:
        _syncing.discard(obj.as_pointer())


class StonefishSettings(bpy.types.PropertyGroup):
    enabled: bpy.props.BoolProperty(
        name="Include in simulation",
        description="Write this object into the Stonefish world when the simulator starts",
        default=False,
        update=_write_id_properties,
    )
    stonefish_name: bpy.props.StringProperty(
        name="Stonefish name",
        description="Entity name in the simulator. Defaults to the Blender object name",
        update=_write_id_properties,
    )
    material: bpy.props.EnumProperty(
        name="Material",
        items=_material_items,
        update=_write_id_properties,
    )
    look: bpy.props.EnumProperty(
        name="Look",
        items=_look_items,
        update=_write_id_properties,
    )
    cls_name: bpy.props.EnumProperty(
        name="Class",
        items=_class_items,
        default=0,
        update=_write_id_properties,
    )
    physics_mesh: bpy.props.StringProperty(
        name="Physics OBJ",
        description="Path relative to auvs/stonefish_sim/data/. Empty exports the Blender mesh at startup",
        update=_write_id_properties,
    )
    visual_mesh: bpy.props.StringProperty(
        name="Visual OBJ",
        description="Optional visual mesh. Empty uses the physics OBJ",
        update=_write_id_properties,
    )
    convex: bpy.props.BoolProperty(
        name="Convex collision",
        default=False,
        update=_write_id_properties,
    )
    pulled: bpy.props.BoolProperty(default=False, options={"HIDDEN"})


class STONEFISH_PT_object(bpy.types.Panel):
    bl_label = "Stonefish"
    bl_idname = "STONEFISH_PT_object"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"

    def draw(self, context):
        obj = context.object
        layout = self.layout
        if obj is None:
            layout.label(text="Select an object")
            return
        _pull_from_id_properties(obj)
        settings = obj.stonefish_settings
        layout.prop(settings, "enabled")
        col = layout.column()
        col.enabled = settings.enabled
        col.prop(settings, "stonefish_name")
        col.prop(settings, "material")
        col.prop(settings, "look")
        col.prop(settings, "cls_name")
        col.separator()
        col.prop(settings, "physics_mesh")
        col.prop(settings, "visual_mesh")
        col.prop(settings, "convex")
        if not settings.physics_mesh:
            col.label(text="Empty OBJ paths export this mesh at startup")
        layout.separator()
        layout.operator("stonefish.stamp_pool", icon="MESH_DATA")


class STONEFISH_OT_stamp_pool(bpy.types.Operator):
    bl_idname = "stonefish.stamp_pool"
    bl_label = "Stamp pool objects"
    bl_description = "Fill Stonefish properties on the eight pool meshes from the current pool mapping"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        stamped = 0
        for obj in context.scene.objects:
            spec = POOL_MESHES.get(obj.name)
            if spec is None:
                continue
            stonefish_name, material, look, cls_name, physics, visual, convex = spec
            obj["stonefish"] = "1"
            obj["stonefish_name"] = stonefish_name
            obj["material"] = material
            obj["look"] = look
            obj["cls"] = cls_name
            obj["physics_mesh"] = physics
            obj["visual_mesh"] = visual
            obj["convex"] = "true" if convex else "false"
            obj.stonefish_settings.pulled = False
            stamped += 1
        self.report({"INFO"}, f"Stamped {stamped} pool objects")
        return {"FINISHED"}


classes = (
    StonefishSettings,
    STONEFISH_PT_object,
    STONEFISH_OT_stamp_pool,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Object.stonefish_settings = bpy.props.PointerProperty(type=StonefishSettings)


def unregister():
    del bpy.types.Object.stonefish_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
