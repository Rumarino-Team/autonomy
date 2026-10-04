"""Build a Stonefish world from a .blend file through the C ABI."""

import hashlib
import logging
import struct
from pathlib import Path

import yaml

from .blender_extractor import BlenderExtractor
from . import stonefish_c

logger = logging.getLogger("blender_stonefish")

# Fallback name, material, look, class, and convex flag for pool meshes that
# have not been stamped by the Blender addon. Geometry comes from the blend
# mesh block. The OBJ paths in this table are unused. Keep the tuples in sync
# with POOL_MESHES in blender_stonefish_addon/__init__.py.
KNOWN_MESHES = {
    "pool_tile": ("PoolShell", "ceramic", "pool_tile", "scenery", "models/pool_tile.obj", "models/pool_tile.obj", 0),
    "pool_deck": ("PoolDeck", "concrete", "deck", "scenery", "models/pool_deck.obj", "models/pool_deck.obj", 0),
    "pool_stainless": ("PoolFittings", "steel", "stainless", "scenery", "models/pool_stainless.obj", "models/pool_stainless.obj", 0),
    "pool_band": ("PoolBand", "ceramic", "pool_band", "scenery", "models/pool_buried.obj", "models/pool_band.obj", 0),
    "pool_coping": ("PoolCoping", "stone", "coping", "scenery", "models/pool_buried.obj", "models/pool_coping.obj", 0),
    "pool_light": ("PoolLights", "glass", "light", "scenery", "models/pool_buried.obj", "models/pool_light.obj", 0),
    "pool_glass": ("PoolWindow", "glass", "glass", "scenery", "models/pool_buried.obj", "models/pool_glass.obj", 0),
    "pool_hall": ("PoolHall", "concrete", "hall", "scenery", "models/pool_buried.obj", "models/pool_hall.obj", 0),
}


def _bytes(value) -> bytes | None:
    if value is None:
        return None
    text = str(value)
    if text == "":
        return None
    return text.encode("utf-8")


def _static(name, material, look, cls, xyz, rpy) -> stonefish_c.SfStatic:
    body = stonefish_c.SfStatic()
    body.name = _bytes(name)
    body.material = _bytes(material)
    body.look = _bytes(look)
    body.cls = _bytes(cls) if cls else None
    body.world = stonefish_c.pose(xyz, rpy)
    body.origin = stonefish_c.pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    return body


def _stonefish_pose(location, rotation):
    # Y matches the pool mesh. X is mirrored, and Z is mirrored because the
    # simulator pool OBJs have the basin above zero while Blender has it below.
    xyz = (-location[0], location[1], -location[2])
    rpy = (-rotation[0], rotation[1], -rotation[2])
    return xyz, rpy


def _identity_pose():
    return stonefish_c.pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _geometry_key(vertices, faces, face_uvs) -> bytes:
    """Hash of the exported triangles. Exact copies share one OBJ."""
    digest = hashlib.sha256()
    digest.update(struct.pack("<I", len(vertices)))
    for x, y, z in vertices:
        digest.update(struct.pack("<3d", float(x), float(y), float(z)))
    digest.update(struct.pack("<I", len(faces)))
    for face in faces:
        digest.update(struct.pack("<I", len(face)))
        if face:
            digest.update(struct.pack(f"<{len(face)}i", *face))
    digest.update(struct.pack("<I", len(face_uvs)))
    for uvs in face_uvs:
        digest.update(struct.pack("<I", len(uvs)))
        for u, v in uvs:
            digest.update(struct.pack("<2d", float(u), float(v)))
    return digest.digest()


class ScenarioBuilder:
    def __init__(self, config_path: str):
        with open(config_path, "r", encoding="utf-8") as handle:
            self.config = yaml.safe_load(handle)

    def build(self, lib, world, blend_file: str, data_dir: str) -> int:
        if self._materials(lib, world) != 0:
            return -1
        if self._looks(lib, world) != 0:
            return -1
        if self._friction(lib, world) != 0:
            return -1
        if self._environment(lib, world) != 0:
            return -1
        return self._statics(lib, world, blend_file, data_dir)

    def _materials(self, lib, world) -> int:
        for name, props in self.config["materials"].items():
            material = stonefish_c.SfMaterial(
                _bytes(name),
                float(props["density"]),
                float(props["restitution"]),
                float(props.get("magnetic", 0.0)),
            )
            if lib.sf_material(world, material) != 0:
                logger.error("material %s failed", name)
                return -1
        return 0

    def _looks(self, lib, world) -> int:
        for name, props in self.config["looks"].items():
            color = props["color"]
            look = stonefish_c.SfLook(
                _bytes(name),
                stonefish_c._vec3(color),
                float(props.get("roughness", 0.5)),
                float(props.get("metalness", 0.0)),
                float(props.get("reflectivity", 0.5)),
                _bytes(props.get("texture")),
                _bytes(props.get("normal_map")),
            )
            if lib.sf_look(world, look) != 0:
                logger.error("look %s failed", name)
                return -1
        return 0

    def _friction(self, lib, world) -> int:
        for pair in self.config["friction"]["pairs"]:
            mat1, mat2, static, dynamic = pair
            friction = stonefish_c.SfFriction(_bytes(mat1), _bytes(mat2), float(static), float(dynamic))
            if lib.sf_friction(world, friction) != 0:
                logger.error("friction %s/%s failed", mat1, mat2)
                return -1
        return 0

    def _environment(self, lib, world) -> int:
        env_config = self.config["environment"]
        ocean = env_config["ocean"]
        sun = env_config["sun"]
        ned = env_config["ned"]
        current = ocean["current"]["velocity"]
        environment = stonefish_c.SfEnvironment(
            float(ned["latitude"]),
            float(ned["longitude"]),
            float(ocean["water_density"]),
            float(ocean["jerlov"]),
            float(ocean["wave_height"]),
            float(env_config["atmosphere"]["temperature"]),
            stonefish_c._vec3(current),
            float(sun["azimuth"]),
            float(sun["elevation"]),
            1 if ocean.get("particles", True) else 0,
            0,
        )
        if not ocean.get("enabled", True):
            return 0
        if lib.sf_environment(world, environment) != 0:
            logger.error("environment failed")
            return -1
        return 0

    def _statics(self, lib, world, blend_file: str, data_dir: str) -> int:
        defaults = {
            "material": self.config.get("defaults", {}).get("material", "steel"),
            "look": self.config.get("defaults", {}).get("look", "gray"),
        }
        extractor = BlenderExtractor(blend_file, defaults)
        extracted = extractor.extract(also_include_names=KNOWN_MESHES.keys())
        mesh_dir = Path(data_dir) / "models" / "blender"
        mesh_dir.mkdir(parents=True, exist_ok=True)
        mesh_scale = float(self.config.get("meshes", {}).get("scale", 1.0))
        # geometry hash -> OBJ path already written this build
        mesh_cache: dict[bytes, Path] = {}

        for obj in extracted:
            xyz, rpy = _stonefish_pose(obj.location, obj.rotation)
            # Blender scale is local. Bake it into the vertices: Stonefish's
            # mesh scale is one number and cannot express a non-uniform scale.
            sx, sy, sz = (component * mesh_scale for component in obj.scale)
            known = KNOWN_MESHES.get(obj.name)
            if known is not None and not obj.stonefish_name:
                entity_name, material, look, cls, _physics, _visual, convex = known
            else:
                entity_name = obj.stonefish_name or obj.name
                material = obj.material
                look = obj.look
                cls = obj.cls
                convex = 1 if obj.convex else 0
            body = _static(entity_name, material, look, cls, xyz, rpy)
            if obj.mesh_data_block is None:
                logger.error("mesh %s has no data", obj.name)
                return -1
            triangles = extractor.read_mesh_triangles(
                obj.mesh_data_block, scale=(sx, sy, sz), transform_coords=True
            )
            if triangles is None:
                logger.error("failed to read %s", obj.name)
                return -1
            vertices, faces, face_uvs = triangles
            key = _geometry_key(vertices, faces, face_uvs)
            relative = mesh_cache.get(key)
            if relative is None:
                unit = abs(sx - mesh_scale) < 1e-9 and abs(sy - mesh_scale) < 1e-9 and abs(sz - mesh_scale) < 1e-9
                stem = (obj.mesh_name or obj.name) if unit else obj.name
                relative = Path("models") / "blender" / f"{stem}.obj"
                filename = Path(data_dir) / relative
                if not extractor.write_mesh_triangles(str(filename), vertices, faces, face_uvs):
                    logger.error("failed to export %s", obj.name)
                    return -1
                mesh_cache[key] = relative
            else:
                logger.info("reusing %s for %s", relative.as_posix(), entity_name)
            origin = _identity_pose()
            mesh = stonefish_c.SfMesh(
                _bytes(relative.as_posix()),
                1.0,
                origin,
                None,
                1.0,
                origin,
                int(convex),
                0,
            )
            status = lib.sf_static_mesh(world, body, mesh)
            if status != 0:
                logger.error("static %s failed", entity_name)
                return -1
        logger.info("built %d statics from %s", len(extracted), blend_file)
        return 0
