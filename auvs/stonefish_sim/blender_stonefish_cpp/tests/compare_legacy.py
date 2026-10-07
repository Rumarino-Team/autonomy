"""Regression checks only: the native plugin never imports this Python code.

Compare native geometry to the existing reader on the real pool scene, then
exercise mirrored scale, compressed input and rejected file/transform formats.
"""
import ctypes
import gzip
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "auvs/stonefish_sim"))
from blender_stonefish.blender_extractor import BlenderExtractor
from blender_stonefish.builder import KNOWN_MESHES, _stonefish_pose

INSPECT = Path(sys.argv[1]).resolve()
BLEND = ROOT / "auvs/stonefish_sim/data/pool_scene.blend"
CONFIG = ROOT / "auvs/stonefish_sim/blender_stonefish/config.yaml"


def run(path, export=None, ok=True):
    args = [str(INSPECT), str(path), str(CONFIG)]
    if export is not None:
        args.append(str(export))
    result = subprocess.run(args, text=True, capture_output=True)
    assert (result.returncode == 0) == ok, result.stderr
    return json.loads(result.stdout) if ok else result.stderr


def obj_lines(path):
    return [line.split() for line in path.read_text().splitlines()
            if line.startswith(("v ", "vn ", "vt ", "f "))]


def compare(path, temporary):
    export = temporary / "native"
    native = run(path, export)
    extractor = BlenderExtractor(str(path))
    objects = extractor.extract(KNOWN_MESHES.keys())
    assert len(native) == len(objects) == 20
    for old, new in zip(objects, native):
        if old.name in KNOWN_MESHES and not old.stonefish_name:
            name, material, look, cls, *_ = KNOWN_MESHES[old.name]
        else:
            name, material, look, cls = old.stonefish_name or old.name, old.material, old.look, old.cls
        assert (name, material, look, cls) == (new["name"], new["material"], new["look"], new["cls"])
        xyz, rpy = _stonefish_pose(old.location, old.rotation)
        assert list(xyz) == new["position"] and list(rpy) == new["rotation"]
        assert list(old.scale) == new["scale"] and old.convex == new["convex"]
        vertices, faces, face_uvs = extractor.read_mesh_triangles(old.mesh_data_block, old.scale, True)
        assert len(vertices) == new["vertices"] and len(faces) == new["faces"]
        assert sum(map(len, face_uvs)) == new["uv_corners"]
        expected = temporary / "legacy" / f"{name}.obj"
        assert extractor.write_mesh_triangles(str(expected), vertices, faces, face_uvs)
        old_lines, new_lines = obj_lines(expected), obj_lines(export / f"{name}.obj")
        assert len(old_lines) == len(new_lines), name
        for a, b in zip(old_lines, new_lines):
            assert a[0] == b[0] and len(a) == len(b), name
            if a[0] == "f":
                assert a == b, (name, a, b)
            else:
                assert all(math.isclose(float(x), float(y), abs_tol=1.1e-6) for x, y in zip(a[1:], b[1:])), (name, a, b)
    extractor.blend.close()
    return native


with tempfile.TemporaryDirectory(prefix="stonefish-blend-parity-") as tmp:
    tmp = Path(tmp)
    baseline = compare(BLEND, tmp / "pool")
    original = BLEND.read_bytes()
    compressed = tmp / "gzip.blend"
    compressed.write_bytes(gzip.compress(original))
    assert run(compressed) == baseline

    zstd = ctypes.CDLL("libzstd.so.1")
    zstd.ZSTD_compressBound.argtypes = [ctypes.c_size_t]
    zstd.ZSTD_compressBound.restype = ctypes.c_size_t
    zstd.ZSTD_compress.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
    zstd.ZSTD_compress.restype = ctypes.c_size_t
    buffer = ctypes.create_string_buffer(zstd.ZSTD_compressBound(len(original)))
    size = zstd.ZSTD_compress(buffer, len(buffer), original, len(original), 3)
    compressed = tmp / "zstd.blend"
    compressed.write_bytes(buffer.raw[:size])
    assert run(compressed) == baseline
    # Blender can write multiple compressed frames.
    compressed.write_bytes(buffer.raw[:size] + buffer.raw[:size])
    assert "CBlend could not parse" in run(compressed, ok=False)

    extractor = BlenderExtractor(str(BLEND))
    extractor.extract(KNOWN_MESHES.keys())
    first = extractor.blend.find_blocks_from_code(b"OB")[0]
    mirrored = bytearray(original)
    offset, _ = first.get_file_offset(b"size")
    struct.pack_into("<3f", mirrored, offset, -2, 0.5, 3)
    changed = tmp / "mirrored.blend"
    changed.write_bytes(mirrored)
    compare(changed, tmp / "mirror")

    offset, _ = first.get_file_offset(b"rotmode")
    unsupported = bytearray(original)
    struct.pack_into("<h", unsupported, offset, 0)
    changed.write_bytes(unsupported)
    assert "XYZ Euler" in run(changed, ok=False)
    offset, _ = first.get_file_offset(b"parent")
    unsupported = bytearray(original)
    struct.pack_into("<Q", unsupported, offset, first.addr_old)
    changed.write_bytes(unsupported)
    assert "parent transform" in run(changed, ok=False)
    extractor.blend.close()

    changed.write_bytes(b"not a blend")
    run(changed, ok=False)
    changed.write_bytes(original[:512])
    run(changed, ok=False)
    changed.write_bytes(original[:9] + b"500" + original[12:])
    assert "version is 500" in run(changed, ok=False)

print("PASS: all 20 objects match legacy metadata, geometry, UVs, normals and winding; mirrored scale, gzip/zstd and error cases passed")
