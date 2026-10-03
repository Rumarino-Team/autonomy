"""Blender scene builder for the Stonefish C ABI."""

import logging

from .builder import ScenarioBuilder
from . import stonefish_c

logger = logging.getLogger("blender_stonefish")


def build(world_ptr: int, library_path: str, blend_path: str, config_path: str, data_dir: str) -> int:
    """Fill an existing SfWorld from a .blend file. Returns 0 on success."""
    try:
        lib = stonefish_c.load(library_path)
        world = stonefish_c.c_void_p(world_ptr)
        return ScenarioBuilder(config_path).build(lib, world, blend_path, data_dir)
    except Exception:
        logger.exception("blend build failed")
        return -1
