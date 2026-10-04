"""ctypes layout for auvs/stonefish_sim/stonefish_c/include/stonefish_c.h."""

import ctypes
from ctypes import CDLL, c_char_p, c_double, c_int, c_void_p


class SfPose(ctypes.Structure):
    _fields_ = [
        ("xyz", c_double * 3),
        ("rpy", c_double * 3),
    ]


class SfMaterial(ctypes.Structure):
    _fields_ = [
        ("name", c_char_p),
        ("density", c_double),
        ("restitution", c_double),
        ("magnetic", c_double),
    ]


class SfLook(ctypes.Structure):
    _fields_ = [
        ("name", c_char_p),
        ("rgb", c_double * 3),
        ("roughness", c_double),
        ("metalness", c_double),
        ("reflectivity", c_double),
        ("texture", c_char_p),
        ("normal_map", c_char_p),
    ]


class SfFriction(ctypes.Structure):
    _fields_ = [
        ("material1", c_char_p),
        ("material2", c_char_p),
        ("static_friction", c_double),
        ("dynamic_friction", c_double),
    ]


class SfEnvironment(ctypes.Structure):
    _fields_ = [
        ("ned_latitude", c_double),
        ("ned_longitude", c_double),
        ("water_density", c_double),
        ("jerlov", c_double),
        ("wave_height", c_double),
        ("water_temperature", c_double),
        ("current_xyz", c_double * 3),
        ("sun_azimuth_deg", c_double),
        ("sun_elevation_deg", c_double),
        ("particles", c_int),
        ("pad0", c_int),
    ]


class SfStatic(ctypes.Structure):
    _fields_ = [
        ("name", c_char_p),
        ("material", c_char_p),
        ("look", c_char_p),
        ("cls", c_char_p),
        ("world", SfPose),
        ("origin", SfPose),
    ]


class SfMesh(ctypes.Structure):
    _fields_ = [
        ("physics_path", c_char_p),
        ("physics_scale", c_double),
        ("physics_origin", SfPose),
        ("visual_path", c_char_p),
        ("visual_scale", c_double),
        ("visual_origin", SfPose),
        ("convex", c_int),
        ("pad0", c_int),
    ]


def _vec3(values):
    return (c_double * 3)(*values)


def pose(xyz, rpy) -> SfPose:
    value = SfPose()
    value.xyz = _vec3(xyz)
    value.rpy = _vec3(rpy)
    return value


def load(library_path: str):
    lib = CDLL(library_path)
    lib.sf_material.argtypes = [c_void_p, ctypes.POINTER(SfMaterial)]
    lib.sf_material.restype = c_int
    lib.sf_look.argtypes = [c_void_p, ctypes.POINTER(SfLook)]
    lib.sf_look.restype = c_int
    lib.sf_friction.argtypes = [c_void_p, ctypes.POINTER(SfFriction)]
    lib.sf_friction.restype = c_int
    lib.sf_environment.argtypes = [c_void_p, ctypes.POINTER(SfEnvironment)]
    lib.sf_environment.restype = c_int
    lib.sf_static_plane.argtypes = [c_void_p, ctypes.POINTER(SfStatic), c_double]
    lib.sf_static_plane.restype = c_int
    lib.sf_static_box.argtypes = [c_void_p, ctypes.POINTER(SfStatic), ctypes.POINTER(c_double)]
    lib.sf_static_box.restype = c_int
    lib.sf_static_cylinder.argtypes = [c_void_p, ctypes.POINTER(SfStatic), c_double, c_double]
    lib.sf_static_cylinder.restype = c_int
    lib.sf_static_sphere.argtypes = [c_void_p, ctypes.POINTER(SfStatic), c_double]
    lib.sf_static_sphere.restype = c_int
    lib.sf_static_mesh.argtypes = [c_void_p, ctypes.POINTER(SfStatic), ctypes.POINTER(SfMesh)]
    lib.sf_static_mesh.restype = c_int
    return lib
