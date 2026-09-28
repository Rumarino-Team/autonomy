"""ctypes binding for libhydrus_mujoco_rl.so (auvs/hydrus_mujoco/hydrus_batch.h)."""

from __future__ import annotations

import ctypes
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
DEFAULT_LIB = REPO / "zig-out" / "lib" / "libhydrus_mujoco_rl.so"
DEFAULT_XML = REPO / "auvs" / "hydrus_mujoco" / "hydrus.xml"

THRUSTERS = 8
INIT_SIZE = 13
RANDOMIZATION_SIZE = 8
STATE_SIZE = 21
CAM_W = 96
CAM_H = 72
CAM_BOXES = 4
CAM_BOX_STRIDE = 7
CAM_EDGES = 48
CAM_FLOW = 96
CAM_FEATURE_SIZE = CAM_BOXES * CAM_BOX_STRIDE + CAM_EDGES + CAM_FLOW
MOCAP_POSE = 7
BOXES = slice(0, CAM_BOXES * CAM_BOX_STRIDE)
EDGES = slice(BOXES.stop, BOXES.stop + CAM_EDGES)
FLOW = slice(EDGES.stop, EDGES.stop + CAM_FLOW)

# Column slices into the state rows returned by step().
POS = slice(0, 3)
QUAT = slice(3, 7)  # xyzw
LIN_VEL = slice(7, 10)  # world
ANG_VEL = slice(10, 13)  # world
GYRO = slice(13, 16)  # body
ACCEL = slice(16, 19)
DEPTH = 19
TIME = 20

_f64p = ctypes.POINTER(ctypes.c_double)
_f32p = ctypes.POINTER(ctypes.c_float)
_u8p = ctypes.POINTER(ctypes.c_uint8)


def _load(path: Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))
    lib.hydrus_batch_create.restype = ctypes.c_void_p
    lib.hydrus_batch_create.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
    lib.hydrus_batch_destroy.argtypes = [ctypes.c_void_p]
    lib.hydrus_batch_num_envs.restype = ctypes.c_int
    lib.hydrus_batch_num_envs.argtypes = [ctypes.c_void_p]
    lib.hydrus_batch_timestep.restype = ctypes.c_double
    lib.hydrus_batch_timestep.argtypes = [ctypes.c_void_p]
    lib.hydrus_batch_reset.argtypes = [ctypes.c_void_p, _u8p, _f64p, _f64p]
    lib.hydrus_batch_step.argtypes = [ctypes.c_void_p, _f32p, ctypes.c_int, _f64p]
    lib.hydrus_batch_get_state.argtypes = [ctypes.c_void_p, _f64p]
    lib.hydrus_batch_mocap_count.restype = ctypes.c_int
    lib.hydrus_batch_mocap_count.argtypes = [ctypes.c_void_p]
    lib.hydrus_batch_set_mocap.argtypes = [ctypes.c_void_p, _f64p]
    lib.hydrus_batch_camera.argtypes = [ctypes.c_void_p, _f32p]
    lib.hydrus_batch_camera_image.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_uint8)]
    return lib


def _ptr(array: np.ndarray | None, ctype):
    if array is None:
        return None
    return array.ctypes.data_as(ctypes.POINTER(ctype))


class HydrusBatch:
    """num_envs Hydrus vehicles with the Stonefish-matched physics, stepped in C++ threads."""

    def __init__(
        self,
        num_envs: int,
        num_threads: int = 0,
        xml_path: Path | str = DEFAULT_XML,
        lib_path: Path | str = DEFAULT_LIB,
    ):
        self._lib = _load(Path(lib_path))
        self._handle = self._lib.hydrus_batch_create(str(xml_path).encode(), num_envs, num_threads)
        if not self._handle:
            raise RuntimeError(f"hydrus_batch_create failed for {xml_path}")
        self.num_envs = num_envs
        self.timestep = self._lib.hydrus_batch_timestep(self._handle)
        self.mocap_count = self._lib.hydrus_batch_mocap_count(self._handle)
        self._state = np.zeros((num_envs, STATE_SIZE), dtype=np.float64)
        self._features = np.zeros((num_envs, CAM_FEATURE_SIZE), dtype=np.float32)

    def close(self) -> None:
        if self._handle:
            self._lib.hydrus_batch_destroy(self._handle)
            self._handle = None

    def __del__(self):
        self.close()

    def reset(
        self,
        mask: np.ndarray | None = None,
        init_state: np.ndarray | None = None,
        randomization: np.ndarray | None = None,
    ) -> np.ndarray:
        """init_state rows: pos, quat xyzw, lin_vel, ang_vel (world).
        randomization rows: dry_mass, volume, drag, thrust, rotor_inertia scales, current xyz."""
        m = None if mask is None else np.ascontiguousarray(mask, dtype=np.uint8)
        s = None if init_state is None else np.ascontiguousarray(init_state, dtype=np.float64)
        r = None if randomization is None else np.ascontiguousarray(randomization, dtype=np.float64)
        if s is not None:
            assert s.shape == (self.num_envs, INIT_SIZE)
        if r is not None:
            assert r.shape == (self.num_envs, RANDOMIZATION_SIZE)
        self._lib.hydrus_batch_reset(self._handle, _ptr(m, ctypes.c_uint8), _ptr(s, ctypes.c_double), _ptr(r, ctypes.c_double))
        self._lib.hydrus_batch_get_state(self._handle, _ptr(self._state, ctypes.c_double))
        return self._state.copy()

    def step(self, thrusters: np.ndarray, substeps: int = 1) -> np.ndarray:
        cmd = np.ascontiguousarray(thrusters, dtype=np.float32)
        assert cmd.shape == (self.num_envs, THRUSTERS)
        self._lib.hydrus_batch_step(self._handle, _ptr(cmd, ctypes.c_float), substeps, _ptr(self._state, ctypes.c_double))
        return self._state.copy()

    def set_mocap(self, poses: np.ndarray) -> None:
        """poses: (num_envs, mocap_count, 7) xyz + quaternion xyzw. Gate is mocap 0, marker is mocap 1."""
        mocap = np.ascontiguousarray(poses, dtype=np.float64)
        assert mocap.shape == (self.num_envs, self.mocap_count, MOCAP_POSE)
        self._lib.hydrus_batch_set_mocap(self._handle, _ptr(mocap, ctypes.c_double))

    def camera(self) -> np.ndarray:
        """YOLO boxes, pooled Sobel edges, and pooled optical flow. Shape (num_envs, CAM_FEATURE_SIZE)."""
        self._lib.hydrus_batch_camera(self._handle, _ptr(self._features, ctypes.c_float))
        return self._features.copy()

    def camera_image(self, env: int = 0) -> np.ndarray:
        """Last 96x72 raster for one env, after camera()."""
        pixels = np.zeros(CAM_W * CAM_H, dtype=np.uint8)
        self._lib.hydrus_batch_camera_image(self._handle, env, _ptr(pixels, ctypes.c_uint8))
        return pixels.reshape(CAM_H, CAM_W)

    def state(self) -> np.ndarray:
        self._lib.hydrus_batch_get_state(self._handle, _ptr(self._state, ctypes.c_double))
        return self._state.copy()


if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description="Measure batched Hydrus physics throughput.")
    parser.add_argument("--envs", type=int, nargs="+", default=[1, 16, 64, 256])
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0, help="simulated seconds per env")
    args = parser.parse_args()

    for n in args.envs:
        batch = HydrusBatch(n, args.threads)
        batch.reset()
        rng = np.random.default_rng(0)
        policy_steps = int(args.seconds * 60)
        cmd = rng.uniform(-0.3, 0.3, size=(n, THRUSTERS)).astype(np.float32)
        t0 = time.perf_counter()
        for _ in range(policy_steps):
            state = batch.step(cmd, substeps=6)
        wall = time.perf_counter() - t0
        physics = n * policy_steps * 6
        print(
            f"envs={n:4d}  {physics / wall:10.0f} physics steps/s  {n * policy_steps / wall:9.0f} policy steps/s  "
            f"{n * args.seconds / wall:7.1f}x realtime  finite={np.isfinite(state).all()}"
        )
        batch.close()
