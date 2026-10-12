"""Pose-holding task on the batched Hydrus physics, as a Stable-Baselines3 VecEnv.

The policy replaces the global-navigation PID. Every observation term is computed from
what src/navigation/global.zig sees (frame pose and timestamp) so src/policy.zig can
rebuild it bit for bit; keep build_observation() and policy.zig in sync.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import gymnasium as gym
import numpy as np
from stable_baselines3.common.vec_env import VecEnv

import auv_mujoco as hm

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO / "auvs" / "hydrus_mujoco" / "auv.json"

PHYSICS_HZ = 360
POLICY_HZ = 60
SUBSTEPS = PHYSICS_HZ // POLICY_HZ
POLICY_DT = SUBSTEPS / PHYSICS_HZ

OBS_SIZE = 20
ACT_SIZE = 6
# Same as MissionContext.close_enough: farther than this in xy, the heading target points
# at the goal; closer, it is the goal yaw.
CLOSE_ENOUGH = 1.0
# Horizontal goal error fed to the policy is clipped to this length (far goals look like a
# goal this far away in the same direction); depth error is clipped to ± this separately so
# it stays visible during long transits.
MAX_GOAL_DIST = 2.0
# MissionContext.thrustor_saturate / thrustor_output_scale.
THRUSTOR_SATURATE = 5.0
THRUSTOR_OUTPUT_SCALE = 1.0

SURFACE_LIMIT = 0.2
FLOOR_Z = 3.0
TILT_LIMIT = 0.3  # terminate when body +Z (down) has less than this down component


def load_config(path: Path | str = DEFAULT_CONFIG) -> dict:
    with open(path) as f:
        cfg = json.load(f)
    cfg["tam"] = np.asarray(cfg["tam"], dtype=np.float64)
    # Global-navigation PID gains live under "odometry" (src/ConfigLoader.zig).
    gains = cfg["odometry"] if "odometry" in cfg else cfg
    for key in ("kp", "ki", "kd"):
        cfg[key] = np.asarray(gains[key], dtype=np.float64)
    return cfg


# Quaternions are xyzw, as in include/math.h and src/math.zig.
def quat_conj(q: np.ndarray) -> np.ndarray:
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    av, aw = a[..., :3], a[..., 3:4]
    bv, bw = b[..., :3], b[..., 3:4]
    xyz = aw * bv + bw * av + np.cross(av, bv)
    w = aw * bw - np.sum(av * bv, axis=-1, keepdims=True)
    return np.concatenate([xyz, w], axis=-1)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv, qw = q[..., :3], q[..., 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + qw * t + np.cross(qv, t)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def wrap_angle(a: np.ndarray) -> np.ndarray:
    return np.mod(a + np.pi, 2.0 * np.pi) - np.pi


def heading(q: np.ndarray) -> np.ndarray:
    """Heading of the nose (body +Y) in the world xy plane, as in global.zig pidStep."""
    fwd = quat_rotate(q, np.broadcast_to([0.0, 1.0, 0.0], q[..., :3].shape))
    return np.arctan2(fwd[..., 1], fwd[..., 0])


def quat_from_heading(psi: np.ndarray, roll: np.ndarray, pitch: np.ndarray) -> np.ndarray:
    """Nose heading psi, then small roll (about body Y) and pitch (about body X)."""
    yaw = psi - np.pi / 2.0
    qz = np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], axis=-1)
    qy = np.stack([np.zeros_like(roll), np.sin(roll / 2), np.zeros_like(roll), np.cos(roll / 2)], axis=-1)
    qx = np.stack([np.sin(pitch / 2), np.zeros_like(pitch), np.zeros_like(pitch), np.cos(pitch / 2)], axis=-1)
    return quat_normalize(quat_mul(quat_mul(qz, qy), qx))


def yaw_error(pos: np.ndarray, quat: np.ndarray, goal: np.ndarray) -> np.ndarray:
    """global.zig pidStep: face the goal while far, then turn to the goal yaw."""
    d = goal[..., :2] - pos[..., :2]
    far = np.linalg.norm(d, axis=-1) > CLOSE_ENOUGH
    target = np.where(far, np.arctan2(d[..., 1], d[..., 0]), goal[..., 3])
    return wrap_angle(target - heading(quat))


def build_observation(
    pos: np.ndarray,
    quat: np.ndarray,
    prev_pos: np.ndarray,
    prev_quat: np.ndarray,
    dt: float | np.ndarray,
    goal: np.ndarray,
    prev_action: np.ndarray,
) -> np.ndarray:
    """goal rows are (x, y, z, yaw). Layout (all body frame):
    [0:3] goal position error, world xy length and z each clipped to MAX_GOAL_DIST
    [3:6] world down axis (0, 0, 1)
    [6:8] sin, cos of the heading error
    [8:11] linear velocity from the pose difference
    [11:14] angular velocity from the quaternion difference
    [14:20] previous action
    """
    quat = quat_normalize(quat)
    prev_quat = quat_normalize(prev_quat)
    inv = quat_conj(quat)
    world_err = goal[..., :3] - pos
    n = np.linalg.norm(world_err[..., :2], axis=-1, keepdims=True)
    world_err = np.concatenate(
        [
            world_err[..., :2] * np.minimum(1.0, MAX_GOAL_DIST / np.maximum(n, 1e-9)),
            np.clip(world_err[..., 2:3], -MAX_GOAL_DIST, MAX_GOAL_DIST),
        ],
        axis=-1,
    )
    err = quat_rotate(inv, world_err)
    down = quat_rotate(inv, np.broadcast_to([0.0, 0.0, 1.0], pos.shape))
    ye = yaw_error(pos, quat, goal)
    dt = np.asarray(dt, dtype=np.float64)[..., None] if np.ndim(dt) else dt
    vel = quat_rotate(inv, (pos - prev_pos)) / dt
    rel = quat_mul(quat_conj(prev_quat), quat)
    sign = np.where(rel[..., 3:4] < 0, -1.0, 1.0)
    ang = 2.0 * sign * rel[..., :3] / dt
    return np.concatenate(
        [err, down, np.sin(ye)[..., None], np.cos(ye)[..., None], vel, ang, prev_action], axis=-1
    ).astype(np.float32)


def action_to_thrusters(action: np.ndarray, tam: np.ndarray) -> np.ndarray:
    """controls.tamToThrusters on the raw action; the plugin then clamps to [-1, 1]."""
    t = np.clip(action, -1.0, 1.0) @ tam.T
    return (np.clip(t, -THRUSTOR_SATURATE, THRUSTOR_SATURATE) / THRUSTOR_OUTPUT_SCALE).astype(np.float32)


@dataclass
class TaskConfig:
    episode_seconds: float = 15.0
    xy_range: float = 4.0
    z_range: tuple[float, float] = (0.7, 2.4)
    goal_dist: tuple[float, float] = (0.0, 6.0)
    init_tilt: float = 0.15
    init_speed: float = 0.2
    init_ang_speed: float = 0.2
    # Chance per episode of moving the goal once, at a random time.
    goal_switch_prob: float = 0.5
    randomize: bool = True
    dry_mass_scale: tuple[float, float] = (0.97, 1.03)
    volume_scale: tuple[float, float] = (0.97, 1.03)
    drag_scale: tuple[float, float] = (0.7, 1.4)
    thrust_scale: tuple[float, float] = (0.8, 1.2)
    rotor_inertia_scale: tuple[float, float] = (0.7, 1.5)
    current_speed: float = 0.15
    reward_weights: dict = field(
        default_factory=lambda: {
            "pos": 1.0,
            "pos_fine": 0.5,
            "yaw": 0.5,
            "yaw_fine": 0.25,
            "tilt": 2.0,
            "action": 0.01,
            "action_rate": 0.05,
            "terminate": 10.0,
        }
    )


def sample_scenarios(rng: np.random.Generator, n: int, task: TaskConfig):
    """Initial states (auv_batch init rows), goals (x, y, z, yaw), randomization rows."""
    pos = np.column_stack(
        [
            rng.uniform(-task.xy_range, task.xy_range, n),
            rng.uniform(-task.xy_range, task.xy_range, n),
            rng.uniform(*task.z_range, n),
        ]
    )
    quat = quat_from_heading(
        rng.uniform(-np.pi, np.pi, n),
        rng.uniform(-task.init_tilt, task.init_tilt, n),
        rng.uniform(-task.init_tilt, task.init_tilt, n),
    )
    lin = rng.normal(0, task.init_speed / np.sqrt(3), (n, 3))
    ang = rng.normal(0, task.init_ang_speed / np.sqrt(3), (n, 3))
    init = np.concatenate([pos, quat, lin, ang], axis=1)
    goals = sample_goals(rng, pos, task)
    return init, goals, sample_randomization(rng, n, task)


def sample_goals(rng: np.random.Generator, pos: np.ndarray, task: TaskConfig) -> np.ndarray:
    n = pos.shape[0]
    direction = rng.normal(size=(n, 3))
    direction /= np.linalg.norm(direction, axis=1, keepdims=True)
    dist = rng.uniform(*task.goal_dist, n)[:, None]
    goal = pos + direction * dist
    goal[:, 2] = np.clip(goal[:, 2], *task.z_range)
    return np.column_stack([goal, rng.uniform(-np.pi, np.pi, n)])


def sample_randomization(rng: np.random.Generator, n: int, task: TaskConfig) -> np.ndarray:
    r = np.zeros((n, hm.RANDOMIZATION_SIZE))
    if not task.randomize:
        r[:, :5] = 1.0
        return r
    r[:, 0] = rng.uniform(*task.dry_mass_scale, n)
    r[:, 1] = rng.uniform(*task.volume_scale, n)
    r[:, 2] = rng.uniform(*task.drag_scale, n)
    r[:, 3] = rng.uniform(*task.thrust_scale, n)
    r[:, 4] = rng.uniform(*task.rotor_inertia_scale, n)
    theta = rng.uniform(-np.pi, np.pi, n)
    speed = rng.uniform(0, task.current_speed, n)
    r[:, 5] = speed * np.cos(theta)
    r[:, 6] = speed * np.sin(theta)
    return r


def compute_reward(pos, quat, goal, action, prev_action, w: dict) -> tuple[np.ndarray, dict]:
    dist = np.linalg.norm(goal[:, :3] - pos, axis=1)
    ye = np.abs(yaw_error(pos, quat, goal))
    down = quat_rotate(quat_conj(quat), np.broadcast_to([0.0, 0.0, 1.0], pos.shape))
    tilt = 1.0 - down[:, 2]
    terms = {
        "pos": w["pos"] * (1.0 - np.tanh(dist / 2.0)),
        "pos_fine": w["pos_fine"] * np.exp(-((dist / 0.2) ** 2)),
        "yaw": w["yaw"] * (1.0 - ye / np.pi),
        "yaw_fine": w["yaw_fine"] * np.exp(-((ye / 0.1) ** 2)),
        "tilt": -w["tilt"] * tilt,
        "action": -w["action"] * np.sum(action**2, axis=1),
        "action_rate": -w["action_rate"] * np.sum((action - prev_action) ** 2, axis=1),
    }
    return sum(terms.values()), {"dist": dist, "yaw_err": ye, "tilt": tilt}


class HydrusPoseVecEnv(VecEnv):
    def __init__(
        self,
        num_envs: int = 64,
        num_threads: int = 0,
        task: TaskConfig | None = None,
        config_path: Path | str = DEFAULT_CONFIG,
        seed: int = 0,
    ):
        self.task = task or TaskConfig()
        self.cfg = load_config(config_path)
        self.tam = self.cfg["tam"]
        self.batch = hm.AuvBatch(num_envs, num_threads)
        assert abs(self.batch.timestep - 1.0 / PHYSICS_HZ) < 1e-9
        observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE,), np.float32)
        action_space = gym.spaces.Box(-1.0, 1.0, (ACT_SIZE,), np.float32)
        self.render_mode = None
        super().__init__(num_envs, observation_space, action_space)
        self.rng = np.random.default_rng(seed)
        self.max_steps = int(round(self.task.episode_seconds * POLICY_HZ))
        n = num_envs
        self.goal = np.zeros((n, 4))
        self.prev_action = np.zeros((n, ACT_SIZE))
        self.prev_pos = np.zeros((n, 3))
        self.prev_quat = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        self.steps = np.zeros(n, dtype=np.int64)
        self.switch_step = np.full(n, -1, dtype=np.int64)
        self.episode_return = np.zeros(n)
        self._actions = np.zeros((n, ACT_SIZE))

    def _reset_envs(self, mask: np.ndarray) -> np.ndarray:
        idx = np.flatnonzero(mask)
        init, goals, rand = sample_scenarios(self.rng, self.num_envs, self.task)
        state = self.batch.reset(mask.astype(np.uint8), init, rand)
        self.goal[idx] = goals[idx]
        self.prev_action[idx] = 0.0
        # The host has no previous pose on its first policy call, so velocities start at 0.
        self.prev_pos[idx] = state[idx, hm.POS]
        self.prev_quat[idx] = state[idx, hm.QUAT]
        self.steps[idx] = 0
        switch = self.rng.random(self.num_envs) < self.task.goal_switch_prob
        when = self.rng.integers(self.max_steps // 4, 3 * self.max_steps // 4, self.num_envs)
        self.switch_step[idx] = np.where(switch, when, -1)[idx]
        self.episode_return[idx] = 0.0
        return state

    def _observe(self, state: np.ndarray) -> np.ndarray:
        return build_observation(
            state[:, hm.POS], state[:, hm.QUAT], self.prev_pos, self.prev_quat, POLICY_DT, self.goal, self.prev_action
        )

    def reset(self):
        state = self._reset_envs(np.ones(self.num_envs, dtype=bool))
        return self._observe(state)

    def step_async(self, actions: np.ndarray) -> None:
        self._actions = np.clip(np.asarray(actions, dtype=np.float64), -1.0, 1.0)

    def step_wait(self):
        action = self._actions
        state = self.batch.step(action_to_thrusters(action, self.tam), SUBSTEPS)
        pos, quat = state[:, hm.POS], state[:, hm.QUAT]
        obs = build_observation(pos, quat, self.prev_pos, self.prev_quat, POLICY_DT, self.goal, action)
        reward, info_terms = compute_reward(pos, quat, self.goal, action, self.prev_action, self.task.reward_weights)
        self.steps += 1

        down_z = 1.0 - info_terms["tilt"]
        bad = (pos[:, 2] < SURFACE_LIMIT) | (down_z < TILT_LIMIT) | ~np.isfinite(state).all(axis=1)
        reward = np.where(bad, -self.task.reward_weights["terminate"], reward)
        truncated = (self.steps >= self.max_steps) & ~bad
        done = bad | truncated
        self.episode_return += reward

        infos = [{} for _ in range(self.num_envs)]
        self.prev_pos = pos.copy()
        self.prev_quat = quat.copy()
        self.prev_action = action.copy()

        switch = self.steps == self.switch_step
        if switch.any():
            self.goal[switch] = sample_goals(self.rng, pos[switch], self.task)

        if done.any():
            for i in np.flatnonzero(done):
                infos[i]["terminal_observation"] = obs[i].copy()
                infos[i]["TimeLimit.truncated"] = bool(truncated[i])
                infos[i]["episode"] = {
                    "r": float(self.episode_return[i]),
                    "l": int(self.steps[i]),
                    "final_dist": float(info_terms["dist"][i]),
                    "final_yaw_err": float(info_terms["yaw_err"][i]),
                }
            fresh = self._reset_envs(done)
            obs[done] = self._observe(fresh)[done]
        return obs, reward.astype(np.float32), done, infos

    def close(self) -> None:
        self.batch.close()

    def get_attr(self, attr_name, indices=None):
        return [getattr(self, attr_name)] * len(self._get_indices(indices))

    def set_attr(self, attr_name, value, indices=None) -> None:
        setattr(self, attr_name, value)

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        return [getattr(self, method_name)(*method_args, **method_kwargs)] * len(self._get_indices(indices))

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * len(self._get_indices(indices))


if __name__ == "__main__":
    from stable_baselines3.common.env_checker import check_env  # noqa: F401

    env = HydrusPoseVecEnv(num_envs=8)
    obs = env.reset()
    print("obs", obs.shape, obs[0])
    total = 0.0
    for _ in range(1200):
        obs, r, d, info = env.step(np.zeros((8, ACT_SIZE)))
        total += r.mean()
        for i in info:
            if "episode" in i:
                print("episode", i["episode"])
    print("mean reward/step (zero action)", total / 1200)
