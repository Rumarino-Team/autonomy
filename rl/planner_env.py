"""Reactive prequalify planner on the batched MuJoCo camera.

The observation is YOLO boxes, pooled Sobel edges, pooled optical flow, and the body
IMU. It has no depth and no world pose. The 22D action is a body wrench plus four body
waypoints; the configured PID and TAM turn that into thruster commands.
"""

from __future__ import annotations

import gymnasium as gym
import numpy as np
from stable_baselines3.common.vec_env import VecEnv

import hydrus_mujoco as hm
import pose_env as pe

PLANNER_HZ = 2
CONTROL_HZ = 60
CONTROLS_PER_PLAN = CONTROL_HZ // PLANNER_HZ
PHYSICS_PER_CONTROL = pe.PHYSICS_HZ // CONTROL_HZ

BOX_FEATURES = hm.CAM_BOXES * hm.CAM_BOX_STRIDE
OBS_SIZE = hm.CAM_FEATURE_SIZE + 6
ACT_SIZE = 22
WAYPOINTS = 4

WRENCH_SCALE = 1.0
FORWARD_SCALE = 3.0
RIGHT_SCALE = 3.0
DOWN_SCALE = 1.0
YAW_SCALE = np.pi

REACHED = 1.0
# Gate opening in the gate frame. The mesh spans the posts; this is the hole.
OPENING_X = 0.9
OPENING_Z = (-1.3, -0.25)


def _yaw_quat(yaw: np.ndarray) -> np.ndarray:
    return np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], axis=-1)


def _body_down(quat: np.ndarray) -> np.ndarray:
    return pe.quat_rotate(quat, np.broadcast_to([0.0, 0.0, 1.0], quat[..., :3].shape))


class PlannerVecEnv(VecEnv):
    def __init__(self, num_envs: int = 64, num_threads: int = 0, stage: str = "gate", seed: int = 0):
        if stage not in ("gate", "full"):
            raise ValueError("stage must be 'gate' or 'full'")
        self.stage = stage
        self.cfg = pe.load_config()
        self.tam = self.cfg["tam"]
        # Roll and pitch hold the vehicle upright. The other gains follow the body waypoint.
        # Roll is negative, matching the Hydrus TAM sign in auv.json before it was flattened.
        # Mx restores body-Y tilt with a negative gain; My restores body-X tilt with a positive gain.
        self.kp = np.array([0.8, 0.8, 0.8, -8.0, 8.0, 1.2])
        self.ki = np.zeros(6)
        self.kd = np.array([0.4, 0.4, 0.3, -2.0, 2.0, 0.4])
        self.batch = hm.HydrusBatch(num_envs, num_threads)
        if self.batch.mocap_count < 2:
            raise RuntimeError("hydrus.xml gate and marker must be mocap bodies")
        self.render_mode = None
        super().__init__(
            num_envs,
            gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE,), np.float32),
            gym.spaces.Box(-1.0, 1.0, (ACT_SIZE,), np.float32),
        )
        self.rng = np.random.default_rng(seed)
        self.max_steps = 80 if stage == "gate" else 120
        n = num_envs
        self.gate_pos = np.zeros((n, 3))
        self.gate_quat = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        self.marker_pos = np.zeros((n, 3))
        self.start_pos = np.zeros((n, 3))
        self.phase = np.zeros(n, dtype=np.int64)
        self.crossed = np.zeros(n, dtype=bool)
        self.prev_local_y = np.zeros(n)
        self.prev_dist = np.zeros(n)
        self.steps = np.zeros(n, dtype=np.int64)
        self.episode_return = np.zeros(n)
        self.pos0 = np.zeros((n, 3))
        self.quat0 = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))
        self.waypoints = np.zeros((n, WAYPOINTS, 4))
        self.wrench = np.zeros((n, 6))
        self.active = np.zeros(n, dtype=np.int64)
        self.active_ticks = np.zeros(n, dtype=np.int64)
        self.pid_sum = np.zeros((n, 6))
        self.pid_prev = np.zeros((n, 6))
        self._action = np.zeros((n, ACT_SIZE))
        self.mocap = np.zeros((n, self.batch.mocap_count, hm.MOCAP_POSE))
        self.mocap[:, :, 6] = 1.0

    def _sample_layout(self, idx: np.ndarray) -> None:
        n = len(idx)
        rng = self.rng
        yaw = rng.uniform(-0.6, 0.6, n)
        gate = np.column_stack([rng.uniform(-2.0, 4.0, n), rng.uniform(6.0, 14.0, n), np.full(n, 3.0)])
        gq = _yaw_quat(yaw)
        dist = rng.uniform(3.0, 8.0, n)
        ahead = np.column_stack([np.zeros(n), -dist, np.full(n, -1.0)])
        veh = gate + pe.quat_rotate(gq, ahead)
        to_gate = gate[:, :2] - veh[:, :2]
        cam_heading = np.arctan2(to_gate[:, 1], to_gate[:, 0]) + rng.uniform(-0.3, 0.3, n)
        # The camera site looks opposite body +Y, so the nose heading is the camera heading plus pi.
        quat = pe.quat_from_heading(cam_heading + np.pi, rng.uniform(-0.05, 0.05, n), rng.uniform(-0.05, 0.05, n))
        init = np.zeros((self.num_envs, hm.INIT_SIZE))
        init[idx, 0:3] = veh
        init[idx, 3:7] = quat
        marker_dist = rng.uniform(6.0, 12.0, n)
        marker = gate + pe.quat_rotate(gq, np.column_stack([np.zeros(n), marker_dist, np.full(n, -1.0)]))
        self.mocap[idx, 0, 0:3] = gate
        self.mocap[idx, 0, 3:7] = gq
        self.mocap[idx, 1, 0:3] = marker
        self.mocap[idx, 1, 3:7] = gq
        mask = np.zeros(self.num_envs, dtype=np.uint8)
        mask[idx] = 1
        self.batch.reset(mask=mask, init_state=init)
        self.batch.set_mocap(self.mocap)
        self.gate_pos[idx] = gate
        self.gate_quat[idx] = gq
        self.marker_pos[idx] = marker
        self.start_pos[idx] = veh
        self.phase[idx] = 0
        self.crossed[idx] = False
        state = self.batch.state()
        local = self._gate_local(state[:, hm.POS])
        self.prev_local_y[idx] = local[idx]
        self.prev_dist[idx] = self._goal_dist(state[:, hm.POS])[idx]
        self.steps[idx] = 0
        self.episode_return[idx] = 0.0

    def _subgoal(self, phase: np.ndarray) -> np.ndarray:
        goals = np.zeros((self.num_envs, 3))
        front = self.gate_pos + pe.quat_rotate(self.gate_quat, np.array([0.0, -2.0, -1.0]))
        past = self.gate_pos + pe.quat_rotate(self.gate_quat, np.array([0.0, 2.0, -1.0]))
        corners = []
        for sx, sy in ((2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 2.0)):
            corner = self.marker_pos.copy()
            corner[:, 0] += sx
            corner[:, 1] += sy
            corner[:, 2] = 2.0
            corners.append(corner)
        table = [front, past, *corners, self.start_pos]
        if self.stage == "gate":
            table = [front, past]
        for i, goal in enumerate(table):
            goals[phase == i] = goal[phase == i]
        last = len(table) - 1
        goals[phase > last] = table[last][phase > last]
        return goals

    def _nphases(self) -> int:
        return 2 if self.stage == "gate" else 7

    def _gate_local(self, pos: np.ndarray) -> np.ndarray:
        return pe.quat_rotate(pe.quat_conj(self.gate_quat), pos - self.gate_pos)[:, 1]

    def _goal_dist(self, pos: np.ndarray) -> np.ndarray:
        return np.linalg.norm(self._subgoal(self.phase) - pos, axis=1)

    def _observe(self, state: np.ndarray) -> np.ndarray:
        features = self.batch.camera()
        imu = np.concatenate([state[:, hm.GYRO], state[:, hm.ACCEL]], axis=1).astype(np.float32)
        return np.concatenate([features, imu], axis=1)

    def _decode(self, action: np.ndarray) -> None:
        self.wrench = np.clip(action[:, 0:6], -1.0, 1.0) * WRENCH_SCALE
        wp = action[:, 6:].reshape(self.num_envs, WAYPOINTS, 4)
        scale = np.array([FORWARD_SCALE, RIGHT_SCALE, DOWN_SCALE, YAW_SCALE])
        self.waypoints = np.clip(wp, -1.0, 1.0) * scale
        state = self.batch.state()
        self.pos0 = state[:, hm.POS].copy()
        self.quat0 = state[:, hm.QUAT].copy()
        self.active[:] = 0
        self.active_ticks[:] = 0
        self.pid_sum[:] = 0.0
        err = self._waypoint_error(state)
        self.pid_prev = err

    def _waypoint_error(self, state: np.ndarray) -> np.ndarray:
        pos, quat = state[:, hm.POS], pe.quat_normalize(state[:, hm.QUAT])
        delta = pe.quat_rotate(pe.quat_conj(self.quat0), pos - self.pos0)
        wp = self.waypoints[np.arange(self.num_envs), self.active]
        remain = np.column_stack([wp[:, 1] - delta[:, 0], wp[:, 0] - delta[:, 1], wp[:, 2] - delta[:, 2]])
        body = pe.quat_rotate(pe.quat_conj(quat), pe.quat_rotate(self.quat0, remain))
        dyaw = pe.wrap_angle(pe.heading(quat) - pe.heading(self.quat0))
        yaw_err = pe.wrap_angle(wp[:, 3] - dyaw)
        # World down in the body. Upright in NED is (0, 0, 1). TAM Mx moves the body-Y
        # component and My moves the body-X component, so the two are swapped here.
        down = pe.quat_rotate(pe.quat_conj(quat), np.broadcast_to([0.0, 0.0, 1.0], pos.shape))
        return np.column_stack([body, down[:, 1], down[:, 0], yaw_err])

    def _thrusters(self, err: np.ndarray) -> np.ndarray:
        dt = 1.0 / CONTROL_HZ
        self.pid_sum += err * dt
        vel = (err - self.pid_prev) / dt
        self.pid_prev = err
        pid = self.kp * err + self.ki * self.pid_sum + self.kd * vel
        wrench = pid + self.wrench
        t = wrench @ self.tam.T
        return np.clip(t, -pe.THRUSTOR_SATURATE, pe.THRUSTOR_SATURATE).astype(np.float32) / pe.THRUSTOR_OUTPUT_SCALE

    def reset(self):
        self._sample_layout(np.arange(self.num_envs))
        return self._observe(self.batch.state())

    def step_async(self, actions: np.ndarray) -> None:
        self._action = np.asarray(actions, dtype=np.float64)

    def step_wait(self):
        self._decode(self._action)
        state = self.batch.state()
        for _ in range(CONTROLS_PER_PLAN):
            err = self._waypoint_error(state)
            close = np.linalg.norm(err[:, :3], axis=1) < 0.6
            timed = self.active_ticks >= CONTROL_HZ // 2 - 1
            advance = (close | timed) & (self.active < WAYPOINTS - 1)
            self.active[advance] += 1
            self.active_ticks[advance] = 0
            self.active_ticks += 1
            if advance.any():
                err = self._waypoint_error(state)
                self.pid_prev[advance] = err[advance]
            state = self.batch.step(self._thrusters(err), PHYSICS_PER_CONTROL)

        pos, quat = state[:, hm.POS], state[:, hm.QUAT]
        local_y = self._gate_local(pos)
        local = pe.quat_rotate(pe.quat_conj(self.gate_quat), pos - self.gate_pos)
        inside = (np.abs(local[:, 0]) < OPENING_X) & (local[:, 2] > OPENING_Z[0]) & (local[:, 2] < OPENING_Z[1])
        just_crossed = (self.prev_local_y < 0) & (local_y >= 0) & inside & ~self.crossed
        self.crossed |= just_crossed
        self.prev_local_y = local_y

        dist = self._goal_dist(pos)
        reward = 2.0 * (self.prev_dist - dist)
        reward = reward + 5.0 * just_crossed
        reached = (dist < REACHED) & (self.phase != 1)
        # Phase 1 advances only by passing through the opening.
        reward = reward + 2.0 * reached
        self.phase[reached] += 1
        opening = (self.phase == 1) & just_crossed
        self.phase[opening] += 1
        moved = reached | opening
        if moved.any():
            dist = self._goal_dist(pos)
        self.prev_dist = dist

        goal = self._subgoal(self.phase)
        direction = pe.quat_rotate(pe.quat_conj(pe.quat_normalize(quat)), goal - pos)
        direction[:, 2] = 0.0
        dn = np.linalg.norm(direction[:, :2], axis=1, keepdims=True)
        direction[:, :2] /= np.maximum(dn, 1e-6)
        wp = self.waypoints
        wp_body = np.stack([wp[:, :, 1], wp[:, :, 0]], axis=-1)
        wn = np.linalg.norm(wp_body, axis=-1, keepdims=True)
        align = np.sum((wp_body / np.maximum(wn, 1e-6)) * direction[:, None, :2], axis=-1).mean(axis=1)
        reward = reward + 0.2 * align - 0.02 * np.sum(self.wrench**2, axis=1)

        down = _body_down(pe.quat_normalize(quat))[:, 2]
        bad = (pos[:, 2] < 0.25) | (pos[:, 2] > 2.85) | (down < 0.3) | ~np.isfinite(state).all(axis=1)
        bad |= np.abs(pos[:, 0]) > 25
        reward = np.where(bad, -10.0, reward)
        self.steps += 1
        done_phase = self.phase >= self._nphases()
        truncated = (self.steps >= self.max_steps) & ~bad & ~done_phase
        done = bad | truncated | done_phase
        reward = np.where(done_phase, reward + 5.0, reward)
        self.episode_return += reward

        obs = self._observe(state)
        infos = [{} for _ in range(self.num_envs)]
        if done.any():
            for i in np.flatnonzero(done):
                infos[i]["terminal_observation"] = obs[i].copy()
                infos[i]["TimeLimit.truncated"] = bool(truncated[i])
                infos[i]["episode"] = {
                    "r": float(self.episode_return[i]),
                    "l": int(self.steps[i]),
                    "phase": int(self.phase[i]),
                    "crossed": bool(self.crossed[i]),
                }
            self._sample_layout(np.flatnonzero(done))
            obs[done] = self._observe(self.batch.state())[done]
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
    env = PlannerVecEnv(num_envs=4, stage="gate")
    obs = env.reset()
    print("obs", obs.shape, "boxes", obs[0, :7], "edge", obs[0, BOX_FEATURES:BOX_FEATURES + 4])
    total = 0.0
    for _ in range(3):
        obs, reward, done, info = env.step(np.zeros((4, ACT_SIZE)))
        total += reward.mean()
        print("reward", reward.round(3), "done", done)
    print("mean", total / 3)
    env.close()
