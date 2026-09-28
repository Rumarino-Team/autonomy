"""The host's global-navigation PID (src/navigation/global.zig + src/controls.zig) in numpy,
plus the rollout harness and metrics shared with eval.py."""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np

import hydrus_mujoco as hm
import pose_env as pe

YAW_GATE = np.pi / 8.0
SUCCESS_DIST = 0.3
SUCCESS_YAW = 0.2


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """math.quaternionToEuler: roll, pitch, yaw from xyzw."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1.0, np.sign(sinp) * np.pi / 2.0, np.arcsin(np.clip(sinp, -1.0, 1.0)))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.column_stack([roll, pitch, yaw])


class HostPID:
    """Runs every frame (360 Hz), exactly like MissionContext.fetchFrameAndUpdate."""

    hz = pe.PHYSICS_HZ

    def __init__(self, cfg: dict):
        self.kp, self.ki, self.kd, self.tam = cfg["kp"], cfg["ki"], cfg["kd"], cfg["tam"]

    def reset(self, n: int) -> None:
        self.sum_error = np.zeros((n, 6))
        self.prev_error = np.zeros((n, 6))
        self.first = True

    def act(self, state: np.ndarray, goal: np.ndarray) -> np.ndarray:
        pos, quat = state[:, hm.POS], pe.quat_normalize(state[:, hm.QUAT])
        goal6 = np.column_stack([goal[:, :3], np.zeros((len(goal), 2)), goal[:, 3]])
        err = goal6 - np.column_stack([pos, quat_to_euler(quat)])
        ye = pe.yaw_error(pos, quat, goal)
        err[:, 5] = ye
        if self.first:
            vel_err = np.zeros_like(err)
            self.first = False
        else:
            dt = 1.0 / self.hz
            self.sum_error += err * dt
            vel_err = (err - self.prev_error) / dt
        wrench = self.kp * err + self.ki * self.sum_error + self.kd * vel_err
        self.prev_error = err

        body = pe.quat_rotate(pe.quat_conj(quat), wrench[:, :3])
        gated = np.abs(ye) > YAW_GATE
        bx = np.where(gated, 0.0, np.maximum(body[:, 0], 0.0))
        by = np.where(gated, 0.0, np.maximum(body[:, 1], 0.0))
        tam_input = np.column_stack([bx, by, wrench[:, 2], -wrench[:, 3], wrench[:, 4], wrench[:, 5]])
        t = tam_input @ self.tam.T
        return (np.clip(t, -pe.THRUSTOR_SATURATE, pe.THRUSTOR_SATURATE) / pe.THRUSTOR_OUTPUT_SCALE).astype(np.float32)


@dataclass
class Scenarios:
    init: np.ndarray
    goals: np.ndarray
    randomization: np.ndarray

    @staticmethod
    def sample(n: int, seed: int, task: pe.TaskConfig) -> "Scenarios":
        init, goals, rand = pe.sample_scenarios(np.random.default_rng(seed), n, task)
        return Scenarios(init, goals, rand)


def run_episodes(controller, sc: Scenarios, seconds: float, num_threads: int = 0, record: bool = False) -> dict:
    """Runs every scenario for `seconds` with a fixed goal. Controllers expose hz, reset(n)
    and act(state, goal) -> thruster commands."""
    n = len(sc.init)
    batch = hm.HydrusBatch(n, num_threads)
    state = batch.reset(None, sc.init, sc.randomization)
    controller.reset(n)
    substeps = pe.PHYSICS_HZ // controller.hz
    steps = int(round(seconds * controller.hz))
    failed = np.zeros(n, dtype=bool)
    fail_time = np.full(n, np.inf)
    dist_hist, yaw_hist, tilt_hist, effort_hist, traj = [], [], [], [], []
    for k in range(steps):
        cmd = controller.act(state, sc.goals)
        effort_hist.append(np.mean(np.clip(np.abs(cmd), 0, 1), axis=1))
        state = batch.step(cmd, substeps)
        pos, quat = state[:, hm.POS], state[:, hm.QUAT]
        down = pe.quat_rotate(pe.quat_conj(pe.quat_normalize(quat)), np.broadcast_to([0.0, 0.0, 1.0], pos.shape))
        bad = (pos[:, 2] < pe.SURFACE_LIMIT) | (down[:, 2] < pe.TILT_LIMIT) | ~np.isfinite(state).all(axis=1)
        new_fail = bad & ~failed
        fail_time[new_fail] = (k + 1) / controller.hz
        failed |= bad
        dist_hist.append(np.linalg.norm(sc.goals[:, :3] - pos, axis=1))
        yaw_hist.append(np.abs(pe.yaw_error(pos, quat, sc.goals)))
        tilt_hist.append(np.degrees(np.arccos(np.clip(down[:, 2], -1, 1))))
        if record:
            traj.append(state.copy())
    batch.close()
    dist = np.array(dist_hist)
    yaw = np.array(yaw_hist)
    tilt = np.array(tilt_hist)
    t = np.arange(1, steps + 1) / controller.hz
    inside = (dist < SUCCESS_DIST) & (yaw < SUCCESS_YAW)
    # Settling time: first time after which the vehicle stays inside the tolerance.
    stays = np.flip(np.cumprod(np.flip(inside, axis=0), axis=0), axis=0).astype(bool)
    settle = np.where(stays.any(axis=0), t[np.argmax(stays, axis=0)], np.inf)
    last = t > seconds - 5.0
    success = inside[-1] & ~failed
    result = {
        "success": success,
        "failed": failed,
        "fail_time": fail_time,
        "final_dist": dist[-1],
        "final_yaw": yaw[-1],
        "hold_dist": dist[last].mean(axis=0),
        "settle": settle,
        "max_tilt": tilt.max(axis=0),
        "effort": np.array(effort_hist).mean(axis=0),
    }
    if record:
        result["traj"] = np.array(traj)
        result["time"] = t
    return result


def summarize(name: str, r: dict) -> str:
    ok = ~r["failed"]
    settled = np.isfinite(r["settle"]) & r["success"]
    return (
        f"{name:>8}: success {100 * r['success'].mean():5.1f}%  failed {100 * r['failed'].mean():5.1f}%  "
        f"final dist {np.median(r['final_dist'][ok]) if ok.any() else np.nan:6.3f} m (median)  "
        f"hold dist {r['hold_dist'][ok].mean() if ok.any() else np.nan:6.3f} m  "
        f"final yaw {np.degrees(np.median(r['final_yaw'][ok])) if ok.any() else np.nan:5.1f} deg  "
        f"settle {np.median(r['settle'][settled]) if settled.any() else np.nan:5.2f} s  "
        f"max tilt {np.median(r['max_tilt']):5.1f} deg  effort {r['effort'].mean():.3f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate the host PID on random pose goals.")
    parser.add_argument("--episodes", type=int, default=256)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--nominal", action="store_true", help="disable domain randomization")
    args = parser.parse_args()
    task = pe.TaskConfig(randomize=not args.nominal, goal_switch_prob=0.0)
    sc = Scenarios.sample(args.episodes, args.seed, task)
    print(summarize("pid", run_episodes(HostPID(pe.load_config()), sc, args.seconds)))


if __name__ == "__main__":
    main()
