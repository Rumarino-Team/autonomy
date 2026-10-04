"""Compare an exported policy against the host PID on the same random pose goals."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import auv_mujoco as hm
import pose_env as pe
from export_policy import ExportedPolicy
from pid_baseline import HostPID, Scenarios, run_episodes, summarize


class PolicyController:
    """What src/policy.zig does: a new action every 6 frames from pose differences."""

    hz = pe.POLICY_HZ

    def __init__(self, policy: ExportedPolicy, tam: np.ndarray):
        self.policy = policy
        self.tam = tam

    def reset(self, n: int) -> None:
        self.prev_pos = None
        self.prev_quat = None
        self.prev_action = np.zeros((n, pe.ACT_SIZE))

    def act(self, state: np.ndarray, goal: np.ndarray) -> np.ndarray:
        pos, quat = state[:, hm.POS], state[:, hm.QUAT]
        if self.prev_pos is None:
            self.prev_pos, self.prev_quat = pos, quat
        obs = pe.build_observation(pos, quat, self.prev_pos, self.prev_quat, pe.POLICY_DT, goal, self.prev_action)
        action = self.policy(obs).astype(np.float64)
        self.prev_pos, self.prev_quat, self.prev_action = pos.copy(), quat.copy(), action
        return pe.action_to_thrusters(action, self.tam)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=pe.REPO / "auvs" / "hydrus_mujoco" / "policy.bin")
    parser.add_argument("--episodes", type=int, default=256)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=1234)
    args = parser.parse_args()

    cfg = pe.load_config()
    policy = ExportedPolicy.load(args.policy)
    for label, randomize in (("nominal physics", False), ("randomized physics + current", True)):
        task = pe.TaskConfig(randomize=randomize, goal_switch_prob=0.0)
        sc = Scenarios.sample(args.episodes, args.seed, task)
        print(f"== {label}: {args.episodes} goals up to {task.goal_dist[1]:.0f} m away, {args.seconds:.0f} s each")
        print(summarize("pid", run_episodes(HostPID(cfg), sc, args.seconds)))
        print(summarize("policy", run_episodes(PolicyController(policy, cfg["tam"]), sc, args.seconds)))


if __name__ == "__main__":
    main()
