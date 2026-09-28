"""Train the reactive prequalify planner. Start with --stage gate, then --stage full."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize

import planner_env as pl

RUNS = Path(__file__).resolve().parent / "runs"


class EpisodeStats(BaseCallback):
    def __init__(self):
        super().__init__()
        self.phase: list[float] = []
        self.crossed: list[float] = []
        self.t0 = time.time()
        self.start_steps = 0

    def _on_training_start(self) -> None:
        self.t0 = time.time()
        self.start_steps = self.num_timesteps

    def _on_step(self) -> bool:
        for info in self.locals["infos"]:
            ep = info.get("episode")
            if ep is None:
                continue
            self.phase.append(ep["phase"])
            self.crossed.append(ep["crossed"])
        return True

    def _on_rollout_end(self) -> None:
        if not self.phase:
            return
        self.logger.record("task/phase", float(np.mean(self.phase)))
        self.logger.record("task/gate_pass", float(np.mean(self.crossed)))
        self.logger.record("task/steps_per_s", (self.num_timesteps - self.start_steps) / (time.time() - self.t0))
        self.phase.clear()
        self.crossed.clear()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="planner_gate")
    parser.add_argument("--stage", choices=("gate", "full"), default="gate")
    parser.add_argument("--timesteps", type=float, default=2e6)
    parser.add_argument("--envs", type=int, default=64)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--torch-threads", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--resume", type=Path)
    args = parser.parse_args()

    torch.set_num_threads(args.torch_threads)
    run_dir = RUNS / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    env = pl.PlannerVecEnv(num_envs=args.envs, num_threads=args.threads, stage=args.stage, seed=args.seed)
    if args.resume:
        env = VecNormalize.load(str(args.resume.parent / "vecnormalize.pkl"), env)
        env.training = True
        model = PPO.load(args.resume, env=env, device="cpu", tensorboard_log=str(RUNS))
    else:
        env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.99)
        model = PPO(
            "MlpPolicy",
            env,
            n_steps=max(16, 4096 // args.envs),
            batch_size=2048,
            n_epochs=4,
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0,
            target_kl=0.03,
            policy_kwargs={
                "net_arch": {"pi": [128, 128], "vf": [256, 256]},
                "activation_fn": torch.nn.Tanh,
                "log_std_init": -1.0,
            },
            tensorboard_log=str(RUNS),
            seed=args.seed,
            device="cpu",
            verbose=1,
        )

    class SaveNormalize(CheckpointCallback):
        def _on_step(self) -> bool:
            if self.n_calls % self.save_freq == 0:
                env.save(str(run_dir / "vecnormalize.pkl"))
            return super()._on_step()

    checkpoint = SaveNormalize(save_freq=max(1, 200_000 // args.envs), save_path=str(run_dir / "checkpoints"))
    try:
        model.learn(
            total_timesteps=int(args.timesteps),
            callback=[EpisodeStats(), checkpoint],
            tb_log_name=args.name,
            reset_num_timesteps=args.resume is None,
        )
    finally:
        model.save(run_dir / "model.zip")
        env.save(str(run_dir / "vecnormalize.pkl"))
        print(f"saved {run_dir / 'model.zip'}")
        env.close()


if __name__ == "__main__":
    main()
