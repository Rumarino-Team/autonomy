"""Train a PPO pose-holding policy on the batched Hydrus physics."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize

import pose_env as pe

RUNS = Path(__file__).resolve().parent / "runs"


class EpisodeStats(BaseCallback):
    """Logs task metrics from the episode infos emitted by HydrusPoseVecEnv."""

    def __init__(self):
        super().__init__()
        self.dist: list[float] = []
        self.yaw: list[float] = []
        self.fail: list[float] = []
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
            self.dist.append(ep["final_dist"])
            self.yaw.append(ep["final_yaw_err"])
            self.fail.append(0.0 if info.get("TimeLimit.truncated") else 1.0)
        return True

    def _on_rollout_end(self) -> None:
        if self.dist:
            self.logger.record("task/final_dist", float(np.mean(self.dist)))
            self.logger.record("task/final_yaw_err", float(np.mean(self.yaw)))
            self.logger.record("task/fail_rate", float(np.mean(self.fail)))
            self.logger.record("task/steps_per_s", (self.num_timesteps - self.start_steps) / (time.time() - self.t0))
            self.dist.clear()
            self.yaw.clear()
            self.fail.clear()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="pose_ppo")
    parser.add_argument("--timesteps", type=float, default=20e6)
    parser.add_argument("--envs", type=int, default=256)
    parser.add_argument("--threads", type=int, default=0, help="physics threads (0 = all cores)")
    parser.add_argument("--torch-threads", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-randomize", action="store_true")
    parser.add_argument(
        "--resume", type=Path, help="model .zip to continue from (expects vecnormalize.pkl beside it); trains --timesteps more"
    )
    args = parser.parse_args()

    torch.set_num_threads(args.torch_threads)
    run_dir = RUNS / args.name
    run_dir.mkdir(parents=True, exist_ok=True)

    task = pe.TaskConfig(randomize=not args.no_randomize)
    env = pe.HydrusPoseVecEnv(num_envs=args.envs, num_threads=args.threads, task=task, seed=args.seed)
    if args.resume:
        env = VecNormalize.load(str(args.resume.parent / "vecnormalize.pkl"), env)
        env.training = True
        model = PPO.load(args.resume, env=env, device="cpu", tensorboard_log=str(RUNS))
    else:
        env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.995)
        model = PPO(
            "MlpPolicy",
            env,
            n_steps=max(32, 32768 // args.envs),
            batch_size=8192,
            n_epochs=5,
            learning_rate=3e-4,
            gamma=0.995,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0,
            vf_coef=0.5,
            max_grad_norm=0.5,
            target_kl=0.03,
            policy_kwargs={
                "net_arch": {"pi": [128, 128], "vf": [256, 256]},
                "activation_fn": torch.nn.Tanh,
                "log_std_init": -0.5,
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

    checkpoint = SaveNormalize(save_freq=max(1, 2_000_000 // args.envs), save_path=str(run_dir / "checkpoints"))
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
        print(f"saved {run_dir / 'model.zip'} and {run_dir / 'vecnormalize.pkl'}")
        env.close()


if __name__ == "__main__":
    main()
