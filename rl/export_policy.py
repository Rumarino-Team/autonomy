"""Export a trained SB3 policy plus its VecNormalize statistics for src/policy.zig.

File layout, little endian:
    magic "HYPL", u32 version (1), u32 obs_size, u32 act_size, u32 num_layers,
    f32 policy_hz, f32 max_goal_dist, f32 close_enough, f32 clip_obs,
    f32 obs_mean[obs_size], f32 obs_inv_std[obs_size],
    per layer: u32 in, u32 out, u32 activation (0 identity, 1 tanh),
               f32 weight[out][in], f32 bias[out]
The action is the last layer's output clamped to [-1, 1] and fed to the TAM.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

import pose_env as pe

MAGIC = b"HYPL"
VERSION = 1
IDENTITY, TANH = 0, 1


class ExportedPolicy:
    def __init__(self, obs_mean, obs_inv_std, clip_obs, layers, policy_hz, max_goal_dist, close_enough):
        self.obs_mean = np.asarray(obs_mean, np.float32)
        self.obs_inv_std = np.asarray(obs_inv_std, np.float32)
        self.clip_obs = np.float32(clip_obs)
        self.layers = [(np.asarray(w, np.float32), np.asarray(b, np.float32), act) for w, b, act in layers]
        self.policy_hz = policy_hz
        self.max_goal_dist = max_goal_dist
        self.close_enough = close_enough

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        x = np.clip((obs.astype(np.float32) - self.obs_mean) * self.obs_inv_std, -self.clip_obs, self.clip_obs)
        for w, b, act in self.layers:
            x = x @ w.T + b
            if act == TANH:
                x = np.tanh(x)
        return np.clip(x, -1.0, 1.0)

    def save(self, path: Path) -> None:
        with open(path, "wb") as f:
            f.write(MAGIC)
            f.write(struct.pack("<4I", VERSION, len(self.obs_mean), self.layers[-1][0].shape[0], len(self.layers)))
            f.write(struct.pack("<4f", self.policy_hz, self.max_goal_dist, self.close_enough, self.clip_obs))
            f.write(self.obs_mean.astype("<f4").tobytes())
            f.write(self.obs_inv_std.astype("<f4").tobytes())
            for w, b, act in self.layers:
                f.write(struct.pack("<3I", w.shape[1], w.shape[0], act))
                f.write(w.astype("<f4").tobytes())
                f.write(b.astype("<f4").tobytes())

    @staticmethod
    def load(path: Path) -> "ExportedPolicy":
        data = Path(path).read_bytes()
        assert data[:4] == MAGIC, "not a Hydrus policy file"
        version, obs_size, act_size, num_layers = struct.unpack_from("<4I", data, 4)
        assert version == VERSION
        policy_hz, max_goal_dist, close_enough, clip_obs = struct.unpack_from("<4f", data, 20)
        off = 36
        mean = np.frombuffer(data, "<f4", obs_size, off)
        off += 4 * obs_size
        inv_std = np.frombuffer(data, "<f4", obs_size, off)
        off += 4 * obs_size
        layers = []
        for _ in range(num_layers):
            n_in, n_out, act = struct.unpack_from("<3I", data, off)
            off += 12
            w = np.frombuffer(data, "<f4", n_in * n_out, off).reshape(n_out, n_in)
            off += 4 * n_in * n_out
            b = np.frombuffer(data, "<f4", n_out, off)
            off += 4 * n_out
            layers.append((w, b, act))
        assert off == len(data)
        assert layers[-1][0].shape[0] == act_size
        return ExportedPolicy(mean, inv_std, clip_obs, layers, policy_hz, max_goal_dist, close_enough)


def from_sb3(model_path: Path, vecnorm_path: Path) -> ExportedPolicy:
    import pickle

    import torch
    from stable_baselines3 import PPO

    model = PPO.load(model_path, device="cpu")
    with open(vecnorm_path, "rb") as f:
        vecnorm = pickle.load(f)
    rms = vecnorm.obs_rms
    policy = model.policy
    mods = list(policy.mlp_extractor.policy_net) + [policy.action_net]
    layers = []
    for i, m in enumerate(mods):
        if isinstance(m, torch.nn.Linear):
            nxt = mods[i + 1] if i + 1 < len(mods) else None
            act = TANH if isinstance(nxt, torch.nn.Tanh) else IDENTITY
            layers.append((m.weight.detach().numpy(), m.bias.detach().numpy(), act))
        elif not isinstance(m, torch.nn.Tanh):
            raise ValueError(f"unsupported layer {m}")
    return ExportedPolicy(
        rms.mean,
        1.0 / np.sqrt(rms.var + vecnorm.epsilon),
        vecnorm.clip_obs,
        layers,
        pe.POLICY_HZ,
        pe.MAX_GOAL_DIST,
        pe.CLOSE_ENOUGH,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path, help="run directory with model.zip and vecnormalize.pkl")
    parser.add_argument("--model", type=Path, help="override model .zip (e.g. a checkpoint)")
    parser.add_argument("-o", "--output", type=Path, default=pe.REPO / "auvs" / "hydrus_mujoco" / "policy.bin")
    args = parser.parse_args()

    model_path = args.model or args.run / "model.zip"
    exported = from_sb3(model_path, args.run / "vecnormalize.pkl")
    exported.save(args.output)
    reloaded = ExportedPolicy.load(args.output)

    # The exported forward pass must match SB3's deterministic action.
    import pickle

    from stable_baselines3 import PPO

    model = PPO.load(model_path, device="cpu")
    with open(args.run / "vecnormalize.pkl", "rb") as f:
        vecnorm = pickle.load(f)
    rng = np.random.default_rng(0)
    obs = rng.normal(size=(512, pe.OBS_SIZE)).astype(np.float32) * np.sqrt(vecnorm.obs_rms.var) + vecnorm.obs_rms.mean
    sb3_action, _ = model.predict(vecnorm.normalize_obs(obs), deterministic=True)
    err = np.abs(reloaded(obs) - np.clip(sb3_action, -1, 1)).max()
    sizes = " -> ".join(str(w.shape[1]) for w, _, _ in reloaded.layers) + f" -> {reloaded.layers[-1][0].shape[0]}"
    print(f"wrote {args.output} ({args.output.stat().st_size} bytes, layers {sizes}), max |diff| vs SB3 = {err:.2e}")
    assert err < 1e-4


if __name__ == "__main__":
    main()
