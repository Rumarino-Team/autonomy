"""Evolution strategy over the host PID gains.

Searches kp, ki, and kd (6 axes each: x, y, z, roll, pitch, yaw) by rolling HostPID
out on a fixed set of pose goals. Prints the best gains. Does not write auv.json.

Add a cost by writing a function (result, seconds, gamma) -> float and registering it
in COSTS. Lower is better. The search only sees that number.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

import auv_mujoco as auv
import pose_env as pe
from pid_baseline import HostPID, Scenarios, run_episodes, summarize

N_GAINS = 18


def gain_bounds() -> tuple[np.ndarray, np.ndarray]:
    """Box around the Stonefish gains. Roll and pitch may be negative; the host flips roll before the TAM."""
    lo = np.array(
        [
            0, 0, 0, -12, -12, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, -12, -12, 0,
        ],
        dtype=np.float64,
    )
    hi = np.full(N_GAINS, 12.0)
    return lo, hi


def pack(cfg: dict) -> np.ndarray:
    return np.concatenate([cfg["kp"], cfg["ki"], cfg["kd"]]).astype(np.float64)


def make_pid(cfg: dict, gains: np.ndarray) -> HostPID:
    tuned = dict(cfg)
    tuned["kp"] = np.array(gains[0:6], dtype=np.float64)
    tuned["ki"] = np.array(gains[6:12], dtype=np.float64)
    tuned["kd"] = np.array(gains[12:18], dtype=np.float64)
    return HostPID(tuned)


def cost_task(result: dict, seconds: float, gamma: float) -> float:
    """Failures and misses dominate; settle time and effort are tie breaks."""
    del gamma
    fail = float(result["failed"].mean())
    success = float(result["success"].mean())
    dist = float(np.median(result["final_dist"]))
    yaw = float(np.median(result["final_yaw"]))
    settle = np.array(result["settle"], dtype=np.float64)
    settle[~np.isfinite(settle)] = seconds
    effort = float(result["effort"].mean())
    return 8.0 * fail + 4.0 * (1.0 - success) + dist + yaw + 0.15 * float(np.median(settle)) + 0.2 * effort


def _finite_penalty(values: list[np.ndarray], failed: np.ndarray) -> np.ndarray:
    bad = np.array(failed, dtype=bool)
    for value in values:
        bad |= ~np.isfinite(value)
    return bad


def cost_itae(result: dict, seconds: float, gamma: float) -> float:
    """J = ∫ t |e| dt + γ ∫ Σ |u_i| dt. e is position error (m) plus absolute yaw (rad)."""
    err = np.array(result["weighted_error"], dtype=np.float64)
    thrust = np.array(result["abs_thrust"], dtype=np.float64)
    bad = _finite_penalty([err, thrust], result["failed"])
    # A diverged trial is worse than tracking a large error for the whole window.
    err = np.where(bad, 50.0 * seconds * seconds, err)
    thrust = np.where(bad, 0.0, thrust)
    return float(err.mean() + gamma * thrust.mean())


def cost_stable(result: dict, seconds: float, gamma: float) -> float:
    """Position tracking plus time-weighted roll, pitch, yaw error and body rotation rate.

    Attitude is in radians and weighted by 8 so a tenth of a radian costs as much as
    0.8 m of position error. Rotation rate is weighted by 2 so a pitch oscillation
    costs more than a slow yaw toward the goal.
    """
    pos = np.array(result["weighted_position"], dtype=np.float64)
    attitude = np.array(result["weighted_attitude"], dtype=np.float64)
    rate = np.array(result["weighted_rate"], dtype=np.float64)
    thrust = np.array(result["abs_thrust"], dtype=np.float64)
    bad = _finite_penalty([pos, attitude, rate, thrust], result["failed"])
    penalty = 50.0 * seconds * seconds
    pos = np.where(bad, penalty, pos)
    attitude = np.where(bad, 0.0, attitude)
    rate = np.where(bad, 0.0, rate)
    thrust = np.where(bad, 0.0, thrust)
    return float((pos + 8.0 * attitude + 2.0 * rate + gamma * thrust).mean())


# name -> (result, seconds, gamma) -> float
COSTS = {
    "task": cost_task,
    "itae": cost_itae,
    "stable": cost_stable,
}


def evaluate(
    cfg: dict,
    gains: np.ndarray,
    scenarios: Scenarios,
    seconds: float,
    batch: auv.AuvBatch,
    score,
    gamma: float,
) -> tuple[float, dict]:
    result = run_episodes(make_pid(cfg, gains), scenarios, seconds, batch=batch)
    return score(result, seconds, gamma), result


def format_gains(gains: np.ndarray) -> str:
    chunks = []
    for name, row in zip(("kp", "ki", "kd"), np.split(gains, 3)):
        nums = ", ".join(f"{v:.4f}" for v in row)
        chunks.append(f'"{name}": [{nums}]')
    return "\n".join(chunks)


def search(
    cfg: dict,
    scenarios: Scenarios,
    seconds: float,
    batch: auv.AuvBatch,
    generations: int,
    population: int,
    elite: int,
    seed: int,
    score,
    gamma: float,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    lo, hi = gain_bounds()
    span = hi - lo
    sigma = 0.2 * span
    sigma_floor = 0.02 * span
    best = np.clip(pack(cfg), lo, hi)
    center = best.copy()
    best_cost, _ = evaluate(cfg, best, scenarios, seconds, batch, score, gamma)
    print(f"baseline cost {best_cost:.3f}")
    print(format_gains(best))

    for gen in range(generations):
        t0 = time.perf_counter()
        noise = rng.normal(size=(population, N_GAINS))
        candidates = np.clip(center + sigma * noise, lo, hi)
        candidates[0] = best
        scored: list[tuple[float, np.ndarray]] = []
        for gains in candidates:
            value, _ = evaluate(cfg, gains, scenarios, seconds, batch, score, gamma)
            scored.append((value, gains))
        scored.sort(key=lambda item: item[0])
        if scored[0][0] < best_cost:
            best_cost = scored[0][0]
            best = scored[0][1].copy()
            sigma = np.minimum(sigma * 1.05, 0.5 * span)
        else:
            sigma = np.maximum(sigma * 0.85, sigma_floor)
        parents = np.stack([gains for _, gains in scored[:elite]])
        center = np.clip(parents.mean(axis=0), lo, hi)
        dt = time.perf_counter() - t0
        print(
            f"gen {gen + 1:3d}/{generations}  cost {best_cost:.3f}  "
            f"sigma {float(np.mean(sigma / np.maximum(span, 1e-9))):.3f}  {dt:.1f}s",
            flush=True,
        )
    return best


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=32)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--generations", type=int, default=12)
    parser.add_argument("--population", type=int, default=8)
    parser.add_argument("--elite", type=int, default=3)
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--cost", choices=tuple(COSTS), default="task")
    parser.add_argument("--gamma", type=float, default=0.05, help="thruster weight for --cost itae and --cost stable")
    parser.add_argument("--randomize", action="store_true", help="tune on randomized physics and current")
    args = parser.parse_args()
    if args.elite < 1 or args.elite > args.population:
        raise SystemExit("--elite must be between 1 and --population")

    score = COSTS[args.cost]
    cfg = pe.load_config()
    task = pe.TaskConfig(randomize=args.randomize, goal_switch_prob=0.0)
    train = Scenarios.sample(args.episodes, args.seed, task)
    held_out = Scenarios.sample(args.episodes, args.seed + 1, task)
    batch = auv.AuvBatch(args.episodes, args.threads)
    try:
        best = search(
            cfg,
            train,
            args.seconds,
            batch,
            args.generations,
            args.population,
            args.elite,
            args.seed,
            score,
            args.gamma,
        )
        print("\nbest gains")
        print(format_gains(best))
        print("\nheld-out")
        base = run_episodes(make_pid(cfg, pack(cfg)), held_out, args.seconds, batch=batch)
        tuned = run_episodes(make_pid(cfg, best), held_out, args.seconds, batch=batch)
        print(f"{args.cost} baseline {score(base, args.seconds, args.gamma):.3f}  tuned {score(tuned, args.seconds, args.gamma):.3f}")
        print(summarize("baseline", base))
        print(summarize("tuned", tuned))
    finally:
        batch.close()


if __name__ == "__main__":
    main()
