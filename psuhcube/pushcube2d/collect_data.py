"""Collect scripted PushCube2D demonstrations."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

from .dataset import episode_from_lists, save_dataset
from .env import PushCube2DEnv
from .scripted_policy import make_policy_from_env


def str2bool(value) -> bool:
    if isinstance(value, bool):
        return value
    value = str(value).lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected boolean value, got {value!r}")


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("data") / "pushcube2d" / f"dataset_{stamp}.npz"


def rollout_episode(env: PushCube2DEnv, policy, seed: int | None = None, render: bool = False, reset_options=None):
    obs, _ = env.reset(seed=seed, options=reset_options)
    if hasattr(policy, "reset"):
        policy.reset()

    obs_list, action_list, state_list, reward_list, done_list = [], [], [], [], []
    info = {}
    for _ in range(env.config.max_steps):
        action = policy(obs)
        state = env.get_state()
        next_obs, reward, terminated, truncated, info = env.step(action)
        done = bool(terminated or truncated)

        obs_list.append(obs.copy())
        action_list.append(np.asarray(action, dtype=np.float32).copy())
        state_list.append(state.copy())
        reward_list.append(float(reward))
        done_list.append(done)

        if render:
            env.render("human")
        obs = next_obs
        if done:
            break

    return episode_from_lists(obs_list, action_list, state_list, reward_list, done_list, bool(info.get("success", False)))


def collect_dataset(
    episodes: int,
    seed: int,
    render: bool,
    noise_std: float,
    max_steps: int | None = None,
    success_only: bool = False,
    max_attempts: int | None = None,
    reset_mode: str = "near",
):
    env_kwargs = {}
    if max_steps is not None:
        env_kwargs["max_steps"] = max_steps
    env_kwargs["pusher_reset_mode"] = reset_mode
    env = PushCube2DEnv(**env_kwargs)
    policy = make_policy_from_env(env, noise_std=noise_std, seed=seed)
    demos = []
    attempts = 0
    max_attempts = max_attempts or (episodes * (20 if success_only else 1))
    try:
        while len(demos) < episodes and attempts < max_attempts:
            demo = rollout_episode(env, policy, seed=seed + attempts, render=render)
            attempts += 1
            if success_only and not demo["success"]:
                print(
                    f"attempt {attempts:04d}/{max_attempts:04d} "
                    f"discarded len={len(demo['actions']):03d} success=0"
                )
                continue
            demos.append(demo)
            print(
                f"episode {len(demos):04d}/{episodes:04d} "
                f"attempt={attempts:04d} len={len(demo['actions']):03d} success={int(demo['success'])}"
            )
    finally:
        env.close()
    if len(demos) < episodes:
        raise RuntimeError(f"Collected {len(demos)} episodes after {attempts} attempts; requested {episodes}.")
    return demos, env


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--render", type=str2bool, default=False)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--noise-std", type=float, default=0.02)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--success-only", type=str2bool, default=False)
    parser.add_argument("--max-attempts", type=int, default=None)
    parser.add_argument("--reset-mode", choices=["aligned", "near", "random"], default="near")
    args = parser.parse_args()

    out = args.out or default_output_path()
    demos, env = collect_dataset(
        args.episodes,
        args.seed,
        args.render,
        args.noise_std,
        args.max_steps,
        success_only=args.success_only,
        max_attempts=args.max_attempts,
        reset_mode=args.reset_mode,
    )
    metadata = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "env_config": env.config.__dict__,
        "collector": "scripted_policy",
        "seed": args.seed,
        "noise_std": args.noise_std,
        "success_only": args.success_only,
        "reset_mode": args.reset_mode,
    }
    path = save_dataset(out, demos, metadata)
    success_rate = float(np.mean([demo["success"] for demo in demos]))
    print(f"saved {len(demos)} episodes to {path}")
    print(f"success_rate={success_rate:.3f}")


if __name__ == "__main__":
    main()
