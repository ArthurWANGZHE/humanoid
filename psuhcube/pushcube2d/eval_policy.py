"""Evaluate a trained PushCube2D behavior-cloning policy."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .collect_data import str2bool
from .env import PushCube2DEnv
from .train_bc import load_policy_model, predict_action


def latest_model() -> Path:
    candidates = sorted((Path("models") / "pushcube2d").glob("bc_*.npz"))
    if not candidates:
        raise FileNotFoundError("No model found under models/pushcube2d. Pass --model explicitly.")
    return candidates[-1]


def evaluate(model_path: Path, episodes: int, seed: int, render: bool, reset_mode: str):
    model = load_policy_model(model_path)
    env = PushCube2DEnv(pusher_reset_mode=reset_mode)
    successes, final_distances, lengths, episode_rows = [], [], [], []
    try:
        for ep in range(episodes):
            obs, _ = env.reset(seed=seed + ep)
            info = {}
            for t in range(env.config.max_steps):
                action = predict_action(model, obs)
                obs, _reward, terminated, truncated, info = env.step(action)
                if render:
                    env.render("human")
                if terminated or truncated:
                    break
            successes.append(bool(info.get("success", False)))
            final_distances.append(float(info.get("distance_to_goal", env.distance_to_goal())))
            lengths.append(t + 1)
            episode_rows.append(
                {
                    "episode": ep,
                    "seed": seed + ep,
                    "length": int(t + 1),
                    "success": bool(successes[-1]),
                    "final_distance": float(final_distances[-1]),
                }
            )
            print(f"episode {ep + 1:04d}/{episodes:04d} len={t + 1:03d} success={int(successes[-1])} final_dist={final_distances[-1]:.4f}")
    finally:
        env.close()
    aggregate = {
        "success_rate": float(np.mean(successes)),
        "mean_final_distance": float(np.mean(final_distances)),
        "mean_length": float(np.mean(lengths)),
        "episodes": int(episodes),
    }
    return aggregate, episode_rows


def save_eval_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--render", type=str2bool, default=False)
    parser.add_argument("--reset-mode", choices=["aligned", "near", "random"], default="near")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--label", type=str, default="")
    parser.add_argument("--train-reset-mode", choices=["aligned", "near", "random"], default=None)
    parser.add_argument("--train-variant", type=str, default="")
    args = parser.parse_args()

    model_path = args.model or latest_model()
    metrics, episode_rows = evaluate(model_path, args.episodes, args.seed, args.render, args.reset_mode)
    print(f"model={model_path}")
    print(f"reset_mode={args.reset_mode}")
    print(f"success_rate={metrics['success_rate']:.3f}")
    print(f"mean_final_distance={metrics['mean_final_distance']:.4f}")
    print(f"mean_length={metrics['mean_length']:.1f}")
    if args.out is not None:
        payload = {
            "label": args.label or model_path.stem,
            "model": str(model_path),
            "train_reset_mode": args.train_reset_mode,
            "train_variant": args.train_variant,
            "eval_reset_mode": args.reset_mode,
            "seed": args.seed,
            "aggregate": metrics,
            "episodes": episode_rows,
        }
        save_eval_json(args.out, payload)
        print(f"saved eval json to {args.out}")


if __name__ == "__main__":
    main()
