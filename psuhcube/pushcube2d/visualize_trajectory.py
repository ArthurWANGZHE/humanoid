"""Roll out or load a PushCube2D trajectory and save a plot."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

from .collect_data import rollout_episode
from .dataset import load_dataset
from .env import PushCube2DEnv
from .scripted_policy import make_policy_from_env
from .train_bc import load_policy_model, predict_action


def default_out() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("outputs") / "pushcube2d" / f"trajectory_{stamp}.png"


def require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for visualization. Install it with `pip install matplotlib`.") from exc
    return plt


def episode_from_dataset(path: Path, index: int):
    data = load_dataset(path)
    length = int(data["episode_lengths"][index])
    return {
        "obs": data["obs"][index, :length],
        "actions": data["actions"][index, :length],
        "states": data["states"][index, :length],
        "rewards": data["rewards"][index, :length],
        "dones": data["dones"][index, :length],
        "success": bool(data["success"][index]),
    }


def rollout_model(model_path: Path, seed: int, reset_mode: str = "near"):
    env = PushCube2DEnv(pusher_reset_mode=reset_mode)
    model = load_policy_model(model_path)
    obs, _ = env.reset(seed=seed)
    obs_list, action_list, state_list, reward_list, done_list = [], [], [], [], []
    info = {}
    for _ in range(env.config.max_steps):
        action = predict_action(model, obs)
        state = env.get_state()
        next_obs, reward, terminated, truncated, info = env.step(action)
        done = bool(terminated or truncated)
        obs_list.append(obs.copy())
        action_list.append(action.copy())
        state_list.append(state.copy())
        reward_list.append(float(reward))
        done_list.append(done)
        obs = next_obs
        if done:
            break
    env.close()
    return {"obs": np.asarray(obs_list), "actions": np.asarray(action_list), "states": np.asarray(state_list), "rewards": np.asarray(reward_list), "dones": np.asarray(done_list), "success": bool(info.get("success", False))}


def plot_episode(episode, out: Path, title: str) -> Path:
    plt = require_matplotlib()
    obs = np.asarray(episode["obs"])
    pusher = obs[:, 0:2]
    cube = obs[:, 2:4]
    goal = obs[0, 5:7]

    out.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title(title)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.plot(pusher[:, 0], pusher[:, 1], color="#d95f4f", linewidth=1.7, label="pusher")
    ax.plot(cube[:, 0], cube[:, 1], color="#467bc2", linewidth=2.0, label="cube")
    ax.scatter(pusher[0, 0], pusher[0, 1], color="#d95f4f", marker="o", s=35)
    ax.scatter(cube[0, 0], cube[0, 1], color="#467bc2", marker="s", s=60)
    ax.scatter(cube[-1, 0], cube[-1, 1], color="#1f4f9a", marker="s", s=80, label="cube final")
    goal_circle = plt.Circle(goal, 0.055, color="#4f9f5f", alpha=0.22)
    ax.add_patch(goal_circle)
    ax.scatter(goal[0], goal[1], color="#2f7f45", marker="*", s=160, label="goal")

    every = max(1, len(pusher) // 20)
    actions = np.asarray(episode["actions"])
    ax.quiver(pusher[::every, 0], pusher[::every, 1], actions[::every, 0], actions[::every, 1], angles="xy", scale_units="xy", scale=18, color="#8f3d32", alpha=0.55)
    ax.legend(loc="upper left")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=["scripted", "dataset", "model"], default="scripted")
    parser.add_argument("--dataset", type=Path, default=None)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--reset-mode", choices=["aligned", "near", "random"], default="near")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    out = args.out or default_out()
    if args.source == "scripted":
        env = PushCube2DEnv(pusher_reset_mode=args.reset_mode)
        policy = make_policy_from_env(env, seed=args.seed)
        episode = rollout_episode(env, policy, seed=args.seed, render=False)
        env.close()
        title = f"Scripted rollout ({args.reset_mode}), success={int(episode['success'])}"
    elif args.source == "dataset":
        if args.dataset is None:
            raise SystemExit("--dataset is required for --source dataset")
        episode = episode_from_dataset(args.dataset, args.episode)
        title = f"Dataset episode {args.episode}, success={int(episode['success'])}"
    else:
        if args.model is None:
            raise SystemExit("--model is required for --source model")
        episode = rollout_model(args.model, args.seed, reset_mode=args.reset_mode)
        title = f"Model rollout ({args.reset_mode}), success={int(episode['success'])}"

    path = plot_episode(episode, out, title)
    print(f"saved trajectory plot to {path}")


if __name__ == "__main__":
    main()
