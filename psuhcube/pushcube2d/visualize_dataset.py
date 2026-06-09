"""Create sanity-check plots for a PushCube2D dataset."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

from .dataset import load_dataset, valid_mask


def default_out() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("outputs") / "pushcube2d" / f"dataset_summary_{stamp}.png"


def require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for visualization. Install it with `pip install matplotlib`.") from exc
    return plt


def visualize_dataset(path: Path, out: Path, max_episodes: int) -> Path:
    plt = require_matplotlib()
    data = load_dataset(path)
    lengths = data["episode_lengths"].astype(int)
    success = data["success"].astype(bool)
    mask = valid_mask(data)
    obs = data["obs"]
    actions = data["actions"][mask]
    distances = np.linalg.norm(obs[..., 2:4] - obs[..., 5:7], axis=-1)
    final_distances = np.array([distances[i, lengths[i] - 1] for i in range(len(lengths))])

    out.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    ax = axes[0, 0]
    ax.set_title("Trajectories")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    for i in range(min(max_episodes, len(lengths))):
        length = lengths[i]
        color = "#2f7f45" if success[i] else "#a44f45"
        ax.plot(obs[i, :length, 2], obs[i, :length, 3], color=color, alpha=0.65, linewidth=1.4)
        ax.plot(obs[i, :length, 0], obs[i, :length, 1], color="#777777", alpha=0.25, linewidth=1.0)
        ax.scatter(obs[i, 0, 5], obs[i, 0, 6], color="#2f7f45", marker="*", s=45, alpha=0.7)
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    ax = axes[0, 1]
    ax.set_title("Episode Lengths")
    ax.hist(lengths, bins=min(20, max(5, len(lengths))), color="#5d83bd", edgecolor="white")
    ax.set_xlabel("steps")
    ax.set_ylabel("episodes")

    ax = axes[1, 0]
    ax.set_title("Final Cube-Goal Distance")
    ax.hist(final_distances, bins=min(20, max(5, len(final_distances))), color="#6aa56a", edgecolor="white")
    ax.axvline(0.055, color="#333333", linestyle="--", linewidth=1.2, label="default success threshold")
    ax.set_xlabel("distance")
    ax.legend()

    ax = axes[1, 1]
    ax.set_title("Action Distribution")
    ax.hist(actions[:, 0], bins=30, alpha=0.65, label="dx", color="#d95f4f")
    ax.hist(actions[:, 1], bins=30, alpha=0.65, label="dy", color="#467bc2")
    ax.set_xlabel("normalized action")
    ax.legend()

    fig.suptitle(f"{path} | episodes={len(lengths)} success_rate={success.mean():.3f}", y=0.995)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--max-episodes", type=int, default=40)
    args = parser.parse_args()

    path = visualize_dataset(args.dataset, args.out or default_out(), args.max_episodes)
    print(f"saved dataset visualization to {path}")


if __name__ == "__main__":
    main()

