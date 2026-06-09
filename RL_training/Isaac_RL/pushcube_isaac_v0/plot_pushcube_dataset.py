"""Plot top-down PushCube trajectories and summary charts."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    compute_episode_metrics,
    load_all_episodes,
    optional_import,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot PushCube dataset trajectories and summaries.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--output-dir", type=str, required=True)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--episode", type=int, default=None)
    parser.add_argument("--save-video", nargs="?", const="", default=None)
    return parser


def _parse_table_bounds(metadata: dict[str, object]) -> tuple[float, float, float, float]:
    bounds = metadata.get("table_bounds")
    if isinstance(bounds, str):
        import json

        bounds = json.loads(bounds)
    if not isinstance(bounds, dict):
        return (0.20, 0.76, -0.52, 0.04)
    return (
        float(bounds["x_min"]),
        float(bounds["x_max"]),
        float(bounds["y_min"]),
        float(bounds["y_max"]),
    )


def _trajectory_plot(plt, episode_name: str, episode, metadata: dict[str, object], output_path: Path) -> None:
    x_min, x_max, y_min, y_max = _parse_table_bounds(metadata)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.add_patch(plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, linewidth=2.0, color="black"))
    cube_xy = episode.cube_pose[:, :2]
    ee_xy = episode.ee_pose[:, :2]
    target_xy = episode.target_pose[-1, :2]
    target_size_xy = episode.obs[-1, 7:9]
    ax.plot(cube_xy[:, 0], cube_xy[:, 1], color="#d55e00", linewidth=2.0, label="cube")
    ax.plot(ee_xy[:, 0], ee_xy[:, 1], color="#0072b2", linewidth=1.5, label="ee")
    ax.scatter(cube_xy[0, 0], cube_xy[0, 1], color="#d55e00", marker="o", label="cube start")
    ax.scatter(cube_xy[-1, 0], cube_xy[-1, 1], color="#d55e00", marker="X", label="cube end")
    ax.scatter(ee_xy[0, 0], ee_xy[0, 1], color="#0072b2", marker="o", label="ee start")
    ax.scatter(ee_xy[-1, 0], ee_xy[-1, 1], color="#0072b2", marker="X", label="ee end")
    ax.add_patch(
        plt.Rectangle(
            (
                float(target_xy[0] - (target_size_xy[0] / 2.0)),
                float(target_xy[1] - (target_size_xy[1] / 2.0)),
            ),
            float(target_size_xy[0]),
            float(target_size_xy[1]),
            fill=False,
            linewidth=2.0,
            linestyle="--",
            color="#009e73",
            label="target",
        )
    )
    ax.set_title(episode_name)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def _summary_plots(plt, episodes, output_dir: Path) -> None:
    metrics = [(name, compute_episode_metrics(episode)) for name, episode in episodes]
    lengths = np.array([metric.length for _, metric in metrics], dtype=np.float64)
    final_distances = np.array([metric.final_cube_to_target_distance for _, metric in metrics], dtype=np.float64)
    progress = np.array([metric.cube_progress for _, metric in metrics], dtype=np.float64)
    success = np.array([metric.success for _, metric in metrics], dtype=np.bool_)
    actions = np.concatenate([episode.action.reshape(-1) for _, episode in episodes], axis=0)

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    axes = axes.reshape(-1)
    axes[0].hist(lengths, bins=min(20, len(lengths)))
    axes[0].set_title("Episode Length")
    axes[1].hist(final_distances, bins=min(20, len(final_distances)))
    axes[1].set_title("Final Cube-Target Distance")
    axes[2].hist(progress, bins=min(20, len(progress)))
    axes[2].set_title("Cube Progress")
    axes[3].hist(actions, bins=40)
    axes[3].set_title("Action Distribution")
    axes[4].bar(["success", "failure"], [int(np.sum(success)), int(np.sum(~success))], color=["#009e73", "#cc79a7"])
    axes[4].set_title("Success / Failure")
    for name, episode in episodes:
        if not bool(episode.success):
            continue
        cube_xy = episode.cube_pose[:, :2]
        axes[5].plot(cube_xy[:, 0], cube_xy[:, 1], alpha=0.5)
    axes[5].set_title("Successful Cube Trajectory Overlay")
    for axis in axes:
        axis.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(output_dir / "summary.png", dpi=160)
    plt.close(fig)


def _save_video(metadata: dict[str, object], episode_name: str, episode, video_path: Path) -> None:
    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")
    imageio = optional_import("imageio", "pip install imageio")
    x_min, x_max, y_min, y_max = _parse_table_bounds(metadata)
    frames: list[np.ndarray] = []
    for step in range(episode.obs.shape[0]):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.add_patch(plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, linewidth=2.0, color="black"))
        cube_xy = episode.cube_pose[: step + 1, :2]
        ee_xy = episode.ee_pose[: step + 1, :2]
        target_xy = episode.target_pose[-1, :2]
        target_size_xy = episode.obs[-1, 7:9]
        ax.plot(cube_xy[:, 0], cube_xy[:, 1], color="#d55e00", linewidth=2.0)
        ax.plot(ee_xy[:, 0], ee_xy[:, 1], color="#0072b2", linewidth=1.5)
        ax.add_patch(
            plt.Rectangle(
                (
                    float(target_xy[0] - (target_size_xy[0] / 2.0)),
                    float(target_xy[1] - (target_size_xy[1] / 2.0)),
                ),
                float(target_size_xy[0]),
                float(target_size_xy[1]),
                fill=False,
                linewidth=2.0,
                linestyle="--",
                color="#009e73",
            )
        )
        ax.set_title(f"{episode_name} step {step}")
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.canvas.draw()
        frame = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
        frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (4,))[:, :, :3]
        frames.append(frame.copy())
        plt.close(fig)
    video_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(video_path, frames, fps=10)


def main() -> None:
    args = build_arg_parser().parse_args()
    metadata, episodes = load_all_episodes(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")

    if args.all:
        for idx, (episode_name, episode) in enumerate(episodes):
            print(f"[{idx+1}/{len(episodes)}] plotting {episode_name}...")
            _trajectory_plot(plt, episode_name, episode, metadata, output_dir / f"{episode_name}.png")
            if args.save_video is not None:
                video_path = output_dir / "videos" / f"{episode_name}.mp4"
                print(f"  saving video -> {video_path}")
                _save_video(metadata, episode_name, episode, video_path)
        _summary_plots(plt, episodes, output_dir)

    if args.episode is not None:
        episode_name, episode = episodes[int(args.episode)]
        _trajectory_plot(plt, episode_name, episode, metadata, output_dir / f"{episode_name}.png")
        if args.save_video is not None:
            video_path = Path(args.save_video) if args.save_video else (output_dir / f"{episode_name}.mp4")
            _save_video(metadata, episode_name, episode, video_path)


if __name__ == "__main__":
    main()

