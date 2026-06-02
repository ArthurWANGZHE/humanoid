"""Dataset analysis and thesis plotting for PushCube HDF5 demos."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_stats import (  # noqa: E402
    OBSERVATION_LABELS,
    build_layout_regions,
    summarize_dataset,
)
from pushcube_isaac_v0.pushcube_dataset_utils import load_all_episodes, optional_import  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze a PushCube dataset and generate thesis figures.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--output-dir", type=str, required=True)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--episode", type=int, default=None)
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--save-pdf", action="store_true")
    parser.add_argument("--low-motion-threshold", type=float, default=0.03)
    return parser


def _save_figure(fig, base_path: Path, *, dpi: int, save_pdf: bool) -> None:
    fig.tight_layout()
    fig.savefig(base_path.with_suffix(".png"), dpi=dpi, bbox_inches="tight")
    if save_pdf:
        fig.savefig(base_path.with_suffix(".pdf"), bbox_inches="tight")


def _draw_layout(ax, layout: dict[str, object], plt) -> None:
    bounds = layout["table_bounds"]
    ax.add_patch(
        plt.Rectangle(
            (bounds["x_min"], bounds["y_min"]),
            bounds["x_max"] - bounds["x_min"],
            bounds["y_max"] - bounds["y_min"],
            fill=False,
            linewidth=2.0,
            color="black",
            label="table",
        )
    )
    spawn_center_xy = np.asarray(layout["spawn_center_xy"], dtype=np.float64)
    spawn_size_xy = np.asarray(layout["spawn_size_xy"], dtype=np.float64)
    target_center_xy = np.asarray(layout["target_center_xy"], dtype=np.float64)
    target_size_xy = np.asarray(layout["target_size_xy"], dtype=np.float64)
    ax.add_patch(
        plt.Rectangle(
            (spawn_center_xy[0] - (spawn_size_xy[0] / 2.0), spawn_center_xy[1] - (spawn_size_xy[1] / 2.0)),
            spawn_size_xy[0],
            spawn_size_xy[1],
            facecolor="#4c72b0",
            edgecolor="#1f4d8f",
            alpha=0.12,
            linewidth=1.5,
            label="spawn region",
        )
    )
    ax.add_patch(
        plt.Rectangle(
            (target_center_xy[0] - (target_size_xy[0] / 2.0), target_center_xy[1] - (target_size_xy[1] / 2.0)),
            target_size_xy[0],
            target_size_xy[1],
            facecolor="#55a868",
            edgecolor="#2f7f3f",
            alpha=0.14,
            linewidth=1.5,
            label="target region",
        )
    )
    ax.set_xlim(bounds["x_min"] - 0.02, bounds["x_max"] + 0.02)
    ax.set_ylim(bounds["y_min"] - 0.02, bounds["y_max"] + 0.02)


def _plot_all_overlay(plt, episodes, layout, output_dir: Path, *, dpi: int, save_pdf: bool) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 6.8))
    _draw_layout(ax, layout, plt)
    for index, (name, episode) in enumerate(episodes):
        cube_xy = np.asarray(episode.cube_pose[:, :2], dtype=np.float64)
        ee_xy = np.asarray(episode.ee_pose[:, :2], dtype=np.float64)
        cube_label = "cube trajectory" if index == 0 else None
        ee_label = "ee trajectory" if index == 0 else None
        ax.plot(cube_xy[:, 0], cube_xy[:, 1], color="#dd8452", linewidth=1.5, alpha=0.75, label=cube_label)
        ax.plot(ee_xy[:, 0], ee_xy[:, 1], color="#4c72b0", linewidth=1.0, alpha=0.45, label=ee_label)
        ax.scatter(cube_xy[0, 0], cube_xy[0, 1], color="#dd8452", s=22, marker="o")
        ax.scatter(cube_xy[-1, 0], cube_xy[-1, 1], color="#c44e52", s=30, marker="X")
        ax.scatter(ee_xy[0, 0], ee_xy[0, 1], color="#4c72b0", s=14, marker="o")
        ax.scatter(ee_xy[-1, 0], ee_xy[-1, 1], color="#1f4d8f", s=18, marker="X")
    ax.set_title("PushCube Demonstration Overlay")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=9)
    _save_figure(fig, output_dir / "all_demo_topdown_overlay", dpi=dpi, save_pdf=save_pdf)
    plt.close(fig)


def _plot_episode_trajectory(plt, episode_name: str, episode, summary_row: dict[str, object], layout, output_path: Path, *, dpi: int, save_pdf: bool) -> None:
    fig, ax = plt.subplots(figsize=(7.4, 6.2))
    _draw_layout(ax, layout, plt)
    cube_xy = np.asarray(episode.cube_pose[:, :2], dtype=np.float64)
    ee_xy = np.asarray(episode.ee_pose[:, :2], dtype=np.float64)
    target_xy = np.asarray(episode.target_pose[-1, :2], dtype=np.float64)
    target_size_xy = np.asarray(episode.obs[-1, 7:9], dtype=np.float64)
    ax.add_patch(
        plt.Rectangle(
            (target_xy[0] - (target_size_xy[0] / 2.0), target_xy[1] - (target_size_xy[1] / 2.0)),
            target_size_xy[0],
            target_size_xy[1],
            fill=False,
            edgecolor="#2f7f3f",
            linewidth=1.8,
            linestyle="--",
        )
    )
    ax.plot(cube_xy[:, 0], cube_xy[:, 1], color="#dd8452", linewidth=2.0, label="cube")
    ax.plot(ee_xy[:, 0], ee_xy[:, 1], color="#4c72b0", linewidth=1.4, label="ee")
    ax.scatter(cube_xy[0, 0], cube_xy[0, 1], color="#dd8452", s=28, marker="o")
    ax.scatter(cube_xy[-1, 0], cube_xy[-1, 1], color="#c44e52", s=34, marker="X")
    ax.scatter(ee_xy[0, 0], ee_xy[0, 1], color="#4c72b0", s=18, marker="o")
    ax.scatter(ee_xy[-1, 0], ee_xy[-1, 1], color="#1f4d8f", s=24, marker="X")
    annotation = (
        f"len={summary_row['length']}\n"
        f"success={summary_row['success']}\n"
        f"final cube-target={float(summary_row['final_cube_to_target_distance']):.4f} m"
    )
    ax.text(
        0.02,
        0.98,
        annotation,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85, "edgecolor": "#bbbbbb"},
    )
    ax.set_title(episode_name)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower right", fontsize=9)
    _save_figure(fig, output_path, dpi=dpi, save_pdf=save_pdf)
    plt.close(fig)


def _hist_plot(plt, values: np.ndarray, title: str, xlabel: str, output_path: Path, *, dpi: int, save_pdf: bool, color: str) -> None:
    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    ax.hist(np.asarray(values, dtype=np.float64), bins=min(24, max(8, len(values))), color=color, edgecolor="white", alpha=0.9)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.25)
    _save_figure(fig, output_path, dpi=dpi, save_pdf=save_pdf)
    plt.close(fig)


def _plot_action_distributions(plt, episodes, output_dir: Path, *, dpi: int, save_pdf: bool) -> None:
    actions = np.concatenate([episode.action for _, episode in episodes], axis=0)
    _hist_plot(plt, actions[:, 0], "Action Distribution: x", "normalized action x", output_dir / "action_distribution_x", dpi=dpi, save_pdf=save_pdf, color="#4c72b0")
    _hist_plot(plt, actions[:, 1], "Action Distribution: y", "normalized action y", output_dir / "action_distribution_y", dpi=dpi, save_pdf=save_pdf, color="#55a868")
    fig, ax = plt.subplots(figsize=(5.8, 5.4))
    ax.scatter(actions[:, 0], actions[:, 1], s=8, alpha=0.25, color="#8172b3", edgecolors="none")
    ax.set_title("Action XY Scatter")
    ax.set_xlabel("normalized action x")
    ax.set_ylabel("normalized action y")
    ax.grid(True, alpha=0.25)
    ax.axvline(0.0, color="#999999", linewidth=0.8)
    ax.axhline(0.0, color="#999999", linewidth=0.8)
    _save_figure(fig, output_dir / "action_xy_scatter", dpi=dpi, save_pdf=save_pdf)
    plt.close(fig)


def _plot_obs_distribution(plt, episodes, output_dir: Path, *, dpi: int, save_pdf: bool) -> None:
    observations = np.concatenate([episode.obs for _, episode in episodes], axis=0)
    columns = min(observations.shape[1], len(OBSERVATION_LABELS))
    rows = int(np.ceil(columns / 3.0))
    fig, axes = plt.subplots(rows, 3, figsize=(12.0, 3.6 * rows))
    axes = np.atleast_1d(axes).reshape(-1)
    for index in range(columns):
        axis = axes[index]
        axis.hist(observations[:, index], bins=30, color="#937860", edgecolor="white", alpha=0.9)
        axis.set_title(OBSERVATION_LABELS[index])
        axis.grid(True, alpha=0.2)
    for index in range(columns, len(axes)):
        axes[index].axis("off")
    fig.suptitle("Observation Dimension Distributions", fontsize=14)
    _save_figure(fig, output_dir / "obs_distribution", dpi=dpi, save_pdf=save_pdf)
    plt.close(fig)


def _write_summary_csv(output_path: Path, per_episode_rows: list[dict[str, object]]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(per_episode_rows[0].keys())
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(per_episode_rows)


def _print_required_stats(dataset_summary: dict[str, object]) -> None:
    print(f"num demos: {dataset_summary['num_demos']}")
    print(f"success count: {dataset_summary['success_count']}")
    episode_length = dataset_summary["episode_length"]
    print(f"episode length min / mean / max: {int(episode_length['min'])} / {episode_length['mean']:.2f} / {int(episode_length['max'])}")
    print(f"obs_dim: {dataset_summary['obs_dim']}")
    print(f"action_dim: {dataset_summary['action_dim']}")
    action = dataset_summary["action"]
    print(f"action min / max / mean / std: min={action['min']} max={action['max']} mean={action['mean']} std={action['std']}")
    print(f"cube initial xy mean/std: mean={dataset_summary['cube_initial_xy']['mean']} std={dataset_summary['cube_initial_xy']['std']}")
    print(f"cube final xy mean/std: mean={dataset_summary['cube_final_xy']['mean']} std={dataset_summary['cube_final_xy']['std']}")
    print(f"target xy mean/std: mean={dataset_summary['target_xy']['mean']} std={dataset_summary['target_xy']['std']}")
    print(f"cube total motion per episode min / mean / max: {dataset_summary['cube_total_motion']['min']:.4f} / {dataset_summary['cube_total_motion']['mean']:.4f} / {dataset_summary['cube_total_motion']['max']:.4f}")
    print(f"cube progress toward target min / mean / max: {dataset_summary['cube_progress_toward_target']['min']:.4f} / {dataset_summary['cube_progress_toward_target']['mean']:.4f} / {dataset_summary['cube_progress_toward_target']['max']:.4f}")
    print(f"final cube-to-target distance min / mean / max: {dataset_summary['final_cube_to_target_distance']['min']:.4f} / {dataset_summary['final_cube_to_target_distance']['mean']:.4f} / {dataset_summary['final_cube_to_target_distance']['max']:.4f}")
    print(f"final ee-to-cube distance min / mean / max: {dataset_summary['final_ee_to_cube_distance']['min']:.4f} / {dataset_summary['final_ee_to_cube_distance']['mean']:.4f} / {dataset_summary['final_ee_to_cube_distance']['max']:.4f}")
    print(f"number of demos with very low cube motion: {dataset_summary['very_low_cube_motion_count']}")
    print(f"number of demos with NaN: {dataset_summary['demos_with_nan']}")
    if dataset_summary["normalized_actions"]:
        print(f"action saturation ratio if actions normalized: {dataset_summary['action_saturation_ratio']:.6f}")
    else:
        print("action saturation ratio if actions normalized: not reported (actions appear unnormalized)")


def main() -> None:
    args = build_arg_parser().parse_args()
    metadata, episodes = load_all_episodes(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    dataset_summary, summaries = summarize_dataset(metadata, episodes, low_motion_threshold=float(args.low_motion_threshold))
    per_episode_rows = dataset_summary["per_episode"]
    _print_required_stats(dataset_summary)

    with (output_dir / "dataset_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(dataset_summary, handle, indent=2, sort_keys=True)
    _write_summary_csv(output_dir / "dataset_summary.csv", per_episode_rows)

    if not args.all and args.episode is None:
        return

    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")
    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "legend.fontsize": 9,
            "figure.titlesize": 14,
        }
    )
    layout = build_layout_regions(metadata, episodes)
    summary_lookup = {row["name"]: row for row in per_episode_rows}

    if args.all:
        _plot_all_overlay(plt, episodes, layout, output_dir, dpi=int(args.dpi), save_pdf=bool(args.save_pdf))
        demo_dir = output_dir / "demo_each_trajectory"
        demo_dir.mkdir(parents=True, exist_ok=True)
        for episode_name, episode in episodes:
            _plot_episode_trajectory(
                plt,
                episode_name,
                episode,
                summary_lookup[episode_name],
                layout,
                demo_dir / episode_name,
                dpi=int(args.dpi),
                save_pdf=bool(args.save_pdf),
            )
        _hist_plot(
            plt,
            np.array([row["length"] for row in per_episode_rows], dtype=np.float64),
            "Episode Length Distribution",
            "episode length (steps)",
            output_dir / "episode_length_hist",
            dpi=int(args.dpi),
            save_pdf=bool(args.save_pdf),
            color="#4c72b0",
        )
        _hist_plot(
            plt,
            np.array([row["final_cube_to_target_distance"] for row in per_episode_rows], dtype=np.float64),
            "Final Cube-to-Target Distance",
            "distance (m)",
            output_dir / "final_distance_hist",
            dpi=int(args.dpi),
            save_pdf=bool(args.save_pdf),
            color="#c44e52",
        )
        _hist_plot(
            plt,
            np.array([row["cube_total_motion"] for row in per_episode_rows], dtype=np.float64),
            "Cube Total Motion",
            "path length (m)",
            output_dir / "cube_motion_hist",
            dpi=int(args.dpi),
            save_pdf=bool(args.save_pdf),
            color="#dd8452",
        )
        _hist_plot(
            plt,
            np.array([row["cube_progress"] for row in per_episode_rows], dtype=np.float64),
            "Cube Progress Toward Target",
            "progress (m)",
            output_dir / "cube_progress_hist",
            dpi=int(args.dpi),
            save_pdf=bool(args.save_pdf),
            color="#55a868",
        )
        _plot_action_distributions(plt, episodes, output_dir, dpi=int(args.dpi), save_pdf=bool(args.save_pdf))
        _plot_obs_distribution(plt, episodes, output_dir, dpi=int(args.dpi), save_pdf=bool(args.save_pdf))

    if args.episode is not None:
        episode_name, episode = episodes[int(args.episode)]
        _plot_episode_trajectory(
            plt,
            episode_name,
            episode,
            summary_lookup[episode_name],
            layout,
            output_dir / episode_name,
            dpi=int(args.dpi),
            save_pdf=bool(args.save_pdf),
        )


if __name__ == "__main__":
    main()

