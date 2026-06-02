"""Inspect a PushCube HDF5 dataset and print summary statistics."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    aggregate_action_stats,
    compute_episode_metrics,
    load_all_episodes,
    summarize_lengths,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a PushCube dataset.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--short-episode-threshold", type=int, default=20)
    parser.add_argument("--action-saturation-threshold", type=float, default=0.25)
    parser.add_argument("--cube-motion-threshold", type=float, default=0.03)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    metadata, episodes = load_all_episodes(args.input)
    if not episodes:
        raise RuntimeError("Dataset contains no demos.")

    lengths = summarize_lengths(episodes)
    action_stats = aggregate_action_stats(episodes)
    metrics = [(name, compute_episode_metrics(episode)) for name, episode in episodes]

    print("top-level keys:", ["data"])
    print("metadata attrs:")
    for key in sorted(metadata.keys()):
        print(f"  {key}: {metadata[key]}")
    print(f"num demos: {len(episodes)}")
    print(f"obs shape example: {episodes[0][1].obs.shape}")
    print(f"action shape example: {episodes[0][1].action.shape}")
    print(
        "episode length min/mean/max:",
        f"{int(lengths['min'])}/{lengths['mean']:.2f}/{int(lengths['max'])}",
    )
    success_count = sum(1 for _, metric in metrics if metric.success)
    print(f"success count: {success_count}")
    print(f"NaN count: {sum(metric.nan_count for _, metric in metrics)}")
    print(
        "action min/max/mean/std:",
        {
            "min": np.round(action_stats["min"], 6).tolist(),
            "max": np.round(action_stats["max"], 6).tolist(),
            "mean": np.round(action_stats["mean"], 6).tolist(),
            "std": np.round(action_stats["std"], 6).tolist(),
        },
    )
    print("per-episode stats:")
    for name, metric in metrics:
        print(
            f"  {name}: "
            f"len={metric.length} success={metric.success} "
            f"cube_initial_xy={np.round(metric.cube_initial_xy, 4).tolist()} "
            f"cube_final_xy={np.round(metric.cube_final_xy, 4).tolist()} "
            f"cube_progress={metric.cube_progress:.4f} "
            f"final_cube_to_target_distance={metric.final_cube_to_target_distance:.4f} "
            f"cube_motion={metric.cube_motion:.4f} "
            f"gripper_to_cube_mean={None if metric.gripper_to_cube_distance_mean is None else round(metric.gripper_to_cube_distance_mean, 4)} "
            f"action_saturation_fraction={metric.action_saturation_fraction:.4f}"
        )
        if metric.length < int(args.short_episode_threshold):
            print(f"    warning: short episode ({metric.length} < {args.short_episode_threshold})")
        if metric.action_saturation_fraction > float(args.action_saturation_threshold):
            print(
                f"    warning: action saturation high "
                f"({metric.action_saturation_fraction:.3f} > {float(args.action_saturation_threshold):.3f})"
            )
        if metric.cube_motion < float(args.cube_motion_threshold):
            print(
                f"    warning: cube barely moves "
                f"({metric.cube_motion:.3f} < {float(args.cube_motion_threshold):.3f})"
            )


if __name__ == "__main__":
    main()

