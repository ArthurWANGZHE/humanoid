"""Filter a PushCube dataset by success and quality heuristics."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    PushCubeWriter,
    compute_episode_metrics,
    created_at_timestamp,
    load_all_episodes,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Filter a PushCube dataset.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    parser.add_argument("--success-only", action="store_true")
    parser.add_argument("--keep-failed", action="store_true")
    parser.add_argument("--min-episode-length", type=int, default=20)
    parser.add_argument("--min-cube-motion", type=float, default=0.03)
    parser.add_argument("--max-final-distance", type=float, default=0.15)
    parser.add_argument("--max-action-saturation-fraction", type=float, default=0.25)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    metadata, episodes = load_all_episodes(args.input)
    keep_failed = bool(args.keep_failed) and not bool(args.success_only)
    writer = PushCubeWriter(
        args.output,
        {
            **metadata,
            "created_at": created_at_timestamp(),
            "success_only": bool(not keep_failed),
            "source_dataset": str(args.input),
        },
    )
    kept = 0
    rejected = 0
    rejection_reasons: Counter[str] = Counter()
    for episode_name, episode in episodes:
        metric = compute_episode_metrics(episode)
        reasons: list[str] = []
        if not keep_failed and not metric.success:
            reasons.append("failed")
        if metric.length < int(args.min_episode_length):
            reasons.append("too_short")
        if metric.cube_motion < float(args.min_cube_motion):
            reasons.append("cube_motion_too_small")
        if metric.final_cube_to_target_distance > float(args.max_final_distance):
            reasons.append("final_distance_too_large")
        if metric.nan_count > 0:
            reasons.append("contains_nan")
        if metric.action_saturation_fraction > float(args.max_action_saturation_fraction):
            reasons.append("action_saturated")
        if reasons:
            rejected += 1
            for reason in reasons:
                rejection_reasons[reason] += 1
            print(f"reject {episode_name}: {', '.join(reasons)}")
            continue
        writer.write_episode(episode)
        kept += 1
        print(f"keep {episode_name}")

    print(f"kept episodes: {kept}")
    print(f"rejected episodes: {rejected}")
    print("rejection reasons:")
    for reason, count in sorted(rejection_reasons.items()):
        print(f"  {reason}: {count}")


if __name__ == "__main__":
    main()

