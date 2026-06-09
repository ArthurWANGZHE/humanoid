"""Generate a markdown quality report for a PushCube HDF5 dataset."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_stats import summarize_dataset  # noqa: E402
from pushcube_isaac_v0.pushcube_dataset_utils import load_all_episodes  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create a markdown quality report for a PushCube dataset.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    parser.add_argument("--min-episode-length", type=int, default=100)
    parser.add_argument("--min-cube-motion", type=float, default=0.03)
    parser.add_argument("--max-final-distance", type=float, default=0.12)
    parser.add_argument("--max-action-saturation", type=float, default=0.35)
    parser.add_argument("--min-ee-motion", type=float, default=0.03)
    return parser


def _format_bool(value: bool) -> str:
    return "yes" if value else "no"


def main() -> None:
    args = build_arg_parser().parse_args()
    metadata, episodes = load_all_episodes(args.input)
    dataset_summary, summaries = summarize_dataset(metadata, episodes, low_motion_threshold=float(args.min_cube_motion))
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    warning_counts: Counter[str] = Counter()
    pass_count = 0
    demo_rows: list[str] = []
    warnings: list[str] = []

    for summary in summaries:
        reasons: list[str] = []
        if summary.length < int(args.min_episode_length):
            reasons.append("short_episode")
        if summary.cube_max_displacement <= float(args.min_cube_motion):
            reasons.append("stationary_cube")
        if summary.final_cube_to_target_distance >= float(args.max_final_distance):
            reasons.append("large_final_distance")
        if summary.action_saturation_ratio >= float(args.max_action_saturation):
            reasons.append("action_saturation")
        if summary.nan_count > 0 or summary.inf_count > 0:
            reasons.append("non_finite_values")
        if summary.ee_total_motion <= float(args.min_ee_motion):
            reasons.append("stationary_ee")
        if not reasons:
            pass_count += 1
        for reason in reasons:
            warning_counts[reason] += 1
        demo_rows.append(
            "| {name} | {length} | {success} | {cube_motion:.4f} | {final_distance:.4f} | {ee_motion:.4f} | {sat:.4f} | {non_finite} | {reasons} |".format(
                name=summary.name,
                length=summary.length,
                success=_format_bool(summary.success),
                cube_motion=summary.cube_max_displacement,
                final_distance=summary.final_cube_to_target_distance,
                ee_motion=summary.ee_total_motion,
                sat=summary.action_saturation_ratio,
                non_finite=summary.nan_count + summary.inf_count,
                reasons=", ".join(reasons) if reasons else "pass",
            )
        )

    if not warning_counts:
        warnings.append("- No quality warnings were triggered with the current thresholds.")
    else:
        for reason, count in sorted(warning_counts.items()):
            warnings.append(f"- `{reason}`: {count} demos")

    notes = [
        f"- The raw manual dataset contains {dataset_summary['num_demos']} real demonstrations; any later augmentation should be reported separately.",
        f"- With the proposed thresholds, {pass_count} / {dataset_summary['num_demos']} demos are immediately usable for BC training.",
        "- The thesis should distinguish task success from trajectory quality: a successful demo can still be too short, overly saturated, or nearly stationary.",
        "- 2D trajectory figures support quantitative claims, while 3D Isaac renders should be reserved for qualitative examples and failure-case discussion.",
    ]

    markdown = "\n".join(
        [
            "# PushCube Dataset Quality Report",
            "",
            "## Dataset Overview",
            "",
            f"- Input dataset: `{args.input}`",
            f"- Number of demos: {dataset_summary['num_demos']}",
            f"- Success count: {dataset_summary['success_count']}",
            f"- Episode length min / mean / max: {int(dataset_summary['episode_length']['min'])} / {dataset_summary['episode_length']['mean']:.2f} / {int(dataset_summary['episode_length']['max'])}",
            f"- Cube final-distance mean: {dataset_summary['final_cube_to_target_distance']['mean']:.4f} m",
            f"- Cube-motion mean: {dataset_summary['cube_max_displacement']['mean']:.4f} m",
            "",
            "## Demo Table",
            "",
            "| demo | len | success | cube_motion | final_dist | ee_motion | action_sat | non_finite | status |",
            "| --- | ---: | :---: | ---: | ---: | ---: | ---: | ---: | --- |",
            *demo_rows,
            "",
            "## Warning List",
            "",
            *warnings,
            "",
            "## Recommended Filtering Thresholds",
            "",
            f"- Minimum episode length: `{int(args.min_episode_length)}` steps",
            f"- Minimum cube motion: `{float(args.min_cube_motion):.3f}` m",
            f"- Maximum final cube-to-target distance: `{float(args.max_final_distance):.3f}` m",
            f"- Maximum action saturation ratio: `{float(args.max_action_saturation):.3f}`",
            f"- Minimum end-effector motion: `{float(args.min_ee_motion):.3f}` m",
            "",
            "## Suggested Number of Usable Demos",
            "",
            f"- Suggested usable demos with the above thresholds: **{pass_count}**",
            f"- Demos needing review or removal: **{dataset_summary['num_demos'] - pass_count}**",
            "",
            "## Notes for Thesis Writing",
            "",
            *notes,
            "",
        ]
    )
    output_path.write_text(markdown, encoding="utf-8")
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()

