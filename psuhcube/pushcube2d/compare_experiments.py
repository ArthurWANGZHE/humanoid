"""Aggregate PushCube2D evaluation JSON files into tables and paper figures."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List

import numpy as np


RESET_ORDER = ["aligned", "near", "random"]
VARIANT_ORDER = ["all_data", "success_only"]


def require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for comparison plots. Install it with `pip install matplotlib`.") from exc
    return plt


def set_paper_style(plt) -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 160,
            "savefig.dpi": 300,
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.family": "DejaVu Sans",
        }
    )


def load_eval_rows(eval_dir: Path) -> List[Dict]:
    rows = []
    for path in sorted(eval_dir.glob("*.json")):
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        aggregate = data.get("aggregate", {})
        rows.append(
            {
                "file": str(path),
                "label": data.get("label", path.stem),
                "model": data.get("model", ""),
                "train_reset_mode": data.get("train_reset_mode") or infer_train_reset(data.get("label", path.stem)),
                "train_variant": data.get("train_variant") or infer_variant(data.get("label", path.stem)),
                "eval_reset_mode": data.get("eval_reset_mode", ""),
                "episodes": int(aggregate.get("episodes", len(data.get("episodes", [])))),
                "success_rate": float(aggregate.get("success_rate", np.nan)),
                "mean_final_distance": float(aggregate.get("mean_final_distance", np.nan)),
                "mean_length": float(aggregate.get("mean_length", np.nan)),
            }
        )
    if not rows:
        raise FileNotFoundError(f"No evaluation JSON files found in {eval_dir}")
    return rows


def infer_train_reset(label: str) -> str:
    for reset in RESET_ORDER:
        if reset in label:
            return reset
    return ""


def infer_variant(label: str) -> str:
    for variant in VARIANT_ORDER:
        if variant in label:
            return variant
    return ""


def write_summary_csv(rows: List[Dict], out_prefix: Path) -> Path:
    path = out_prefix.with_name(f"{out_prefix.name}_eval_summary.csv")
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "label",
        "train_reset_mode",
        "train_variant",
        "eval_reset_mode",
        "episodes",
        "success_rate",
        "mean_final_distance",
        "mean_length",
        "model",
        "file",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})
    return path


def write_delta_csv(rows: List[Dict], out_prefix: Path) -> Path:
    path = out_prefix.with_name(f"{out_prefix.name}_success_only_delta.csv")
    path.parent.mkdir(parents=True, exist_ok=True)
    by_key = {
        (row["train_reset_mode"], row["train_variant"], row["eval_reset_mode"]): row
        for row in rows
    }
    fields = [
        "train_reset_mode",
        "eval_reset_mode",
        "all_data_success_rate",
        "success_only_success_rate",
        "delta_success_rate",
        "all_data_mean_final_distance",
        "success_only_mean_final_distance",
        "delta_mean_final_distance",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for train_reset in RESET_ORDER:
            for eval_reset in RESET_ORDER:
                all_row = by_key.get((train_reset, "all_data", eval_reset))
                success_row = by_key.get((train_reset, "success_only", eval_reset))
                if all_row is None or success_row is None:
                    continue
                writer.writerow(
                    {
                        "train_reset_mode": train_reset,
                        "eval_reset_mode": eval_reset,
                        "all_data_success_rate": all_row["success_rate"],
                        "success_only_success_rate": success_row["success_rate"],
                        "delta_success_rate": success_row["success_rate"] - all_row["success_rate"],
                        "all_data_mean_final_distance": all_row["mean_final_distance"],
                        "success_only_mean_final_distance": success_row["mean_final_distance"],
                        "delta_mean_final_distance": success_row["mean_final_distance"] - all_row["mean_final_distance"],
                    }
                )
    return path


def matrix_for(rows: List[Dict], variant: str, metric: str) -> np.ndarray:
    mat = np.full((len(RESET_ORDER), len(RESET_ORDER)), np.nan, dtype=np.float64)
    for row in rows:
        if row["train_variant"] != variant:
            continue
        if row["train_reset_mode"] not in RESET_ORDER or row["eval_reset_mode"] not in RESET_ORDER:
            continue
        i = RESET_ORDER.index(row["train_reset_mode"])
        j = RESET_ORDER.index(row["eval_reset_mode"])
        mat[i, j] = row[metric]
    return mat


def plot_heatmaps(rows: List[Dict], out_prefix: Path, metric: str, title: str, cmap: str, vmin=None, vmax=None, formats=None) -> List[Path]:
    formats = formats or ["png", "pdf", "svg"]
    plt = require_matplotlib()
    set_paper_style(plt)
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 3.6), constrained_layout=True)
    images = []
    for ax, variant in zip(axes, VARIANT_ORDER):
        mat = matrix_for(rows, variant, metric)
        im = ax.imshow(mat, cmap=cmap, vmin=vmin, vmax=vmax)
        images.append(im)
        ax.set_title(variant.replace("_", " "))
        ax.set_xticks(range(len(RESET_ORDER)), RESET_ORDER)
        ax.set_yticks(range(len(RESET_ORDER)), RESET_ORDER)
        ax.set_xlabel("Evaluation Reset")
        ax.set_ylabel("Training Reset")
        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                if np.isfinite(mat[i, j]):
                    text = f"{mat[i, j]:.2f}" if metric == "success_rate" else f"{mat[i, j]:.3f}"
                    ax.text(j, i, text, ha="center", va="center", color="black", fontsize=9)
    fig.suptitle(title)
    fig.colorbar(images[-1], ax=axes, shrink=0.82)

    paths = []
    for fmt in formats:
        path = out_prefix.with_name(f"{out_prefix.name}_{metric}_heatmap.{fmt}")
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
        paths.append(path)
    plt.close(fig)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eval-dir", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, default=Path("outputs") / "pushcube2d" / "bc_comparison")
    parser.add_argument("--formats", type=str, default="png,svg")
    args = parser.parse_args()

    formats = [fmt.strip().lower() for fmt in args.formats.split(",") if fmt.strip()]
    rows = load_eval_rows(args.eval_dir)
    paths = [
        write_summary_csv(rows, args.out_prefix),
        write_delta_csv(rows, args.out_prefix),
    ]
    paths.extend(plot_heatmaps(rows, args.out_prefix, "success_rate", "Policy Success Rate", "YlGn", vmin=0.0, vmax=1.0, formats=formats))
    paths.extend(plot_heatmaps(rows, args.out_prefix, "mean_final_distance", "Mean Final Cube-Goal Distance", "YlOrRd_r", formats=formats))

    for path in paths:
        print(f"saved {path}")


if __name__ == "__main__":
    main()
