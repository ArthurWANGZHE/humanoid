"""Generate statistical comparison charts from cross-evaluation results."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.size'] = 10

EVAL_DIR = Path("outputs/pushcube2d/experiment_compare/eval_json")
OUT_DIR = Path("outputs/pushcube2d/experiment_compare")

TRAIN_MODES = ["aligned", "near", "random"]
TRAIN_VARIANTS = ["all_data", "success_only"]
EVAL_MODES = ["aligned", "near", "random"]


def load_all_results():
    """Load all JSON evaluation results into a nested dict."""
    results = {}
    for f in sorted(EVAL_DIR.glob("*.json")):
        with open(f) as fp:
            data = json.load(fp)
        key = f.stem  # e.g. bc_aligned_all_data_on_near
        results[key] = data
    return results


def get_key(train_mode, variant, eval_mode):
    return f"bc_{train_mode}_{variant}_on_{eval_mode}"


def plot_success_rate_bars(results):
    """Grouped bar chart: success rate per model across eval environments."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)

    for ax_idx, variant in enumerate(TRAIN_VARIANTS):
        ax = axes[ax_idx]
        x = np.arange(len(EVAL_MODES))
        width = 0.25

        for i, train_mode in enumerate(TRAIN_MODES):
            rates = []
            for eval_mode in EVAL_MODES:
                key = get_key(train_mode, variant, eval_mode)
                rates.append(results[key]["aggregate"]["success_rate"])
            bars = ax.bar(x + i * width, rates, width, label=f"train: {train_mode}")
            # Add value labels on bars
            for bar, val in zip(bars, rates):
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                        f"{val:.0%}", ha='center', va='bottom', fontsize=8)

        ax.set_xlabel("Eval Environment")
        ax.set_ylabel("Success Rate")
        ax.set_title(f"Success Rate — {variant.replace('_', ' ').title()}")
        ax.set_xticks(x + width)
        ax.set_xticklabels(EVAL_MODES)
        ax.set_ylim(0, 1.15)
        ax.legend(loc="lower right")
        ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.3)

    plt.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(OUT_DIR / f"success_rate_bars.{ext}", dpi=150, bbox_inches='tight')
    plt.close(fig)
    print("Saved: success_rate_bars")


def plot_distance_bars(results):
    """Grouped bar chart: mean final distance per model across eval environments."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)

    for ax_idx, variant in enumerate(TRAIN_VARIANTS):
        ax = axes[ax_idx]
        x = np.arange(len(EVAL_MODES))
        width = 0.25

        for i, train_mode in enumerate(TRAIN_MODES):
            dists = []
            for eval_mode in EVAL_MODES:
                key = get_key(train_mode, variant, eval_mode)
                dists.append(results[key]["aggregate"]["mean_final_distance"])
            bars = ax.bar(x + i * width, dists, width, label=f"train: {train_mode}")
            for bar, val in zip(bars, dists):
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.005,
                        f"{val:.3f}", ha='center', va='bottom', fontsize=7)

        ax.set_xlabel("Eval Environment")
        ax.set_ylabel("Mean Final Distance")
        ax.set_title(f"Mean Final Distance — {variant.replace('_', ' ').title()}")
        ax.set_xticks(x + width)
        ax.set_xticklabels(EVAL_MODES)
        ax.legend(loc="upper left")

    plt.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(OUT_DIR / f"mean_final_distance_bars.{ext}", dpi=150, bbox_inches='tight')
    plt.close(fig)
    print("Saved: mean_final_distance_bars")


def plot_distance_boxplots(results):
    """Box plots showing final_distance distribution for each model × eval combination."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    for ax_idx, variant in enumerate(TRAIN_VARIANTS):
        ax = axes[ax_idx]
        data_list = []
        labels = []

        for train_mode in TRAIN_MODES:
            for eval_mode in EVAL_MODES:
                key = get_key(train_mode, variant, eval_mode)
                episodes = results[key]["episodes"]
                dists = [ep["final_distance"] for ep in episodes]
                data_list.append(dists)
                labels.append(f"{train_mode}\n→{eval_mode}")

        bp = ax.boxplot(data_list, tick_labels=labels, patch_artist=True, showfliers=True,
                        flierprops=dict(marker='o', markersize=3, alpha=0.5))

        # Color by train mode
        colors = ['#4C72B0', '#55A868', '#C44E52']
        for i, patch in enumerate(bp['boxes']):
            patch.set_facecolor(colors[i // 3])
            patch.set_alpha(0.6)

        ax.set_xlabel("Train → Eval")
        ax.set_ylabel("Final Distance")
        ax.set_title(f"Final Distance Distribution — {variant.replace('_', ' ').title()}")
        ax.tick_params(axis='x', rotation=45)

        # Add legend
        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor=c, alpha=0.6, label=f"train: {m}")
                           for c, m in zip(colors, TRAIN_MODES)]
        ax.legend(handles=legend_elements, loc="upper left")

    plt.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(OUT_DIR / f"final_distance_boxplots.{ext}", dpi=150, bbox_inches='tight')
    plt.close(fig)
    print("Saved: final_distance_boxplots")


def plot_length_boxplots(results):
    """Box plots showing episode length distribution."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    for ax_idx, variant in enumerate(TRAIN_VARIANTS):
        ax = axes[ax_idx]
        data_list = []
        labels = []

        for train_mode in TRAIN_MODES:
            for eval_mode in EVAL_MODES:
                key = get_key(train_mode, variant, eval_mode)
                episodes = results[key]["episodes"]
                lengths = [ep["length"] for ep in episodes]
                data_list.append(lengths)
                labels.append(f"{train_mode}\n→{eval_mode}")

        bp = ax.boxplot(data_list, tick_labels=labels, patch_artist=True,
                        flierprops=dict(marker='o', markersize=3, alpha=0.5))

        colors = ['#4C72B0', '#55A868', '#C44E52']
        for i, patch in enumerate(bp['boxes']):
            patch.set_facecolor(colors[i // 3])
            patch.set_alpha(0.6)

        ax.set_xlabel("Train → Eval")
        ax.set_ylabel("Episode Length (steps)")
        ax.set_title(f"Episode Length Distribution — {variant.replace('_', ' ').title()}")
        ax.tick_params(axis='x', rotation=45)

        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor=c, alpha=0.6, label=f"train: {m}")
                           for c, m in zip(colors, TRAIN_MODES)]
        ax.legend(handles=legend_elements, loc="upper left")

    plt.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(OUT_DIR / f"episode_length_boxplots.{ext}", dpi=150, bbox_inches='tight')
    plt.close(fig)
    print("Saved: episode_length_boxplots")


def plot_radar(results):
    """Radar chart showing each training mode's performance across eval environments."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), subplot_kw=dict(polar=True))

    angles = np.linspace(0, 2 * np.pi, len(EVAL_MODES), endpoint=False).tolist()
    angles += angles[:1]  # close the polygon

    colors = ['#4C72B0', '#55A868', '#C44E52']

    for ax_idx, variant in enumerate(TRAIN_VARIANTS):
        ax = axes[ax_idx]

        for i, train_mode in enumerate(TRAIN_MODES):
            values = []
            for eval_mode in EVAL_MODES:
                key = get_key(train_mode, variant, eval_mode)
                values.append(results[key]["aggregate"]["success_rate"])
            values += values[:1]
            ax.plot(angles, values, 'o-', color=colors[i], linewidth=2, label=f"train: {train_mode}")
            ax.fill(angles, values, alpha=0.1, color=colors[i])

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels([f"eval: {m}" for m in EVAL_MODES])
        ax.set_ylim(0, 1.1)
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(["20%", "40%", "60%", "80%", "100%"], fontsize=7)
        ax.set_title(f"Success Rate Radar — {variant.replace('_', ' ').title()}", pad=20)
        ax.legend(loc="lower right", bbox_to_anchor=(1.3, -0.1), fontsize=9)

    plt.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(OUT_DIR / f"success_rate_radar.{ext}", dpi=150, bbox_inches='tight')
    plt.close(fig)
    print("Saved: success_rate_radar")


def plot_generalization_gap(results):
    """Bar chart showing the 'generalization gap': in-distribution vs out-of-distribution performance."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    for ax_idx, variant in enumerate(TRAIN_VARIANTS):
        ax = axes[ax_idx]
        in_dist = []
        out_dist = []

        for train_mode in TRAIN_MODES:
            # In-distribution: eval on same mode as training
            key_in = get_key(train_mode, variant, train_mode)
            in_dist.append(results[key_in]["aggregate"]["success_rate"])

            # Out-of-distribution: average over other eval modes
            ood_rates = []
            for eval_mode in EVAL_MODES:
                if eval_mode != train_mode:
                    key_out = get_key(train_mode, variant, eval_mode)
                    ood_rates.append(results[key_out]["aggregate"]["success_rate"])
            out_dist.append(np.mean(ood_rates))

        x = np.arange(len(TRAIN_MODES))
        width = 0.35
        bars1 = ax.bar(x - width / 2, in_dist, width, label="In-distribution", color='#4C72B0')
        bars2 = ax.bar(x + width / 2, out_dist, width, label="Out-of-distribution", color='#C44E52')

        for bar, val in zip(bars1, in_dist):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                    f"{val:.0%}", ha='center', va='bottom', fontsize=9)
        for bar, val in zip(bars2, out_dist):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                    f"{val:.0%}", ha='center', va='bottom', fontsize=9)

        ax.set_xlabel("Training Mode")
        ax.set_ylabel("Success Rate")
        ax.set_title(f"Generalization Gap — {variant.replace('_', ' ').title()}")
        ax.set_xticks(x)
        ax.set_xticklabels(TRAIN_MODES)
        ax.set_ylim(0, 1.2)
        ax.legend()
        ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.3)

    plt.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(OUT_DIR / f"generalization_gap.{ext}", dpi=150, bbox_inches='tight')
    plt.close(fig)
    print("Saved: generalization_gap")


def main():
    results = load_all_results()
    print(f"Loaded {len(results)} evaluation results.\n")

    plot_success_rate_bars(results)
    plot_distance_bars(results)
    plot_distance_boxplots(results)
    plot_length_boxplots(results)
    plot_radar(results)
    plot_generalization_gap(results)

    print(f"\nAll charts saved to: {OUT_DIR}")


if __name__ == "__main__":
    main()
