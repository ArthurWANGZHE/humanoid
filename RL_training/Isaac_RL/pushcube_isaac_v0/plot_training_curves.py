"""Regenerate BC training figures from a saved run directory."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_utils import optional_import  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot saved PushCube training curves.")
    parser.add_argument("--run-dir", type=str, required=True)
    return parser


def _read_csv(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        return [{key: float(value) for key, value in row.items()} for row in reader]


def main() -> None:
    args = build_arg_parser().parse_args()
    run_dir = Path(args.run_dir)
    train_rows = _read_csv(run_dir / "train_loss.csv")
    val_rows = _read_csv(run_dir / "val_loss.csv")
    prediction_data = np.load(run_dir / "val_predictions.npz")
    prediction = prediction_data["prediction"]
    target = prediction_data["target"]

    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")

    epochs = [row["epoch"] for row in train_rows]
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.plot(epochs, [row["loss"] for row in train_rows], label="train", color="#4c72b0")
    ax.plot(epochs, [row["loss"] for row in val_rows], label="val", color="#c44e52")
    ax.set_xlabel("epoch")
    ax.set_ylabel("MSE loss")
    ax.set_title("Loss Curve")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(run_dir / "loss_curve.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.plot(epochs, [row["rmse"] for row in train_rows], label="train", color="#4c72b0")
    ax.plot(epochs, [row["rmse"] for row in val_rows], label="val", color="#55a868")
    ax.set_xlabel("epoch")
    ax.set_ylabel("action RMSE")
    ax.set_title("Action RMSE Curve")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(run_dir / "action_rmse_curve.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    gap = np.array([val_row["loss"] - train_row["loss"] for train_row, val_row in zip(train_rows, val_rows)], dtype=np.float64)
    ax.plot(epochs, gap, color="#8172b3")
    ax.axhline(0.0, color="#999999", linewidth=1.0, linestyle="--")
    ax.set_xlabel("epoch")
    ax.set_ylabel("val_loss - train_loss")
    ax.set_title("Train / Val Gap")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(run_dir / "train_val_gap.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    for action_index, label, color in [(0, "x", "#4c72b0"), (1, "y", "#55a868")]:
        fig, ax = plt.subplots(figsize=(5.6, 5.2))
        ax.scatter(target[:, action_index], prediction[:, action_index], s=10, alpha=0.3, color=color, edgecolors="none")
        lower = float(min(np.min(target[:, action_index]), np.min(prediction[:, action_index])))
        upper = float(max(np.max(target[:, action_index]), np.max(prediction[:, action_index])))
        ax.plot([lower, upper], [lower, upper], linestyle="--", color="#999999", linewidth=1.0)
        ax.set_xlabel(f"ground-truth action {label}")
        ax.set_ylabel(f"predicted action {label}")
        ax.set_title(f"Predicted vs GT Action: {label}")
        ax.grid(True, alpha=0.25)
        fig.tight_layout()
        fig.savefig(run_dir / f"pred_vs_gt_action_{label}.png", dpi=220, bbox_inches="tight")
        plt.close(fig)

    errors = prediction - target
    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    ax.hist(errors[:, 0], bins=40, alpha=0.7, color="#4c72b0", label="x error")
    ax.hist(errors[:, 1], bins=40, alpha=0.6, color="#55a868", label="y error")
    ax.set_xlabel("prediction error")
    ax.set_ylabel("count")
    ax.set_title("Action Error Histogram")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(run_dir / "action_error_hist.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()

