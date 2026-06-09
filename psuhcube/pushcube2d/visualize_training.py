"""Create paper-style training figures from PushCube2D training logs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List

import numpy as np

from .collect_data import str2bool


def require_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for plotting. Install it with `pip install matplotlib`.") from exc
    return plt


def metrics_path_from_input(path: Path) -> Path:
    if path.is_dir():
        path = path / "metrics.csv"
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def read_metrics(path: Path) -> Dict[str, np.ndarray]:
    rows: List[Dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for key, value in row.items():
                if value == "" or value is None:
                    continue
                try:
                    parsed[key] = float(value)
                except ValueError:
                    pass
            rows.append(parsed)
    if not rows:
        raise ValueError(f"No metric rows found in {path}")
    keys = sorted({key for row in rows for key in row})
    return {key: np.asarray([row.get(key, np.nan) for row in rows], dtype=np.float64) for key in keys}


def read_config(metrics_path: Path) -> Dict:
    config_path = metrics_path.parent / "config.json"
    if not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as f:
        return json.load(f)


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
            "axes.grid": True,
            "grid.alpha": 0.22,
            "grid.linewidth": 0.6,
            "lines.linewidth": 1.8,
            "font.family": "DejaVu Sans",
        }
    )


def save_all(fig, out_prefix: Path, suffix: str, formats: List[str]) -> List[Path]:
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    paths = []
    for fmt in formats:
        path = out_prefix.with_name(f"{out_prefix.name}_{suffix}.{fmt}")
        fig.savefig(path, bbox_inches="tight")
        paths.append(path)
    return paths


def plot_training_curves(metrics: Dict[str, np.ndarray], out_prefix: Path, formats: List[str]) -> List[Path]:
    plt = require_matplotlib()
    set_paper_style(plt)
    epoch = metrics.get("epoch", np.arange(len(next(iter(metrics.values())))) + 1)

    fig, axes = plt.subplots(2, 2, figsize=(8.0, 5.8))
    ax = axes[0, 0]
    if "train_mse" in metrics:
        ax.plot(epoch, metrics["train_mse"], label="train", color="#3b6fb6")
    if "val_mse" in metrics:
        ax.plot(epoch, metrics["val_mse"], label="validation", color="#c84b3a")
    ax.set_title("Behavior Cloning Loss")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MSE")
    ax.set_yscale("log")
    ax.legend(frameon=False)

    ax = axes[0, 1]
    if "train_mae" in metrics:
        ax.plot(epoch, metrics["train_mae"], label="train", color="#3b6fb6")
    if "val_mae" in metrics:
        ax.plot(epoch, metrics["val_mae"], label="validation", color="#c84b3a")
    ax.set_title("Action Error")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MAE")
    ax.legend(frameon=False)

    ax = axes[1, 0]
    if "val_dx_mse" in metrics:
        ax.plot(epoch, metrics["val_dx_mse"], label="dx", color="#4b8f5a")
    if "val_dy_mse" in metrics:
        ax.plot(epoch, metrics["val_dy_mse"], label="dy", color="#8d5fbf")
    ax.set_title("Validation Error by Action Dimension")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MSE")
    ax.set_yscale("log")
    ax.legend(frameon=False)

    ax = axes[1, 1]
    if "lr" in metrics:
        ax.plot(epoch, metrics["lr"], label="learning rate", color="#4c4c4c")
    if "grad_norm" in metrics:
        ax2 = ax.twinx()
        ax2.plot(epoch, metrics["grad_norm"], label="grad norm", color="#d28b2f", alpha=0.9)
        ax2.set_ylabel("Grad Norm")
        ax2.spines["right"].set_visible(True)
    ax.set_title("Optimization")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("Learning Rate")
    ax.set_yscale("log")

    fig.tight_layout()
    paths = save_all(fig, out_prefix, "curves", formats)
    plt.close(fig)
    return paths


def flatten_config(config: Dict) -> Dict[str, str]:
    keep = [
        "dataset",
        "epochs",
        "batch_size",
        "lr",
        "lr_schedule",
        "warmup_epochs",
        "min_lr_ratio",
        "hidden_dim",
        "num_layers",
        "activation",
        "weight_decay",
        "grad_clip",
        "val_split",
        "seed",
        "num_train_steps",
        "num_val_steps",
    ]
    flat = {}
    for key in keep:
        if key in config:
            value = config[key]
            if isinstance(value, float):
                flat[key] = f"{value:.4g}"
            else:
                flat[key] = str(value)
    return flat


def plot_hparams_table(config: Dict, out_prefix: Path, formats: List[str]) -> List[Path]:
    plt = require_matplotlib()
    set_paper_style(plt)
    flat = flatten_config(config)
    if not flat:
        return []
    rows = [[key.replace("_", " "), value] for key, value in flat.items()]
    height = max(2.5, 0.28 * len(rows) + 0.8)
    fig, ax = plt.subplots(figsize=(7.2, height))
    ax.axis("off")
    table = ax.table(cellText=rows, colLabels=["Parameter", "Value"], loc="center", cellLoc="left", colLoc="left")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.18)
    for (row, _col), cell in table.get_celld().items():
        cell.set_linewidth(0.4)
        if row == 0:
            cell.set_facecolor("#e9eef5")
            cell.set_text_props(weight="bold")
        elif row % 2 == 0:
            cell.set_facecolor("#f7f7f7")
    fig.tight_layout()
    paths = save_all(fig, out_prefix, "hparams", formats)
    plt.close(fig)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path, required=True, help="Run directory or metrics.csv path.")
    parser.add_argument("--out-prefix", type=Path, default=None)
    parser.add_argument("--formats", type=str, default="png,svg")
    parser.add_argument("--hparams", type=str2bool, default=True)
    args = parser.parse_args()

    metrics_path = metrics_path_from_input(args.log_dir)
    out_prefix = args.out_prefix or (Path("outputs") / "pushcube2d" / metrics_path.parent.name)
    formats = [fmt.strip().lower() for fmt in args.formats.split(",") if fmt.strip()]
    metrics = read_metrics(metrics_path)
    config = read_config(metrics_path)

    paths = plot_training_curves(metrics, out_prefix, formats)
    if args.hparams:
        paths.extend(plot_hparams_table(config, out_prefix, formats))

    for path in paths:
        print(f"saved {path}")


if __name__ == "__main__":
    main()
