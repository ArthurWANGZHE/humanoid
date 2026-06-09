"""Train a behavior-cloning MLP baseline for PushCube."""

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

from pushcube_isaac_v0.pushcube_dataset_utils import load_all_episodes, optional_import  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train a BC baseline on PushCube demonstrations.")
    parser.add_argument("--dataset", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    return parser


def _stack_episode_split(episodes, indices, torch):
    observations = []
    actions = []
    for index in indices:
        _, episode = episodes[index]
        observations.append(torch.tensor(episode.obs, dtype=torch.float32))
        actions.append(torch.tensor(episode.action, dtype=torch.float32))
    if not observations:
        return torch.empty((0, 13), dtype=torch.float32), torch.empty((0, 2), dtype=torch.float32)
    return torch.cat(observations, dim=0), torch.cat(actions, dim=0)


def _metrics(prediction, target, torch):
    mse = torch.mean((prediction - target) ** 2)
    rmse_xy = torch.sqrt(torch.mean((prediction - target) ** 2, dim=0))
    rmse = torch.sqrt(torch.mean((prediction - target) ** 2))
    return {
        "loss": float(mse.detach().cpu().item()),
        "rmse": float(rmse.detach().cpu().item()),
        "x_rmse": float(rmse_xy[0].detach().cpu().item()),
        "y_rmse": float(rmse_xy[1].detach().cpu().item()),
    }


def _save_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["epoch", "loss", "rmse", "x_rmse", "y_rmse"])
        writer.writeheader()
        writer.writerows(rows)


def _plot_training_artifacts(output_dir: Path, train_rows: list[dict[str, float]], val_rows: list[dict[str, float]], val_target: np.ndarray, val_pred: np.ndarray) -> None:
    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")

    epochs = [row["epoch"] for row in train_rows]
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.plot(epochs, [row["loss"] for row in train_rows], label="train", color="#4c72b0")
    ax.plot(epochs, [row["loss"] for row in val_rows], label="val", color="#c44e52")
    ax.set_xlabel("epoch")
    ax.set_ylabel("MSE loss")
    ax.set_title("PushCube BC Loss Curve")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "loss_curve.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.8))
    axes[0].scatter(val_target[:, 0], val_pred[:, 0], s=10, alpha=0.3, color="#4c72b0", edgecolors="none")
    axes[1].scatter(val_target[:, 1], val_pred[:, 1], s=10, alpha=0.3, color="#55a868", edgecolors="none")
    for axis, label in zip(axes, ["x", "y"]):
        axis.set_xlabel(f"ground-truth action {label}")
        axis.set_ylabel(f"predicted action {label}")
        axis.grid(True, alpha=0.25)
        lower = float(min(np.min(val_target), np.min(val_pred)))
        upper = float(max(np.max(val_target), np.max(val_pred)))
        axis.plot([lower, upper], [lower, upper], linestyle="--", color="#999999", linewidth=1.0)
    axes[0].set_title("Predicted vs GT Action: x")
    axes[1].set_title("Predicted vs GT Action: y")
    fig.tight_layout()
    fig.savefig(output_dir / "pred_vs_gt_action_scatter.png", dpi=220, bbox_inches="tight")
    fig.savefig(output_dir / "pred_vs_gt_action_x.png", dpi=220, bbox_inches="tight")
    fig.savefig(output_dir / "pred_vs_gt_action_y.png", dpi=220, bbox_inches="tight")
    plt.close(fig)

    errors = val_pred - val_target
    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    ax.hist(errors[:, 0], bins=40, alpha=0.7, color="#4c72b0", label="x error")
    ax.hist(errors[:, 1], bins=40, alpha=0.6, color="#55a868", label="y error")
    ax.set_xlabel("prediction error")
    ax.set_ylabel("count")
    ax.set_title("Action Prediction Error Histogram")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "action_error_hist.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = build_arg_parser().parse_args()
    torch = optional_import("torch", "pip install torch")
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset

    metadata, episodes = load_all_episodes(args.dataset)
    if not episodes:
        raise RuntimeError("Dataset contains no episodes.")
    if not 0.0 < float(args.val_split) < 1.0:
        raise RuntimeError(f"val-split must be in (0, 1), got {args.val_split}")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(int(args.seed))
    episode_indices = np.arange(len(episodes))
    rng.shuffle(episode_indices)
    val_episode_count = max(1, int(round(len(episodes) * float(args.val_split))))
    val_indices = sorted(episode_indices[:val_episode_count].tolist())
    train_indices = sorted(episode_indices[val_episode_count:].tolist())
    if not train_indices:
        raise RuntimeError("Validation split consumed every episode; reduce --val-split.")

    train_obs, train_action = _stack_episode_split(episodes, train_indices, torch)
    val_obs, val_action = _stack_episode_split(episodes, val_indices, torch)
    torch.manual_seed(int(args.seed))
    obs_mean = train_obs.mean(dim=0)
    obs_std = train_obs.std(dim=0).clamp_min(1e-6)
    train_loader = DataLoader(
        TensorDataset(train_obs, train_action),
        batch_size=int(args.batch_size),
        shuffle=True,
        drop_last=False,
    )

    class PolicyMLP(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(13, 256),
                nn.ReLU(),
                nn.Linear(256, 256),
                nn.ReLU(),
                nn.Linear(256, 128),
                nn.ReLU(),
                nn.Linear(128, 2),
            )

        def forward(self, obs):
            return self.net((obs - obs_mean) / obs_std)

    model = PolicyMLP()
    optimizer = torch.optim.Adam(model.parameters(), lr=float(args.lr))
    criterion = nn.MSELoss()
    train_rows: list[dict[str, float]] = []
    val_rows: list[dict[str, float]] = []
    best_val_loss = float("inf")
    best_payload = None

    for epoch in range(1, int(args.epochs) + 1):
        model.train()
        for batch_obs, batch_action in train_loader:
            prediction = model(batch_obs)
            loss = criterion(prediction, batch_action)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        model.eval()
        with torch.no_grad():
            train_prediction = model(train_obs)
            val_prediction = model(val_obs)
            train_metrics = _metrics(train_prediction, train_action, torch)
            val_metrics = _metrics(val_prediction, val_action, torch)
        train_row = {"epoch": epoch, **train_metrics}
        val_row = {"epoch": epoch, **val_metrics}
        train_rows.append(train_row)
        val_rows.append(val_row)
        print(
            "epoch={epoch} train_loss={train_loss:.8f} val_loss={val_loss:.8f} action_rmse={rmse:.6f} x_rmse={x_rmse:.6f} y_rmse={y_rmse:.6f}".format(
                epoch=epoch,
                train_loss=train_metrics["loss"],
                val_loss=val_metrics["loss"],
                rmse=val_metrics["rmse"],
                x_rmse=val_metrics["x_rmse"],
                y_rmse=val_metrics["y_rmse"],
            )
        )
        if val_metrics["loss"] < best_val_loss:
            best_val_loss = val_metrics["loss"]
            best_payload = {
                "model_state_dict": model.state_dict(),
                "obs_mean": obs_mean,
                "obs_std": obs_std,
                "metadata": metadata,
                "train_args": vars(args),
                "val_metrics": val_metrics,
                "architecture": [13, 256, 256, 128, 2],
            }

    if best_payload is None:
        raise RuntimeError("Training failed to produce a checkpoint.")
    torch.save(best_payload, output_dir / "model.pt")
    _save_csv(output_dir / "train_loss.csv", train_rows)
    _save_csv(output_dir / "val_loss.csv", val_rows)

    with (output_dir / "config.json").open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "dataset": args.dataset,
                "epochs": int(args.epochs),
                "batch_size": int(args.batch_size),
                "lr": float(args.lr),
                "val_split": float(args.val_split),
                "seed": int(args.seed),
                "obs_dim": 13,
                "action_dim": 2,
                "train_episode_indices": train_indices,
                "val_episode_indices": val_indices,
                "dataset_metadata": metadata,
                "best_val_loss": best_val_loss,
            },
            handle,
            indent=2,
            sort_keys=True,
        )

    model.load_state_dict(best_payload["model_state_dict"])
    model.eval()
    with torch.no_grad():
        val_prediction = model(val_obs).cpu().numpy()
    val_target = val_action.cpu().numpy()
    np.savez(output_dir / "val_predictions.npz", prediction=val_prediction, target=val_target)
    _plot_training_artifacts(output_dir, train_rows, val_rows, val_target, val_prediction)


if __name__ == "__main__":
    main()

