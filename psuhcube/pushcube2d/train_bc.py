"""Train a NumPy behavior-cloning policy for PushCube2D.

The trainer is intentionally dependency-light but experiment-friendly:
configurable MLP depth/width, learning-rate schedules, CSV/JSON logs, optional
TensorBoard scalars, checkpoints, and paper-figure-ready metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from copy import deepcopy
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np

from .collect_data import str2bool
from .dataset import load_dataset, valid_mask


METRIC_FIELDS = [
    "epoch",
    "step",
    "lr",
    "train_mse",
    "val_mse",
    "train_rmse",
    "val_rmse",
    "train_mae",
    "val_mae",
    "train_dx_mse",
    "train_dy_mse",
    "val_dx_mse",
    "val_dy_mse",
    "grad_norm",
    "epoch_seconds",
]


@dataclass
class TrainConfig:
    dataset: str
    out: str
    log_dir: str
    run_name: str
    epochs: int = 300
    batch_size: int = 256
    lr: float = 1e-3
    min_lr_ratio: float = 0.05
    lr_schedule: str = "cosine"
    warmup_epochs: int = 5
    hidden_dim: int = 128
    num_layers: int = 4
    activation: str = "silu"
    weight_decay: float = 1e-5
    grad_clip: float = 5.0
    val_split: float = 0.1
    seed: int = 0
    log_every: int = 1
    save_every: int = 0
    save_best: bool = True
    tensorboard: bool = False
    success_only_data: bool = True


def default_model_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("models") / "pushcube2d" / f"bc_{stamp}.npz"


def default_run_name() -> str:
    return datetime.now().strftime("bc_%Y%m%d_%H%M%S")


def weight_keys(params: Dict[str, np.ndarray]) -> List[str]:
    return sorted(
        [key for key in params if key.startswith("W") and key[1:].isdigit()],
        key=lambda key: int(key[1:]),
    )


def bias_key(weight_key: str) -> str:
    return "b" + weight_key[1:]


def activation_name_from_model(params: Dict[str, np.ndarray]) -> str:
    if "activation_name" not in params:
        return "tanh"
    value = np.asarray(params["activation_name"])
    return str(value.item() if value.shape == () else value)


def init_model(
    rng: np.random.Generator,
    in_dim: int,
    hidden_dim: int,
    out_dim: int,
    num_layers: int,
    activation: str,
) -> Dict[str, np.ndarray]:
    if num_layers < 1:
        raise ValueError("--num-layers must be >= 1")
    dims = [in_dim] + [hidden_dim] * num_layers + [out_dim]
    params: Dict[str, np.ndarray] = {}
    for i in range(len(dims) - 1):
        fan_in = max(dims[i], 1)
        gain = 2.0 if activation in {"relu", "silu"} and i < len(dims) - 2 else 1.0
        scale = math.sqrt(gain / fan_in)
        params[f"W{i + 1}"] = rng.normal(0.0, scale, size=(dims[i], dims[i + 1])).astype(np.float32)
        params[f"b{i + 1}"] = np.zeros(dims[i + 1], dtype=np.float32)
    return params


def activate(z: np.ndarray, name: str) -> np.ndarray:
    if name == "tanh":
        return np.tanh(z)
    if name == "relu":
        return np.maximum(z, 0.0)
    if name == "silu":
        sig = 1.0 / (1.0 + np.exp(-z))
        return z * sig
    raise ValueError(f"Unsupported activation: {name!r}")


def activation_grad(z: np.ndarray, h: np.ndarray, name: str) -> np.ndarray:
    if name == "tanh":
        return 1.0 - h**2
    if name == "relu":
        return (z > 0.0).astype(np.float32)
    if name == "silu":
        sig = 1.0 / (1.0 + np.exp(-z))
        return sig * (1.0 + z * (1.0 - sig))
    raise ValueError(f"Unsupported activation: {name!r}")


def forward(params: Dict[str, np.ndarray], x: np.ndarray):
    activation = activation_name_from_model(params)
    keys = weight_keys(params)
    hidden = x
    zs: List[np.ndarray] = []
    hs: List[np.ndarray] = [x]
    for key in keys[:-1]:
        z = hidden @ params[key] + params[bias_key(key)]
        hidden = activate(z, activation)
        zs.append(z)
        hs.append(hidden)
    out_key = keys[-1]
    z_out = hidden @ params[out_key] + params[bias_key(out_key)]
    y = np.tanh(z_out)
    cache = {"x": x, "zs": zs, "hs": hs, "z_out": z_out, "y": y, "keys": keys, "activation": activation}
    return y, cache


def backward(params: Dict[str, np.ndarray], cache, target: np.ndarray, weight_decay: float = 0.0):
    keys = cache["keys"]
    activation = cache["activation"]
    y = cache["y"]
    grads: Dict[str, np.ndarray] = {}

    dz = (2.0 / max(1, y.size)) * (y - target) * (1.0 - y**2)
    out_key = keys[-1]
    grads[out_key] = cache["hs"][-1].T @ dz
    grads[bias_key(out_key)] = dz.sum(axis=0)
    dh = dz @ params[out_key].T

    for layer_idx in range(len(keys) - 2, -1, -1):
        key = keys[layer_idx]
        z = cache["zs"][layer_idx]
        h = cache["hs"][layer_idx + 1]
        dz = dh * activation_grad(z, h, activation)
        grads[key] = cache["hs"][layer_idx].T @ dz
        grads[bias_key(key)] = dz.sum(axis=0)
        if layer_idx > 0:
            dh = dz @ params[key].T

    if weight_decay > 0.0:
        for key in keys:
            grads[key] = grads[key] + weight_decay * params[key]
    return grads


def global_grad_norm(grads: Dict[str, np.ndarray]) -> float:
    total = 0.0
    for grad in grads.values():
        total += float(np.sum(grad.astype(np.float64) ** 2))
    return float(math.sqrt(total))


def clip_grads(grads: Dict[str, np.ndarray], max_norm: float | None) -> float:
    norm = global_grad_norm(grads)
    if max_norm is not None and max_norm > 0.0 and norm > max_norm:
        scale = max_norm / (norm + 1e-12)
        for key in grads:
            grads[key] *= scale
    return norm


def adam_update(params, grads, opt, lr, step):
    beta1, beta2, eps = 0.9, 0.999, 1e-8
    for key in weight_keys(params):
        for param_key in (key, bias_key(key)):
            opt["m"][param_key] = beta1 * opt["m"][param_key] + (1.0 - beta1) * grads[param_key]
            opt["v"][param_key] = beta2 * opt["v"][param_key] + (1.0 - beta2) * (grads[param_key] ** 2)
            m_hat = opt["m"][param_key] / (1.0 - beta1**step)
            v_hat = opt["v"][param_key] / (1.0 - beta2**step)
            params[param_key] -= lr * m_hat / (np.sqrt(v_hat) + eps)


def learning_rate(epoch: int, cfg: TrainConfig) -> float:
    if cfg.warmup_epochs > 0 and epoch <= cfg.warmup_epochs:
        return cfg.lr * epoch / cfg.warmup_epochs
    if cfg.lr_schedule == "constant":
        return cfg.lr
    if cfg.lr_schedule == "cosine":
        start = cfg.warmup_epochs + 1
        progress = (epoch - start) / max(1, cfg.epochs - start)
        progress = float(np.clip(progress, 0.0, 1.0))
        min_lr = cfg.lr * cfg.min_lr_ratio
        return min_lr + 0.5 * (cfg.lr - min_lr) * (1.0 + math.cos(math.pi * progress))
    if cfg.lr_schedule == "linear":
        progress = (epoch - cfg.warmup_epochs) / max(1, cfg.epochs - cfg.warmup_epochs)
        progress = float(np.clip(progress, 0.0, 1.0))
        return cfg.lr * (1.0 - progress * (1.0 - cfg.min_lr_ratio))
    raise ValueError(f"Unsupported lr schedule: {cfg.lr_schedule!r}")


def flatten_training_steps(data: Dict, success_only_data: bool) -> Tuple[np.ndarray, np.ndarray]:
    mask = valid_mask(data)
    if success_only_data:
        success = np.asarray(data["success"], dtype=bool)
        mask = mask & success[:, None]
    if not np.any(mask):
        mode = "successful" if success_only_data else "valid"
        raise ValueError(f"No {mode} transitions found in dataset.")
    return data["obs"][mask].astype(np.float32), data["actions"][mask].astype(np.float32)


def prepare_data(dataset_path: Path, val_split: float, seed: int, success_only_data: bool):
    data = load_dataset(dataset_path)
    x, y = flatten_training_steps(data, success_only_data)
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(x))
    x, y = x[order], y[order]
    n_val = int(round(len(x) * val_split))
    n_val = min(max(n_val, 1), max(len(x) - 1, 1))
    x_val, y_val = x[:n_val], y[:n_val]
    x_train, y_train = x[n_val:], y[n_val:]
    x_mean = x_train.mean(axis=0, keepdims=True)
    x_std = x_train.std(axis=0, keepdims=True) + 1e-6
    return (x_train - x_mean) / x_std, y_train, (x_val - x_mean) / x_std, y_val, x_mean.squeeze(0), x_std.squeeze(0), data


def evaluate_regression(params: Dict[str, np.ndarray], x: np.ndarray, y: np.ndarray, prefix: str) -> Dict[str, float]:
    pred, _ = forward(params, x)
    err = pred - y
    mse = float(np.mean(err**2))
    mae = float(np.mean(np.abs(err)))
    per_dim = np.mean(err**2, axis=0)
    return {
        f"{prefix}_mse": mse,
        f"{prefix}_rmse": float(math.sqrt(mse)),
        f"{prefix}_mae": mae,
        f"{prefix}_dx_mse": float(per_dim[0]) if len(per_dim) > 0 else 0.0,
        f"{prefix}_dy_mse": float(per_dim[1]) if len(per_dim) > 1 else 0.0,
    }


class OptionalTensorBoard:
    def __init__(self, enabled: bool, log_dir: Path):
        self.writer = None
        if not enabled:
            return
        try:
            from tensorboardX import SummaryWriter
        except ImportError:
            try:
                from torch.utils.tensorboard import SummaryWriter
            except ImportError:
                print("TensorBoard logging requested, but neither tensorboardX nor torch is installed.")
                print("Install with `pip install tensorboard tensorboardX`, or rerun with --tensorboard false.")
                return
        self.writer = SummaryWriter(str(log_dir))

    def add_scalars(self, metrics: Dict[str, float], epoch: int) -> None:
        if self.writer is None:
            return
        for key, value in metrics.items():
            if isinstance(value, (int, float, np.floating)):
                self.writer.add_scalar(key, float(value), epoch)

    def add_text(self, tag: str, text: str) -> None:
        if self.writer is not None:
            self.writer.add_text(tag, text, 0)

    def close(self) -> None:
        if self.writer is not None:
            self.writer.close()


def append_metrics_csv(path: Path, row: Dict[str, float]) -> None:
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=METRIC_FIELDS)
        if not exists:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in METRIC_FIELDS})


def append_jsonl(path: Path, row: Dict[str, float]) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True) + "\n")


def save_model(
    path: Path,
    params: Dict[str, np.ndarray],
    x_mean: np.ndarray,
    x_std: np.ndarray,
    cfg: TrainConfig,
    metadata: Dict,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(params)
    payload["x_mean"] = x_mean.astype(np.float32)
    payload["x_std"] = x_std.astype(np.float32)
    payload["num_hidden_layers"] = np.array(cfg.num_layers, dtype=np.int32)
    payload["hidden_dim"] = np.array(cfg.hidden_dim, dtype=np.int32)
    payload["activation_name"] = np.array(cfg.activation)
    payload["metadata_json"] = np.array(json.dumps(metadata, indent=2))
    np.savez_compressed(path, **payload)


def train_with_config(cfg: TrainConfig) -> Path:
    dataset_path = Path(cfg.dataset)
    out = Path(cfg.out)
    run_dir = Path(cfg.log_dir) / cfg.run_name
    run_dir.mkdir(parents=True, exist_ok=True)
    metrics_csv = run_dir / "metrics.csv"
    metrics_jsonl = run_dir / "metrics.jsonl"
    config_json = run_dir / "config.json"
    checkpoints_dir = run_dir / "checkpoints"

    rng = np.random.default_rng(cfg.seed)
    x_train, y_train, x_val, y_val, x_mean, x_std, data = prepare_data(dataset_path, cfg.val_split, cfg.seed, cfg.success_only_data)
    params = init_model(rng, x_train.shape[-1], cfg.hidden_dim, y_train.shape[-1], cfg.num_layers, cfg.activation)
    params["activation_name"] = np.array(cfg.activation)
    opt = {
        "m": {key: np.zeros_like(value) for key, value in params.items() if key.startswith(("W", "b"))},
        "v": {key: np.zeros_like(value) for key, value in params.items() if key.startswith(("W", "b"))},
    }

    with config_json.open("w", encoding="utf-8") as f:
        json.dump({**asdict(cfg), "num_train_steps": int(len(x_train)), "num_val_steps": int(len(x_val))}, f, indent=2)

    tb = OptionalTensorBoard(cfg.tensorboard, run_dir)
    tb.add_text("config", json.dumps(asdict(cfg), indent=2))

    best_val = float("inf")
    best_epoch = 0
    best_params = deepcopy(params)
    step = 0
    last_grad_norm = 0.0

    try:
        for epoch in range(1, cfg.epochs + 1):
            epoch_start = time.time()
            lr_now = learning_rate(epoch, cfg)
            order = rng.permutation(len(x_train))
            x_train, y_train = x_train[order], y_train[order]
            for start in range(0, len(x_train), cfg.batch_size):
                xb = x_train[start : start + cfg.batch_size]
                yb = y_train[start : start + cfg.batch_size]
                _pred, cache = forward(params, xb)
                grads = backward(params, cache, yb, cfg.weight_decay)
                last_grad_norm = clip_grads(grads, cfg.grad_clip)
                step += 1
                adam_update(params, grads, opt, lr_now, step)

            train_metrics = evaluate_regression(params, x_train, y_train, "train")
            val_metrics = evaluate_regression(params, x_val, y_val, "val")
            metrics = {
                "epoch": epoch,
                "step": step,
                "lr": lr_now,
                **train_metrics,
                **val_metrics,
                "grad_norm": last_grad_norm,
                "epoch_seconds": time.time() - epoch_start,
            }
            append_metrics_csv(metrics_csv, metrics)
            append_jsonl(metrics_jsonl, metrics)
            tb.add_scalars(metrics, epoch)

            if metrics["val_mse"] < best_val:
                best_val = metrics["val_mse"]
                best_epoch = epoch
                best_params = deepcopy(params)

            should_print = epoch == 1 or epoch % max(1, cfg.log_every) == 0 or epoch == cfg.epochs
            if should_print:
                print(
                    f"epoch {epoch:04d}/{cfg.epochs:04d} "
                    f"lr={lr_now:.2e} train_mse={metrics['train_mse']:.6f} "
                    f"val_mse={metrics['val_mse']:.6f} val_mae={metrics['val_mae']:.5f} "
                    f"grad_norm={last_grad_norm:.3f}"
                )

            if cfg.save_every > 0 and epoch % cfg.save_every == 0:
                ckpt_meta = build_metadata(cfg, data, run_dir, epoch, metrics, best_epoch, best_val)
                save_model(checkpoints_dir / f"epoch_{epoch:04d}.npz", params, x_mean, x_std, cfg, ckpt_meta)
    finally:
        tb.close()

    final_params = best_params if cfg.save_best else params
    final_metrics = evaluate_regression(final_params, x_val, y_val, "val")
    metadata = build_metadata(cfg, data, run_dir, cfg.epochs, final_metrics, best_epoch, best_val)
    metadata["saved_params"] = "best_val" if cfg.save_best else "last_epoch"
    save_model(out, final_params, x_mean, x_std, cfg, metadata)
    print(f"best_epoch={best_epoch} best_val_mse={best_val:.6f}")
    print(f"logs={run_dir}")
    return out


def build_metadata(cfg: TrainConfig, data: Dict, run_dir: Path, epoch: int, metrics: Dict, best_epoch: int, best_val: float) -> Dict:
    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "dataset_path": cfg.dataset,
        "dataset_metadata": data.get("metadata", {}),
        "run_dir": str(run_dir),
        "epoch": epoch,
        "best_epoch": best_epoch,
        "best_val_mse": best_val,
        "metrics": metrics,
        "train_config": asdict(cfg),
        "input_normalization": "x_norm=(x-x_mean)/x_std",
        "output_activation": "tanh actions in [-1,1]",
    }


def train(
    dataset_path: Path,
    out: Path,
    epochs: int,
    batch_size: int,
    lr: float,
    hidden_dim: int,
    val_split: float,
    seed: int,
) -> Path:
    cfg = TrainConfig(
        dataset=str(dataset_path),
        out=str(out),
        log_dir=str(Path("logs") / "pushcube2d"),
        run_name=default_run_name(),
        epochs=epochs,
        batch_size=batch_size,
        lr=lr,
        hidden_dim=hidden_dim,
        val_split=val_split,
        seed=seed,
    )
    return train_with_config(cfg)


def load_policy_model(path: str | Path) -> Dict[str, np.ndarray]:
    raw = np.load(Path(path), allow_pickle=False)
    return {key: raw[key] for key in raw.files}


def predict_action(model: Dict[str, np.ndarray], obs: np.ndarray) -> np.ndarray:
    obs = np.asarray(obs, dtype=np.float32)
    single = obs.ndim == 1
    if single:
        obs = obs[None, :]
    x = (obs - model["x_mean"]) / model["x_std"]
    y, _ = forward(model, x.astype(np.float32))
    y = np.clip(y, -1.0, 1.0).astype(np.float32)
    return y[0] if single else y


def parse_args() -> TrainConfig:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--log-dir", type=Path, default=Path("logs") / "pushcube2d")
    parser.add_argument("--run-name", type=str, default=None)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--min-lr-ratio", type=float, default=0.05)
    parser.add_argument("--lr-schedule", choices=["constant", "cosine", "linear"], default="cosine")
    parser.add_argument("--warmup-epochs", type=int, default=5)
    parser.add_argument("--hidden-dim", type=int, default=128)
    parser.add_argument("--num-layers", type=int, default=4)
    parser.add_argument("--activation", choices=["tanh", "relu", "silu"], default="silu")
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--grad-clip", type=float, default=5.0)
    parser.add_argument("--val-split", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--save-best", type=str2bool, default=True)
    parser.add_argument("--tensorboard", type=str2bool, default=False)
    parser.add_argument("--success-only-data", type=str2bool, default=True)
    args = parser.parse_args()

    out = args.out or default_model_path()
    run_name = args.run_name or out.stem or default_run_name()
    return TrainConfig(
        dataset=str(args.dataset),
        out=str(out),
        log_dir=str(args.log_dir),
        run_name=run_name,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        min_lr_ratio=args.min_lr_ratio,
        lr_schedule=args.lr_schedule,
        warmup_epochs=args.warmup_epochs,
        hidden_dim=args.hidden_dim,
        num_layers=args.num_layers,
        activation=args.activation,
        weight_decay=args.weight_decay,
        grad_clip=args.grad_clip,
        val_split=args.val_split,
        seed=args.seed,
        log_every=args.log_every,
        save_every=args.save_every,
        save_best=args.save_best,
        tensorboard=args.tensorboard,
        success_only_data=args.success_only_data,
    )


def main() -> None:
    cfg = parse_args()
    path = train_with_config(cfg)
    print(f"saved model to {path}")


if __name__ == "__main__":
    main()
