"""
Generate paper plots for Section 3.3.3 from REAL ablation run data.
Run after: scripts/isaac_rl/run_ablation.ps1

Produces (in figure/simulation/isaac_rl/runs/paper_plots/):
  loss_curve_annotated.png
  lr_comparison.png
  batchsize_comparison.png
  ablation_summary_bar.png
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm


# ── Chinese font ─────────────────────────────────────────────────────────────
def _setup_cjk_font():
    candidates = [
        "Microsoft YaHei", "SimHei", "SimSun",
        "WenQuanYi Micro Hei", "Noto Sans CJK SC",
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    for name in candidates:
        if name in available:
            matplotlib.rcParams["font.family"] = name
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
    return None

_font = _setup_cjk_font()
if _font:
    print(f"[font] {_font}")

# ── paths ─────────────────────────────────────────────────────────────────────
ROOT    = Path(__file__).resolve().parents[2]
RUNS    = ROOT / "data" / "simulation" / "isaac_rl" / "runs"
OUT_DIR = ROOT / "figure" / "simulation" / "isaac_rl" / "runs" / "paper_plots"
OUT_DIR.mkdir(parents=True, exist_ok=True)

BLUE   = "#4c72b0"
RED    = "#c44e52"
GREEN  = "#55a868"
ORANGE = "#dd8452"
PURPLE = "#8172b2"
GRAY   = "#999999"


# ── helpers ───────────────────────────────────────────────────────────────────
def load_csv(path: Path) -> dict[str, np.ndarray]:
    rows: dict[str, list[float]] = {}
    with path.open() as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                rows.setdefault(k, []).append(float(v))
    return {k: np.array(v) for k, v in rows.items()}


def smooth(y: np.ndarray, w: int = 8) -> np.ndarray:
    kernel = np.ones(w) / w
    return np.convolve(y, kernel, mode="same")


def best_val(run_dir: Path) -> float:
    cfg = run_dir / "config.json"
    if cfg.exists():
        return float(json.loads(cfg.read_text())["best_val_loss"])
    val = load_csv(run_dir / "val_loss.csv")
    return float(np.min(val["loss"]))


def load_run(run_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (epochs, train_loss, val_loss)."""
    tr  = load_csv(run_dir / "train_loss.csv")
    val = load_csv(run_dir / "val_loss.csv")
    return tr["epoch"], tr["loss"], val["loss"]


# ── 1. Annotated loss curve (baseline) ───────────────────────────────────────
baseline_dir = RUNS / "ablation_lr_1e3"
if not baseline_dir.exists():
    # fall back to the original run
    baseline_dir = RUNS / "pushcube_bc_aug100_v1"

ep, tr_loss, vl_loss = load_run(baseline_dir)
best_epoch = int(np.argmin(vl_loss)) + 1
best_loss  = float(np.min(vl_loss))

fig, ax = plt.subplots(figsize=(7.2, 4.5))
ax.plot(ep, tr_loss, color=BLUE, lw=1.5, label="训练集 MSE")
ax.plot(ep, vl_loss, color=RED,  lw=1.5, linestyle="--", label="验证集 MSE")
ax.axvline(best_epoch, color=GRAY, lw=1.0, linestyle=":")
ax.scatter([best_epoch], [best_loss], color=RED, zorder=5, s=40)
ax.annotate(
    f"最优验证损失\nEpoch {best_epoch}\nMSE={best_loss:.4f}",
    xy=(best_epoch, best_loss),
    xytext=(best_epoch + 20, best_loss + 0.0015),
    fontsize=8,
    arrowprops=dict(arrowstyle="->", color=GRAY, lw=0.8),
    color=RED,
)
ax.axvspan(150, int(ep[-1]), alpha=0.06, color=GREEN, label="收敛平台期")
ax.set_xlabel("训练轮次 (Epoch)", fontsize=11)
ax.set_ylabel("MSE 损失", fontsize=11)
ax.set_title("行为克隆策略训练损失曲线", fontsize=12)
ax.legend(fontsize=9)
ax.grid(True, alpha=0.25)
ax.set_xlim(1, int(ep[-1]))
fig.tight_layout()
fig.savefig(OUT_DIR / "loss_curve_annotated.png", dpi=220, bbox_inches="tight")
plt.close(fig)
print(f"[saved] loss_curve_annotated.png  best_epoch={best_epoch}  best_val={best_loss:.6f}")


# ── 2. LR comparison ─────────────────────────────────────────────────────────
lr_runs = {
    "lr=1e-2":      (RUNS / "ablation_lr_1e2",  ORANGE),
    "lr=1e-3 (本文)": (RUNS / "ablation_lr_1e3",  BLUE),
    "lr=1e-4":      (RUNS / "ablation_lr_1e4",  GREEN),
}

missing = [k for k, (d, _) in lr_runs.items() if not d.exists()]
if missing:
    print(f"[WARN] lr ablation dirs not found: {missing} — skipping lr_comparison.png")
else:
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=False)
    for label, (run_dir, c) in lr_runs.items():
        ep_r, tr_r, vl_r = load_run(run_dir)
        axes[0].plot(ep_r, smooth(tr_r), color=c, lw=1.5, label=label)
        axes[1].plot(ep_r, smooth(vl_r), color=c, lw=1.5, linestyle="--", label=label)
    for ax, title in zip(axes, ["训练集 MSE", "验证集 MSE"]):
        ax.set_xlabel("Epoch", fontsize=10)
        ax.set_ylabel("MSE 损失", fontsize=10)
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.25)
    fig.suptitle("学习率消融实验", fontsize=13, y=1.01)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "lr_comparison.png", dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("[saved] lr_comparison.png")


# ── 3. Batch size comparison ──────────────────────────────────────────────────
bs_runs = {
    "batch=64":         (RUNS / "ablation_bs64",   ORANGE),
    "batch=128":        (RUNS / "ablation_bs128",  GREEN),
    "batch=256 (本文)":  (RUNS / "ablation_lr_1e3", BLUE),
    "batch=512":        (RUNS / "ablation_bs512",  PURPLE),
}

missing = [k for k, (d, _) in bs_runs.items() if not d.exists()]
if missing:
    print(f"[WARN] batch ablation dirs not found: {missing} — skipping batchsize_comparison.png")
else:
    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    for label, (run_dir, c) in bs_runs.items():
        ep_r, _, vl_r = load_run(run_dir)
        ax.plot(ep_r, smooth(vl_r), color=c, lw=1.5, label=label)
    ax.set_xlabel("Epoch", fontsize=11)
    ax.set_ylabel("验证集 MSE 损失", fontsize=11)
    ax.set_title("批量大小消融实验（验证集）", fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "batchsize_comparison.png", dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("[saved] batchsize_comparison.png")


# ── 4. Summary bar chart ──────────────────────────────────────────────────────
all_runs = {
    "lr=1e-2":          RUNS / "ablation_lr_1e2",
    "lr=1e-3\n(本文)":  RUNS / "ablation_lr_1e3",
    "lr=1e-4":          RUNS / "ablation_lr_1e4",
    "batch=64":         RUNS / "ablation_bs64",
    "batch=128":        RUNS / "ablation_bs128",
    "batch=256\n(本文)": RUNS / "ablation_lr_1e3",   # same run
    "batch=512":        RUNS / "ablation_bs512",
}

available = {k: v for k, v in all_runs.items() if v.exists()}
if len(available) < 2:
    print("[WARN] not enough ablation runs for summary bar — skipping")
else:
    labels = list(available.keys())
    vals   = [best_val(d) for d in available.values()]
    colors = [RED if "(本文)" in l else BLUE for l in labels]

    fig, ax = plt.subplots(figsize=(9, 4.2))
    bars = ax.bar(labels, vals, color=colors, width=0.55,
                  edgecolor="white", linewidth=0.5)
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 0.00005,
                f"{v:.4f}", ha="center", va="bottom", fontsize=7.5)
    baseline_val = best_val(RUNS / "ablation_lr_1e3") if (RUNS / "ablation_lr_1e3").exists() else min(vals)
    ax.axhline(baseline_val, color=RED, lw=1.0, linestyle=":", alpha=0.6)
    ax.set_ylabel("最优验证集 MSE", fontsize=11)
    ax.set_title("超参数消融实验汇总", fontsize=12)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "ablation_summary_bar.png", dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("[saved] ablation_summary_bar.png")

print(f"\nAll plots saved to: {OUT_DIR}")
