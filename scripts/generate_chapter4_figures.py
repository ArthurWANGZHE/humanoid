#!/usr/bin/env python3
"""
Generate thesis figures for Chapter 4 from the real rosbag-derived dataset.

The script intentionally avoids importing torch so it can run in a lightweight
Python environment. It reads scalar checkpoint metadata through pickle and uses
existing offline-evaluation figures that were already produced by the model
evaluation script.
"""

import io
import json
import pickle
import re
import shutil
import zipfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
DATASET_DIR = ROOT / "data/processed/real_robot" / "training_episodes"
FIGURE_DIR = ROOT / "figure" / "chapter4"
CHECKPOINT = ROOT / "data" / "checkpoints" / "diffusion_policy" / "latest.pt"
REPORT = ROOT / "doc" / "real_robot_experiment_report.md"

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["font.size"] = 10

BLUE = "#1A5CB0"
RED = "#C0392B"
GREEN = "#1E8449"
ORANGE = "#D35400"
PURPLE = "#6C3483"
GRAY = "#5D6D7E"
LIGHT_BLUE = "#EBF5FB"
LIGHT_ORANGE = "#FEF5E7"

JOINT_NAMES = [
    "base_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_yaw",
]


class _Dummy:
    def __init__(self, *args, **kwargs):
        pass

    def __call__(self, *args, **kwargs):
        return _Dummy()

    def __setstate__(self, state):
        pass


def _dummy_rebuild(*args, **kwargs):
    return _Dummy()


class _TorchlessUnpickler(pickle.Unpickler):
    def persistent_load(self, pid):
        return _Dummy()

    def find_class(self, module, name):
        if module.startswith("torch"):
            if name.startswith("_rebuild") or name.endswith("Storage"):
                return _dummy_rebuild
            return _Dummy
        return super().find_class(module, name)


def load_checkpoint_scalars(path: Path):
    """Load non-tensor fields from a torch checkpoint without importing torch."""
    if not path.exists() or not zipfile.is_zipfile(path):
        return {}
    with zipfile.ZipFile(path) as zf:
        data_name = next((n for n in zf.namelist() if n.endswith("data.pkl")), None)
        if data_name is None:
            return {}
        raw = zf.read(data_name)
    obj = _TorchlessUnpickler(io.BytesIO(raw)).load()
    return {
        "config": obj.get("config", {}),
        "stats": obj.get("stats", {}),
        "loss_history": obj.get("loss_history", []),
    }


def load_summary():
    with (DATASET_DIR / "dataset_summary.json").open(encoding="utf-8") as f:
        return json.load(f)


def load_arrays():
    episodes = []
    for ep_dir in sorted(DATASET_DIR.glob("episode_*")):
        state = np.load(ep_dir / "robot_state.npy")
        action = np.load(ep_dir / "action.npy")
        ts = np.load(ep_dir / "timestamps.npy")
        right_error = np.load(ep_dir / "right_error_pos.npy")
        episodes.append(
            {
                "name": ep_dir.name,
                "state": state,
                "action": action,
                "timestamps": ts,
                "right_error": right_error,
            }
        )
    return episodes


def ema(values, alpha=0.25):
    values = np.asarray(values, dtype=np.float64)
    out = np.zeros_like(values)
    out[0] = values[0]
    for i in range(1, len(values)):
        out[i] = alpha * values[i] + (1.0 - alpha) * out[i - 1]
    return out


def training_window_count(lengths, obs_horizon=2, pred_horizon=16):
    min_len = obs_horizon + pred_horizon - 1
    return [max(0, int(n) - min_len + 1) for n in lengths]


def plot_dataset_overview(summary, episodes):
    lengths = np.asarray([e["num_samples"] for e in summary["episodes"]], dtype=np.int64)
    rates = np.asarray([e["sample_rate_hz"] for e in summary["episodes"]], dtype=np.float64)
    durations = np.asarray([e["duration_sec"] for e in summary["episodes"]], dtype=np.float64)
    usable = np.asarray(training_window_count(lengths), dtype=np.int64)
    all_pos = np.vstack([ep["state"][:, :6] for ep in episodes])
    all_dt_ms = np.concatenate([np.diff(ep["timestamps"]) * 1000.0 for ep in episodes])

    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(2, 3, hspace=0.42, wspace=0.34)
    fig.suptitle("图4-3 真机 rosbag 数据集统计", fontsize=15, fontweight="bold")

    ax0 = fig.add_subplot(gs[0, :2])
    x = np.arange(len(lengths))
    bars = ax0.bar(x, lengths, color=BLUE, alpha=0.82, label="原始帧数")
    ax0.bar(x, usable, color=ORANGE, alpha=0.72, label="窗口化训练样本")
    ax0.axhline(lengths.mean(), color=RED, linestyle="--", linewidth=1.2, label=f"平均 {lengths.mean():.0f} 帧")
    for bar, val in zip(bars, lengths):
        ax0.text(bar.get_x() + bar.get_width() / 2, val + 70, f"{val}", ha="center", fontsize=8)
    ax0.set_xticks(x, [f"ep{i:02d}" for i in x])
    ax0.set_ylabel("样本数")
    ax0.set_title("(a) 各轨迹长度")
    ax0.grid(axis="y", alpha=0.25)
    ax0.legend(fontsize=9)

    ax1 = fig.add_subplot(gs[0, 2])
    ax1.hist(all_dt_ms, bins=80, color=GREEN, alpha=0.82, edgecolor="white")
    ax1.axvline(all_dt_ms.mean(), color=RED, linestyle="--", label=f"均值 {all_dt_ms.mean():.2f} ms")
    ax1.set_title("(b) 采样间隔分布")
    ax1.set_xlabel("采样间隔 (ms)")
    ax1.set_ylabel("频数")
    ax1.grid(alpha=0.25)
    ax1.legend(fontsize=8)

    ax2 = fig.add_subplot(gs[1, :2])
    ax2.boxplot(
        [all_pos[:, i] for i in range(6)],
        tick_labels=JOINT_NAMES,
        patch_artist=True,
        boxprops=dict(facecolor=LIGHT_BLUE, color=BLUE),
        medianprops=dict(color=RED),
        whiskerprops=dict(color=BLUE),
        capprops=dict(color=BLUE),
    )
    ax2.set_title("(c) 右臂关节位置分布")
    ax2.set_ylabel("关节角 (rad)")
    ax2.tick_params(axis="x", labelrotation=18)
    ax2.grid(axis="y", alpha=0.25)

    ax3 = fig.add_subplot(gs[1, 2])
    ax3.axis("off")
    stat_rows = [
        ("rosbag 数量", f"{len(summary['episodes'])}"),
        ("有效轨迹数量", f"{summary['num_episodes']}"),
        ("总帧数", f"{summary['total_samples']:,}"),
        ("窗口化样本数", f"{usable.sum():,}"),
        ("总时长", f"{summary['total_duration_sec']:.1f} s"),
        ("平均采样率", f"{rates.mean():.2f} Hz"),
        ("采样率范围", f"{rates.min():.2f} - {rates.max():.2f} Hz"),
        ("状态 / 动作维度", f"{summary['robot_state_dim']} / {summary['action_dim']}"),
        ("窗口长度", "obs=2, pred=16"),
    ]
    y = 0.96
    for key, value in stat_rows:
        ax3.text(0.03, y, key, fontsize=9.5, fontweight="bold", va="top", transform=ax3.transAxes)
        ax3.text(0.55, y, value, fontsize=9.5, color=BLUE, va="top", transform=ax3.transAxes)
        y -= 0.10
    ax3.set_title("(d) 数据集汇总")

    out = FIGURE_DIR / "fig4-3_real_dataset_statistics.png"
    fig.savefig(out, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out}")


def plot_training_loss(checkpoint_scalars):
    loss = checkpoint_scalars.get("loss_history", [])
    if not loss:
        print("skip loss curve: loss_history not found")
        return
    loss = np.asarray(loss, dtype=np.float64)
    epochs = np.arange(1, len(loss) + 1)
    smooth = ema(loss, alpha=0.20)
    best_idx = int(np.argmin(loss))

    fig, ax = plt.subplots(figsize=(11, 5.8))
    fig.suptitle("图4-4 Diffusion Policy 训练损失曲线", fontsize=15, fontweight="bold")
    ax.plot(epochs, loss, color=BLUE, alpha=0.34, linewidth=0.9, label="每轮训练 loss")
    ax.plot(epochs, smooth, color=RED, linewidth=2.1, label="指数平滑曲线")
    ax.scatter(epochs[best_idx], loss[best_idx], color=RED, s=52, zorder=4)
    ax.annotate(
        f"最低 {loss[best_idx]:.4f}\nepoch {epochs[best_idx]}",
        xy=(epochs[best_idx], loss[best_idx]),
        xytext=(epochs[best_idx] - 130, loss[best_idx] + 0.08),
        arrowprops=dict(arrowstyle="->", color=RED, lw=1.2),
        fontsize=9,
        color=RED,
    )
    ax.text(
        0.99,
        0.92,
        f"初始: {loss[0]:.4f}\n最终: {loss[-1]:.4f}\n训练轮数: {len(loss)}",
        ha="right",
        va="top",
        transform=ax.transAxes,
        bbox=dict(facecolor="white", edgecolor=GRAY, boxstyle="round,pad=0.35", alpha=0.9),
        fontsize=9,
    )
    ax.set_xlabel("Epoch")
    ax.set_ylabel("噪声预测 MSE")
    ax.set_xlim(1, len(loss))
    ax.set_ylim(0, loss[0] * 1.08)
    ax.grid(alpha=0.25)
    ax.legend()

    out = FIGURE_DIR / "fig4-4_training_loss_curve.png"
    fig.savefig(out, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out}")


def parse_eval_metrics():
    if not REPORT.exists():
        return None
    text = REPORT.read_text(encoding="utf-8")
    result = {}
    for key, pattern in {
        "mse": r"\| MSE \(rad\) \| ([0-9.]+) \|",
        "rmse": r"\| RMSE \(rad\) \| ([0-9.]+) \|",
        "mae": r"\| MAE \(rad\) \| ([0-9.]+) \|",
        "mae_deg": r"\| MAE \(deg\) \| ([0-9.]+)",
    }.items():
        match = re.search(pattern, text)
        if match:
            result[key] = float(match.group(1))

    joint_rows = re.findall(r"\| ([a-z_]+) \| ([0-9.]+) \| ([0-9.]+).*\|", text)
    per_joint = [(name, float(rad), float(deg)) for name, rad, deg in joint_rows if name in JOINT_NAMES]
    if per_joint:
        result["per_joint"] = per_joint
    return result or None


def plot_eval_metrics_summary():
    metrics = parse_eval_metrics()
    if not metrics:
        print("skip eval metric summary: report metrics not found")
        return

    per_joint = metrics.get("per_joint", [])
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={"width_ratios": [1.0, 1.5]})
    fig.suptitle("图4-5 离线评估误差统计", fontsize=15, fontweight="bold")

    ax0 = axes[0]
    ax0.axis("off")
    rows = [
        ("MSE", f"{metrics.get('mse', 0):.6f} rad$^2$"),
        ("RMSE", f"{metrics.get('rmse', 0):.4f} rad"),
        ("MAE", f"{metrics.get('mae', 0):.4f} rad"),
        ("MAE", f"{metrics.get('mae_deg', 0):.2f} deg"),
    ]
    y = 0.84
    for key, value in rows:
        box = FancyBboxPatch(
            (0.08, y - 0.09),
            0.82,
            0.12,
            boxstyle="round,pad=0.02",
            facecolor=LIGHT_BLUE,
            edgecolor=BLUE,
            transform=ax0.transAxes,
        )
        ax0.add_patch(box)
        ax0.text(0.18, y - 0.03, key, fontweight="bold", transform=ax0.transAxes)
        ax0.text(0.48, y - 0.03, value, color=BLUE, transform=ax0.transAxes)
        y -= 0.18
    ax0.set_title("(a) 总体指标")

    ax1 = axes[1]
    names = [x[0] for x in per_joint]
    values = [x[1] for x in per_joint]
    ax1.bar(names, values, color=ORANGE, alpha=0.82)
    ax1.axhline(np.mean(values), color=RED, linestyle="--", label=f"平均 {np.mean(values):.4f} rad")
    ax1.set_title("(b) 各关节 MAE")
    ax1.set_ylabel("MAE (rad)")
    ax1.tick_params(axis="x", labelrotation=20)
    ax1.grid(axis="y", alpha=0.25)
    ax1.legend(fontsize=9)

    out = FIGURE_DIR / "fig4-5_offline_eval_metrics.png"
    fig.savefig(out, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out}")


def copy_existing_eval_figures():
    mapping = {
        "eval_pred_vs_true_sample.png": "fig4-6_predicted_vs_true_action.png",
        "eval_error_distribution.png": "fig4-7_prediction_error_distribution.png",
        "eval_error_vs_horizon.png": "fig4-8_error_vs_prediction_horizon.png",
        "right_arm_tracking_error.png": "fig4-9_controller_tracking_error.png",
        "trajectory_all_episodes_overlay.png": "fig4-10_real_trajectory_overlay.png",
    }
    src_dir = ROOT / "figure" / "real_robot"
    for src_name, dst_name in mapping.items():
        src = src_dir / src_name
        dst = FIGURE_DIR / dst_name
        if src.exists():
            shutil.copy2(src, dst)
            print(f"copied {dst}")
        else:
            print(f"missing {src}")


def plot_deployment_pipeline():
    fig, ax = plt.subplots(figsize=(14.8, 7.0))
    fig.suptitle("图4-11 视觉输入策略真机部署流程与安全约束", fontsize=15, fontweight="bold")
    ax.set_xlim(0, 14.8)
    ax.set_ylim(0, 7.0)
    ax.axis("off")

    def box(x, y, w, h, title, body, fc, ec):
        patch = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08", facecolor=fc, edgecolor=ec, linewidth=1.4)
        ax.add_patch(patch)
        ax.text(x + w / 2, y + h - 0.28, title, ha="center", va="top", fontsize=10, fontweight="bold")
        ax.text(x + w / 2, y + h / 2 - 0.08, body, ha="center", va="center", fontsize=8.2, linespacing=1.35)

    def arrow(x1, y1, x2, y2, label=""):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1), arrowprops=dict(arrowstyle="-|>", color=GRAY, lw=1.5))
        if label:
            ax.text((x1 + x2) / 2, (y1 + y2) / 2 + 0.18, label, ha="center", fontsize=8, color=GRAY)

    box(0.3, 5.0, 2.2, 1.15, "图像订阅", "/head_camera/depth_image\n头部 RealSense 深度图", LIGHT_BLUE, BLUE)
    box(0.3, 3.2, 2.2, 1.15, "状态订阅", "/joint_states\n右臂位置 + 速度", LIGHT_BLUE, BLUE)
    box(3.0, 5.0, 2.1, 1.15, "图像预处理", "resize / normalize\n时间戳同步", "#EAFAF1", GREEN)
    box(3.0, 3.2, 2.1, 1.15, "状态缓存", "最近 2 帧状态\n归一化 robot_state", "#EAFAF1", GREEN)
    box(5.8, 4.1, 2.25, 1.35, "观测融合", "图像特征 + 关节状态\n构成策略条件输入", "#F4ECF7", PURPLE)
    box(8.8, 4.1, 2.25, 1.35, "策略推理", "视觉编码器 + Diffusion\nDDPM 去噪采样", "#F4ECF7", PURPLE)
    box(11.7, 4.1, 2.2, 1.35, "动作筛选", "取第 1 步动作\n反归一化", LIGHT_ORANGE, ORANGE)
    box(11.7, 2.0, 2.2, 1.35, "安全约束", "关节限位\n单步增量限幅\n急停/干运行", "#F9EBEA", RED)
    box(8.8, 2.0, 2.25, 1.35, "命令发布", "/right_joint_command\n6 维关节目标", LIGHT_BLUE, BLUE)
    box(5.8, 2.0, 2.25, 1.35, "执行验证", "dry-run → 小幅度\n→ 完整轨迹", "#FEF9E7", ORANGE)

    arrow(2.5, 5.58, 3.0, 5.58)
    arrow(2.5, 3.78, 3.0, 3.78)
    arrow(5.1, 5.58, 5.8, 4.86)
    arrow(5.1, 3.78, 5.8, 4.58)
    arrow(8.05, 4.78, 8.8, 4.78)
    arrow(11.05, 4.78, 11.7, 4.78)
    arrow(12.8, 4.1, 12.8, 3.35, "限幅检查")
    arrow(11.7, 2.68, 11.05, 2.68)
    arrow(8.8, 2.68, 8.05, 2.68)
    arrow(6.9, 3.35, 6.9, 4.1, "通过后执行")
    arrow(9.9, 2.0, 9.9, 1.1, "反馈闭环")
    ax.text(9.9, 0.78, "执行后状态与图像重新进入下一控制周期", ha="center", fontsize=8.5, color=GRAY)

    out = FIGURE_DIR / "fig4-11_deployment_pipeline.png"
    fig.savefig(out, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out}")


def main():
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    summary = load_summary()
    episodes = load_arrays()
    checkpoint_scalars = load_checkpoint_scalars(CHECKPOINT)

    plot_dataset_overview(summary, episodes)
    plot_training_loss(checkpoint_scalars)
    plot_eval_metrics_summary()
    copy_existing_eval_figures()
    plot_deployment_pipeline()
    print(f"\nChapter 4 figures saved to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
