#!/usr/bin/env python3
"""
论文图表生成脚本
图5-12  真机低维数据集结构
图5-13  MLP Diffusion Policy 输入输出结构
图5-14  训练 loss 曲线
图5-15  训练样本窗口切分示意图
"""

import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import matplotlib.gridspec as gridspec

# ── 字体 ──────────────────────────────────────────────────────────────────────
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["font.size"] = 10

FIGURE_DIR = Path("figure/real_robot")
FIGURE_DIR.mkdir(parents=True, exist_ok=True)

DATASET_DIR = Path("data/processed/real_robot/training_episodes")
CHECKPOINT = Path("data/checkpoints/diffusion_policy/latest.pt")

# ── 颜色方案 ──────────────────────────────────────────────────────────────────
C_BLUE   = "#1A5CB0"
C_RED    = "#C0392B"
C_GREEN  = "#1E8449"
C_ORANGE = "#D35400"
C_PURPLE = "#6C3483"
C_GRAY   = "#5D6D7E"
C_LIGHT  = "#EBF5FB"
C_LIGHT2 = "#FDFEFE"


# ══════════════════════════════════════════════════════════════════════════════
# 图5-12  真机低维数据集结构
# ══════════════════════════════════════════════════════════════════════════════
def plot_dataset_structure():
    summary_path = DATASET_DIR / "dataset_summary.json"
    with open(summary_path, encoding="utf-8") as f:
        summary = json.load(f)

    episodes = summary["episodes"]
    n_ep = len(episodes)
    lengths = [e["num_samples"] for e in episodes]
    durations = [e["duration_sec"] for e in episodes]
    rates = [e["sample_rate_hz"] for e in episodes]

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("图5-12  真机低维数据集结构", fontsize=13, fontweight="bold", y=0.98)

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.50, wspace=0.38)

    # ── (a) 各 episode 帧数条形图 ──────────────────────────────────────────
    ax0 = fig.add_subplot(gs[0, :2])
    ep_labels = [f"ep{i:02d}" for i in range(n_ep)]
    colors = plt.cm.Blues(np.linspace(0.45, 0.85, n_ep))
    bars = ax0.bar(ep_labels, lengths, color=colors, edgecolor="white", linewidth=0.8)
    ax0.axhline(np.mean(lengths), color=C_RED, linewidth=1.2, linestyle="--",
                label=f"均值 {np.mean(lengths):.0f} 帧")
    for bar, val in zip(bars, lengths):
        ax0.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 60,
                 str(val), ha="center", va="bottom", fontsize=7.5)
    ax0.set_title("(a) 各 episode 帧数", fontsize=10)
    ax0.set_ylabel("帧数")
    ax0.set_ylim(0, max(lengths) * 1.18)
    ax0.legend(fontsize=8)
    ax0.grid(axis="y", alpha=0.3)
    ax0.tick_params(axis="x", labelsize=8)

    # ── (b) 采样率分布 ─────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 2])
    ax1.hist(rates, bins=8, color=C_BLUE, edgecolor="white", alpha=0.85)
    ax1.axvline(np.mean(rates), color=C_RED, linewidth=1.2, linestyle="--",
                label=f"均值 {np.mean(rates):.1f} Hz")
    ax1.set_title("(b) 采样率分布", fontsize=10)
    ax1.set_xlabel("采样率 (Hz)")
    ax1.set_ylabel("episode 数")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    # ── (c) 数据维度示意表 ─────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, :2])
    ax2.axis("off")

    state_names = [
        "right_base_pitch:pos", "right_shoulder_roll:pos", "right_shoulder_yaw:pos",
        "right_elbow_pitch:pos", "right_wrist_pitch:pos", "right_wrist_yaw:pos",
        "right_base_pitch:vel", "right_shoulder_roll:vel", "right_shoulder_yaw:vel",
        "right_elbow_pitch:vel", "right_wrist_pitch:vel", "right_wrist_yaw:vel",
    ]
    action_names = [
        "right_base_pitch:desired", "right_shoulder_roll:desired",
        "right_shoulder_yaw:desired", "right_elbow_pitch:desired",
        "right_wrist_pitch:desired", "right_wrist_yaw:desired",
    ]

    # 绘制两列色块
    col_w, row_h = 0.46, 0.072
    for idx, name in enumerate(state_names):
        y = 1.0 - idx * row_h
        rect = FancyBboxPatch((0.01, y - row_h * 0.85), col_w, row_h * 0.82,
                               boxstyle="round,pad=0.005",
                               facecolor=C_LIGHT, edgecolor=C_BLUE, linewidth=0.6,
                               transform=ax2.transAxes)
        ax2.add_patch(rect)
        ax2.text(0.01 + col_w / 2, y - row_h * 0.42, name,
                 ha="center", va="center", fontsize=7.2, transform=ax2.transAxes)

    for idx, name in enumerate(action_names):
        y = 1.0 - idx * row_h
        rect = FancyBboxPatch((0.52, y - row_h * 0.85), col_w, row_h * 0.82,
                               boxstyle="round,pad=0.005",
                               facecolor="#FEF9E7", edgecolor=C_ORANGE, linewidth=0.6,
                               transform=ax2.transAxes)
        ax2.add_patch(rect)
        ax2.text(0.52 + col_w / 2, y - row_h * 0.42, name,
                 ha="center", va="center", fontsize=7.2, transform=ax2.transAxes)

    ax2.text(0.01 + col_w / 2, 1.04, "robot_state  (12 维)",
             ha="center", va="bottom", fontsize=9, fontweight="bold",
             color=C_BLUE, transform=ax2.transAxes)
    ax2.text(0.52 + col_w / 2, 1.04, "action  (6 维)",
             ha="center", va="bottom", fontsize=9, fontweight="bold",
             color=C_ORANGE, transform=ax2.transAxes)
    ax2.set_title("(c) 数据字段说明", fontsize=10, pad=18)

    # ── (d) 数据集总览文字 ─────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 2])
    ax3.axis("off")
    total_frames = sum(lengths)
    total_dur = sum(durations)
    info = [
        ("episode 数量", f"{n_ep} 条"),
        ("总帧数", f"{total_frames:,} 帧"),
        ("总时长", f"{total_dur:.1f} s  ({total_dur/60:.1f} min)"),
        ("平均采样率", f"{np.mean(rates):.1f} Hz"),
        ("robot_state 维度", "12  (6 pos + 6 vel)"),
        ("action 维度", "6  (desired position)"),
        ("obs_horizon", "2 帧"),
        ("pred_horizon", "16 帧"),
        ("可用训练样本", f"≈ {total_frames - n_ep * (2 + 16 - 1):,} 个"),
    ]
    y_start = 0.95
    for key, val in info:
        ax3.text(0.05, y_start, f"• {key}：", fontsize=8.5, fontweight="bold",
                 transform=ax3.transAxes, va="top")
        ax3.text(0.05, y_start - 0.055, f"   {val}", fontsize=8.5,
                 transform=ax3.transAxes, va="top", color=C_BLUE)
        y_start -= 0.105
    ax3.set_title("(d) 数据集总览", fontsize=10)

    plt.savefig(FIGURE_DIR / "fig5-12_dataset_structure.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  图5-12 已保存: fig5-12_dataset_structure.png")


# ══════════════════════════════════════════════════════════════════════════════
# 图5-13  MLP Diffusion Policy 输入输出结构
# ══════════════════════════════════════════════════════════════════════════════
def plot_model_architecture():
    fig, ax = plt.subplots(figsize=(15, 7))
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 7)
    ax.axis("off")
    fig.suptitle("图5-13  MLP Diffusion Policy 输入输出结构", fontsize=13,
                 fontweight="bold", y=0.97)

    def box(ax, x, y, w, h, label, sublabel="", fc="#EBF5FB", ec=C_BLUE, fs=9):
        rect = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08",
                               facecolor=fc, edgecolor=ec, linewidth=1.5)
        ax.add_patch(rect)
        ax.text(x + w / 2, y + h / 2 + (0.15 if sublabel else 0),
                label, ha="center", va="center", fontsize=fs, fontweight="bold")
        if sublabel:
            ax.text(x + w / 2, y + h / 2 - 0.22, sublabel,
                    ha="center", va="center", fontsize=7.5, color=C_GRAY)

    def arrow(ax, x1, y1, x2, y2, label=""):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color=C_GRAY,
                                   lw=1.5, mutation_scale=14))
        if label:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(mx + 0.08, my, label, fontsize=7.5, color=C_GRAY, va="center")

    # ── 输入区 ────────────────────────────────────────────────────────────
    # robot_state: obs_horizon=2 帧，每帧 12 维
    box(ax, 0.2, 4.8, 2.2, 1.6, "robot_state", "obs_horizon=2\n每帧 12 维 → 展平 24",
        fc="#EBF5FB", ec=C_BLUE)

    # noisy_action: pred_horizon=16 帧，每帧 6 维
    box(ax, 0.2, 2.6, 2.2, 1.6, "noisy_action", "pred_horizon=16\n每帧 6 维 → 展平 96",
        fc="#FEF9E7", ec=C_ORANGE)

    # timestep t
    box(ax, 0.2, 0.5, 2.2, 1.6, "timestep  t", "扩散步 t ∈ [0, T)\nSinusoidal Embed → 64",
        fc="#F9EBEA", ec=C_RED)

    # ── 拼接 ──────────────────────────────────────────────────────────────
    arrow(ax, 2.4, 5.6, 3.6, 4.0)
    arrow(ax, 2.4, 3.4, 3.6, 4.0)
    arrow(ax, 2.4, 1.3, 3.6, 4.0)

    box(ax, 3.6, 3.4, 1.6, 1.2, "Concat", "24 + 96 + 64\n= 184 维",
        fc="#F4ECF7", ec=C_PURPLE, fs=8)

    # ── MLP ──────────────────────────────────────────────────────────────
    arrow(ax, 5.2, 4.0, 6.0, 4.0)

    # Linear 1
    box(ax, 6.0, 3.3, 1.8, 1.4, "Linear", "184 → 256",
        fc="#EBF5FB", ec=C_BLUE, fs=8)
    box(ax, 6.0, 2.5, 1.8, 0.7, "ReLU", "", fc="#EAFAF1", ec=C_GREEN, fs=8)

    arrow(ax, 7.8, 4.0, 8.4, 4.0)

    # Linear 2
    box(ax, 8.4, 3.3, 1.8, 1.4, "Linear", "256 → 256",
        fc="#EBF5FB", ec=C_BLUE, fs=8)
    box(ax, 8.4, 2.5, 1.8, 0.7, "ReLU", "", fc="#EAFAF1", ec=C_GREEN, fs=8)

    arrow(ax, 10.2, 4.0, 10.8, 4.0)

    # Linear 3
    box(ax, 10.8, 3.3, 1.8, 1.4, "Linear", "256 → 96",
        fc="#EBF5FB", ec=C_BLUE, fs=8)

    arrow(ax, 12.6, 4.0, 13.2, 4.0)

    # ── 输出 ──────────────────────────────────────────────────────────────
    box(ax, 13.2, 3.2, 1.6, 1.6, "pred_noise", "reshape\n(16, 6)",
        fc="#FEF9E7", ec=C_ORANGE)

    # ── 训练目标标注 ──────────────────────────────────────────────────────
    ax.annotate("", xy=(14.0, 2.5), xytext=(14.0, 1.8),
                arrowprops=dict(arrowstyle="-|>", color=C_RED, lw=1.5, mutation_scale=12))
    box(ax, 12.8, 0.5, 2.0, 1.2, "MSE Loss", "‖pred_noise − ε‖²",
        fc="#F9EBEA", ec=C_RED, fs=8)

    # ── 标注 MLP 范围 ─────────────────────────────────────────────────────
    rect_mlp = FancyBboxPatch((5.8, 2.3), 6.6, 2.5,
                               boxstyle="round,pad=0.1",
                               facecolor="none", edgecolor=C_PURPLE,
                               linewidth=1.5, linestyle="--")
    ax.add_patch(rect_mlp)
    ax.text(9.1, 5.0, "MLP  (3 层全连接)", ha="center", fontsize=9,
            color=C_PURPLE, fontweight="bold")

    # ── 维度标注 ──────────────────────────────────────────────────────────
    ax.text(7.5, 0.3, "训练时：随机采样扩散步 t，加噪后预测噪声；推理时：从纯噪声逐步去噪（DDPM）",
            ha="center", fontsize=8, color=C_GRAY, style="italic")

    plt.savefig(FIGURE_DIR / "fig5-13_model_architecture.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  图5-13 已保存: fig5-13_model_architecture.png")


# ══════════════════════════════════════════════════════════════════════════════
# 图5-14  训练 loss 曲线
# ══════════════════════════════════════════════════════════════════════════════
def plot_loss_curve():
    import torch
    ckpt = torch.load(CHECKPOINT, map_location="cpu", weights_only=False)
    loss_history = ckpt.get("loss_history")
    if not loss_history:
        print("  WARNING: checkpoint 中无 loss_history，跳过图5-14")
        return

    epochs = np.arange(1, len(loss_history) + 1)
    loss = np.array(loss_history)

    # 平滑曲线（指数移动平均）
    def ema(x, alpha=0.3):
        s = np.zeros_like(x)
        s[0] = x[0]
        for i in range(1, len(x)):
            s[i] = alpha * x[i] + (1 - alpha) * s[i - 1]
        return s

    loss_smooth = ema(loss, alpha=0.4)

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle("图5-14  训练 Loss 曲线（DDPM 噪声预测 MSE）",
                 fontsize=13, fontweight="bold")

    ax.plot(epochs, loss, color=C_BLUE, linewidth=1.0, alpha=0.4, label="每 epoch loss")
    ax.plot(epochs, loss_smooth, color=C_RED, linewidth=2.0, label="平滑曲线 (EMA α=0.4)")

    # 标注最低点
    best_epoch = int(np.argmin(loss)) + 1
    best_loss = loss.min()
    ax.scatter([best_epoch], [best_loss], color=C_RED, s=60, zorder=5)
    ax.annotate(f"最低 {best_loss:.4f}\n(epoch {best_epoch})",
                xy=(best_epoch, best_loss),
                xytext=(best_epoch + 2, best_loss + 0.04),
                fontsize=8.5, color=C_RED,
                arrowprops=dict(arrowstyle="->", color=C_RED, lw=1.2))

    # 阶段分区
    ax.axvspan(1, 10, alpha=0.06, color=C_ORANGE, label="快速下降阶段")
    ax.axvspan(10, 30, alpha=0.06, color=C_GREEN, label="稳定收敛阶段")
    ax.axvspan(30, len(epochs), alpha=0.06, color=C_PURPLE, label="精细优化阶段")

    ax.set_xlabel("Epoch", fontsize=11)
    ax.set_ylabel("MSE Loss", fontsize=11)
    ax.set_xlim(1, len(epochs))
    ax.set_ylim(0, loss[0] * 1.05)
    ax.legend(fontsize=9, loc="upper right")
    ax.grid(True, alpha=0.3)

    # 右侧 y 轴标注最终 loss
    ax2 = ax.twinx()
    ax2.set_ylim(ax.get_ylim())
    ax2.set_yticks([loss[-1]])
    ax2.set_yticklabels([f"最终\n{loss[-1]:.4f}"], fontsize=8, color=C_GRAY)
    ax2.tick_params(axis="y", colors=C_GRAY)

    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "fig5-14_loss_curve.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  图5-14 已保存: fig5-14_loss_curve.png")


# ══════════════════════════════════════════════════════════════════════════════
# 图5-15  训练样本窗口切分示意图
# ══════════════════════════════════════════════════════════════════════════════
def plot_window_slicing():
    obs_horizon = 2
    pred_horizon = 16
    total_show = 28   # 展示的时间步数
    n_windows = 3     # 展示几个滑动窗口

    fig, ax = plt.subplots(figsize=(14, 6))
    fig.suptitle("图5-15  训练样本窗口切分示意图", fontsize=13, fontweight="bold")
    ax.set_xlim(-0.5, total_show + 0.5)
    ax.set_ylim(-1.5, 5.5)
    ax.axis("off")

    cell_w = 0.92
    cell_h = 0.7
    y_timeline = 3.8

    # ── 时间轴帧格子 ──────────────────────────────────────────────────────
    for i in range(total_show):
        rect = FancyBboxPatch((i * cell_w, y_timeline), cell_w * 0.92, cell_h,
                               boxstyle="round,pad=0.04",
                               facecolor="#D6EAF8", edgecolor="#2E86C1", linewidth=0.8)
        ax.add_patch(rect)
        ax.text(i * cell_w + cell_w * 0.46, y_timeline + cell_h / 2,
                str(i), ha="center", va="center", fontsize=7.5, color="#1A5276")

    ax.text(total_show * cell_w + 0.2, y_timeline + cell_h / 2,
            "...", ha="left", va="center", fontsize=12, color=C_GRAY)
    ax.text(-0.3, y_timeline + cell_h / 2, "时间步",
            ha="right", va="center", fontsize=9, color=C_GRAY)

    # ── 滑动窗口 ──────────────────────────────────────────────────────────
    window_starts = [0, 3, 6]
    colors_obs    = ["#1A5CB0", "#1E8449", "#6C3483"]
    colors_pred   = ["#D35400", "#C0392B", "#7D6608"]
    y_levels      = [2.4, 1.2, 0.0]

    for win_idx, (t_start, y_lv) in enumerate(zip(window_starts, y_levels)):
        t_obs_start  = t_start
        t_obs_end    = t_start + obs_horizon - 1
        t_pred_start = t_obs_end + 1
        t_pred_end   = t_pred_start + pred_horizon - 1

        c_obs  = colors_obs[win_idx]
        c_pred = colors_pred[win_idx]

        # obs 窗口
        x0 = t_obs_start * cell_w
        w_obs = obs_horizon * cell_w * 0.92
        rect_obs = FancyBboxPatch((x0, y_lv), w_obs, cell_h * 0.85,
                                   boxstyle="round,pad=0.05",
                                   facecolor=c_obs, edgecolor=c_obs,
                                   linewidth=1.2, alpha=0.85)
        ax.add_patch(rect_obs)
        ax.text(x0 + w_obs / 2, y_lv + cell_h * 0.42,
                f"obs\n({obs_horizon}帧)", ha="center", va="center",
                fontsize=7.5, color="white", fontweight="bold")

        # pred 窗口
        x1 = t_pred_start * cell_w
        w_pred = pred_horizon * cell_w * 0.92
        rect_pred = FancyBboxPatch((x1, y_lv), w_pred, cell_h * 0.85,
                                    boxstyle="round,pad=0.05",
                                    facecolor=c_pred, edgecolor=c_pred,
                                    linewidth=1.2, alpha=0.85)
        ax.add_patch(rect_pred)
        ax.text(x1 + w_pred / 2, y_lv + cell_h * 0.42,
                f"pred  ({pred_horizon}帧)", ha="center", va="center",
                fontsize=7.5, color="white", fontweight="bold")

        # 连接线到时间轴
        for t in range(t_obs_start, t_obs_end + 1):
            ax.plot([t * cell_w + cell_w * 0.46, t * cell_w + cell_w * 0.46],
                    [y_timeline, y_lv + cell_h * 0.85],
                    color=c_obs, linewidth=0.6, alpha=0.5, linestyle=":")
        for t in range(t_pred_start, min(t_pred_end + 1, total_show)):
            ax.plot([t * cell_w + cell_w * 0.46, t * cell_w + cell_w * 0.46],
                    [y_timeline, y_lv + cell_h * 0.85],
                    color=c_pred, linewidth=0.6, alpha=0.5, linestyle=":")

        # 样本标签
        ax.text(-0.3, y_lv + cell_h * 0.42,
                f"样本\n{win_idx + 1}", ha="right", va="center",
                fontsize=8, color=C_GRAY)

    # ── 图例 & 说明 ───────────────────────────────────────────────────────
    legend_y = -1.1
    obs_patch  = mpatches.Patch(color=colors_obs[0],  label=f"观测窗口 (obs_horizon = {obs_horizon})")
    pred_patch = mpatches.Patch(color=colors_pred[0], label=f"预测窗口 (pred_horizon = {pred_horizon})")
    ax.legend(handles=[obs_patch, pred_patch], loc="lower center",
              bbox_to_anchor=(0.5, -0.22), ncol=2, fontsize=9,
              framealpha=0.9)

    ax.text(total_show * cell_w / 2, -1.0,
            f"每次滑动步长 = 1；最小 episode 长度 ≥ obs_horizon + pred_horizon − 1 = {obs_horizon + pred_horizon - 1} 帧",
            ha="center", va="center", fontsize=8.5, color=C_GRAY, style="italic")

    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "fig5-15_window_slicing.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  图5-15 已保存: fig5-15_window_slicing.png")


# ══════════════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("生成论文图表...")
    plot_dataset_structure()
    plot_model_architecture()
    plot_loss_curve()
    plot_window_slicing()
    print(f"\n全部完成，图片保存在: {FIGURE_DIR.resolve()}")
