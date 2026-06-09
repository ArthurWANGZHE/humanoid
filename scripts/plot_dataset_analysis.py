#!/usr/bin/env python3
"""
数据集轨迹与误差分析图
生成四张图：
  A. 单条 episode 轨迹（desired vs actual，6 关节 2x3）
  B. 全部 episode 叠加轨迹（6 关节 2x3）
  C. 全部 episode 误差分布（箱线图 + 小提琴图）
  D. 误差时序热力图（每条 episode 的逐帧误差）
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable

# ── 字体 ─────────────────────────────────────────────────────────────────────
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["pdf.fonttype"] = 42

# ── 路径 ─────────────────────────────────────────────────────────────────────
DATASET_DIR = Path("data/processed/real_robot/training_episodes")
OUT_DIR     = Path("figure/dataset_analysis")
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── 关节名 ───────────────────────────────────────────────────────────────────
JOINTS = [
    "base_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_yaw",
]

# ── 颜色 ─────────────────────────────────────────────────────────────────────
C_DESIRED = "#C0392B"   # 深红
C_ACTUAL  = "#1A5276"   # 深蓝
C_ERROR   = "#E67E22"   # 橙
C_FILL    = "#F0B27A"   # 浅橙填充


# ─────────────────────────────────────────────────────────────────────────────
# 图A  单条 episode 轨迹（episode_000000）
# ─────────────────────────────────────────────────────────────────────────────
def plot_single_trajectory():
    ep_dirs = sorted(DATASET_DIR.glob("episode_*"))
    # 选 RMSE 最低的那条（ep0）
    ep_dir = ep_dirs[0]

    ts      = np.load(ep_dir / "timestamps.npy")
    actual  = np.load(ep_dir / "right_actual_pos.npy")
    desired = np.load(ep_dir / "right_desired_pos.npy")
    error   = desired - actual
    t = ts - ts[0]   # 从 0 开始

    rmse_per_joint = np.sqrt(np.mean(error**2, axis=0))
    max_per_joint  = np.abs(error).max(axis=0)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"右臂关节轨迹：期望 vs 实际  |  {ep_dir.name}  "
        f"({len(ts)} 帧, {t[-1]:.1f} s)",
        fontsize=14, fontweight="bold", y=0.99,
    )
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.52, wspace=0.38)

    for i, joint in enumerate(JOINTS):
        ax = fig.add_subplot(gs[i // 3, i % 3])

        ax.plot(t, desired[:, i], color=C_DESIRED, linewidth=1.6,
                label="期望 (desired)", zorder=3)
        ax.plot(t, actual[:, i],  color=C_ACTUAL,  linewidth=1.4,
                alpha=0.9, label="实际 (actual)", zorder=2)
        ax.fill_between(t, desired[:, i], actual[:, i],
                        alpha=0.25, color=C_FILL, label="误差区域", zorder=1)

        ax.set_title(
            f"{joint}\n"
            f"RMSE = {rmse_per_joint[i]*1000:.2f} mrad  |  "
            f"max = {max_per_joint[i]*1000:.1f} mrad",
            fontsize=9.5,
        )
        ax.set_xlabel("时间 (s)", fontsize=8.5)
        ax.set_ylabel("位置 (rad)", fontsize=8.5)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.35, linewidth=0.6)
        ax.legend(fontsize=7.5, loc="upper right")
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2f"))

    plt.savefig(OUT_DIR / "A_single_episode_trajectory.png",
                dpi=200, bbox_inches="tight")
    plt.close()
    print("  A 已保存: A_single_episode_trajectory.png")


# ─────────────────────────────────────────────────────────────────────────────
# 图B  全部 episode 叠加轨迹
# ─────────────────────────────────────────────────────────────────────────────
def plot_overlay_trajectory():
    ep_dirs = sorted(DATASET_DIR.glob("episode_*"))
    n = len(ep_dirs)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"右臂关节轨迹叠加（全部 {n} 条 episode）",
        fontsize=14, fontweight="bold", y=0.99,
    )
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.52, wspace=0.38)
    axes = [fig.add_subplot(gs[i // 3, i % 3]) for i in range(6)]

    # 颜色渐变区分不同 episode
    cmap = plt.cm.tab10
    alpha_line = max(0.35, 0.9 / n)

    all_rmse = np.zeros(6)
    all_max  = np.zeros(6)
    total_frames = 0

    for ep_idx, ep_dir in enumerate(ep_dirs):
        ts      = np.load(ep_dir / "timestamps.npy")
        actual  = np.load(ep_dir / "right_actual_pos.npy")
        desired = np.load(ep_dir / "right_desired_pos.npy")
        error   = desired - actual
        t = ts - ts[0]
        total_frames += len(ts)

        c = cmap(ep_idx % 10)
        lbl = ep_dir.name if ep_idx < 10 else None

        for i in range(6):
            axes[i].plot(t, desired[:, i], color=C_DESIRED,
                         linewidth=0.9, alpha=alpha_line,
                         label="期望" if ep_idx == 0 else None)
            axes[i].plot(t, actual[:, i], color=c,
                         linewidth=0.9, alpha=alpha_line, label=lbl)
            all_rmse[i] += np.mean(error[:, i]**2) * len(ts)
            all_max[i]   = max(all_max[i], np.abs(error[:, i]).max())

    all_rmse = np.sqrt(all_rmse / total_frames)

    for i, joint in enumerate(JOINTS):
        axes[i].set_title(
            f"{joint}\n"
            f"RMSE = {all_rmse[i]*1000:.2f} mrad  |  "
            f"max = {all_max[i]*1000:.1f} mrad",
            fontsize=9.5,
        )
        axes[i].set_xlabel("时间 (s)", fontsize=8.5)
        axes[i].set_ylabel("位置 (rad)", fontsize=8.5)
        axes[i].tick_params(labelsize=8)
        axes[i].grid(True, alpha=0.35, linewidth=0.6)
        axes[i].yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2f"))

    # 统一图例放在最后一个子图
    axes[0].legend(fontsize=7, loc="upper right", ncol=1)

    plt.savefig(OUT_DIR / "B_overlay_trajectory.png",
                dpi=200, bbox_inches="tight")
    plt.close()
    print("  B 已保存: B_overlay_trajectory.png")


# ─────────────────────────────────────────────────────────────────────────────
# 图C  误差分布（箱线图 + 散点抖动）
# ─────────────────────────────────────────────────────────────────────────────
def plot_error_distribution():
    ep_dirs = sorted(DATASET_DIR.glob("episode_*"))

    # 收集每条 episode 的 per-joint RMSE 和 max
    rmse_mat = []   # shape (n_ep, 6)
    max_mat  = []

    for ep_dir in ep_dirs:
        actual  = np.load(ep_dir / "right_actual_pos.npy")
        desired = np.load(ep_dir / "right_desired_pos.npy")
        err = desired - actual
        rmse_mat.append(np.sqrt(np.mean(err**2, axis=0)) * 1000)
        max_mat.append(np.abs(err).max(axis=0) * 1000)

    rmse_mat = np.array(rmse_mat)   # (n_ep, 6)
    max_mat  = np.array(max_mat)

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle("右臂关节跟踪误差分布（全部 episode）",
                 fontsize=14, fontweight="bold")

    short_names = ["base\npitch", "shoulder\nroll", "shoulder\nyaw",
                   "elbow\npitch", "wrist\npitch", "wrist\nyaw"]
    x = np.arange(6)
    rng = np.random.default_rng(42)

    for ax, mat, ylabel, title in [
        (axes[0], rmse_mat, "RMSE (mrad)", "per-joint RMSE（各 episode）"),
        (axes[1], max_mat,  "max |error| (mrad)", "per-joint 最大误差（各 episode）"),
    ]:
        # 箱线图
        bp = ax.boxplot(
            [mat[:, j] for j in range(6)],
            positions=x,
            widths=0.45,
            patch_artist=True,
            medianprops=dict(color=C_DESIRED, linewidth=2.2),
            boxprops=dict(facecolor="#D6EAF8", edgecolor=C_ACTUAL, linewidth=1.5),
            whiskerprops=dict(color=C_ACTUAL, linewidth=1.4),
            capprops=dict(color=C_ACTUAL, linewidth=1.8),
            flierprops=dict(marker="o", color=C_ERROR, markersize=5, alpha=0.7),
        )

        # 散点抖动（每个 episode 一个点）
        for j in range(6):
            jitter = rng.uniform(-0.18, 0.18, size=len(mat))
            ax.scatter(x[j] + jitter, mat[:, j],
                       color=C_ERROR, s=28, alpha=0.75, zorder=3,
                       edgecolors="white", linewidths=0.5)

        # 均值线
        means = mat.mean(axis=0)
        ax.plot(x, means, "D--", color="#1E8449", markersize=7,
                linewidth=1.5, label="均值", zorder=4)

        ax.set_xticks(x)
        ax.set_xticklabels(short_names, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(title, fontsize=11, fontweight="bold")
        ax.grid(axis="y", alpha=0.35, linewidth=0.7)
        ax.legend(fontsize=9)
        ax.set_xlim(-0.6, 5.6)

    plt.tight_layout()
    plt.savefig(OUT_DIR / "C_error_distribution.png",
                dpi=200, bbox_inches="tight")
    plt.close()
    print("  C 已保存: C_error_distribution.png")


# ─────────────────────────────────────────────────────────────────────────────
# 图D  误差时序热力图（每条 episode 逐帧绝对误差，6 关节并排）
# ─────────────────────────────────────────────────────────────────────────────
def plot_error_heatmap():
    ep_dirs = sorted(DATASET_DIR.glob("episode_*"))
    n = len(ep_dirs)

    fig, axes = plt.subplots(n, 6, figsize=(18, n * 1.4),
                             gridspec_kw={"hspace": 0.08, "wspace": 0.04})
    fig.suptitle("各 episode 逐帧绝对误差热力图（mrad）",
                 fontsize=13, fontweight="bold", y=1.01)

    # 全局色阶上限（95 百分位，避免极值压缩色阶）
    all_abs = []
    for ep_dir in ep_dirs:
        actual  = np.load(ep_dir / "right_actual_pos.npy")
        desired = np.load(ep_dir / "right_desired_pos.npy")
        all_abs.append(np.abs(desired - actual) * 1000)
    vmax = np.percentile(np.concatenate(all_abs, axis=0), 95)

    norm = Normalize(vmin=0, vmax=vmax)
    cmap = plt.cm.YlOrRd

    for row, ep_dir in enumerate(ep_dirs):
        actual  = np.load(ep_dir / "right_actual_pos.npy")
        desired = np.load(ep_dir / "right_desired_pos.npy")
        abs_err = np.abs(desired - actual) * 1000   # (T, 6)

        for col in range(6):
            ax = axes[row, col]
            # 每行是一条时间序列，reshape 成 (1, T) 显示为色带
            data = abs_err[:, col].reshape(1, -1)
            ax.imshow(data, aspect="auto", cmap=cmap, norm=norm,
                      interpolation="nearest")
            ax.set_yticks([])
            ax.set_xticks([])

            if row == 0:
                ax.set_title(JOINTS[col], fontsize=8.5, fontweight="bold")
            if col == 0:
                ax.set_ylabel(f"ep{row:02d}", fontsize=8, rotation=0,
                              labelpad=28, va="center")

    # 颜色条
    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=axes, orientation="vertical",
                        fraction=0.015, pad=0.01)
    cbar.set_label("|error| (mrad)", fontsize=10)

    plt.savefig(OUT_DIR / "D_error_heatmap.png",
                dpi=200, bbox_inches="tight")
    plt.close()
    print("  D 已保存: D_error_heatmap.png")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("生成数据集分析图...")
    plot_single_trajectory()
    plot_overlay_trajectory()
    plot_error_distribution()
    plot_error_heatmap()
    print(f"\n全部完成，图片保存在: {OUT_DIR.resolve()}")
