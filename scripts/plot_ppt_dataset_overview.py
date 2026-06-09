#!/usr/bin/env python3
"""
答辩 PPT 数据集综合可视化
16:9 宽幅，单张图，包含：
  左上  数据集统计概览（关键数字）
  右上  各 episode 时长 & 帧数条形图
  中    episode_000000 右臂6关节轨迹（desired vs actual）
  左下  全局 per-joint RMSE 柱状图
  右下  误差分布箱线图
"""

from pathlib import Path
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

# ── 字体 ─────────────────────────────────────────────────────────────────────
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["pdf.fonttype"] = 42

DATASET_DIR = Path("data/processed/real_robot/training_episodes")
OUT_DIR     = Path("figure/real_robot")
OUT_DIR.mkdir(parents=True, exist_ok=True)

JOINTS_SHORT = ["base\npitch", "shoulder\nroll", "shoulder\nyaw",
                "elbow\npitch", "wrist\npitch", "wrist\nyaw"]
JOINTS_FULL  = ["base_pitch", "shoulder_roll", "shoulder_yaw",
                "elbow_pitch", "wrist_pitch", "wrist_yaw"]

# ── 配色（深色，适合投影）────────────────────────────────────────────────────
C_DESIRED = "#E74C3C"   # 鲜红
C_ACTUAL  = "#1A6FA8"   # 深蓝
C_FILL    = "#F5CBA7"   # 浅橙
C_BAR     = "#2471A3"   # 柱状图蓝
C_RMSE    = "#1E8449"   # 绿
C_MAX     = "#C0392B"   # 红
C_ACCENT  = "#D35400"   # 橙
C_BG      = "#F8F9FA"   # 浅灰背景
C_TITLE   = "#1B2631"   # 深色标题

# ─────────────────────────────────────────────────────────────────────────────
def load_all():
    eps = sorted(DATASET_DIR.glob("episode_*"))
    data = []
    for ep in eps:
        ts  = np.load(ep / "timestamps.npy")
        act = np.load(ep / "right_actual_pos.npy")
        des = np.load(ep / "right_desired_pos.npy")
        err = des - act
        data.append(dict(
            name=ep.name, ts=ts, actual=act, desired=des, error=err,
            dur=float(ts[-1]-ts[0]), frames=len(ts),
            rmse=np.sqrt(np.mean(err**2, axis=0)),
            maxe=np.abs(err).max(axis=0),
        ))
    with open(DATASET_DIR / "dataset_summary.json") as f:
        summary = json.load(f)
    return data, summary


def main():
    data, summary = load_all()
    n_ep = len(data)

    # ── 画布：16:9，300dpi ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(19.2, 10.8), facecolor="white")
    fig.patch.set_facecolor("white")

    # 总标题
    fig.text(0.5, 0.975,
             "真机模仿学习数据集可视化",
             ha="center", va="top",
             fontsize=22, fontweight="bold", color=C_TITLE)

    # ── GridSpec 布局 ─────────────────────────────────────────────────────────
    # 三行：
    #   row0 (h=2.2): 左=统计卡片  右=episode条形图
    #   row1 (h=3.8): 跨全宽 = 轨迹图（6关节）
    #   row2 (h=3.0): 左=RMSE柱状  右=误差箱线
    outer = gridspec.GridSpec(
        3, 1,
        figure=fig,
        top=0.94, bottom=0.06,
        left=0.04, right=0.97,
        hspace=0.42,
        height_ratios=[2.2, 3.8, 3.0],
    )

    # ── Row 0 ─────────────────────────────────────────────────────────────────
    gs0 = gridspec.GridSpecFromSubplotSpec(
        1, 2, subplot_spec=outer[0], wspace=0.06, width_ratios=[1, 2.2]
    )

    # ── 统计卡片（左上）──────────────────────────────────────────────────────
    ax_info = fig.add_subplot(gs0[0])
    ax_info.set_facecolor(C_BG)
    ax_info.axis("off")

    total_frames = sum(d["frames"] for d in data)
    total_dur    = sum(d["dur"] for d in data)
    avg_rate     = total_frames / total_dur
    total_samples = total_frames - n_ep * (2 + 16 - 1)   # obs+pred window

    stats = [
        ("示教轨迹数",    f"{n_ep} 条"),
        ("总帧数",        f"{total_frames:,} 帧"),
        ("总时长",        f"{total_dur:.0f} s  /  {total_dur/60:.1f} min"),
        ("平均采样率",    f"{avg_rate:.1f} Hz"),
        ("状态维度",      "12  (6 pos + 6 vel)"),
        ("动作维度",      "6  (desired pos)"),
        ("可用训练样本",  f"≈ {total_samples:,} 个"),
    ]

    # 绘制卡片
    card_colors = ["#D6EAF8", "#D5F5E3", "#FDEBD0", "#F9EBEA",
                   "#EBF5FB", "#F4ECF7", "#EAFAF1"]
    n_cards = len(stats)
    card_h  = 0.88 / n_cards
    for idx, ((key, val), cc) in enumerate(zip(stats, card_colors)):
        y = 0.96 - idx * (card_h + 0.015)
        rect = FancyBboxPatch((0.03, y - card_h), 0.94, card_h * 0.92,
                               boxstyle="round,pad=0.01",
                               facecolor=cc, edgecolor="none",
                               transform=ax_info.transAxes, clip_on=False)
        ax_info.add_patch(rect)
        ax_info.text(0.08, y - card_h * 0.48, key,
                     transform=ax_info.transAxes,
                     fontsize=9.5, fontweight="bold", color="#2C3E50",
                     va="center")
        ax_info.text(0.97, y - card_h * 0.48, val,
                     transform=ax_info.transAxes,
                     fontsize=9.5, color=C_BAR, va="center", ha="right")

    ax_info.set_title("数据集统计", fontsize=11, fontweight="bold",
                      color=C_TITLE, pad=6)

    # ── Episode 条形图（右上）────────────────────────────────────────────────
    ax_bar = fig.add_subplot(gs0[1])
    ax_bar.set_facecolor(C_BG)

    ep_labels = [f"ep{i:02d}" for i in range(n_ep)]
    frames_list = [d["frames"] for d in data]
    durs_list   = [d["dur"] for d in data]

    x = np.arange(n_ep)
    w = 0.38

    ax_bar2 = ax_bar.twinx()

    b1 = ax_bar.bar(x - w/2, frames_list, width=w,
                    color=C_BAR, alpha=0.85, label="帧数", zorder=3)
    b2 = ax_bar2.bar(x + w/2, durs_list, width=w,
                     color=C_ACCENT, alpha=0.85, label="时长 (s)", zorder=3)

    # 帧数标注
    for bar, v in zip(b1, frames_list):
        ax_bar.text(bar.get_x() + bar.get_width()/2,
                    bar.get_height() + 60,
                    str(v), ha="center", va="bottom",
                    fontsize=7.5, color=C_BAR, fontweight="bold")

    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(ep_labels, fontsize=9)
    ax_bar.set_ylabel("帧数", fontsize=10, color=C_BAR)
    ax_bar.tick_params(axis="y", labelcolor=C_BAR)
    ax_bar2.set_ylabel("时长 (s)", fontsize=10, color=C_ACCENT)
    ax_bar2.tick_params(axis="y", labelcolor=C_ACCENT)
    ax_bar.set_ylim(0, max(frames_list) * 1.22)
    ax_bar2.set_ylim(0, max(durs_list) * 1.22)
    ax_bar.grid(axis="y", alpha=0.3, linewidth=0.7, zorder=0)
    ax_bar.set_title("各 Episode 帧数与时长", fontsize=11,
                     fontweight="bold", color=C_TITLE, pad=6)

    lines = [mpatches.Patch(color=C_BAR, label="帧数"),
             mpatches.Patch(color=C_ACCENT, label="时长 (s)")]
    ax_bar.legend(handles=lines, fontsize=9, loc="upper right")

    # ── Row 1：轨迹图（全宽，6关节）─────────────────────────────────────────
    gs1 = gridspec.GridSpecFromSubplotSpec(
        2, 3, subplot_spec=outer[1], hspace=0.55, wspace=0.32
    )

    # 用 ep0（最长、最稳定）
    ep0 = data[0]
    t   = ep0["ts"] - ep0["ts"][0]

    for i in range(6):
        ax = fig.add_subplot(gs1[i // 3, i % 3])
        ax.set_facecolor(C_BG)

        ax.plot(t, ep0["desired"][:, i], color=C_DESIRED,
                linewidth=1.8, label="期望", zorder=3)
        ax.plot(t, ep0["actual"][:, i],  color=C_ACTUAL,
                linewidth=1.5, alpha=0.9, label="实际", zorder=2)
        ax.fill_between(t, ep0["desired"][:, i], ep0["actual"][:, i],
                        alpha=0.28, color=C_FILL, zorder=1)

        rmse = ep0["rmse"][i] * 1000
        maxe = ep0["maxe"][i] * 1000
        ax.set_title(
            f"{JOINTS_FULL[i]}   "
            f"RMSE={rmse:.2f} mrad  max={maxe:.1f} mrad",
            fontsize=9, fontweight="bold", color=C_TITLE,
        )
        ax.set_xlabel("时间 (s)", fontsize=8.5)
        ax.set_ylabel("位置 (rad)", fontsize=8.5)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.3, linewidth=0.6)
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2f"))
        if i == 0:
            ax.legend(fontsize=8.5, loc="upper right",
                      framealpha=0.85, edgecolor="none")

    # 行标题
    fig.text(0.505, outer[1].get_position(fig).y1 + 0.005,
             f"右臂关节轨迹（episode_000000，{ep0['frames']} 帧，{ep0['dur']:.0f} s）"
             "    ■ 期望  ■ 实际  ■ 误差区域",
             ha="center", va="bottom",
             fontsize=11, fontweight="bold", color=C_TITLE)

    # ── Row 2：RMSE + 箱线图 ─────────────────────────────────────────────────
    gs2 = gridspec.GridSpecFromSubplotSpec(
        1, 2, subplot_spec=outer[2], wspace=0.32
    )

    # ── 左：全局 per-joint RMSE 柱状图 ───────────────────────────────────────
    ax_rmse = fig.add_subplot(gs2[0])
    ax_rmse.set_facecolor(C_BG)

    global_rmse = np.sqrt(np.mean(
        np.array([d["rmse"] for d in data])**2, axis=0
    )) * 1000
    global_max = np.array([d["maxe"] for d in data]).max(axis=0) * 1000

    x6 = np.arange(6)
    w2 = 0.35
    ax_rmse.bar(x6 - w2/2, global_rmse, width=w2,
                color=C_RMSE, alpha=0.88, label="全局 RMSE (mrad)", zorder=3)
    ax_rmse.bar(x6 + w2/2, global_max,  width=w2,
                color=C_MAX,  alpha=0.75, label="全局 max |err| (mrad)", zorder=3)

    for j, (r, m) in enumerate(zip(global_rmse, global_max)):
        ax_rmse.text(j - w2/2, r + 0.3, f"{r:.1f}",
                     ha="center", va="bottom", fontsize=8, color=C_RMSE,
                     fontweight="bold")
        ax_rmse.text(j + w2/2, m + 0.3, f"{m:.0f}",
                     ha="center", va="bottom", fontsize=8, color=C_MAX,
                     fontweight="bold")

    ax_rmse.set_xticks(x6)
    ax_rmse.set_xticklabels(JOINTS_SHORT, fontsize=9)
    ax_rmse.set_ylabel("误差 (mrad)", fontsize=10)
    ax_rmse.set_title("全局 per-joint 跟踪误差",
                      fontsize=11, fontweight="bold", color=C_TITLE, pad=6)
    ax_rmse.legend(fontsize=9, loc="upper right")
    ax_rmse.grid(axis="y", alpha=0.3, linewidth=0.7, zorder=0)
    ax_rmse.set_ylim(0, global_max.max() * 1.25)

    # ── 右：per-episode RMSE 箱线图 ──────────────────────────────────────────
    ax_box = fig.add_subplot(gs2[1])
    ax_box.set_facecolor(C_BG)

    rmse_mat = np.array([d["rmse"] for d in data]) * 1000   # (10, 6)
    rng = np.random.default_rng(0)

    bp = ax_box.boxplot(
        [rmse_mat[:, j] for j in range(6)],
        positions=x6,
        widths=0.45,
        patch_artist=True,
        medianprops=dict(color=C_DESIRED, linewidth=2.5),
        boxprops=dict(facecolor="#D6EAF8", edgecolor=C_ACTUAL, linewidth=1.6),
        whiskerprops=dict(color=C_ACTUAL, linewidth=1.5, linestyle="--"),
        capprops=dict(color=C_ACTUAL, linewidth=2.0),
        flierprops=dict(marker="o", color=C_ACCENT,
                        markersize=6, alpha=0.8, markeredgewidth=0.5),
    )

    # 散点（每个 episode 一个点）
    for j in range(6):
        jitter = rng.uniform(-0.18, 0.18, size=len(rmse_mat))
        ax_box.scatter(x6[j] + jitter, rmse_mat[:, j],
                       color=C_ACCENT, s=32, alpha=0.8, zorder=4,
                       edgecolors="white", linewidths=0.6)

    # 均值
    ax_box.plot(x6, rmse_mat.mean(axis=0), "D--",
                color="#1E8449", markersize=8, linewidth=1.8,
                label="均值", zorder=5)

    ax_box.set_xticks(x6)
    ax_box.set_xticklabels(JOINTS_SHORT, fontsize=9)
    ax_box.set_ylabel("RMSE (mrad)", fontsize=10)
    ax_box.set_title("各 Episode per-joint RMSE 分布",
                     fontsize=11, fontweight="bold", color=C_TITLE, pad=6)
    ax_box.legend(fontsize=9, loc="upper right")
    ax_box.grid(axis="y", alpha=0.3, linewidth=0.7, zorder=0)

    # ── 保存 ─────────────────────────────────────────────────────────────────
    out = OUT_DIR / "ppt_dataset_overview.png"
    plt.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"已保存: {out}")


if __name__ == "__main__":
    main()
