#!/usr/bin/env python3
"""
右臂关节轨迹误差图
2×3 布局，每个子图显示一个关节的 desired vs actual 轨迹
支持指定单条 episode，或叠加所有 episode
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# 中文字体配置
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DATASET_DIR = Path("data/processed/real_robot/training_episodes")
FIGURE_DIR = Path("figure/real_robot")

JOINT_NAMES = [
    "base_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_yaw",
]


def plot_single_episode(ep_dir: Path, save_path: Path):
    """绘制单条 episode 的轨迹误差图"""
    ts = np.load(ep_dir / "timestamps.npy")
    actual = np.load(ep_dir / "right_actual_pos.npy")
    desired = np.load(ep_dir / "right_desired_pos.npy")
    error = desired - actual  # 或直接用 right_error_pos.npy

    ep_name = ep_dir.name
    duration = ts[-1] - ts[0]
    n_frames = len(ts)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"右臂关节轨迹：期望 vs 实际  |  {ep_name}  "
        f"({n_frames} 帧, {duration:.1f}s)",
        fontsize=14,
        fontweight="bold",
    )

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    for i, joint in enumerate(JOINT_NAMES):
        ax = fig.add_subplot(gs[i // 3, i % 3])

        ax.plot(ts, desired[:, i], color="#E05C5C", linewidth=1.0,
                label="期望 (desired)", zorder=3)
        ax.plot(ts, actual[:, i], color="#4C8BF5", linewidth=1.0,
                alpha=0.85, label="实际 (actual)", zorder=2)

        # 误差填充
        ax.fill_between(ts, desired[:, i], actual[:, i],
                        alpha=0.18, color="#F5A623", label="误差区域")

        rmse = np.sqrt(np.mean(error[:, i] ** 2))
        max_err = np.abs(error[:, i]).max()

        ax.set_title(
            f"{joint}\n"
            f"RMSE={rmse*1000:.2f} mrad  |  max={max_err*1000:.2f} mrad",
            fontsize=9,
        )
        ax.set_xlabel("时间 (s)", fontsize=8)
        ax.set_ylabel("位置 (rad)", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.legend(fontsize=7, loc="upper right")

    save_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  已保存: {save_path}")


def plot_all_episodes_overlay(save_path: Path):
    """所有 episode 叠加，展示整体轨迹分布"""
    ep_dirs = sorted(DATASET_DIR.glob("episode_*"))
    if not ep_dirs:
        print("未找到任何 episode 数据")
        return

    colors_desired = "#C0392B"
    colors_actual = "#1A5CB0"

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"右臂关节轨迹（全部 {len(ep_dirs)} 条 episode 叠加）",
        fontsize=14,
        fontweight="bold",
    )

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)
    axes = [fig.add_subplot(gs[i // 3, i % 3]) for i in range(6)]

    all_rmse = np.zeros(6)
    all_max_err = np.zeros(6)
    total_frames = 0

    for ep_idx, ep_dir in enumerate(ep_dirs):
        ts = np.load(ep_dir / "timestamps.npy")
        actual = np.load(ep_dir / "right_actual_pos.npy")
        desired = np.load(ep_dir / "right_desired_pos.npy")
        error = desired - actual
        total_frames += len(ts)

        alpha = max(0.15, 0.6 / len(ep_dirs))

        for i in range(6):
            label_d = "期望" if ep_idx == 0 else None
            label_a = "实际" if ep_idx == 0 else None
            axes[i].plot(ts, desired[:, i], color=colors_desired,
                         linewidth=0.6, alpha=alpha, label=label_d)
            axes[i].plot(ts, actual[:, i], color=colors_actual,
                         linewidth=0.6, alpha=alpha, label=label_a)

            all_rmse[i] += np.mean(error[:, i] ** 2) * len(ts)
            all_max_err[i] = max(all_max_err[i], np.abs(error[:, i]).max())

    all_rmse = np.sqrt(all_rmse / total_frames)

    for i, joint in enumerate(JOINT_NAMES):
        axes[i].set_title(
            f"{joint}\n"
            f"RMSE={all_rmse[i]*1000:.2f} mrad  |  max={all_max_err[i]*1000:.2f} mrad",
            fontsize=9,
        )
        axes[i].set_xlabel("时间 (s)", fontsize=8)
        axes[i].set_ylabel("位置 (rad)", fontsize=8)
        axes[i].tick_params(labelsize=7)
        axes[i].grid(True, alpha=0.3, linewidth=0.5)
        axes[i].legend(fontsize=7, loc="upper right")

    save_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  已保存: {save_path}")


def main():
    parser = argparse.ArgumentParser(description="绘制右臂关节轨迹误差图")
    parser.add_argument(
        "--episode", "-e",
        type=str,
        default=None,
        help="指定 episode 名称，如 episode_000000；不指定则绘制所有 episode 叠加图",
    )
    parser.add_argument(
        "--all-separate", "-a",
        action="store_true",
        help="为每条 episode 单独生成一张图",
    )
    args = parser.parse_args()

    FIGURE_DIR.mkdir(parents=True, exist_ok=True)

    if args.all_separate:
        ep_dirs = sorted(DATASET_DIR.glob("episode_*"))
        print(f"为 {len(ep_dirs)} 条 episode 分别生成轨迹图...")
        for ep_dir in ep_dirs:
            save_path = FIGURE_DIR / f"trajectory_{ep_dir.name}.png"
            plot_single_episode(ep_dir, save_path)

    elif args.episode:
        ep_dir = DATASET_DIR / args.episode
        if not ep_dir.exists():
            print(f"ERROR: 找不到 {ep_dir}")
            return
        save_path = FIGURE_DIR / f"trajectory_{args.episode}.png"
        plot_single_episode(ep_dir, save_path)

    else:
        # 默认：画第一条 episode + 全部叠加图
        ep_dirs = sorted(DATASET_DIR.glob("episode_*"))
        if not ep_dirs:
            print("未找到任何 episode 数据")
            return

        print(f"绘制 episode_000000 单条轨迹图...")
        plot_single_episode(ep_dirs[0], FIGURE_DIR / "trajectory_episode_000000.png")

        print(f"绘制全部 {len(ep_dirs)} 条 episode 叠加图...")
        plot_all_episodes_overlay(FIGURE_DIR / "trajectory_all_episodes_overlay.png")

    print("完成！")


if __name__ == "__main__":
    main()
