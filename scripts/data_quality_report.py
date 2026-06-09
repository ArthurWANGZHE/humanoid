#!/usr/bin/env python3
"""
数据质量报告 - 补充统计
包括：关节间相关性、动作平滑度、episode 一致性、训练可用性检查
"""

import json
from pathlib import Path

import numpy as np

DATA_DIR = Path("data/processed/real_robot/training_episodes")


def load_all_episodes():
    episodes = []
    for ep_dir in sorted(DATA_DIR.glob("episode_*")):
        state = np.load(ep_dir / "robot_state.npy")
        action = np.load(ep_dir / "action.npy")
        ts = np.load(ep_dir / "timestamps.npy")
        episodes.append({"state": state, "action": action, "timestamps": ts, "name": ep_dir.name})
    return episodes


def main():
    episodes = load_all_episodes()
    print("=" * 60)
    print("数据质量补充报告")
    print("=" * 60)

    # 1. 动作平滑度分析
    print("\n--- 动作平滑度 (相邻帧 action 差值) ---")
    all_action_diffs = []
    for ep in episodes:
        diff = np.diff(ep["action"], axis=0)
        all_action_diffs.append(diff)
    all_diffs = np.vstack(all_action_diffs)
    print(f"  action delta mean abs: {np.abs(all_diffs).mean(axis=0)}")
    print(f"  action delta max abs:  {np.abs(all_diffs).max(axis=0)}")
    print(f"  action delta std:      {all_diffs.std(axis=0)}")

    # 2. 关节间相关性
    print("\n--- 关节位置相关性矩阵 (右臂 6 关节) ---")
    all_pos = np.vstack([ep["state"][:, :6] for ep in episodes])
    corr = np.corrcoef(all_pos.T)
    joint_short = ["base_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "wrist_pitch", "wrist_yaw"]
    print(f"  {'':18s}", end="")
    for name in joint_short:
        print(f"{name[:8]:>9s}", end="")
    print()
    for i, name in enumerate(joint_short):
        print(f"  {name:18s}", end="")
        for j in range(6):
            print(f"{corr[i, j]:+.4f}  ", end="")
        print()

    # 3. 采样间隔一致性
    print("\n--- 采样间隔一致性 ---")
    for ep in episodes:
        dt = np.diff(ep["timestamps"])
        jitter = dt.std() / dt.mean() * 100
        print(f"  {ep['name']}: mean_dt={dt.mean()*1000:.2f}ms, std={dt.std()*1000:.2f}ms, jitter={jitter:.1f}%")

    # 4. 数据范围 (用于归一化参考)
    print("\n--- 数据范围 (训练归一化参考) ---")
    all_states = np.vstack([ep["state"] for ep in episodes])
    all_actions = np.vstack([ep["action"] for ep in episodes])

    print("  robot_state (12 dim):")
    print(f"    min:  {all_states.min(axis=0)}")
    print(f"    max:  {all_states.max(axis=0)}")
    print(f"    mean: {all_states.mean(axis=0)}")
    print(f"    std:  {all_states.std(axis=0)}")

    print("  action (6 dim):")
    print(f"    min:  {all_actions.min(axis=0)}")
    print(f"    max:  {all_actions.max(axis=0)}")
    print(f"    mean: {all_actions.mean(axis=0)}")
    print(f"    std:  {all_actions.std(axis=0)}")

    # 5. 训练可用性检查
    print("\n--- 训练可用性检查 ---")
    obs_horizon = 2
    pred_horizon = 16
    min_len = obs_horizon + pred_horizon - 1
    total_samples = 0
    for ep in episodes:
        n = ep["state"].shape[0]
        usable = max(0, n - min_len + 1)
        total_samples += usable
        status = "OK" if usable > 0 else "TOO SHORT"
        print(f"  {ep['name']}: {n} frames -> {usable} training samples [{status}]")

    print(f"\n  总可用训练样本数: {total_samples}")
    print(f"  obs_horizon={obs_horizon}, pred_horizon={pred_horizon}")
    print(f"  每个样本: 输入 state (2, 12), 输出 action (16, 6)")

    # 6. NaN/Inf 检查
    print("\n--- NaN/Inf 检查 ---")
    has_issue = False
    for ep in episodes:
        nan_state = np.isnan(ep["state"]).sum()
        nan_action = np.isnan(ep["action"]).sum()
        inf_state = np.isinf(ep["state"]).sum()
        inf_action = np.isinf(ep["action"]).sum()
        if nan_state + nan_action + inf_state + inf_action > 0:
            print(f"  {ep['name']}: NaN(state={nan_state}, action={nan_action}), Inf(state={inf_state}, action={inf_action})")
            has_issue = True
    if not has_issue:
        print("  All clean - no NaN or Inf values found.")

    # 7. 运动幅度统计
    print("\n--- 每条 episode 运动幅度 (关节位置 max-min) ---")
    for ep in episodes:
        pos = ep["state"][:, :6]
        motion_range = pos.max(axis=0) - pos.min(axis=0)
        total_motion = motion_range.sum()
        print(f"  {ep['name']}: total_range={total_motion:.3f} rad, per_joint={motion_range}")

    print("\n" + "=" * 60)
    print("报告完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
