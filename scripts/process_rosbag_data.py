#!/usr/bin/env python3
"""
ROS bag 数据处理脚本
功能：
1. 从 rosbag2 (sqlite3) 中提取关节数据
2. 数据统计（时长、采样率、关节范围等）
3. 数据可视化（关节轨迹、速度、跟踪误差）
4. 误差分析（desired vs actual）
5. 导出为训练格式（npy）

使用方法：
    conda activate train-gpu
    python scripts/process_rosbag_data.py
"""

import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

# 中文字体配置（Windows 优先 Microsoft YaHei，备选 SimHei）
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False  # 负号正常显示

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


# ============================================================
# 配置
# ============================================================
BAG_DIR = Path("data/rosbag")
OUTPUT_DIR = Path("data/processed/real_robot")
FIGURE_DIR = Path("figure/real_robot")

RIGHT_ARM_TOPIC = "/right_arm_controller/state"
LEFT_ARM_TOPIC = "/left_arm_controller/state"

RIGHT_JOINT_NAMES = [
    "right_base_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
LEFT_JOINT_NAMES = [
    "left_base_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]


# ============================================================
# 注册消息类型
# ============================================================
def get_configured_typestore():
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    JOINT_TRAJECTORY_CONTROLLER_STATE_MSG = """
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint reference
trajectory_msgs/JointTrajectoryPoint feedback
trajectory_msgs/JointTrajectoryPoint error
trajectory_msgs/JointTrajectoryPoint output
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
string[] multi_dof_joint_names
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_reference
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_feedback
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_error
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_output
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_desired
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_actual
"""
    add_types = get_types_from_msg(
        JOINT_TRAJECTORY_CONTROLLER_STATE_MSG,
        "control_msgs/msg/JointTrajectoryControllerState",
    )
    typestore.register(add_types)
    return typestore


# ============================================================
# 数据提取
# ============================================================
def extract_bag_data(bag_path: Path, typestore) -> Dict[str, Dict]:
    """从单个 rosbag 中提取左右臂的关节数据"""
    data = {
        "right": {
            "timestamps": [],
            "actual_pos": [],
            "actual_vel": [],
            "desired_pos": [],
            "desired_vel": [],
            "error_pos": [],
        },
        "left": {
            "timestamps": [],
            "actual_pos": [],
            "actual_vel": [],
            "desired_pos": [],
            "desired_vel": [],
            "error_pos": [],
        },
    }

    with Reader(bag_path) as reader:
        for conn, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            ts = timestamp * 1e-9  # nanoseconds -> seconds

            if conn.topic == RIGHT_ARM_TOPIC:
                arm = "right"
            elif conn.topic == LEFT_ARM_TOPIC:
                arm = "left"
            else:
                continue

            data[arm]["timestamps"].append(ts)
            data[arm]["actual_pos"].append(np.array(msg.actual.positions, dtype=np.float64))
            data[arm]["actual_vel"].append(
                np.array(msg.actual.velocities, dtype=np.float64)
                if len(msg.actual.velocities) > 0
                else np.zeros(6, dtype=np.float64)
            )
            data[arm]["desired_pos"].append(np.array(msg.desired.positions, dtype=np.float64))
            data[arm]["desired_vel"].append(
                np.array(msg.desired.velocities, dtype=np.float64)
                if len(msg.desired.velocities) > 0
                else np.zeros(6, dtype=np.float64)
            )
            data[arm]["error_pos"].append(np.array(msg.error.positions, dtype=np.float64))

    # Convert to numpy arrays
    for arm in ["right", "left"]:
        for key in data[arm]:
            data[arm][key] = np.array(data[arm][key])
        # Normalize timestamps to start from 0
        if len(data[arm]["timestamps"]) > 0:
            data[arm]["timestamps"] -= data[arm]["timestamps"][0]

    return data


def extract_all_bags(bag_dir: Path, typestore) -> List[Dict]:
    """提取所有 rosbag 数据"""
    bag_dirs = sorted([d for d in bag_dir.iterdir() if d.is_dir() and d.name.startswith("rosbag2_")])
    all_episodes = []

    print(f"找到 {len(bag_dirs)} 个 rosbag 文件")
    print("-" * 60)

    for i, bag_path in enumerate(bag_dirs):
        print(f"  [{i+1}/{len(bag_dirs)}] 处理: {bag_path.name} ...", end=" ")
        try:
            data = extract_bag_data(bag_path, typestore)
            data["bag_name"] = bag_path.name
            all_episodes.append(data)
            n_right = len(data["right"]["timestamps"])
            n_left = len(data["left"]["timestamps"])
            duration = max(
                data["right"]["timestamps"][-1] if n_right > 0 else 0,
                data["left"]["timestamps"][-1] if n_left > 0 else 0,
            )
            print(f"OK (右臂 {n_right} 帧, 左臂 {n_left} 帧, {duration:.1f}s)")
        except Exception as e:
            print(f"FAILED: {e}")

    return all_episodes


# ============================================================
# 数据统计
# ============================================================
def print_statistics(episodes: List[Dict]):
    """打印数据统计信息"""
    print("\n" + "=" * 60)
    print("数据统计")
    print("=" * 60)

    total_frames_right = 0
    total_frames_left = 0
    durations = []

    for ep in episodes:
        n_right = len(ep["right"]["timestamps"])
        n_left = len(ep["left"]["timestamps"])
        total_frames_right += n_right
        total_frames_left += n_left
        duration = max(
            ep["right"]["timestamps"][-1] if n_right > 0 else 0,
            ep["left"]["timestamps"][-1] if n_left > 0 else 0,
        )
        durations.append(duration)

    print(f"\n总 episode 数: {len(episodes)}")
    print(f"总帧数 (右臂): {total_frames_right}")
    print(f"总帧数 (左臂): {total_frames_left}")
    print(f"总时长: {sum(durations):.1f}s ({sum(durations)/60:.1f}min)")
    print(f"单条时长: min={min(durations):.1f}s, max={max(durations):.1f}s, mean={np.mean(durations):.1f}s")

    # 采样率
    rates = []
    for ep in episodes:
        ts = ep["right"]["timestamps"]
        if len(ts) > 1:
            dt = np.diff(ts)
            rates.append(1.0 / np.mean(dt))
    print(f"平均采样率 (右臂): {np.mean(rates):.1f} Hz")

    # 关节范围
    print("\n--- 右臂关节位置范围 (rad) ---")
    all_pos = np.vstack([ep["right"]["actual_pos"] for ep in episodes if len(ep["right"]["actual_pos"]) > 0])
    for i, name in enumerate(RIGHT_JOINT_NAMES):
        col = all_pos[:, i]
        print(f"  {name:35s}: [{col.min():+.4f}, {col.max():+.4f}]  range={col.max()-col.min():.4f}  std={col.std():.4f}")

    print("\n--- 左臂关节位置范围 (rad) ---")
    all_pos_left = np.vstack([ep["left"]["actual_pos"] for ep in episodes if len(ep["left"]["actual_pos"]) > 0])
    for i, name in enumerate(LEFT_JOINT_NAMES):
        col = all_pos_left[:, i]
        print(f"  {name:35s}: [{col.min():+.4f}, {col.max():+.4f}]  range={col.max()-col.min():.4f}  std={col.std():.4f}")

    # 跟踪误差
    print("\n--- 右臂跟踪误差 (desired - actual) ---")
    all_err = np.vstack([ep["right"]["error_pos"] for ep in episodes if len(ep["right"]["error_pos"]) > 0])
    for i, name in enumerate(RIGHT_JOINT_NAMES):
        col = all_err[:, i]
        print(f"  {name:35s}: mean={col.mean():+.6f}  std={col.std():.6f}  max_abs={np.abs(col).max():.6f}")

    print("\n--- 左臂跟踪误差 (desired - actual) ---")
    all_err_left = np.vstack([ep["left"]["error_pos"] for ep in episodes if len(ep["left"]["error_pos"]) > 0])
    for i, name in enumerate(LEFT_JOINT_NAMES):
        col = all_err_left[:, i]
        print(f"  {name:35s}: mean={col.mean():+.6f}  std={col.std():.6f}  max_abs={np.abs(col).max():.6f}")

    return all_pos, all_pos_left, all_err, all_err_left


# ============================================================
# 数据可视化
# ============================================================
def plot_episode_trajectories(episodes: List[Dict], save_dir: Path):
    """为每个 episode 绘制关节轨迹"""
    save_dir.mkdir(parents=True, exist_ok=True)

    with PdfPages(save_dir / "all_episodes_trajectories.pdf") as pdf:
        for ep_idx, ep in enumerate(episodes):
            fig, axes = plt.subplots(3, 2, figsize=(14, 10))
            fig.suptitle(f"Episode {ep_idx}: {ep['bag_name']} - 右臂关节轨迹", fontsize=12)

            ts = ep["right"]["timestamps"]
            actual = ep["right"]["actual_pos"]
            desired = ep["right"]["desired_pos"]

            for i in range(6):
                ax = axes[i // 2, i % 2]
                ax.plot(ts, actual[:, i], "b-", linewidth=0.8, label="actual")
                ax.plot(ts, desired[:, i], "r--", linewidth=0.8, label="desired")
                ax.set_ylabel("rad")
                ax.set_title(RIGHT_JOINT_NAMES[i], fontsize=9)
                ax.legend(fontsize=7)
                ax.grid(True, alpha=0.3)

            axes[-1, 0].set_xlabel("Time (s)")
            axes[-1, 1].set_xlabel("Time (s)")
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)

    print(f"  轨迹图已保存: {save_dir / 'all_episodes_trajectories.pdf'}")


def plot_error_analysis(episodes: List[Dict], save_dir: Path):
    """跟踪误差分析图"""
    save_dir.mkdir(parents=True, exist_ok=True)

    # 右臂误差时间序列（所有 episode 叠加）
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("右臂跟踪误差 (所有 episode 叠加)", fontsize=12)

    colors = plt.cm.tab10(np.linspace(0, 1, len(episodes)))

    for ep_idx, ep in enumerate(episodes):
        ts = ep["right"]["timestamps"]
        err = ep["right"]["error_pos"]
        if len(ts) == 0:
            continue
        for i in range(6):
            ax = axes[i // 2, i % 2]
            ax.plot(ts, err[:, i], linewidth=0.5, alpha=0.6, color=colors[ep_idx])

    for i in range(6):
        ax = axes[i // 2, i % 2]
        ax.set_title(RIGHT_JOINT_NAMES[i], fontsize=9)
        ax.set_ylabel("Error (rad)")
        ax.axhline(0, color="k", linewidth=0.5)
        ax.grid(True, alpha=0.3)

    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(save_dir / "right_arm_tracking_error.png", dpi=150)
    plt.close()

    # 误差分布直方图
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("右臂跟踪误差分布", fontsize=12)

    all_err = np.vstack([ep["right"]["error_pos"] for ep in episodes if len(ep["right"]["error_pos"]) > 0])
    for i in range(6):
        ax = axes[i // 2, i % 2]
        ax.hist(all_err[:, i], bins=100, density=True, alpha=0.7, color="steelblue")
        ax.axvline(0, color="r", linewidth=1)
        mean_err = all_err[:, i].mean()
        std_err = all_err[:, i].std()
        ax.axvline(mean_err, color="orange", linewidth=1, linestyle="--", label=f"mean={mean_err:.5f}")
        ax.set_title(f"{RIGHT_JOINT_NAMES[i]} (std={std_err:.5f})", fontsize=9)
        ax.set_xlabel("Error (rad)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_dir / "right_arm_error_distribution.png", dpi=150)
    plt.close()

    print(f"  误差分析图已保存: {save_dir}")


def plot_joint_ranges(all_pos: np.ndarray, joint_names: List[str], title: str, save_path: Path):
    """关节范围箱线图"""
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.boxplot(all_pos, labels=[n.replace("_joint", "").replace("right_", "R_").replace("left_", "L_") for n in joint_names])
    ax.set_title(title)
    ax.set_ylabel("Position (rad)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()
    print(f"  关节范围图已保存: {save_path}")


def plot_velocity_profiles(episodes: List[Dict], save_dir: Path):
    """速度曲线"""
    save_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("右臂关节速度 (数值微分)", fontsize=12)

    colors = plt.cm.tab10(np.linspace(0, 1, len(episodes)))

    for ep_idx, ep in enumerate(episodes):
        ts = ep["right"]["timestamps"]
        pos = ep["right"]["actual_pos"]
        if len(ts) < 2:
            continue
        dt = np.diff(ts)
        vel = np.diff(pos, axis=0) / dt[:, None]

        for i in range(6):
            ax = axes[i // 2, i % 2]
            ax.plot(ts[1:], vel[:, i], linewidth=0.5, alpha=0.6, color=colors[ep_idx])

    for i in range(6):
        ax = axes[i // 2, i % 2]
        ax.set_title(RIGHT_JOINT_NAMES[i], fontsize=9)
        ax.set_ylabel("Velocity (rad/s)")
        ax.axhline(0, color="k", linewidth=0.5)
        ax.grid(True, alpha=0.3)

    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(save_dir / "right_arm_velocity.png", dpi=150)
    plt.close()
    print(f"  速度图已保存: {save_dir / 'right_arm_velocity.png'}")


# ============================================================
# 导出训练数据
# ============================================================
def export_training_data(episodes: List[Dict], output_dir: Path):
    """
    导出为训练格式：
    每个 episode 一个文件夹，包含:
    - robot_state.npy: 右臂 actual position (6) + actual velocity (6) = 12 维
    - action.npy: 右臂 desired position (6) = 6 维 (作为目标动作)
    - timestamps.npy
    - meta.json
    """
    import json

    train_dir = output_dir / "training_episodes"
    train_dir.mkdir(parents=True, exist_ok=True)

    episode_info = []

    for ep_idx, ep in enumerate(episodes):
        ep_dir = train_dir / f"episode_{ep_idx:06d}"
        ep_dir.mkdir(exist_ok=True)

        ts = ep["right"]["timestamps"]
        actual_pos = ep["right"]["actual_pos"]
        actual_vel = ep["right"]["actual_vel"]
        desired_pos = ep["right"]["desired_pos"]

        if len(ts) == 0:
            continue

        # robot_state: [actual_pos(6), actual_vel(6)] = 12 维
        robot_state = np.hstack([actual_pos, actual_vel]).astype(np.float32)

        # action: desired_pos(6) = 6 维
        action = desired_pos.astype(np.float32)

        # 保存
        np.save(ep_dir / "robot_state.npy", robot_state)
        np.save(ep_dir / "action.npy", action)
        np.save(ep_dir / "timestamps.npy", ts.astype(np.float64))

        # 同时保存完整数据（含左臂）
        np.save(ep_dir / "right_actual_pos.npy", actual_pos.astype(np.float32))
        np.save(ep_dir / "right_desired_pos.npy", desired_pos.astype(np.float32))
        np.save(ep_dir / "right_error_pos.npy", ep["right"]["error_pos"].astype(np.float32))

        if len(ep["left"]["timestamps"]) > 0:
            np.save(ep_dir / "left_actual_pos.npy", ep["left"]["actual_pos"].astype(np.float32))
            np.save(ep_dir / "left_desired_pos.npy", ep["left"]["desired_pos"].astype(np.float32))

        duration = float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        meta = {
            "source_bag": ep["bag_name"],
            "episode_index": ep_idx,
            "num_samples": len(ts),
            "duration_sec": duration,
            "sample_rate_hz": float(len(ts) / duration) if duration > 0 else 0.0,
            "robot_state_dim": robot_state.shape[1],
            "action_dim": action.shape[1],
            "robot_state_names": [f"{n}:position" for n in RIGHT_JOINT_NAMES]
            + [f"{n}:velocity" for n in RIGHT_JOINT_NAMES],
            "action_names": [f"{n}:desired_position" for n in RIGHT_JOINT_NAMES],
            "right_joint_names": RIGHT_JOINT_NAMES,
        }
        with open(ep_dir / "meta.json", "w") as f:
            json.dump(meta, f, indent=2)

        episode_info.append(meta)

    # 保存总体信息
    summary = {
        "num_episodes": len(episode_info),
        "total_samples": sum(e["num_samples"] for e in episode_info),
        "total_duration_sec": sum(e["duration_sec"] for e in episode_info),
        "robot_state_dim": 12,
        "action_dim": 6,
        "episodes": episode_info,
    }
    with open(train_dir / "dataset_summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n训练数据已导出到: {train_dir}")
    print(f"  Episodes: {len(episode_info)}")
    print(f"  总帧数: {summary['total_samples']}")
    print(f"  robot_state 维度: 12 (6 pos + 6 vel)")
    print(f"  action 维度: 6 (desired position)")

    return train_dir


# ============================================================
# 采样率分析
# ============================================================
def plot_sampling_analysis(episodes: List[Dict], save_dir: Path):
    """采样间隔分析"""
    save_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    all_dt = []
    for ep in episodes:
        ts = ep["right"]["timestamps"]
        if len(ts) > 1:
            dt = np.diff(ts) * 1000  # ms
            all_dt.extend(dt.tolist())

    all_dt = np.array(all_dt)

    axes[0].hist(all_dt, bins=100, density=True, alpha=0.7, color="steelblue")
    axes[0].axvline(np.mean(all_dt), color="r", linestyle="--", label=f"mean={np.mean(all_dt):.2f}ms")
    axes[0].axvline(np.median(all_dt), color="orange", linestyle="--", label=f"median={np.median(all_dt):.2f}ms")
    axes[0].set_xlabel("采样间隔 (ms)")
    axes[0].set_ylabel("Density")
    axes[0].set_title("采样间隔分布")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 采样率随时间变化
    for ep_idx, ep in enumerate(episodes):
        ts = ep["right"]["timestamps"]
        if len(ts) > 10:
            dt = np.diff(ts)
            rate = 1.0 / dt
            axes[1].plot(ts[1:], rate, linewidth=0.5, alpha=0.6, label=f"ep{ep_idx}" if ep_idx < 5 else None)

    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Hz")
    axes[1].set_title("采样率随时间变化")
    axes[1].axhline(48, color="r", linestyle="--", alpha=0.5, label="48 Hz")
    axes[1].legend(fontsize=7)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_dir / "sampling_analysis.png", dpi=150)
    plt.close()
    print(f"  采样分析图已保存: {save_dir / 'sampling_analysis.png'}")


# ============================================================
# 主函数
# ============================================================
def main():
    print("=" * 60)
    print("ROS Bag 数据处理与分析")
    print("=" * 60)

    # 1. 初始化
    typestore = get_configured_typestore()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)

    # 2. 提取数据
    print("\n[1/5] 提取 rosbag 数据...")
    episodes = extract_all_bags(BAG_DIR, typestore)

    if not episodes:
        print("ERROR: 没有成功提取任何数据!")
        sys.exit(1)

    # 3. 数据统计
    print("\n[2/5] 数据统计...")
    all_pos_right, all_pos_left, all_err_right, all_err_left = print_statistics(episodes)

    # 4. 可视化
    print("\n[3/5] 生成可视化图表...")
    plot_episode_trajectories(episodes, FIGURE_DIR)
    plot_error_analysis(episodes, FIGURE_DIR)
    plot_joint_ranges(all_pos_right, RIGHT_JOINT_NAMES, "右臂关节位置分布", FIGURE_DIR / "right_arm_joint_ranges.png")
    plot_joint_ranges(all_pos_left, LEFT_JOINT_NAMES, "左臂关节位置分布", FIGURE_DIR / "left_arm_joint_ranges.png")
    plot_velocity_profiles(episodes, FIGURE_DIR)
    plot_sampling_analysis(episodes, FIGURE_DIR)

    # 5. 导出训练数据
    print("\n[4/5] 导出训练数据...")
    train_dir = export_training_data(episodes, OUTPUT_DIR)

    # 6. 总结
    print("\n[5/5] 完成!")
    print("=" * 60)
    print(f"输出目录: {OUTPUT_DIR.resolve()}")
    print(f"  figures/          - 可视化图表")
    print(f"  training_episodes/ - 训练数据 (npy)")
    print("=" * 60)


if __name__ == "__main__":
    main()
