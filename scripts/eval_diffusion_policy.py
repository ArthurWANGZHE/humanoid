#!/usr/bin/env python3
"""
Diffusion Policy 离线评估
- 从训练数据中采样，用模型预测 action 序列
- 对比预测 vs 真实 action
- 计算 MSE、关节误差
- 可视化预测轨迹
"""

import json
import sys
from pathlib import Path

import numpy as np
import torch
import matplotlib.pyplot as plt

# 中文字体配置（Windows 优先 Microsoft YaHei，备选 SimHei）
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False  # 负号正常显示

sys.path.insert(0, str(Path("RL_training/Imitation_Learning")))
from models.diffusion_policy import DiffusionPolicy
from models.imitation_dataset import ImitationDataset


CHECKPOINT_PATH = Path("data/checkpoints/diffusion_policy/latest.pt")
DATASET_DIR = Path("data/processed/real_robot/training_episodes")
FIGURE_DIR = Path("figure/real_robot")


def make_schedule(timesteps: int, device: torch.device):
    betas = torch.linspace(1e-4, 0.02, timesteps, device=device)
    alphas = 1.0 - betas
    alphas_cumprod = torch.cumprod(alphas, dim=0)
    return betas, alphas, alphas_cumprod


@torch.no_grad()
def ddpm_sample(model, cond, config, device):
    """DDPM reverse sampling"""
    timesteps = config["timesteps"]
    pred_horizon = config["pred_horizon"]
    action_dim = config["action_dim"]
    batch_size = cond.shape[0]

    betas, alphas, alphas_cumprod = make_schedule(timesteps, device)

    # Start from pure noise
    x = torch.randn(batch_size, pred_horizon, action_dim, device=device)

    for t_idx in reversed(range(timesteps)):
        t = torch.full((batch_size,), t_idx, device=device, dtype=torch.long)
        pred_noise = model(x, cond, t)

        alpha = alphas[t_idx]
        alpha_bar = alphas_cumprod[t_idx]
        beta = betas[t_idx]

        # DDPM update
        x = (1.0 / torch.sqrt(alpha)) * (
            x - (beta / torch.sqrt(1.0 - alpha_bar)) * pred_noise
        )

        if t_idx > 0:
            noise = torch.randn_like(x)
            x = x + torch.sqrt(beta) * noise

    return x


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Load checkpoint
    ckpt = torch.load(CHECKPOINT_PATH, map_location=device, weights_only=False)
    config = ckpt["config"]
    stats = ckpt["stats"]

    print(f"Model config: state_dim={config['state_dim']}, action_dim={config['action_dim']}")
    print(f"  obs_horizon={config['obs_horizon']}, pred_horizon={config['pred_horizon']}")
    print(f"  timesteps={config['timesteps']}")

    # Load model
    model = DiffusionPolicy(
        state_dim=config["state_dim"],
        action_dim=config["action_dim"],
        obs_horizon=config["obs_horizon"],
        pred_horizon=config["pred_horizon"],
    ).to(device)
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()

    # Load dataset
    dataset = ImitationDataset(
        DATASET_DIR,
        obs_horizon=config["obs_horizon"],
        pred_horizon=config["pred_horizon"],
        processed_dir=Path("data/processed/real_robot/stats"),
    )

    # Sample evaluation indices
    np.random.seed(42)
    n_eval = min(500, len(dataset))
    eval_indices = np.random.choice(len(dataset), n_eval, replace=False)

    # Evaluate
    all_pred_actions = []
    all_true_actions = []

    for idx in eval_indices:
        sample = dataset[idx]
        cond = sample["robot_state"].unsqueeze(0).to(device)  # (1, obs_horizon, state_dim)
        true_action = sample["action"].numpy()  # (pred_horizon, action_dim)

        pred_action = ddpm_sample(model, cond, config, device)
        pred_action = pred_action.squeeze(0).cpu().numpy()  # (pred_horizon, action_dim)

        all_pred_actions.append(pred_action)
        all_true_actions.append(true_action)

    all_pred = np.array(all_pred_actions)  # (n_eval, pred_horizon, action_dim)
    all_true = np.array(all_true_actions)

    # Denormalize for interpretable metrics
    action_mean = np.array(stats["action_mean"], dtype=np.float32)
    action_std = np.array(stats["action_std"], dtype=np.float32)

    all_pred_denorm = all_pred * action_std + action_mean
    all_true_denorm = all_true * action_std + action_mean

    # Metrics
    mse_normalized = ((all_pred - all_true) ** 2).mean()
    mse_denorm = ((all_pred_denorm - all_true_denorm) ** 2).mean()
    mae_denorm = np.abs(all_pred_denorm - all_true_denorm).mean()

    print(f"\n{'='*60}")
    print("Evaluation Results")
    print(f"{'='*60}")
    print(f"  Samples evaluated: {n_eval}")
    print(f"  MSE (normalized):  {mse_normalized:.6f}")
    print(f"  MSE (rad):         {mse_denorm:.6f}")
    print(f"  MAE (rad):         {mae_denorm:.6f}")
    print(f"  RMSE (rad):        {np.sqrt(mse_denorm):.6f}")

    # Per-joint error
    joint_names = ["base_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "wrist_pitch", "wrist_yaw"]
    print(f"\n  Per-joint MAE (rad):")
    per_joint_mae = np.abs(all_pred_denorm - all_true_denorm).mean(axis=(0, 1))
    for i, name in enumerate(joint_names):
        print(f"    {name:20s}: {per_joint_mae[i]:.6f} rad ({np.degrees(per_joint_mae[i]):.3f} deg)")

    # Per-horizon-step error
    print(f"\n  Per-step MAE (averaged over joints, in rad):")
    per_step_mae = np.abs(all_pred_denorm - all_true_denorm).mean(axis=(0, 2))
    for step in range(config["pred_horizon"]):
        bar = "#" * int(per_step_mae[step] * 500)
        print(f"    step {step:2d}: {per_step_mae[step]:.6f}  {bar}")

    # Visualization
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)

    # Plot: predicted vs true for a few samples
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Diffusion Policy: Predicted vs True Action (sample 0)", fontsize=12)

    sample_idx = 0
    pred_sample = all_pred_denorm[sample_idx]  # (16, 6)
    true_sample = all_true_denorm[sample_idx]

    for i in range(6):
        ax = axes[i // 2, i % 2]
        steps = np.arange(config["pred_horizon"])
        ax.plot(steps, true_sample[:, i], "b-o", markersize=3, label="true")
        ax.plot(steps, pred_sample[:, i], "r--x", markersize=3, label="predicted")
        ax.set_title(joint_names[i])
        ax.set_xlabel("Prediction step")
        ax.set_ylabel("Position (rad)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "eval_pred_vs_true_sample.png", dpi=150)
    plt.close()

    # Plot: error distribution
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Prediction Error Distribution (all eval samples)", fontsize=12)

    errors = (all_pred_denorm - all_true_denorm).reshape(-1, 6)
    for i in range(6):
        ax = axes[i // 2, i % 2]
        ax.hist(errors[:, i], bins=80, density=True, alpha=0.7, color="steelblue")
        ax.axvline(0, color="r", linewidth=1)
        ax.set_title(f"{joint_names[i]} (std={errors[:, i].std():.5f})")
        ax.set_xlabel("Error (rad)")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "eval_error_distribution.png", dpi=150)
    plt.close()

    # Plot: error vs prediction horizon
    fig, ax = plt.subplots(figsize=(10, 5))
    for i in range(6):
        per_step_joint = np.abs(all_pred_denorm - all_true_denorm)[:, :, i].mean(axis=0)
        ax.plot(range(config["pred_horizon"]), per_step_joint, "-o", markersize=4, label=joint_names[i])

    ax.set_xlabel("Prediction Horizon Step")
    ax.set_ylabel("MAE (rad)")
    ax.set_title("Prediction Error vs Horizon Step")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "eval_error_vs_horizon.png", dpi=150)
    plt.close()

    print(f"\n  Figures saved to: {FIGURE_DIR}")
    print(f"    eval_pred_vs_true_sample.png")
    print(f"    eval_error_distribution.png")
    print(f"    eval_error_vs_horizon.png")
    print(f"\n{'='*60}")


if __name__ == "__main__":
    main()
