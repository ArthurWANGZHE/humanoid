import argparse
from pathlib import Path

import numpy as np

try:
    from .models.imitation_dataset import ImitationDataset
    from .policy import DiffusionPolicyWrapper
    from .train_diffusion_policy import resolve_dataset_dir
except ImportError:
    from models.imitation_dataset import ImitationDataset
    from policy import DiffusionPolicyWrapper
    from train_diffusion_policy import resolve_dataset_dir


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="logs/diffusion_policy/checkpoints/latest.pt")
    parser.add_argument("--dataset", default="dataset/raw")
    parser.add_argument("--max-samples", type=int, default=None)
    args = parser.parse_args()

    policy = DiffusionPolicyWrapper(args.checkpoint)
    dataset = ImitationDataset(
        resolve_dataset_dir(args.dataset),
        obs_horizon=policy.config["obs_horizon"],
        pred_horizon=policy.config["pred_horizon"],
        processed_dir=Path("dataset/processed"),
    )

    errors = []
    count = len(dataset) if args.max_samples is None else min(len(dataset), args.max_samples)
    for idx in range(count):
        sample = dataset[idx]
        norm_state = sample["robot_state"].numpy()
        norm_gt_action = sample["action"].numpy()[0]

        state_mean = np.asarray(dataset.stats["robot_state_mean"], dtype=np.float32)
        state_std = np.asarray(dataset.stats["robot_state_std"], dtype=np.float32)
        action_mean = np.asarray(dataset.stats["action_mean"], dtype=np.float32)
        action_std = np.asarray(dataset.stats["action_std"], dtype=np.float32)

        state = norm_state * state_std + state_mean
        gt_action = norm_gt_action * action_std + action_mean
        pred_action = policy.act(state)
        errors.append(np.mean((pred_action - gt_action) ** 2))

    print(f"avg MSE: {float(np.mean(errors)):.6f}")


if __name__ == "__main__":
    main()
