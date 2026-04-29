import argparse
from pathlib import Path

import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

try:
    from .models.diffusion_policy import DiffusionPolicy
    from .models.imitation_dataset import ImitationDataset
except ImportError:
    from models.diffusion_policy import DiffusionPolicy
    from models.imitation_dataset import ImitationDataset


def resolve_dataset_dir(path: str) -> Path:
    dataset_dir = Path(path)
    if dataset_dir.exists():
        return dataset_dir
    fallback = Path("data")
    if path == "dataset/raw" and fallback.exists():
        print("dataset/raw not found; using existing data/ directory.")
        return fallback
    return dataset_dir


def make_schedule(timesteps: int, device: torch.device):
    betas = torch.linspace(1e-4, 0.02, timesteps, device=device)
    alphas = 1.0 - betas
    alphas_cumprod = torch.cumprod(alphas, dim=0)
    return betas, alphas, alphas_cumprod


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="dataset/raw")
    parser.add_argument("--checkpoint-dir", default="logs/diffusion_policy/checkpoints")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--timesteps", type=int, default=100)
    parser.add_argument("--obs-horizon", type=int, default=2)
    parser.add_argument("--pred-horizon", type=int, default=16)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()

    device = torch.device(args.device)
    dataset_dir = resolve_dataset_dir(args.dataset)
    dataset = ImitationDataset(
        dataset_dir,
        obs_horizon=args.obs_horizon,
        pred_horizon=args.pred_horizon,
        processed_dir=Path("dataset/processed"),
    )
    dataloader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, drop_last=False)

    stats = dataset.stats
    model = DiffusionPolicy(
        state_dim=stats["state_dim"],
        action_dim=stats["action_dim"],
        obs_horizon=args.obs_horizon,
        pred_horizon=args.pred_horizon,
    ).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    _, _, alphas_cumprod = make_schedule(args.timesteps, device)

    config = {
        "obs_horizon": args.obs_horizon,
        "pred_horizon": args.pred_horizon,
        "timesteps": args.timesteps,
        "beta_start": 1e-4,
        "beta_end": 0.02,
        "state_dim": stats["state_dim"],
        "action_dim": stats["action_dim"],
    }

    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss = 0.0
        total_count = 0
        for batch in dataloader:
            cond = batch["robot_state"].to(device)
            action = batch["action"].to(device)
            batch_size = action.shape[0]

            t = torch.randint(0, args.timesteps, (batch_size,), device=device)
            noise = torch.randn_like(action)
            sqrt_alpha_bar = torch.sqrt(alphas_cumprod[t]).view(batch_size, 1, 1)
            sqrt_one_minus_alpha_bar = torch.sqrt(1.0 - alphas_cumprod[t]).view(batch_size, 1, 1)
            noisy_action = sqrt_alpha_bar * action + sqrt_one_minus_alpha_bar * noise

            pred_noise = model(noisy_action, cond, t)
            loss = F.mse_loss(pred_noise, noise)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item() * batch_size
            total_count += batch_size

        avg_loss = total_loss / max(total_count, 1)
        print(f"epoch {epoch:03d} loss {avg_loss:.6f}")

    checkpoint_dir = Path(args.checkpoint_dir)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_path = checkpoint_dir / "latest.pt"
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "stats": stats,
            "config": config,
        },
        checkpoint_path,
    )
    print(f"saved checkpoint: {checkpoint_path}")


if __name__ == "__main__":
    main()
