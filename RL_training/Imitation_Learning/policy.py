from pathlib import Path

import numpy as np
import torch

try:
    from .models.diffusion_policy import DiffusionPolicy
    from .train_diffusion_policy import make_schedule
except ImportError:
    from models.diffusion_policy import DiffusionPolicy
    from train_diffusion_policy import make_schedule


class DiffusionPolicyWrapper:
    def __init__(self, checkpoint_path, device=None):
        self.checkpoint_path = Path(checkpoint_path)
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        self.stats = checkpoint["stats"]
        self.config = checkpoint["config"]

        self.model = DiffusionPolicy(
            state_dim=self.config["state_dim"],
            action_dim=self.config["action_dim"],
            obs_horizon=self.config["obs_horizon"],
            pred_horizon=self.config["pred_horizon"],
        ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state_dict"])
        self.model.eval()

        self.betas, self.alphas, self.alphas_cumprod = make_schedule(
            self.config["timesteps"], self.device
        )

    def predict(self, robot_state_seq):
        state = np.asarray(robot_state_seq, dtype=np.float32)
        expected_shape = (self.config["obs_horizon"], self.config["state_dim"])
        if state.shape != expected_shape:
            raise ValueError(f"robot_state_seq must have shape {expected_shape}, got {state.shape}")

        state_mean = np.asarray(self.stats["robot_state_mean"], dtype=np.float32)
        state_std = np.asarray(self.stats["robot_state_std"], dtype=np.float32)
        action_mean = np.asarray(self.stats["action_mean"], dtype=np.float32)
        action_std = np.asarray(self.stats["action_std"], dtype=np.float32)

        cond = torch.from_numpy((state - state_mean) / state_std).unsqueeze(0).to(self.device)
        action = torch.randn(
            1,
            self.config["pred_horizon"],
            self.config["action_dim"],
            device=self.device,
        )

        with torch.no_grad():
            for step in reversed(range(self.config["timesteps"])):
                t = torch.full((1,), step, dtype=torch.long, device=self.device)
                pred_noise = self.model(action, cond, t)
                beta = self.betas[step]
                alpha = self.alphas[step]
                alpha_bar = self.alphas_cumprod[step]
                action = (action - beta / torch.sqrt(1.0 - alpha_bar) * pred_noise) / torch.sqrt(
                    alpha
                )
                if step > 0:
                    action = action + torch.sqrt(beta) * torch.randn_like(action)

        action = action.squeeze(0).cpu().numpy()
        return action * action_std + action_mean

    def act(self, robot_state_seq):
        return self.predict(robot_state_seq)[0]


def act(robot_state_seq, checkpoint_path="data/checkpoints/diffusion_policy/latest.pt"):
    policy = DiffusionPolicyWrapper(checkpoint_path)
    return policy.act(robot_state_seq)
