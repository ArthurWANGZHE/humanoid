import math

import torch
from torch import nn


class SinusoidalTimestepEmbedding(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.dim = dim

    def forward(self, timesteps: torch.Tensor) -> torch.Tensor:
        half_dim = self.dim // 2
        scale = math.log(10000) / max(half_dim - 1, 1)
        freqs = torch.exp(
            torch.arange(half_dim, device=timesteps.device, dtype=torch.float32) * -scale
        )
        args = timesteps.float().unsqueeze(1) * freqs.unsqueeze(0)
        emb = torch.cat([torch.sin(args), torch.cos(args)], dim=1)
        if self.dim % 2 == 1:
            emb = torch.cat([emb, torch.zeros_like(emb[:, :1])], dim=1)
        return emb


class DiffusionPolicy(nn.Module):
    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        obs_horizon: int = 2,
        pred_horizon: int = 16,
        hidden_dim: int = 256,
        timestep_embed_dim: int = 64,
    ):
        super().__init__()
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon

        action_size = pred_horizon * action_dim
        cond_size = obs_horizon * state_dim
        self.time_embedding = SinusoidalTimestepEmbedding(timestep_embed_dim)
        self.net = nn.Sequential(
            nn.Linear(action_size + cond_size + timestep_embed_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_size),
        )

    def forward(
        self, noisy_action: torch.Tensor, cond: torch.Tensor, t: torch.Tensor
    ) -> torch.Tensor:
        batch_size = noisy_action.shape[0]
        x = noisy_action.reshape(batch_size, -1)
        c = cond.reshape(batch_size, -1)
        te = self.time_embedding(t)
        pred = self.net(torch.cat([x, c, te], dim=1))
        return pred.reshape(batch_size, self.pred_horizon, self.action_dim)
