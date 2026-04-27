import json
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset


class ImitationDataset(Dataset):
    def __init__(
        self,
        dataset_dir,
        obs_horizon: int = 2,
        pred_horizon: int = 16,
        processed_dir=None,
    ):
        self.dataset_dir = Path(dataset_dir)
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon
        self.processed_dir = (
            Path(processed_dir) if processed_dir is not None else self.dataset_dir.parent / "processed"
        )

        self.episodes = self._load_episodes()
        self.indices: List[Tuple[int, int]] = []
        for episode_idx, episode in enumerate(self.episodes):
            length = episode["robot_state"].shape[0]
            first_t = obs_horizon - 1
            last_t = length - pred_horizon
            for t in range(first_t, last_t + 1):
                self.indices.append((episode_idx, t))

        if not self.indices:
            raise ValueError(
                "No valid samples. Need each episode length >= "
                f"obs_horizon + pred_horizon - 1 ({obs_horizon + pred_horizon - 1})."
            )

        all_states = np.concatenate([ep["robot_state"] for ep in self.episodes], axis=0)
        all_actions = np.concatenate([ep["action"] for ep in self.episodes], axis=0)
        self.stats = self._build_stats(all_states, all_actions)
        self._save_stats()

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        episode_idx, t = self.indices[idx]
        episode = self.episodes[episode_idx]

        state_seq = episode["robot_state"][t - self.obs_horizon + 1 : t + 1]
        action_seq = episode["action"][t : t + self.pred_horizon]

        state_seq = self.normalize_state(state_seq)
        action_seq = self.normalize_action(action_seq)

        return {
            "robot_state": torch.from_numpy(state_seq.astype(np.float32)),
            "action": torch.from_numpy(action_seq.astype(np.float32)),
        }

    def normalize_state(self, state: np.ndarray) -> np.ndarray:
        return (state - np.asarray(self.stats["robot_state_mean"], dtype=np.float32)) / np.asarray(
            self.stats["robot_state_std"], dtype=np.float32
        )

    def normalize_action(self, action: np.ndarray) -> np.ndarray:
        return (action - np.asarray(self.stats["action_mean"], dtype=np.float32)) / np.asarray(
            self.stats["action_std"], dtype=np.float32
        )

    def _load_episodes(self) -> List[Dict[str, np.ndarray]]:
        if self.dataset_dir.name.startswith("episode_") and self.dataset_dir.is_dir():
            episode_dirs = [self.dataset_dir]
        else:
            episode_dirs = sorted(p for p in self.dataset_dir.glob("episode_*") if p.is_dir())
        if not episode_dirs:
            raise FileNotFoundError(f"No episode_* directories found in {self.dataset_dir}")

        episodes = []
        state_dim = None
        action_dim = None
        for episode_dir in episode_dirs:
            state_path = episode_dir / "robot_state.npy"
            action_path = episode_dir / "action.npy"
            if not state_path.exists() or not action_path.exists():
                continue
            state = np.load(state_path, allow_pickle=False).astype(np.float32)
            action = np.load(action_path, allow_pickle=False).astype(np.float32)
            if state.ndim == 1:
                state = state[:, None]
            if action.ndim == 1:
                action = action[:, None]

            length = min(state.shape[0], action.shape[0])
            state = state[:length]
            action = action[:length]
            valid = np.isfinite(state).all(axis=1) & np.isfinite(action).all(axis=1)
            state = state[valid]
            action = action[valid]

            if state.shape[0] < self.obs_horizon + self.pred_horizon - 1:
                continue
            if state_dim is None:
                state_dim = state.shape[1]
                action_dim = action.shape[1]
            if state.shape[1] != state_dim or action.shape[1] != action_dim:
                raise ValueError(f"Dimension mismatch in {episode_dir}")

            episodes.append({"robot_state": state, "action": action})

        if not episodes:
            raise ValueError(f"No usable episodes found in {self.dataset_dir}")
        return episodes

    def _build_stats(self, states: np.ndarray, actions: np.ndarray) -> Dict:
        eps = 1e-6
        state_std = np.maximum(states.std(axis=0), eps)
        action_std = np.maximum(actions.std(axis=0), eps)
        return {
            "obs_horizon": self.obs_horizon,
            "pred_horizon": self.pred_horizon,
            "state_dim": int(states.shape[1]),
            "action_dim": int(actions.shape[1]),
            "robot_state_mean": states.mean(axis=0).tolist(),
            "robot_state_std": state_std.tolist(),
            "action_mean": actions.mean(axis=0).tolist(),
            "action_std": action_std.tolist(),
        }

    def _save_stats(self) -> None:
        self.processed_dir.mkdir(parents=True, exist_ok=True)
        with (self.processed_dir / "stats.json").open("w", encoding="utf-8") as f:
            json.dump(self.stats, f, indent=2)
            f.write("\n")
