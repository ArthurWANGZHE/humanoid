"""Dataset helpers for PushCube2D demonstrations."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

from .env import ACTION_NAMES, OBS_NAMES, STATE_NAMES


def episode_from_lists(obs, actions, states, rewards, dones, success: bool) -> Dict[str, Any]:
    return {
        "obs": np.asarray(obs, dtype=np.float32),
        "actions": np.asarray(actions, dtype=np.float32),
        "states": np.asarray(states, dtype=np.float32),
        "rewards": np.asarray(rewards, dtype=np.float32),
        "dones": np.asarray(dones, dtype=bool),
        "success": bool(success),
    }


def pad_episodes(episodes: Iterable[Dict[str, Any]], max_len: int | None = None) -> Dict[str, np.ndarray]:
    episodes = list(episodes)
    if not episodes:
        raise ValueError("Cannot save an empty dataset.")

    lengths = np.array([len(ep["actions"]) for ep in episodes], dtype=np.int32)
    if np.any(lengths <= 0):
        raise ValueError("All episodes must contain at least one transition.")
    max_len = int(max_len or lengths.max())

    n = len(episodes)
    obs_dim = int(episodes[0]["obs"].shape[-1])
    action_dim = int(episodes[0]["actions"].shape[-1])
    state_dim = int(episodes[0]["states"].shape[-1])

    obs = np.zeros((n, max_len, obs_dim), dtype=np.float32)
    actions = np.zeros((n, max_len, action_dim), dtype=np.float32)
    states = np.zeros((n, max_len, state_dim), dtype=np.float32)
    rewards = np.zeros((n, max_len), dtype=np.float32)
    dones = np.zeros((n, max_len), dtype=bool)
    success = np.zeros(n, dtype=bool)

    for idx, ep in enumerate(episodes):
        length = min(int(lengths[idx]), max_len)
        obs[idx, :length] = ep["obs"][:length]
        actions[idx, :length] = ep["actions"][:length]
        states[idx, :length] = ep["states"][:length]
        rewards[idx, :length] = ep["rewards"][:length]
        dones[idx, :length] = ep["dones"][:length]
        success[idx] = bool(ep["success"])
        lengths[idx] = length

    return {
        "obs": obs,
        "actions": actions,
        "states": states,
        "rewards": rewards,
        "dones": dones,
        "success": success,
        "episode_lengths": lengths,
    }


def save_dataset(path: str | Path, episodes: Iterable[Dict[str, Any]], metadata: Dict[str, Any] | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = pad_episodes(episodes)
    metadata = dict(metadata or {})
    metadata.setdefault("obs_names", OBS_NAMES)
    metadata.setdefault("action_names", ACTION_NAMES)
    metadata.setdefault("state_names", STATE_NAMES)
    metadata.setdefault("num_episodes", int(arrays["obs"].shape[0]))
    metadata.setdefault("max_episode_length", int(arrays["obs"].shape[1]))
    np.savez_compressed(path, **arrays, metadata_json=np.array(json.dumps(metadata, indent=2)))
    return path


def load_dataset(path: str | Path) -> Dict[str, Any]:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    raw = np.load(path, allow_pickle=False)
    data = {key: raw[key] for key in raw.files if key != "metadata_json"}
    if "metadata_json" in raw.files:
        data["metadata"] = json.loads(str(raw["metadata_json"]))
    else:
        data["metadata"] = {}
    data["path"] = str(path)
    return data


def valid_mask(data: Dict[str, Any]) -> np.ndarray:
    lengths = np.asarray(data["episode_lengths"], dtype=np.int32)
    max_len = int(data["obs"].shape[1])
    return np.arange(max_len)[None, :] < lengths[:, None]


def flatten_valid_steps(data: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    mask = valid_mask(data)
    return data["obs"][mask].astype(np.float32), data["actions"][mask].astype(np.float32)


def dataset_summary(data: Dict[str, Any]) -> Dict[str, Any]:
    lengths = np.asarray(data["episode_lengths"])
    success = np.asarray(data["success"], dtype=bool)
    mask = valid_mask(data)
    cube = data["obs"][..., 2:4]
    goal = data["obs"][..., 5:7]
    distances = np.linalg.norm(cube - goal, axis=-1)[mask]
    return {
        "path": data.get("path", ""),
        "episodes": int(data["obs"].shape[0]),
        "max_T": int(data["obs"].shape[1]),
        "transitions": int(mask.sum()),
        "obs_dim": int(data["obs"].shape[-1]),
        "action_dim": int(data["actions"].shape[-1]),
        "success_rate": float(success.mean()) if success.size else 0.0,
        "mean_episode_length": float(lengths.mean()) if lengths.size else 0.0,
        "min_episode_length": int(lengths.min()) if lengths.size else 0,
        "max_episode_length": int(lengths.max()) if lengths.size else 0,
        "mean_cube_goal_distance": float(distances.mean()) if distances.size else 0.0,
        "final_mean_cube_goal_distance": float(np.mean([np.linalg.norm(data["obs"][i, lengths[i] - 1, 2:4] - data["obs"][i, lengths[i] - 1, 5:7]) for i in range(len(lengths))])),
    }


def validate_dataset(data: Dict[str, Any]) -> Tuple[List[str], Dict[str, Any]]:
    issues: List[str] = []
    required = ["obs", "actions", "states", "rewards", "dones", "success", "episode_lengths"]
    for key in required:
        if key not in data:
            issues.append(f"Missing key: {key}")
    if issues:
        return issues, {}

    obs = data["obs"]
    actions = data["actions"]
    states = data["states"]
    rewards = data["rewards"]
    dones = data["dones"]
    success = data["success"]
    lengths = data["episode_lengths"]

    if obs.ndim != 3:
        issues.append(f"obs should have shape [N,T,obs_dim], got {obs.shape}")
    if actions.ndim != 3:
        issues.append(f"actions should have shape [N,T,action_dim], got {actions.shape}")
    if states.ndim != 3:
        issues.append(f"states should have shape [N,T,state_dim], got {states.shape}")
    if rewards.shape != obs.shape[:2]:
        issues.append(f"rewards shape {rewards.shape} does not match obs prefix {obs.shape[:2]}")
    if dones.shape != obs.shape[:2]:
        issues.append(f"dones shape {dones.shape} does not match obs prefix {obs.shape[:2]}")
    if success.shape[0] != obs.shape[0]:
        issues.append(f"success shape {success.shape} does not match number of episodes {obs.shape[0]}")
    if lengths.shape[0] != obs.shape[0]:
        issues.append(f"episode_lengths shape {lengths.shape} does not match number of episodes {obs.shape[0]}")

    if obs.ndim == 3 and obs.shape[-1] != len(OBS_NAMES):
        issues.append(f"obs_dim should be {len(OBS_NAMES)}, got {obs.shape[-1]}")
    if actions.ndim == 3 and actions.shape[-1] != len(ACTION_NAMES):
        issues.append(f"action_dim should be {len(ACTION_NAMES)}, got {actions.shape[-1]}")
    if states.ndim == 3 and states.shape[-1] != len(STATE_NAMES):
        issues.append(f"state_dim should be {len(STATE_NAMES)}, got {states.shape[-1]}")

    if np.any(lengths <= 0):
        issues.append("episode_lengths must all be positive")
    if obs.ndim == 3 and np.any(lengths > obs.shape[1]):
        issues.append("episode_lengths cannot exceed padded T")

    for key in ["obs", "actions", "states", "rewards"]:
        if not np.all(np.isfinite(data[key])):
            issues.append(f"{key} contains non-finite values")

    if actions.ndim == 3 and np.nanmax(np.abs(actions)) > 1.0001:
        issues.append("actions appear outside normalized [-1, 1] range")

    if not issues:
        for i, length in enumerate(lengths.astype(int)):
            if not bool(dones[i, length - 1]):
                issues.append(f"episode {i} final valid done flag is False")
                break

    return issues, dataset_summary(data) if not issues else {}

