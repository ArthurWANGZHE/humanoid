"""Shared dataset helpers for low-dimensional PushCube data."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import importlib
import importlib.util
import json
from pathlib import Path
from typing import Any

import numpy as np


TASK_NAME = "push_cube_manual"
FORMAT_NAME = "push_cube_low_dim_v2"
DATA_GROUP_NAME = "data"
OBS_DIM = 13
ACTION_DIM = 2
DEFAULT_ACTION_SCALE = 0.01
ACTION_SATURATION_EPS = 0.98


def optional_import(module_name: str, install_hint: str):
    try:
        spec = importlib.util.find_spec(module_name)
    except Exception as exc:
        raise RuntimeError(
            f"Failed checking optional dependency {module_name}: {exc}"
        ) from exc

    if spec is None:
        raise RuntimeError(
            f"Missing Python package: {module_name}. Install hint: {install_hint}"
        )

    return importlib.import_module(module_name)


def created_at_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def bool_scalar(value: Any) -> bool:
    if isinstance(value, np.ndarray):
        return bool(value.reshape(-1)[0])
    return bool(value)


def json_ready(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    return value


def quaternion_xyzw_from_rotation(rotation_matrix: np.ndarray) -> np.ndarray:
    rotation_matrix = np.array(rotation_matrix, dtype=np.float64)
    trace = float(np.trace(rotation_matrix))
    if trace > 0.0:
        s_value = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s_value
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s_value
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s_value
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s_value
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        s_value = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s_value
        qx = 0.25 * s_value
        qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s_value
        qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s_value
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s_value = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2.0
        qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s_value
        qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s_value
        qy = 0.25 * s_value
        qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s_value
    else:
        s_value = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2.0
        qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s_value
        qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s_value
        qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s_value
        qz = 0.25 * s_value
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def pose_to_vec7(position: np.ndarray, rotation_matrix: np.ndarray | None = None) -> np.ndarray:
    pose = np.zeros(7, dtype=np.float32)
    pose[:3] = np.array(position, dtype=np.float32)[:3]
    if rotation_matrix is None:
        pose[3:] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    else:
        pose[3:] = quaternion_xyzw_from_rotation(rotation_matrix).astype(np.float32)
    return pose


def cube_yaw_from_rotation(rotation_matrix: np.ndarray | None) -> float:
    if rotation_matrix is None:
        return 0.0
    rotation_matrix = np.array(rotation_matrix, dtype=np.float64)
    return float(np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))


def build_low_dim_observation(
    *,
    ee_xy: np.ndarray,
    cube_xy: np.ndarray,
    cube_yaw: float,
    target_xy: np.ndarray,
    target_size_xy: np.ndarray,
) -> np.ndarray:
    ee_xy = np.array(ee_xy, dtype=np.float64)
    cube_xy = np.array(cube_xy, dtype=np.float64)
    target_xy = np.array(target_xy, dtype=np.float64)
    target_size_xy = np.array(target_size_xy, dtype=np.float64)
    rel_ee_to_cube_xy = cube_xy - ee_xy
    rel_cube_to_target_xy = target_xy - cube_xy
    observation = np.array(
        [
            float(ee_xy[0]),
            float(ee_xy[1]),
            float(cube_xy[0]),
            float(cube_xy[1]),
            float(cube_yaw),
            float(target_xy[0]),
            float(target_xy[1]),
            float(target_size_xy[0]),
            float(target_size_xy[1]),
            float(rel_ee_to_cube_xy[0]),
            float(rel_ee_to_cube_xy[1]),
            float(rel_cube_to_target_xy[0]),
            float(rel_cube_to_target_xy[1]),
        ],
        dtype=np.float32,
    )
    if observation.shape != (OBS_DIM,):
        raise ValueError(f"Expected observation shape {(OBS_DIM,)}, got {observation.shape}")
    return observation


def normalize_action(planar_delta_xy: np.ndarray, action_scale: float) -> np.ndarray:
    scale = max(abs(float(action_scale)), 1e-8)
    normalized = np.array(planar_delta_xy, dtype=np.float32) / scale
    return np.clip(normalized, -1.0, 1.0).astype(np.float32)


@dataclass
class EpisodeData:
    obs: np.ndarray
    action: np.ndarray
    reward: np.ndarray
    done: np.ndarray
    success: bool
    timestamps: np.ndarray
    cube_pose: np.ndarray
    target_pose: np.ndarray
    ee_pose: np.ndarray
    right_joint_pos: np.ndarray
    right_joint_vel: np.ndarray
    left_joint_pos: np.ndarray
    gripper_qpos: np.ndarray
    key_pressed: np.ndarray
    debug_extra: dict[str, np.ndarray] | None = None


def default_file_metadata() -> dict[str, Any]:
    return {
        "task_name": TASK_NAME,
        "format": FORMAT_NAME,
        "obs_dim": OBS_DIM,
        "action_dim": ACTION_DIM,
        "created_at": created_at_timestamp(),
    }


def ensure_h5py():
    return optional_import("h5py", "pip install h5py")


def load_all_episodes(dataset_path: str | Path) -> tuple[dict[str, Any], list[tuple[str, EpisodeData]]]:
    h5py = ensure_h5py()
    dataset_path = Path(dataset_path)
    episodes: list[tuple[str, EpisodeData]] = []
    metadata: dict[str, Any] = {}
    with h5py.File(dataset_path, "r") as h5_file:
        metadata = {str(key): json_ready(value) for key, value in h5_file.attrs.items()}
        data_group = h5_file[DATA_GROUP_NAME]
        for episode_name in sorted(data_group.keys()):
            group = data_group[episode_name]
            debug_group = group["debug"]
            debug_extra: dict[str, np.ndarray] = {}
            for key in debug_group.keys():
                if key in {
                    "cube_pose",
                    "target_pose",
                    "ee_pose",
                    "right_joint_pos",
                    "right_joint_vel",
                    "left_joint_pos",
                    "gripper_qpos",
                    "key_pressed",
                }:
                    continue
                debug_extra[key] = np.array(debug_group[key])
            episodes.append(
                (
                    episode_name,
                    EpisodeData(
                        obs=np.array(group["obs"]),
                        action=np.array(group["action"]),
                        reward=np.array(group["reward"]),
                        done=np.array(group["done"]),
                        success=bool_scalar(group["success"][()]),
                        timestamps=np.array(group["timestamps"]),
                        cube_pose=np.array(debug_group["cube_pose"]),
                        target_pose=np.array(debug_group["target_pose"]),
                        ee_pose=np.array(debug_group["ee_pose"]),
                        right_joint_pos=np.array(debug_group["right_joint_pos"]),
                        right_joint_vel=np.array(debug_group["right_joint_vel"]),
                        left_joint_pos=np.array(debug_group["left_joint_pos"]),
                        gripper_qpos=np.array(debug_group["gripper_qpos"]),
                        key_pressed=np.array(debug_group["key_pressed"]).astype("U"),
                        debug_extra=debug_extra,
                    ),
                )
            )
    return metadata, episodes


def _next_demo_index(data_group) -> int:
    max_index = -1
    for name in data_group.keys():
        if not name.startswith("demo_"):
            continue
        try:
            max_index = max(max_index, int(name.split("_", maxsplit=1)[1]))
        except (IndexError, ValueError):
            continue
    return max_index + 1


class PushCubeWriter:
    def __init__(self, output_path: str | Path, metadata: dict[str, Any]) -> None:
        ensure_h5py()
        self.output_path = Path(output_path)
        self.metadata = dict(default_file_metadata())
        self.metadata.update(metadata)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def _write_metadata(self, h5_file) -> None:
        for key, value in self.metadata.items():
            if isinstance(value, (list, tuple, np.ndarray)):
                h5_file.attrs[key] = np.array(value)
            elif isinstance(value, (dict,)):
                h5_file.attrs[key] = json.dumps(json_ready(value), sort_keys=True)
            else:
                h5_file.attrs[key] = value

    def write_episode(self, episode: EpisodeData) -> str:
        h5py = ensure_h5py()
        with h5py.File(self.output_path, "a") as h5_file:
            self._write_metadata(h5_file)
            data_group = h5_file.require_group(DATA_GROUP_NAME)
            episode_name = f"demo_{_next_demo_index(data_group):06d}"
            group = data_group.create_group(episode_name)
            group.create_dataset("obs", data=np.array(episode.obs, dtype=np.float32))
            group.create_dataset("action", data=np.array(episode.action, dtype=np.float32))
            group.create_dataset("reward", data=np.array(episode.reward, dtype=np.float32))
            group.create_dataset("done", data=np.array(episode.done, dtype=np.bool_))
            group.create_dataset("success", data=np.array(bool(episode.success), dtype=np.bool_))
            group.create_dataset("timestamps", data=np.array(episode.timestamps, dtype=np.float64))
            debug_group = group.create_group("debug")
            debug_group.create_dataset("cube_pose", data=np.array(episode.cube_pose, dtype=np.float32))
            debug_group.create_dataset("target_pose", data=np.array(episode.target_pose, dtype=np.float32))
            debug_group.create_dataset("ee_pose", data=np.array(episode.ee_pose, dtype=np.float32))
            debug_group.create_dataset("right_joint_pos", data=np.array(episode.right_joint_pos, dtype=np.float32))
            debug_group.create_dataset("right_joint_vel", data=np.array(episode.right_joint_vel, dtype=np.float32))
            debug_group.create_dataset("left_joint_pos", data=np.array(episode.left_joint_pos, dtype=np.float32))
            debug_group.create_dataset("gripper_qpos", data=np.array(episode.gripper_qpos, dtype=np.float32))
            string_dtype = h5py.string_dtype(encoding="utf-8")
            debug_group.create_dataset("key_pressed", data=np.array(episode.key_pressed, dtype=string_dtype))
            for key, value in (episode.debug_extra or {}).items():
                debug_group.create_dataset(key, data=np.array(value))
            h5_file.attrs["num_demos"] = len(data_group.keys())
        return episode_name


@dataclass
class EpisodeMetrics:
    length: int
    success: bool
    nan_count: int
    action_min: np.ndarray
    action_max: np.ndarray
    action_mean: np.ndarray
    action_std: np.ndarray
    cube_initial_xy: np.ndarray
    cube_final_xy: np.ndarray
    cube_motion: float
    cube_progress: float
    final_cube_to_target_distance: float
    gripper_to_cube_distance_mean: float | None
    gripper_to_cube_distance_max: float | None
    action_saturation_fraction: float


def compute_episode_metrics(episode: EpisodeData) -> EpisodeMetrics:
    action = np.array(episode.action, dtype=np.float64)
    cube_xy = np.array(episode.cube_pose[:, :2], dtype=np.float64)
    target_xy = np.array(episode.target_pose[:, :2], dtype=np.float64)
    ee_xy = np.array(episode.ee_pose[:, :2], dtype=np.float64)
    initial_distance = float(np.linalg.norm(cube_xy[0] - target_xy[0]))
    final_distance = float(np.linalg.norm(cube_xy[-1] - target_xy[-1]))
    displacement = cube_xy - cube_xy[0]
    cube_motion = float(np.max(np.linalg.norm(displacement, axis=1))) if displacement.size else 0.0
    gripper_to_cube = np.linalg.norm(ee_xy - cube_xy, axis=1) if ee_xy.size and cube_xy.size else None
    nan_count = int(
        np.isnan(episode.obs).sum()
        + np.isnan(episode.action).sum()
        + np.isnan(episode.reward).sum()
        + np.isnan(episode.timestamps).sum()
        + np.isnan(episode.cube_pose).sum()
        + np.isnan(episode.target_pose).sum()
        + np.isnan(episode.ee_pose).sum()
    )
    return EpisodeMetrics(
        length=int(episode.obs.shape[0]),
        success=bool(episode.success),
        nan_count=nan_count,
        action_min=np.min(action, axis=0),
        action_max=np.max(action, axis=0),
        action_mean=np.mean(action, axis=0),
        action_std=np.std(action, axis=0),
        cube_initial_xy=np.array(cube_xy[0], dtype=np.float64),
        cube_final_xy=np.array(cube_xy[-1], dtype=np.float64),
        cube_motion=cube_motion,
        cube_progress=initial_distance - final_distance,
        final_cube_to_target_distance=final_distance,
        gripper_to_cube_distance_mean=(float(np.mean(gripper_to_cube)) if gripper_to_cube is not None else None),
        gripper_to_cube_distance_max=(float(np.max(gripper_to_cube)) if gripper_to_cube is not None else None),
        action_saturation_fraction=float(np.mean(np.abs(action) >= ACTION_SATURATION_EPS)),
    )


def aggregate_action_stats(episodes: list[tuple[str, EpisodeData]]) -> dict[str, np.ndarray]:
    all_actions = np.concatenate([episode.action for _, episode in episodes], axis=0)
    return {
        "min": np.min(all_actions, axis=0),
        "max": np.max(all_actions, axis=0),
        "mean": np.mean(all_actions, axis=0),
        "std": np.std(all_actions, axis=0),
    }


def summarize_lengths(episodes: list[tuple[str, EpisodeData]]) -> dict[str, float]:
    lengths = np.array([episode.obs.shape[0] for _, episode in episodes], dtype=np.float64)
    return {
        "min": float(np.min(lengths)),
        "mean": float(np.mean(lengths)),
        "max": float(np.max(lengths)),
    }
