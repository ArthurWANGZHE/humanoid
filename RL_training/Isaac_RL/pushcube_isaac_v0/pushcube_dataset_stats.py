"""Shared dataset statistics helpers for PushCube pipelines."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

from pushcube_isaac_v0.pushcube_dataset_utils import ACTION_SATURATION_EPS, EpisodeData


OBSERVATION_LABELS = [
    "ee_x",
    "ee_y",
    "cube_x",
    "cube_y",
    "cube_yaw",
    "target_x",
    "target_y",
    "target_size_x",
    "target_size_y",
    "rel_ee_to_cube_x",
    "rel_ee_to_cube_y",
    "rel_cube_to_target_x",
    "rel_cube_to_target_y",
]


@dataclass
class EpisodeSummary:
    name: str
    index: int
    length: int
    success: bool
    obs_dim: int
    action_dim: int
    nan_count: int
    inf_count: int
    cube_initial_xy: list[float]
    cube_final_xy: list[float]
    target_xy: list[float]
    cube_total_motion: float
    cube_max_displacement: float
    cube_progress: float
    initial_cube_to_target_distance: float
    final_cube_to_target_distance: float
    final_ee_to_cube_distance: float
    ee_total_motion: float
    ee_max_displacement: float
    gripper_to_cube_distance_mean: float
    gripper_to_cube_distance_max: float
    action_min: list[float]
    action_max: list[float]
    action_mean: list[float]
    action_std: list[float]
    action_saturation_ratio: float

    def to_row(self) -> dict[str, Any]:
        row = asdict(self)
        for key in ("cube_initial_xy", "cube_final_xy", "target_xy", "action_min", "action_max", "action_mean", "action_std"):
            row[key] = json.dumps(row[key])
        return row


def json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    return value


def parse_json_metadata_value(value: Any) -> Any:
    if isinstance(value, bytes):
        value = value.decode("utf-8")
    if isinstance(value, str):
        text = value.strip()
        if text.startswith("{") or text.startswith("["):
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                return value
    return value


def parse_table_bounds(metadata: dict[str, Any]) -> dict[str, float]:
    bounds = parse_json_metadata_value(metadata.get("table_bounds"))
    if isinstance(bounds, dict):
        return {
            "x_min": float(bounds.get("x_min", 0.17)),
            "x_max": float(bounds.get("x_max", 0.79)),
            "y_min": float(bounds.get("y_min", -0.55)),
            "y_max": float(bounds.get("y_max", -0.05)),
        }
    return {"x_min": 0.17, "x_max": 0.79, "y_min": -0.55, "y_max": -0.05}


def parse_target_size_xy(metadata: dict[str, Any], fallback_xy: np.ndarray | None = None) -> np.ndarray:
    target_size_xy = parse_json_metadata_value(metadata.get("target_size_xy"))
    if isinstance(target_size_xy, list) and len(target_size_xy) >= 2:
        return np.array(target_size_xy[:2], dtype=np.float64)
    if fallback_xy is not None:
        return np.array(fallback_xy[:2], dtype=np.float64)
    return np.array([0.22, 0.22], dtype=np.float64)


def dataset_id_from_path(dataset_path: str | Path) -> str:
    return Path(dataset_path).expanduser().resolve().stem


def _path_length(points_xy: np.ndarray) -> float:
    if points_xy.shape[0] <= 1:
        return 0.0
    deltas = np.diff(points_xy, axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def _max_displacement(points_xy: np.ndarray) -> float:
    if points_xy.shape[0] <= 1:
        return 0.0
    return float(np.max(np.linalg.norm(points_xy - points_xy[0], axis=1)))


def _count_non_finite(*arrays: np.ndarray) -> tuple[int, int]:
    nan_count = 0
    inf_count = 0
    for array in arrays:
        array = np.asarray(array)
        nan_count += int(np.isnan(array).sum())
        inf_count += int(np.isinf(array).sum())
    return nan_count, inf_count


def summarize_episode(name: str, episode: EpisodeData, index: int) -> EpisodeSummary:
    cube_xy = np.asarray(episode.cube_pose[:, :2], dtype=np.float64)
    target_xy = np.asarray(episode.target_pose[:, :2], dtype=np.float64)
    ee_xy = np.asarray(episode.ee_pose[:, :2], dtype=np.float64)
    action = np.asarray(episode.action, dtype=np.float64)
    cube_to_target = np.linalg.norm(target_xy - cube_xy, axis=1)
    ee_to_cube = np.linalg.norm(ee_xy - cube_xy, axis=1)
    nan_count, inf_count = _count_non_finite(
        episode.obs,
        episode.action,
        episode.reward,
        episode.timestamps,
        episode.cube_pose,
        episode.target_pose,
        episode.ee_pose,
    )
    return EpisodeSummary(
        name=name,
        index=index,
        length=int(episode.obs.shape[0]),
        success=bool(episode.success),
        obs_dim=int(episode.obs.shape[1]) if episode.obs.ndim == 2 else 0,
        action_dim=int(episode.action.shape[1]) if episode.action.ndim == 2 else 0,
        nan_count=nan_count,
        inf_count=inf_count,
        cube_initial_xy=np.asarray(cube_xy[0], dtype=np.float64).round(8).tolist(),
        cube_final_xy=np.asarray(cube_xy[-1], dtype=np.float64).round(8).tolist(),
        target_xy=np.asarray(target_xy[-1], dtype=np.float64).round(8).tolist(),
        cube_total_motion=_path_length(cube_xy),
        cube_max_displacement=_max_displacement(cube_xy),
        cube_progress=float(cube_to_target[0] - cube_to_target[-1]),
        initial_cube_to_target_distance=float(cube_to_target[0]),
        final_cube_to_target_distance=float(cube_to_target[-1]),
        final_ee_to_cube_distance=float(ee_to_cube[-1]),
        ee_total_motion=_path_length(ee_xy),
        ee_max_displacement=_max_displacement(ee_xy),
        gripper_to_cube_distance_mean=float(np.mean(ee_to_cube)),
        gripper_to_cube_distance_max=float(np.max(ee_to_cube)),
        action_min=np.min(action, axis=0).round(8).tolist(),
        action_max=np.max(action, axis=0).round(8).tolist(),
        action_mean=np.mean(action, axis=0).round(8).tolist(),
        action_std=np.std(action, axis=0).round(8).tolist(),
        action_saturation_ratio=float(np.mean(np.abs(action) >= ACTION_SATURATION_EPS)),
    )


def _array_stats(values: np.ndarray) -> dict[str, Any]:
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        return {
            "min": float(np.min(values)),
            "mean": float(np.mean(values)),
            "max": float(np.max(values)),
            "std": float(np.std(values)),
        }
    return {
        "min": np.min(values, axis=0).round(8).tolist(),
        "mean": np.mean(values, axis=0).round(8).tolist(),
        "max": np.max(values, axis=0).round(8).tolist(),
        "std": np.std(values, axis=0).round(8).tolist(),
    }


def summarize_dataset(
    metadata: dict[str, Any],
    episodes: list[tuple[str, EpisodeData]],
    *,
    low_motion_threshold: float = 0.03,
) -> tuple[dict[str, Any], list[EpisodeSummary]]:
    summaries = [summarize_episode(name, episode, index) for index, (name, episode) in enumerate(episodes)]
    if not summaries:
        raise RuntimeError("Dataset contains no episodes.")

    actions = np.concatenate([episode.action for _, episode in episodes], axis=0)
    observations = np.concatenate([episode.obs for _, episode in episodes], axis=0)
    cube_initial_xy = np.array([summary.cube_initial_xy for summary in summaries], dtype=np.float64)
    cube_final_xy = np.array([summary.cube_final_xy for summary in summaries], dtype=np.float64)
    target_xy = np.array([summary.target_xy for summary in summaries], dtype=np.float64)
    cube_total_motion = np.array([summary.cube_total_motion for summary in summaries], dtype=np.float64)
    cube_max_displacement = np.array([summary.cube_max_displacement for summary in summaries], dtype=np.float64)
    cube_progress = np.array([summary.cube_progress for summary in summaries], dtype=np.float64)
    final_cube_distance = np.array([summary.final_cube_to_target_distance for summary in summaries], dtype=np.float64)
    final_ee_distance = np.array([summary.final_ee_to_cube_distance for summary in summaries], dtype=np.float64)
    lengths = np.array([summary.length for summary in summaries], dtype=np.float64)
    normalized_actions = bool(np.max(np.abs(actions)) <= 1.05)
    summary = {
        "metadata": json_ready({key: parse_json_metadata_value(value) for key, value in metadata.items()}),
        "num_demos": len(summaries),
        "success_count": int(sum(summary.success for summary in summaries)),
        "episode_length": _array_stats(lengths),
        "obs_dim": int(summaries[0].obs_dim),
        "action_dim": int(summaries[0].action_dim),
        "action": _array_stats(actions),
        "cube_initial_xy": {"mean": np.mean(cube_initial_xy, axis=0).round(8).tolist(), "std": np.std(cube_initial_xy, axis=0).round(8).tolist()},
        "cube_final_xy": {"mean": np.mean(cube_final_xy, axis=0).round(8).tolist(), "std": np.std(cube_final_xy, axis=0).round(8).tolist()},
        "target_xy": {"mean": np.mean(target_xy, axis=0).round(8).tolist(), "std": np.std(target_xy, axis=0).round(8).tolist()},
        "cube_total_motion": _array_stats(cube_total_motion),
        "cube_max_displacement": _array_stats(cube_max_displacement),
        "cube_progress_toward_target": _array_stats(cube_progress),
        "final_cube_to_target_distance": _array_stats(final_cube_distance),
        "final_ee_to_cube_distance": _array_stats(final_ee_distance),
        "low_cube_motion_threshold": float(low_motion_threshold),
        "very_low_cube_motion_count": int(np.sum(cube_max_displacement < float(low_motion_threshold))),
        "demos_with_nan": int(sum(summary.nan_count > 0 for summary in summaries)),
        "demos_with_inf": int(sum(summary.inf_count > 0 for summary in summaries)),
        "normalized_actions": normalized_actions,
        "action_saturation_ratio": (float(np.mean(np.abs(actions) >= ACTION_SATURATION_EPS)) if normalized_actions else None),
        "observation_selected_dimensions": {
            label: _array_stats(observations[:, index])
            for index, label in enumerate(OBSERVATION_LABELS[: min(len(OBSERVATION_LABELS), observations.shape[1])])
        },
        "per_episode": [summary.to_row() for summary in summaries],
    }
    return summary, summaries


def build_layout_regions(
    metadata: dict[str, Any],
    episodes: list[tuple[str, EpisodeData]],
) -> dict[str, Any]:
    table_bounds = parse_table_bounds(metadata)
    target_size_xy = parse_target_size_xy(metadata, fallback_xy=episodes[0][1].obs[-1, 7:9])
    initial_cube_xy = np.array([episode.cube_pose[0, :2] for _, episode in episodes], dtype=np.float64)
    target_xy = np.array([episode.target_pose[-1, :2] for _, episode in episodes], dtype=np.float64)
    layout_preset = str(metadata.get("layout_preset", "default"))
    if layout_preset == "opposite_edges":
        spawn_size_xy = np.array([0.18, 0.14], dtype=np.float64)
    else:
        spread = np.max(initial_cube_xy, axis=0) - np.min(initial_cube_xy, axis=0)
        spawn_size_xy = np.maximum(spread + 0.04, np.array([0.10, 0.10], dtype=np.float64))
    return {
        "table_bounds": table_bounds,
        "spawn_center_xy": np.mean(initial_cube_xy, axis=0),
        "spawn_size_xy": spawn_size_xy,
        "target_center_xy": np.mean(target_xy, axis=0),
        "target_size_xy": target_size_xy,
    }


def linear_resample_array(array: np.ndarray, new_length: int) -> np.ndarray:
    array = np.asarray(array)
    if array.shape[0] == new_length:
        return array.copy()
    if array.shape[0] == 1:
        return np.repeat(array, new_length, axis=0)
    old_grid = np.linspace(0.0, 1.0, num=array.shape[0], dtype=np.float64)
    new_grid = np.linspace(0.0, 1.0, num=new_length, dtype=np.float64)
    flat = array.reshape(array.shape[0], -1)
    resampled = np.empty((new_length, flat.shape[1]), dtype=np.float64)
    for column in range(flat.shape[1]):
        resampled[:, column] = np.interp(new_grid, old_grid, flat[:, column])
    return resampled.reshape((new_length,) + array.shape[1:]).astype(array.dtype, copy=False)


def nearest_resample_array(array: np.ndarray, new_length: int) -> np.ndarray:
    array = np.asarray(array)
    if array.shape[0] == new_length:
        return array.copy()
    if array.shape[0] == 1:
        return np.repeat(array, new_length, axis=0)
    sample_indices = np.linspace(0, array.shape[0] - 1, num=new_length)
    return array[np.rint(sample_indices).astype(np.int64)]

