"""Augment PushCube HDF5 demonstrations to a larger synthetic dataset."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.pushcube_dataset_stats import (  # noqa: E402
    linear_resample_array,
    nearest_resample_array,
    parse_table_bounds,
    summarize_dataset,
)
from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    EpisodeData,
    PushCubeWriter,
    build_low_dim_observation,
    created_at_timestamp,
    load_all_episodes,
    normalize_action,
    optional_import,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Augment PushCube manual demos into a larger HDF5 dataset.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    parser.add_argument("--target-count", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--translation-range", type=float, default=0.03)
    parser.add_argument("--time-scale-min", type=float, default=0.85)
    parser.add_argument("--time-scale-max", type=float, default=1.15)
    parser.add_argument("--cube-noise", type=float, default=0.003)
    parser.add_argument("--ee-noise", type=float, default=0.003)
    parser.add_argument("--action-noise", type=float, default=0.02)
    parser.add_argument("--plots-dir", type=str, default=None)
    return parser


def _resample_string_array(values: np.ndarray, new_length: int) -> np.ndarray:
    return nearest_resample_array(values.astype("U"), new_length).astype(str)


def _clip_shift(xy_sequences: list[np.ndarray], table_bounds: dict[str, float], requested_range: float, rng: np.random.Generator) -> np.ndarray:
    all_xy = np.concatenate(xy_sequences, axis=0)
    dx_low = max(-float(requested_range), float(table_bounds["x_min"] - np.min(all_xy[:, 0])))
    dx_high = min(float(requested_range), float(table_bounds["x_max"] - np.max(all_xy[:, 0])))
    dy_low = max(-float(requested_range), float(table_bounds["y_min"] - np.min(all_xy[:, 1])))
    dy_high = min(float(requested_range), float(table_bounds["y_max"] - np.max(all_xy[:, 1])))
    dx = rng.uniform(dx_low, dx_high) if dx_low <= dx_high else 0.0
    dy = rng.uniform(dy_low, dy_high) if dy_low <= dy_high else 0.0
    return np.array([dx, dy], dtype=np.float64)


def _bounded_noise(shape: tuple[int, ...], max_abs: float, rng: np.random.Generator) -> np.ndarray:
    noise = rng.normal(loc=0.0, scale=max_abs / 2.5, size=shape)
    return np.clip(noise, -max_abs, max_abs)


def _rebuild_obs(ee_pose: np.ndarray, cube_pose: np.ndarray, target_pose: np.ndarray, target_size_xy: np.ndarray) -> np.ndarray:
    observations = []
    for step in range(ee_pose.shape[0]):
        cube_xy = cube_pose[step, :2]
        ee_xy = ee_pose[step, :2]
        target_xy = target_pose[step, :2]
        qw = cube_pose[step, 6]
        qx = cube_pose[step, 3]
        qy = cube_pose[step, 4]
        qz = cube_pose[step, 5]
        yaw = float(np.arctan2(2.0 * ((qw * qz) + (qx * qy)), 1.0 - (2.0 * ((qy * qy) + (qz * qz)))))
        observations.append(
            build_low_dim_observation(
                ee_xy=ee_xy,
                cube_xy=cube_xy,
                cube_yaw=yaw,
                target_xy=target_xy,
                target_size_xy=target_size_xy,
            )
        )
    return np.stack(observations, axis=0).astype(np.float32)


def _recompute_action(ee_pose: np.ndarray, action_scale: float, action_noise: float, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    planar_delta = np.zeros((ee_pose.shape[0], 2), dtype=np.float32)
    if ee_pose.shape[0] > 1:
        planar_delta[:-1] = np.diff(ee_pose[:, :2], axis=0).astype(np.float32)
    action = normalize_action(planar_delta, action_scale)
    action = np.clip(action + _bounded_noise(action.shape, action_noise, rng).astype(np.float32), -1.0, 1.0)
    return action.astype(np.float32), planar_delta.astype(np.float32)


def _resample_episode(episode: EpisodeData, new_length: int) -> EpisodeData:
    debug_extra = {key: linear_resample_array(value, new_length) for key, value in (episode.debug_extra or {}).items() if np.asarray(value).dtype.kind != "U"}
    for key, value in (episode.debug_extra or {}).items():
        if np.asarray(value).dtype.kind == "U":
            debug_extra[key] = _resample_string_array(np.asarray(value), new_length)
    reward = np.zeros(new_length, dtype=np.float32)
    reward[-1] = float(episode.reward[-1]) if episode.reward.size else 0.0
    done = np.zeros(new_length, dtype=np.bool_)
    done[-1] = True
    duration = float(episode.timestamps[-1]) if episode.timestamps.size else float(max(new_length - 1, 0))
    timestamps = np.linspace(0.0, duration, num=new_length, dtype=np.float64)
    return EpisodeData(
        obs=linear_resample_array(episode.obs, new_length).astype(np.float32),
        action=linear_resample_array(episode.action, new_length).astype(np.float32),
        reward=reward,
        done=done,
        success=bool(episode.success),
        timestamps=timestamps,
        cube_pose=linear_resample_array(episode.cube_pose, new_length).astype(np.float32),
        target_pose=linear_resample_array(episode.target_pose, new_length).astype(np.float32),
        ee_pose=linear_resample_array(episode.ee_pose, new_length).astype(np.float32),
        right_joint_pos=linear_resample_array(episode.right_joint_pos, new_length).astype(np.float32),
        right_joint_vel=linear_resample_array(episode.right_joint_vel, new_length).astype(np.float32),
        left_joint_pos=linear_resample_array(episode.left_joint_pos, new_length).astype(np.float32),
        gripper_qpos=linear_resample_array(episode.gripper_qpos, new_length).astype(np.float32),
        key_pressed=_resample_string_array(episode.key_pressed, new_length),
        debug_extra=debug_extra,
    )


def _augment_episode(
    episode: EpisodeData,
    metadata: dict[str, object],
    rng: np.random.Generator,
    args,
) -> tuple[EpisodeData, dict[str, object]]:
    time_scale = float(rng.uniform(float(args.time_scale_min), float(args.time_scale_max)))
    new_length = max(16, int(round(episode.obs.shape[0] * time_scale)))
    resampled = _resample_episode(episode, new_length)
    table_bounds = parse_table_bounds(metadata)
    shift_xy = _clip_shift(
        [resampled.cube_pose[:, :2], resampled.ee_pose[:, :2], resampled.target_pose[:, :2]],
        table_bounds,
        float(args.translation_range),
        rng,
    )
    cube_pose = np.array(resampled.cube_pose, dtype=np.float64)
    ee_pose = np.array(resampled.ee_pose, dtype=np.float64)
    target_pose = np.array(resampled.target_pose, dtype=np.float64)
    cube_pose[:, :2] += shift_xy
    ee_pose[:, :2] += shift_xy
    target_pose[:, :2] += shift_xy
    cube_pose[:, :2] += _bounded_noise(cube_pose[:, :2].shape, float(args.cube_noise), rng)
    ee_pose[:, :2] += _bounded_noise(ee_pose[:, :2].shape, float(args.ee_noise), rng)
    cube_pose[:, 0] = np.clip(cube_pose[:, 0], table_bounds["x_min"], table_bounds["x_max"])
    cube_pose[:, 1] = np.clip(cube_pose[:, 1], table_bounds["y_min"], table_bounds["y_max"])
    ee_pose[:, 0] = np.clip(ee_pose[:, 0], table_bounds["x_min"], table_bounds["x_max"])
    ee_pose[:, 1] = np.clip(ee_pose[:, 1], table_bounds["y_min"], table_bounds["y_max"])
    target_pose[:, 0] = np.clip(target_pose[:, 0], table_bounds["x_min"], table_bounds["x_max"])
    target_pose[:, 1] = np.clip(target_pose[:, 1], table_bounds["y_min"], table_bounds["y_max"])
    target_size_xy = np.asarray(resampled.obs[-1, 7:9], dtype=np.float64)
    obs = _rebuild_obs(ee_pose, cube_pose, target_pose, target_size_xy)
    action_scale = float(metadata.get("action_scale", 0.01))
    action, raw_planar_delta_xy = _recompute_action(ee_pose, action_scale, float(args.action_noise), rng)
    debug_extra = dict(resampled.debug_extra or {})
    debug_extra["raw_planar_delta_xy"] = raw_planar_delta_xy
    debug_extra["source_resampled_only"] = np.ones((new_length, 1), dtype=np.int8)
    augmented = EpisodeData(
        obs=obs,
        action=action,
        reward=resampled.reward,
        done=resampled.done,
        success=bool(resampled.success),
        timestamps=resampled.timestamps,
        cube_pose=cube_pose.astype(np.float32),
        target_pose=target_pose.astype(np.float32),
        ee_pose=ee_pose.astype(np.float32),
        right_joint_pos=resampled.right_joint_pos.astype(np.float32),
        right_joint_vel=resampled.right_joint_vel.astype(np.float32),
        left_joint_pos=resampled.left_joint_pos.astype(np.float32),
        gripper_qpos=resampled.gripper_qpos.astype(np.float32),
        key_pressed=resampled.key_pressed.astype(str),
        debug_extra=debug_extra,
    )
    params = {
        "translation_xy": shift_xy.round(8).tolist(),
        "time_scale": round(time_scale, 8),
        "cube_noise_max_abs": float(args.cube_noise),
        "ee_noise_max_abs": float(args.ee_noise),
        "action_noise_max_abs": float(args.action_noise),
        "mirror": False,
    }
    return augmented, params


def _write_episode_attrs(h5py, output_path: Path, episode_name: str, attrs: dict[str, object]) -> None:
    with h5py.File(output_path, "a") as handle:
        group = handle["data"][episode_name]
        for key, value in attrs.items():
            if isinstance(value, (dict, list, tuple)):
                group.attrs[key] = json.dumps(value, sort_keys=True)
            else:
                group.attrs[key] = value


def _plot_augmented_comparison(metadata, original_episodes, augmented_episodes, output_dir: Path) -> None:
    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")
    output_dir.mkdir(parents=True, exist_ok=True)
    bounds = parse_table_bounds(metadata)
    fig, ax = plt.subplots(figsize=(8.0, 6.5))
    ax.add_patch(
        plt.Rectangle(
            (bounds["x_min"], bounds["y_min"]),
            bounds["x_max"] - bounds["x_min"],
            bounds["y_max"] - bounds["y_min"],
            fill=False,
            linewidth=2.0,
            color="black",
        )
    )
    for _, episode in original_episodes:
        ax.plot(episode.cube_pose[:, 0], episode.cube_pose[:, 1], color="#4c72b0", alpha=0.8, linewidth=1.4)
    for _, episode in augmented_episodes:
        ax.plot(episode.cube_pose[:, 0], episode.cube_pose[:, 1], color="#dd8452", alpha=0.18, linewidth=1.0)
    ax.set_title("Original vs Augmented Cube Trajectories")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "overlay_original_vs_augmented.png", dpi=240, bbox_inches="tight")
    plt.close(fig)

    original_summary, original_ep_summaries = summarize_dataset(metadata, original_episodes)
    augmented_summary, augmented_ep_summaries = summarize_dataset(metadata, augmented_episodes)
    metric_specs = [
        ("episode_length", np.array([s.length for s in original_ep_summaries]), np.array([s.length for s in augmented_ep_summaries]), "episode length", "distribution_episode_length.png"),
        ("final_distance", np.array([s.final_cube_to_target_distance for s in original_ep_summaries]), np.array([s.final_cube_to_target_distance for s in augmented_ep_summaries]), "final cube-target distance (m)", "distribution_final_distance.png"),
        ("cube_motion", np.array([s.cube_total_motion for s in original_ep_summaries]), np.array([s.cube_total_motion for s in augmented_ep_summaries]), "cube total motion (m)", "distribution_cube_motion.png"),
    ]
    original_actions = np.concatenate([episode.action for _, episode in original_episodes], axis=0).reshape(-1)
    augmented_actions = np.concatenate([episode.action for _, episode in augmented_episodes], axis=0).reshape(-1)
    metric_specs.append(("action", original_actions, augmented_actions, "normalized action", "distribution_action.png"))
    for _, original_values, augmented_values, xlabel, filename in metric_specs:
        fig, ax = plt.subplots(figsize=(6.4, 4.6))
        bins = min(30, max(10, len(augmented_values) // 2))
        ax.hist(original_values, bins=bins, alpha=0.65, color="#4c72b0", label="original", density=True)
        ax.hist(augmented_values, bins=bins, alpha=0.55, color="#dd8452", label="augmented", density=True)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("density")
        ax.legend()
        ax.grid(True, alpha=0.25)
        fig.tight_layout()
        fig.savefig(output_dir / filename, dpi=240, bbox_inches="tight")
        plt.close(fig)


def main() -> None:
    args = build_arg_parser().parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)
    if output_path.exists():
        raise RuntimeError(f"Refusing to overwrite existing file: {output_path}")
    metadata, episodes = load_all_episodes(input_path)
    if not episodes:
        raise RuntimeError("Dataset contains no demos.")
    target_count = int(args.target_count)
    if target_count < len(episodes):
        raise RuntimeError(f"target-count={target_count} is smaller than the number of real demos ({len(episodes)}).")
    rng = np.random.default_rng(int(args.seed))
    augmented_needed = target_count - len(episodes)
    output_metadata = {
        **metadata,
        "created_at": created_at_timestamp(),
        "source_dataset": str(input_path),
        "augmented": True,
        "target_count": target_count,
        "real_demo_count": len(episodes),
        "augmented_demo_count": augmented_needed,
        "mirror": False,
        "action_scale": float(metadata.get("action_scale", 0.01)),
    }
    writer = PushCubeWriter(output_path, output_metadata)
    h5py = optional_import("h5py", "pip install h5py")

    written_episodes: list[tuple[str, EpisodeData]] = []
    for episode_index, (_, episode) in enumerate(episodes):
        episode_name = writer.write_episode(episode)
        _write_episode_attrs(
            h5py,
            output_path,
            episode_name,
            {
                "augmented": False,
                "source_demo_id": episode_index,
                "augmentation_type": "original",
                "augmentation_params": {},
            },
        )
        written_episodes.append((episode_name, episode))

    for augmented_index in range(augmented_needed):
        source_index = augmented_index % len(episodes)
        _, source_episode = episodes[source_index]
        augmented_episode, params = _augment_episode(source_episode, metadata, rng, args)
        episode_name = writer.write_episode(augmented_episode)
        _write_episode_attrs(
            h5py,
            output_path,
            episode_name,
            {
                "augmented": True,
                "source_demo_id": source_index,
                "augmentation_type": "translate+time_resample+noise",
                "augmentation_params": params,
                "action_scale": float(metadata.get("action_scale", 0.01)),
            },
        )
        written_episodes.append((episode_name, augmented_episode))
        print(f"[{augmented_index + 1}/{augmented_needed}] wrote {episode_name} from source demo {source_index}")

    plots_dir = Path(args.plots_dir) if args.plots_dir else (Path("plots") / output_path.stem)
    _plot_augmented_comparison(metadata, episodes, written_episodes, plots_dir)
    print(f"wrote {output_path}")
    print(f"plots saved to {plots_dir}")


if __name__ == "__main__":
    main()
