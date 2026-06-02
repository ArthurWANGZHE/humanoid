"""Evaluate a trained PushCube policy in Isaac Sim."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.check_pose_scene import (  # noqa: E402
    create_check_pose_pushcube_scene,
    get_gripper_joint_names,
    get_left_arm_joint_names,
    get_right_arm_joint_names,
)
from pushcube_isaac_v0.isaac_lifecycle import close_simulation_app  # noqa: E402
from pushcube_isaac_v0.isaac_render_utils import CAMERA_PRESET_NAMES, capture_viewport_rgb, configure_active_viewport_camera, write_mp4  # noqa: E402
from pushcube_isaac_v0.manual_ee_teleop import (  # noqa: E402
    LAYOUT_PRESET_DEFAULT,
    LAYOUT_PRESET_OPPOSITE_EDGES,
    LAYOUT_PRESET_RIGHT_ARM_CLOSE,
    LAYOUT_PRESET_RIGHT_ARM_VERY_CLOSE,
    _apply_full_target,
    _apply_initial_pose_preset,
    _apply_table_layout_override,
    _apply_warning_filters,
    _clamp_desired_ee_position,
    _compose_full_target,
    _cube_and_target_xy,
    _cube_inside_target,
    _enforce_fixed_subsets,
    _filter_joint_target,
    _joint_indices,
    _read_joint_positions,
    _read_joint_velocities,
    _reset_scene,
    _resolve_layout_override,
    _right_joint_limits,
)
from pushcube_isaac_v0.pushcube_dataset_stats import summarize_dataset  # noqa: E402
from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    ACTION_DIM,
    EpisodeData,
    PushCubeWriter,
    build_low_dim_observation,
    created_at_timestamp,
    cube_yaw_from_rotation,
    ensure_h5py,
    load_all_episodes,
    optional_import,
    pose_to_vec7,
)
from rl_train.check_pose import _parse_bool_arg  # noqa: E402
from rl_train.pose_math import Pose  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate a PushCube policy.")
    parser.add_argument("--checkpoint", type=str, required=True)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--headless", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--show-ranges", action="store_true")
    parser.add_argument("--disable-lula", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    parser.add_argument(
        "--layout-preset",
        choices=[LAYOUT_PRESET_DEFAULT, LAYOUT_PRESET_RIGHT_ARM_CLOSE, LAYOUT_PRESET_RIGHT_ARM_VERY_CLOSE, LAYOUT_PRESET_OPPOSITE_EDGES],
        default=LAYOUT_PRESET_OPPOSITE_EDGES,
    )
    parser.add_argument("--table-center-x", type=float, default=None)
    parser.add_argument("--table-center-y", type=float, default=None)
    parser.add_argument("--table-top-z", type=float, default=None)
    parser.add_argument("--alpha", type=float, default=0.2)
    parser.add_argument("--joint-alpha", type=float, default=0.2)
    parser.add_argument("--max-joint-delta", type=float, default=0.005)
    parser.add_argument("--expand-right-arm-limits", type=float, default=0.5)
    parser.add_argument("--initial-pose-preset", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--target-size", type=float, default=0.22)
    parser.add_argument("--success-margin", type=float, default=0.02)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--output-dir", type=str, default=None)
    parser.add_argument("--save-video", action="store_true")
    parser.add_argument("--camera", choices=CAMERA_PRESET_NAMES, default="presentation")
    parser.add_argument("--video-fps", type=int, default=20)
    parser.add_argument("--video-resolution", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"), default=(1280, 720))
    return parser


def _episode_from_rollout(rollout: dict[str, list[np.ndarray] | list[float] | list[str]], success: bool) -> EpisodeData:
    count = len(rollout["obs"])
    reward = np.zeros(count, dtype=np.float32)
    reward[-1] = 1.0 if success else 0.0
    done = np.zeros(count, dtype=np.bool_)
    done[-1] = True
    return EpisodeData(
        obs=np.stack(rollout["obs"], axis=0),
        action=np.stack(rollout["action"], axis=0),
        reward=reward,
        done=done,
        success=success,
        timestamps=np.array(rollout["timestamps"], dtype=np.float64),
        cube_pose=np.stack(rollout["cube_pose"], axis=0),
        target_pose=np.stack(rollout["target_pose"], axis=0),
        ee_pose=np.stack(rollout["ee_pose"], axis=0),
        right_joint_pos=np.stack(rollout["right_joint_pos"], axis=0),
        right_joint_vel=np.stack(rollout["right_joint_vel"], axis=0),
        left_joint_pos=np.stack(rollout["left_joint_pos"], axis=0),
        gripper_qpos=np.stack(rollout["gripper_qpos"], axis=0),
        key_pressed=np.array(rollout["key_pressed"], dtype=str),
        debug_extra={"policy_action_xy": np.stack(rollout["policy_action_xy"], axis=0)},
    )


def _save_eval_plots(output_dir: Path) -> None:
    metadata, episodes = load_all_episodes(output_dir / "eval_rollouts.hdf5")
    summary, _ = summarize_dataset(metadata, episodes)
    matplotlib = optional_import("matplotlib", "pip install matplotlib")
    matplotlib.use("Agg")
    plt = optional_import("matplotlib.pyplot", "pip install matplotlib")
    fig, ax = plt.subplots(figsize=(7.6, 6.2))
    for _, episode in episodes:
        cube_xy = episode.cube_pose[:, :2]
        ax.plot(cube_xy[:, 0], cube_xy[:, 1], color="#4c72b0", alpha=0.45, linewidth=1.2)
    ax.set_title("Eval Rollout Cube Trajectory Overlay")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory_overlay.png", dpi=220, bbox_inches="tight")
    plt.close(fig)
    return summary


def main() -> None:
    args = build_arg_parser().parse_args()
    ensure_h5py()
    torch = optional_import("torch", "pip install torch")
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    action_scale = float(checkpoint.get("metadata", {}).get("action_scale", 0.01))
    output_dir = Path(args.output_dir) if args.output_dir else Path(args.checkpoint).resolve().parent / "eval"
    output_dir.mkdir(parents=True, exist_ok=True)
    _apply_warning_filters(False)

    class PolicyMLP(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.net = torch.nn.Sequential(
                torch.nn.Linear(13, 256),
                torch.nn.ReLU(),
                torch.nn.Linear(256, 256),
                torch.nn.ReLU(),
                torch.nn.Linear(256, 128),
                torch.nn.ReLU(),
                torch.nn.Linear(128, 2),
            )

        def forward(self, obs):
            return self.net(obs)

    model = PolicyMLP()
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    obs_mean = checkpoint["obs_mean"]
    obs_std = checkpoint["obs_std"]

    scene_bundle = None
    results: list[dict[str, object]] = []
    action_history: list[np.ndarray] = []
    writer: PushCubeWriter | None = None
    try:
        scene_bundle = create_check_pose_pushcube_scene(
            headless=args.headless,
            show_ranges=args.show_ranges,
            disable_lula=args.disable_lula,
            hold_open=args.hold_open,
        )
        scene = scene_bundle.scene
        resolved_preset, resolved_table_center_xy, resolved_table_top_z, _ = _resolve_layout_override(args, scene)
        actual_table_top_z = _apply_table_layout_override(
            scene_bundle,
            layout_preset=resolved_preset,
            table_center_xy=resolved_table_center_xy,
            table_top_z=resolved_table_top_z,
            target_size_xy=np.array([float(args.target_size), float(args.target_size)], dtype=np.float64),
        )
        viewport_api = None
        if args.save_video:
            viewport_api = configure_active_viewport_camera(
                args.camera,
                np.asarray(resolved_table_center_xy, dtype=np.float64),
                float(actual_table_top_z),
                resolution=(int(args.video_resolution[0]), int(args.video_resolution[1])),
            )

        right_joint_names = get_right_arm_joint_names(scene_bundle)
        left_joint_names = get_left_arm_joint_names(scene_bundle)
        gripper_joint_names = get_gripper_joint_names(scene_bundle)
        right_indices = _joint_indices(scene, right_joint_names)
        left_indices = _joint_indices(scene, left_joint_names)
        gripper_indices = _joint_indices(scene, gripper_joint_names)
        initial_full_positions = np.array(scene.articulation.get_joint_positions(), dtype=np.float64)
        initial_brick_position = np.array(scene.initial_brick_position, dtype=np.float64)
        fixed_left_arm_qpos = _read_joint_positions(scene, left_indices)
        fixed_gripper_qpos = _read_joint_positions(scene, gripper_indices)
        lower_limits, upper_limits = _right_joint_limits(scene_bundle, right_joint_names, expand_by=float(args.expand_right_arm_limits))
        writer = PushCubeWriter(
            output_dir / "eval_rollouts.hdf5",
            {
                "task_name": "push_cube_eval",
                "format": "push_cube_low_dim_v2",
                "control_mode": "ee_policy",
                "action_dim": ACTION_DIM,
                "obs_dim": 13,
                "target_size_xy": np.array([float(args.target_size), float(args.target_size)], dtype=np.float32),
                "layout_preset": str(args.layout_preset),
                "table_top_z": float(actual_table_top_z),
                "table_bounds": dict((scene.pushcube_layout or {}).get("table_bounds", {})),
                "control_hz": float(args.control_hz),
                "action_scale": action_scale,
                "gripper_fixed": True,
                "left_arm_fixed": True,
                "created_at": created_at_timestamp(),
                "success_only": False,
            },
        )
        physics_hz = 1.0 / float(scene.scene_config.physics_dt)
        control_interval_frames = max(1, int(round(physics_hz / float(args.control_hz))))
        for episode_index in range(int(args.episodes)):
            desired_ee_position, filtered_ee_position, desired_ee_rotation, right_joint_target = _reset_scene(
                scene,
                initial_full_positions=initial_full_positions,
                initial_brick_position=initial_brick_position,
                right_indices=right_indices,
                left_indices=left_indices,
                gripper_indices=gripper_indices,
            )
            desired_ee_position, filtered_ee_position, desired_ee_rotation, right_joint_target = _apply_initial_pose_preset(
                scene,
                preset_id=int(args.initial_pose_preset),
                right_joint_names=right_joint_names,
                right_indices=right_indices,
                left_indices=left_indices,
                left_initial=fixed_left_arm_qpos,
                gripper_indices=gripper_indices,
                gripper_initial=fixed_gripper_qpos,
                initial_full_positions=initial_full_positions,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
                max_joint_delta=float(args.max_joint_delta),
            )
            rollout: dict[str, list[np.ndarray] | list[float] | list[str]] = {
                "obs": [],
                "action": [],
                "timestamps": [],
                "cube_pose": [],
                "target_pose": [],
                "ee_pose": [],
                "right_joint_pos": [],
                "right_joint_vel": [],
                "left_joint_pos": [],
                "gripper_qpos": [],
                "key_pressed": [],
                "policy_action_xy": [],
            }
            failure_reason = "timeout"
            success = False
            video_frames: list[np.ndarray] = []
            for step in range(int(args.max_steps)):
                ee_pose = scene.get_end_effector_pose()
                cube_pose = scene.get_brick_pose()
                target_center = np.array((scene.pushcube_layout or {}).get("target_center", [0.0, 0.0, 0.0]), dtype=np.float64)
                target_size_xy = np.array((scene.pushcube_layout or {}).get("target_size", [args.target_size, args.target_size, 0.002]), dtype=np.float64)[:2]
                observation = build_low_dim_observation(
                    ee_xy=np.array(ee_pose.position[:2], dtype=np.float64),
                    cube_xy=np.array(cube_pose.position[:2], dtype=np.float64),
                    cube_yaw=cube_yaw_from_rotation(np.array(cube_pose.rotation, dtype=np.float64)),
                    target_xy=np.array(target_center[:2], dtype=np.float64),
                    target_size_xy=target_size_xy,
                )
                with torch.no_grad():
                    obs_tensor = torch.tensor(observation, dtype=torch.float32).unsqueeze(0)
                    normalized = (obs_tensor - obs_mean) / obs_std
                    action = model(normalized).squeeze(0).cpu().numpy()
                if not np.all(np.isfinite(action)):
                    failure_reason = "invalid_action"
                    break
                action = np.clip(np.array(action, dtype=np.float64), -1.0, 1.0)
                action_history.append(action[:2].copy())
                desired_ee_position[:2] = desired_ee_position[:2] + (action[:2] * action_scale)
                desired_ee_position = _clamp_desired_ee_position(desired_ee_position, actual_table_top_z)
                filtered_ee_position = (float(args.alpha) * desired_ee_position) + ((1.0 - float(args.alpha)) * filtered_ee_position)
                desired_pose = Pose(position=np.array(filtered_ee_position, dtype=np.float64), rotation=np.array(desired_ee_rotation, dtype=np.float64))
                ik_solution, ik_success = scene.solve_ik(desired_pose, warm_start=right_joint_target, position_tolerance=0.01, orientation_tolerance=0.15)
                if not ik_success or ik_solution is None or not np.all(np.isfinite(ik_solution)):
                    failure_reason = "ik_failure"
                    break
                desired_right_joint_target = np.clip(np.array(ik_solution, dtype=np.float64), lower_limits, upper_limits)
                right_joint_target = _filter_joint_target(
                    current_command=right_joint_target,
                    desired_target=desired_right_joint_target,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    joint_alpha=float(args.joint_alpha),
                    max_joint_delta=float(args.max_joint_delta),
                )
                full_target = _compose_full_target(
                    initial_full_positions=initial_full_positions,
                    right_indices=right_indices,
                    right_target=right_joint_target,
                    left_indices=left_indices,
                    left_initial=fixed_left_arm_qpos,
                    gripper_indices=gripper_indices,
                    gripper_initial=fixed_gripper_qpos,
                )
                for _ in range(control_interval_frames):
                    _apply_full_target(scene, full_target)
                    scene.step_world(steps=1)
                    _enforce_fixed_subsets(
                        scene,
                        left_indices=left_indices,
                        left_initial=fixed_left_arm_qpos,
                        gripper_indices=gripper_indices,
                        gripper_initial=fixed_gripper_qpos,
                        frame=step,
                        warning_state={},
                        quiet_warnings=True,
                    )
                ee_pose = scene.get_end_effector_pose()
                cube_pose = scene.get_brick_pose()
                observation_after = build_low_dim_observation(
                    ee_xy=np.array(ee_pose.position[:2], dtype=np.float64),
                    cube_xy=np.array(cube_pose.position[:2], dtype=np.float64),
                    cube_yaw=cube_yaw_from_rotation(np.array(cube_pose.rotation, dtype=np.float64)),
                    target_xy=np.array(target_center[:2], dtype=np.float64),
                    target_size_xy=target_size_xy,
                )
                rollout["obs"].append(observation_after.astype(np.float32))
                rollout["action"].append(np.array(action[:2], dtype=np.float32))
                rollout["timestamps"].append(float(step / max(float(args.control_hz), 1e-6)))
                rollout["cube_pose"].append(pose_to_vec7(np.array(cube_pose.position, dtype=np.float64), np.array(cube_pose.rotation, dtype=np.float64)))
                rollout["target_pose"].append(pose_to_vec7(target_center, None))
                rollout["ee_pose"].append(pose_to_vec7(np.array(ee_pose.position, dtype=np.float64), np.array(ee_pose.rotation, dtype=np.float64)))
                rollout["right_joint_pos"].append(_read_joint_positions(scene, right_indices).astype(np.float32))
                rollout["right_joint_vel"].append(_read_joint_velocities(scene, right_indices).astype(np.float32))
                rollout["left_joint_pos"].append(_read_joint_positions(scene, left_indices).astype(np.float32))
                rollout["gripper_qpos"].append(_read_joint_positions(scene, gripper_indices).astype(np.float32))
                rollout["key_pressed"].append("policy")
                rollout["policy_action_xy"].append(np.array(action[:2], dtype=np.float32))
                if viewport_api is not None:
                    video_frames.append(capture_viewport_rgb(viewport_api))
                if _cube_inside_target(scene, float(args.success_margin)):
                    success = True
                    failure_reason = ""
                    break
            if not rollout["obs"]:
                raise RuntimeError("Policy episode produced no rollout samples.")
            episode = _episode_from_rollout(rollout, success)
            episode_name = writer.write_episode(episode)
            cube_xy, target_xy, _ = _cube_and_target_xy(scene)
            results.append(
                {
                    "episode_index": episode_index,
                    "episode_name": episode_name,
                    "success": success,
                    "final_distance": float(np.linalg.norm(target_xy - cube_xy)),
                    "episode_length": int(len(rollout["obs"])),
                    "cube_motion": float(np.max(np.linalg.norm(np.stack(rollout["cube_pose"], axis=0)[:, :2] - np.stack(rollout["cube_pose"], axis=0)[0, :2], axis=1))),
                    "failure_reason": failure_reason or None,
                }
            )
            if args.save_video and video_frames:
                write_mp4(output_dir / "videos" / f"{episode_name}.mp4", video_frames, fps=int(args.video_fps))
            print(f"episode={episode_index} success={success} failure_reason={failure_reason or 'none'}")

        failure_reasons: dict[str, int] = {}
        for entry in results:
            reason = entry["failure_reason"]
            if reason:
                failure_reasons[str(reason)] = failure_reasons.get(str(reason), 0) + 1
        final_distances = np.array([entry["final_distance"] for entry in results], dtype=np.float64)
        episode_lengths = np.array([entry["episode_length"] for entry in results], dtype=np.float64)
        cube_motion = np.array([entry["cube_motion"] for entry in results], dtype=np.float64)
        action_array = np.stack(action_history, axis=0) if action_history else np.zeros((0, 2), dtype=np.float64)
        summary = {
            "checkpoint": str(args.checkpoint),
            "episodes": int(args.episodes),
            "success_rate": float(np.mean([entry["success"] for entry in results])),
            "mean_final_distance": float(np.mean(final_distances)),
            "median_final_distance": float(np.median(final_distances)),
            "mean_cube_motion": float(np.mean(cube_motion)),
            "mean_episode_length": float(np.mean(episode_lengths)),
            "failure_reasons": failure_reasons,
            "action_statistics": {
                "min": np.min(action_array, axis=0).round(8).tolist() if action_array.size else [0.0, 0.0],
                "max": np.max(action_array, axis=0).round(8).tolist() if action_array.size else [0.0, 0.0],
                "mean": np.mean(action_array, axis=0).round(8).tolist() if action_array.size else [0.0, 0.0],
                "std": np.std(action_array, axis=0).round(8).tolist() if action_array.size else [0.0, 0.0],
            },
            "per_episode": results,
        }
        with (output_dir / "eval_results.json").open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
        with (output_dir / "eval_summary.csv").open("w", encoding="utf-8", newline="") as handle:
            writer_csv = csv.DictWriter(handle, fieldnames=["episode_index", "episode_name", "success", "final_distance", "episode_length", "cube_motion", "failure_reason"])
            writer_csv.writeheader()
            writer_csv.writerows(results)
        _save_eval_plots(output_dir)
    finally:
        if scene_bundle is not None:
            scene_bundle.scene.close()
            close_simulation_app(scene_bundle.scene._app)


if __name__ == "__main__":
    main()

