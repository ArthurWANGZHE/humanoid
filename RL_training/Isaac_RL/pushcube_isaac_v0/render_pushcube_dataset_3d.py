"""Replay a PushCube HDF5 episode in Isaac Sim and render real 3D video."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.check_pose_scene import create_check_pose_pushcube_scene, get_gripper_joint_names, get_left_arm_joint_names, get_right_arm_joint_names  # noqa: E402
from pushcube_isaac_v0.isaac_lifecycle import close_simulation_app  # noqa: E402
from pushcube_isaac_v0.isaac_render_utils import CAMERA_PRESET_NAMES, capture_viewport_rgb, configure_active_viewport_camera, write_mp4  # noqa: E402
from pushcube_isaac_v0.manual_ee_teleop import _apply_full_target, _apply_initial_pose_preset, _apply_table_layout_override, _compose_full_target, _joint_indices, _read_joint_positions, _resolve_layout_override, _right_joint_limits  # noqa: E402
from pushcube_isaac_v0.pushcube_dataset_utils import load_all_episodes, optional_import  # noqa: E402
from rl_train.check_pose import _parse_bool_arg  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay a PushCube dataset episode and render IsaacSim video.")
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--episode", type=int, default=None)
    parser.add_argument("--episodes", type=int, nargs="*", default=None)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--output-dir", type=str, default=None)
    parser.add_argument("--layout-preset", type=str, default="opposite_edges")
    parser.add_argument("--target-size", type=float, default=0.22)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--camera", choices=CAMERA_PRESET_NAMES, default="presentation")
    parser.add_argument("--headless", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--save-frames", action="store_true")
    parser.add_argument("--initial-pose-preset", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--expand-right-arm-limits", type=float, default=0.5)
    parser.add_argument("--video-resolution", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"), default=(1280, 720))
    return parser


def _xyzw_to_wxyz(quat_xyzw: np.ndarray) -> np.ndarray:
    quat_xyzw = np.asarray(quat_xyzw, dtype=np.float64)
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64)


def _set_target_visuals(scene, target_pose: np.ndarray, target_size_xy: np.ndarray) -> None:
    target_path = scene.layout_visual_prim_paths.get("target")
    if not target_path:
        return
    prim = scene._get_prim_at_path(target_path)
    if prim is None or not prim.IsValid():
        return
    xform_api = scene._UsdGeom.XformCommonAPI(prim)
    xform_api.SetTranslate(tuple(float(value) for value in target_pose[:3]))
    xform_api.SetScale((float(target_size_xy[0]), float(target_size_xy[1]), 0.002))


def _render_episode(scene_bundle, episode_name: str, episode, args, output_path: Path, frame_dir: Path | None) -> None:
    scene = scene_bundle.scene
    resolved_preset, resolved_table_center_xy, resolved_table_top_z, _ = _resolve_layout_override(args, scene)
    actual_table_top_z = _apply_table_layout_override(
        scene_bundle,
        layout_preset=resolved_preset,
        table_center_xy=resolved_table_center_xy,
        table_top_z=resolved_table_top_z,
        target_size_xy=np.array([float(args.target_size), float(args.target_size)], dtype=np.float64),
    )
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
    fixed_left_arm_qpos = episode.left_joint_pos[0] if episode.left_joint_pos.size else _read_joint_positions(scene, left_indices)
    fixed_gripper_qpos = episode.gripper_qpos[0] if episode.gripper_qpos.size else _read_joint_positions(scene, gripper_indices)
    # Apply the same initial pose preset used during data collection
    lower_limits, upper_limits = _right_joint_limits(scene_bundle, right_joint_names, expand_by=float(args.expand_right_arm_limits))
    _apply_initial_pose_preset(
        scene,
        preset_id=int(args.initial_pose_preset),
        right_joint_names=right_joint_names,
        right_indices=right_indices,
        left_indices=left_indices,
        left_initial=np.asarray(fixed_left_arm_qpos, dtype=np.float64),
        gripper_indices=gripper_indices,
        gripper_initial=np.asarray(fixed_gripper_qpos, dtype=np.float64),
        initial_full_positions=initial_full_positions,
        lower_limits=lower_limits,
        upper_limits=upper_limits,
        max_joint_delta=0.005,
    )
    frames: list[np.ndarray] = []
    imageio = optional_import("imageio.v2", "pip install imageio")
    if frame_dir is not None:
        frame_dir.mkdir(parents=True, exist_ok=True)
    for step in range(episode.obs.shape[0]):
        right_target = episode.right_joint_pos[step] if episode.right_joint_pos.size else np.zeros(len(right_indices), dtype=np.float64)
        full_target = _compose_full_target(
            initial_full_positions=initial_full_positions,
            right_indices=right_indices,
            right_target=np.asarray(right_target, dtype=np.float64),
            left_indices=left_indices,
            left_initial=np.asarray(fixed_left_arm_qpos, dtype=np.float64),
            gripper_indices=gripper_indices,
            gripper_initial=np.asarray(fixed_gripper_qpos, dtype=np.float64),
        )
        _apply_full_target(scene, full_target)
        scene.brick.set_world_pose(
            position=np.asarray(episode.cube_pose[step, :3], dtype=np.float32),
            orientation=np.asarray(_xyzw_to_wxyz(episode.cube_pose[step, 3:7]), dtype=np.float32),
        )
        scene.brick.set_linear_velocity(np.zeros(3, dtype=np.float32))
        scene.brick.set_angular_velocity(np.zeros(3, dtype=np.float32))
        target_pose = np.asarray(episode.target_pose[step], dtype=np.float64)
        _set_target_visuals(scene, target_pose, np.asarray(episode.obs[step, 7:9], dtype=np.float64))
        scene.step_world(steps=1)
        frame = capture_viewport_rgb(viewport_api)
        frames.append(frame)
        if frame_dir is not None:
            imageio.imwrite(frame_dir / f"frame_{step:06d}.png", frame)
    write_mp4(output_path, frames, fps=int(args.fps))
    print(f"rendered {episode_name} -> {output_path}")


def main() -> None:
    args = build_arg_parser().parse_args()
    metadata, episodes = load_all_episodes(args.input)
    if args.episode is not None:
        episode_indices = [int(args.episode)]
    elif args.episodes:
        episode_indices = [int(index) for index in args.episodes]
    else:
        raise RuntimeError("Provide either --episode or --episodes.")
    if args.output and len(episode_indices) != 1:
        raise RuntimeError("--output is only valid for a single episode.")
    if not args.output and not args.output_dir:
        raise RuntimeError("Provide --output for single-episode mode or --output-dir for batch mode.")

    scene_bundle = None
    try:
        scene_bundle = create_check_pose_pushcube_scene(headless=args.headless, show_ranges=False, disable_lula=True, hold_open=False)
        for episode_index in episode_indices:
            episode_name, episode = episodes[episode_index]
            if args.output:
                output_path = Path(args.output)
                frame_dir = output_path.with_suffix("") if args.save_frames else None
            else:
                output_dir = Path(args.output_dir)
                output_dir.mkdir(parents=True, exist_ok=True)
                output_path = output_dir / f"{episode_name}_{args.camera}.mp4"
                frame_dir = (output_dir / f"{episode_name}_frames") if args.save_frames else None
            _render_episode(scene_bundle, episode_name, episode, args, output_path, frame_dir)
    finally:
        if scene_bundle is not None:
            scene_bundle.scene.close()
            close_simulation_app(scene_bundle.scene._app)


if __name__ == "__main__":
    main()

