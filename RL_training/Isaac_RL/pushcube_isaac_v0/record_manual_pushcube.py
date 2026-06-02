"""Record manual PushCube demonstrations into an HDF5 dataset."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import sys
import time

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
from pushcube_isaac_v0.manual_ee_teleop import (  # noqa: E402
    LAYOUT_PRESET_DEFAULT,
    LAYOUT_PRESET_OPPOSITE_EDGES,
    LAYOUT_PRESET_RIGHT_ARM_CLOSE,
    LAYOUT_PRESET_RIGHT_ARM_VERY_CLOSE,
    STATUS_INTERVAL_FRAMES,
    FIXED_SUBSET_RESTORE_DELTA,
    KEY_DELTAS,
    IK_WARNING_INTERVAL_FRAMES,
    LEFT_ARM_WARNING_INTERVAL_FRAMES,
    GRIPPER_WARNING_INTERVAL_FRAMES,
    TeleopState,
    TerminalKeyReader,
    ViewerKeyReader,
    _apply_full_target,
    _apply_initial_pose_preset,
    _apply_table_layout_override,
    _apply_warning_filters,
    _clamp_desired_ee_position,
    _compose_full_target,
    _cube_and_target_xy,
    _emit_warning,
    _enforce_fixed_subsets,
    _filter_joint_target,
    _joint_indices,
    _print_periodic_status,
    _print_state,
    _read_joint_positions,
    _read_joint_velocities,
    _reset_scene,
    _resolve_input_backend,
    _resolve_layout_override,
    _right_joint_limits,
)
from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    ACTION_DIM,
    DEFAULT_ACTION_SCALE,
    OBS_DIM,
    EpisodeData,
    PushCubeWriter,
    build_low_dim_observation,
    created_at_timestamp,
    cube_yaw_from_rotation,
    ensure_h5py,
    normalize_action,
    pose_to_vec7,
)
from rl_train.check_pose import _parse_bool_arg, _print_layout_logs  # noqa: E402
from rl_train.pose_math import Pose  # noqa: E402


@dataclass
class EpisodeBuffer:
    action_scale: float
    start_time: float | None = None
    obs: list[np.ndarray] = field(default_factory=list)
    ee_pose: list[np.ndarray] = field(default_factory=list)
    cube_pose: list[np.ndarray] = field(default_factory=list)
    target_pose: list[np.ndarray] = field(default_factory=list)
    right_joint_pos: list[np.ndarray] = field(default_factory=list)
    right_joint_vel: list[np.ndarray] = field(default_factory=list)
    left_joint_pos: list[np.ndarray] = field(default_factory=list)
    gripper_qpos: list[np.ndarray] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)
    key_pressed: list[str] = field(default_factory=list)
    raw_planar_delta_xy: list[np.ndarray] = field(default_factory=list)
    raw_joint_target: list[np.ndarray] = field(default_factory=list)

    def reset(self) -> None:
        self.start_time = None
        self.obs.clear()
        self.ee_pose.clear()
        self.cube_pose.clear()
        self.target_pose.clear()
        self.right_joint_pos.clear()
        self.right_joint_vel.clear()
        self.left_joint_pos.clear()
        self.gripper_qpos.clear()
        self.timestamps.clear()
        self.key_pressed.clear()
        self.raw_planar_delta_xy.clear()
        self.raw_joint_target.clear()

    def append(
        self,
        *,
        obs: np.ndarray,
        ee_pose: np.ndarray,
        cube_pose: np.ndarray,
        target_pose: np.ndarray,
        right_joint_pos: np.ndarray,
        right_joint_vel: np.ndarray,
        left_joint_pos: np.ndarray,
        gripper_qpos: np.ndarray,
        key_pressed: str,
        raw_planar_delta_xy: np.ndarray,
        raw_joint_target: np.ndarray,
    ) -> None:
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        self.obs.append(np.array(obs, dtype=np.float32))
        self.ee_pose.append(np.array(ee_pose, dtype=np.float32))
        self.cube_pose.append(np.array(cube_pose, dtype=np.float32))
        self.target_pose.append(np.array(target_pose, dtype=np.float32))
        self.right_joint_pos.append(np.array(right_joint_pos, dtype=np.float32))
        self.right_joint_vel.append(np.array(right_joint_vel, dtype=np.float32))
        self.left_joint_pos.append(np.array(left_joint_pos, dtype=np.float32))
        self.gripper_qpos.append(np.array(gripper_qpos, dtype=np.float32))
        self.timestamps.append(float(now - self.start_time))
        self.key_pressed.append(str(key_pressed))
        self.raw_planar_delta_xy.append(np.array(raw_planar_delta_xy, dtype=np.float32))
        self.raw_joint_target.append(np.array(raw_joint_target, dtype=np.float32))

    def finalize(self, success: bool) -> EpisodeData | None:
        if not self.obs:
            return None
        ee_array = np.stack(self.ee_pose, axis=0)
        planar_delta = np.zeros((len(self.obs), ACTION_DIM), dtype=np.float32)
        if ee_array.shape[0] > 1:
            planar_delta[:-1] = np.diff(ee_array[:, :2], axis=0).astype(np.float32)
        action = normalize_action(planar_delta, self.action_scale)
        reward = np.zeros(len(self.obs), dtype=np.float32)
        reward[-1] = 1.0 if success else 0.0
        done = np.zeros(len(self.obs), dtype=np.bool_)
        done[-1] = True
        episode = EpisodeData(
            obs=np.stack(self.obs, axis=0),
            action=action,
            reward=reward,
            done=done,
            success=success,
            timestamps=np.array(self.timestamps, dtype=np.float64),
            cube_pose=np.stack(self.cube_pose, axis=0),
            target_pose=np.stack(self.target_pose, axis=0),
            ee_pose=ee_array,
            right_joint_pos=np.stack(self.right_joint_pos, axis=0),
            right_joint_vel=np.stack(self.right_joint_vel, axis=0),
            left_joint_pos=np.stack(self.left_joint_pos, axis=0),
            gripper_qpos=np.stack(self.gripper_qpos, axis=0),
            key_pressed=np.array(self.key_pressed, dtype=str),
            debug_extra={
                "raw_planar_delta_xy": np.stack(self.raw_planar_delta_xy, axis=0),
                "raw_joint_target": np.stack(self.raw_joint_target, axis=0),
            },
        )
        self.reset()
        return episode


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record manual PushCube demonstrations.")
    parser.add_argument("--headless", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--show-ranges", action="store_true")
    parser.add_argument("--disable-lula", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    parser.add_argument(
        "--layout-preset",
        choices=[
            LAYOUT_PRESET_DEFAULT,
            LAYOUT_PRESET_RIGHT_ARM_CLOSE,
            LAYOUT_PRESET_RIGHT_ARM_VERY_CLOSE,
            LAYOUT_PRESET_OPPOSITE_EDGES,
        ],
        default=LAYOUT_PRESET_OPPOSITE_EDGES,
    )
    parser.add_argument("--table-center-x", type=float, default=None)
    parser.add_argument("--table-center-y", type=float, default=None)
    parser.add_argument("--table-top-z", type=float, default=None)
    parser.add_argument("--step-size", type=float, default=0.005)
    parser.add_argument("--joint-step", type=float, default=0.01)
    parser.add_argument("--alpha", type=float, default=0.2)
    parser.add_argument("--joint-alpha", type=float, default=0.2)
    parser.add_argument("--max-joint-delta", type=float, default=FIXED_SUBSET_RESTORE_DELTA)
    parser.add_argument("--expand-right-arm-limits", type=float, default=0.5)
    parser.add_argument("--initial-pose-preset", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--target-size", type=float, default=0.22)
    parser.add_argument("--success-margin", type=float, default=0.02)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--input-backend", choices=["terminal", "viewer", "auto", "line"], default="terminal")
    parser.add_argument("--control-mode", choices=["joint", "ee"], default="joint")
    parser.add_argument("--quiet-warnings", type=_parse_bool_arg, nargs="?", const=True, default=True)
    parser.add_argument("--verbose-warnings", action="store_true")
    parser.add_argument("--suppress-isaac-warnings", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--status-every", type=int, default=STATUS_INTERVAL_FRAMES)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--output", type=str, required=True)
    parser.add_argument("--num-episodes", type=int, default=20)
    parser.add_argument("--save-failed", action="store_true")
    parser.add_argument("--action-scale", type=float, default=DEFAULT_ACTION_SCALE)
    return parser


def _capture_frame(
    scene,
    *,
    ee_position: np.ndarray,
    right_indices: np.ndarray,
    left_indices: np.ndarray,
    gripper_indices: np.ndarray,
    right_joint_target: np.ndarray,
    last_key: str,
    raw_planar_delta_xy: np.ndarray,
    target_size_xy: np.ndarray,
) -> dict[str, np.ndarray | str]:
    cube_pose = scene.get_brick_pose()
    ee_pose = scene.get_end_effector_pose()
    cube_position = np.array(cube_pose.position, dtype=np.float64)
    target_center = np.array((scene.pushcube_layout or {}).get("target_center", [0.0, 0.0, 0.0]), dtype=np.float64)
    cube_xy = cube_position[:2]
    target_xy = target_center[:2]
    observation = build_low_dim_observation(
        ee_xy=np.array(ee_position[:2], dtype=np.float64),
        cube_xy=np.array(cube_xy, dtype=np.float64),
        cube_yaw=cube_yaw_from_rotation(np.array(cube_pose.rotation, dtype=np.float64)),
        target_xy=np.array(target_xy, dtype=np.float64),
        target_size_xy=np.array(target_size_xy, dtype=np.float64),
    )
    return {
        "obs": observation,
        "ee_pose": pose_to_vec7(np.array(ee_pose.position, dtype=np.float64), np.array(ee_pose.rotation, dtype=np.float64)),
        "cube_pose": pose_to_vec7(cube_position, np.array(cube_pose.rotation, dtype=np.float64)),
        "target_pose": pose_to_vec7(target_center, None),
        "right_joint_pos": _read_joint_positions(scene, right_indices),
        "right_joint_vel": _read_joint_velocities(scene, right_indices),
        "left_joint_pos": _read_joint_positions(scene, left_indices),
        "gripper_qpos": _read_joint_positions(scene, gripper_indices),
        "key_pressed": last_key,
        "raw_planar_delta_xy": np.array(raw_planar_delta_xy[:2], dtype=np.float32),
        "raw_joint_target": np.array(right_joint_target, dtype=np.float32),
    }


def _build_writer_metadata(args, scene, actual_table_top_z: float) -> dict[str, object]:
    layout = scene.pushcube_layout or {}
    return {
        "task_name": "push_cube_manual",
        "format": "push_cube_low_dim_v2",
        "control_mode": str(args.control_mode),
        "action_dim": ACTION_DIM,
        "obs_dim": OBS_DIM,
        "target_size_xy": np.array(layout.get("target_size", [args.target_size, args.target_size, 0.002]), dtype=np.float32)[:2],
        "layout_preset": str(args.layout_preset),
        "table_top_z": float(actual_table_top_z),
        "table_bounds": json_ready(dict(layout.get("table_bounds", {}))),
        "table_center_xy": np.array(layout.get("table_center_xy", [0.0, 0.0]), dtype=np.float32)[:2],
        "table_size_xy": np.array(layout.get("table_size_xy", [0.0, 0.0]), dtype=np.float32)[:2],
        "control_hz": float(args.control_hz),
        "action_scale": float(args.action_scale),
        "gripper_fixed": True,
        "left_arm_fixed": True,
        "created_at": created_at_timestamp(),
        "success_only": bool(not args.save_failed),
    }


def json_ready(value):
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def main() -> None:
    args = build_arg_parser().parse_args()
    ensure_h5py()
    quiet_warnings = bool(args.quiet_warnings) and not bool(args.verbose_warnings)
    status_every = max(1, int(args.status_every))
    _apply_warning_filters(bool(args.suppress_isaac_warnings))
    resolved_output = Path(args.output).expanduser().resolve()

    print(f"[record] cwd={Path.cwd()}")
    print(f"[record] output={args.output}")
    print(f"[record] output_resolved={resolved_output}")
    print(f"[record] num_episodes={args.num_episodes}")
    print(f"[record] control_mode={args.control_mode}")
    print(f"[record] input_backend={args.input_backend}")
    print(f"[record] save_failed={args.save_failed}")
    print(f"[record] action_scale={float(args.action_scale):.6f}")

    scene_bundle = None
    viewer_keys = None
    active_input_reader = None
    try:
        scene_bundle = create_check_pose_pushcube_scene(
            headless=args.headless,
            show_ranges=args.show_ranges,
            disable_lula=args.disable_lula,
            hold_open=args.hold_open,
        )
        scene = scene_bundle.scene
        viewer_keys = ViewerKeyReader()
        active_input_reader = _resolve_input_backend(args.input_backend, viewer_keys)
        resolved_preset, resolved_table_center_xy, resolved_table_top_z, _ = _resolve_layout_override(args, scene)
        actual_table_top_z = _apply_table_layout_override(
            scene_bundle,
            layout_preset=resolved_preset,
            table_center_xy=resolved_table_center_xy,
            table_top_z=resolved_table_top_z,
            target_size_xy=np.array([float(args.target_size), float(args.target_size)], dtype=np.float64),
        )
        _print_layout_logs(scene, scene_bundle.runtime_options)

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
        lower_limits, upper_limits = _right_joint_limits(
            scene_bundle,
            right_joint_names,
            expand_by=float(args.expand_right_arm_limits),
        )

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

        teleop_state = TeleopState(
            desired_ee_position=desired_ee_position,
            filtered_ee_position=filtered_ee_position,
            desired_ee_rotation=desired_ee_rotation,
            desired_right_joint_target=right_joint_target.copy(),
            right_joint_target=right_joint_target.copy(),
            paused=False,
            success_count=0,
            failure_count=0,
            exit_requested=False,
            status="running",
            ik_fail_count=0,
            last_ik_failure_frame=-30,
            last_ik_error="",
            contact_warning_count=0,
        )
        control_available = not (args.control_mode == "ee" and (args.disable_lula or scene.kinematics is None))
        if not control_available:
            raise RuntimeError("Cartesian control requires Lula/IK. Remove --disable-lula or use --control-mode joint.")

        target_size_xy = np.array((scene.pushcube_layout or {}).get("target_size", [args.target_size, args.target_size, 0.002]), dtype=np.float64)[:2]
        physics_hz = 1.0 / float(scene.scene_config.physics_dt)
        control_interval_frames = max(1, int(round(physics_hz / float(args.control_hz))))
        joint_key_map = {
            "1": ("right_base_pitch_joint", float(args.joint_step)),
            "q": ("right_base_pitch_joint", -float(args.joint_step)),
            "2": ("right_shoulder_yaw_joint", float(args.joint_step)),
            "w": ("right_shoulder_yaw_joint", -float(args.joint_step)),
            "3": ("right_elbow_pitch_joint", float(args.joint_step)),
            "e": ("right_elbow_pitch_joint", -float(args.joint_step)),
            "4": ("right_shoulder_roll_joint", float(args.joint_step)),
            "g": ("right_shoulder_roll_joint", -float(args.joint_step)),
            "5": ("right_wrist_pitch_joint", float(args.joint_step)),
            "t": ("right_wrist_pitch_joint", -float(args.joint_step)),
            "6": ("right_wrist_yaw_joint", float(args.joint_step)),
            "y": ("right_wrist_yaw_joint", -float(args.joint_step)),
        }
        right_joint_name_to_index = {joint_name: index for index, joint_name in enumerate(right_joint_names)}
        writer = PushCubeWriter(resolved_output, _build_writer_metadata(args, scene, actual_table_top_z))
        episode_buffer = EpisodeBuffer(action_scale=float(args.action_scale))
        warning_state: dict[str, dict[str, int]] = {}
        frame = 0
        saved_episodes = 0
        attempted_episodes = 0
        last_key = ""
        last_action_xyz = np.zeros(3, dtype=np.float64)
        sample_due = False

        print("[record] controls:")
        if args.control_mode == "ee":
            print("  q/e: X +/-")
            print("  a/d: Y +/-")
            print("  w/s: Z +/-")
        else:
            print("  1/q: right_base_pitch_joint +/-")
            print("  2/w: right_shoulder_yaw_joint +/-")
            print("  3/e: right_elbow_pitch_joint +/-")
            print("  4/g: right_shoulder_roll_joint +/-")
            print("  5/t: right_wrist_pitch_joint +/-")
            print("  6/y: right_wrist_yaw_joint +/-")
            print("  r: reset current episode without saving")
        print("  p: print current state")
        print("  x: mark success")
        print("  f: mark failure")
        print("  esc or Ctrl+C: exit safely")

        def reset_episode() -> None:
            nonlocal desired_ee_position
            nonlocal filtered_ee_position
            nonlocal desired_ee_rotation
            nonlocal right_joint_target
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
            teleop_state.desired_ee_position = desired_ee_position
            teleop_state.filtered_ee_position = filtered_ee_position
            teleop_state.desired_ee_rotation = desired_ee_rotation
            teleop_state.desired_right_joint_target = right_joint_target.copy()
            teleop_state.right_joint_target = right_joint_target.copy()
            teleop_state.ik_fail_count = 0
            teleop_state.last_ik_failure_frame = -30
            teleop_state.last_ik_error = ""
            teleop_state.status = "running"
            episode_buffer.reset()
            ee_pose = scene.get_end_effector_pose()
            frame_data = _capture_frame(
                scene,
                ee_position=np.array(ee_pose.position, dtype=np.float64),
                right_indices=right_indices,
                left_indices=left_indices,
                gripper_indices=gripper_indices,
                right_joint_target=teleop_state.right_joint_target,
                last_key="",
                raw_planar_delta_xy=np.zeros(2, dtype=np.float32),
                target_size_xy=target_size_xy,
            )
            episode_buffer.append(**frame_data)

        def finalize_episode(success: bool) -> None:
            nonlocal saved_episodes
            nonlocal attempted_episodes
            attempted_episodes += 1
            episode = episode_buffer.finalize(success=success)
            should_save = bool(success or args.save_failed)
            if episode is None:
                print("[record] current episode has no samples; nothing saved")
            elif should_save:
                episode_name = writer.write_episode(episode)
                saved_episodes += 1
                print(
                    f"[record] saved episode={episode_name} success={success} "
                    f"saved={saved_episodes}/{args.num_episodes} attempted={attempted_episodes} "
                    f"output={resolved_output}"
                )
            else:
                print(
                    f"[record] discarded failed episode "
                    f"saved={saved_episodes}/{args.num_episodes} attempted={attempted_episodes}"
                )
            if saved_episodes >= int(args.num_episodes):
                teleop_state.exit_requested = True
                return
            reset_episode()

        def handle_key(key: str) -> None:
            nonlocal last_key
            nonlocal last_action_xyz
            last_key = key
            if key in {"x", "f", "r", "p", "escape"}:
                print(f"[record-key] key={key}")
            if key in KEY_DELTAS and args.control_mode == "ee":
                unclamped = teleop_state.desired_ee_position + (KEY_DELTAS[key] * float(args.step_size))
                teleop_state.desired_ee_position = _clamp_desired_ee_position(unclamped, actual_table_top_z)
                last_action_xyz = KEY_DELTAS[key] * float(args.step_size)
            elif args.control_mode == "joint" and key in joint_key_map:
                joint_name, delta = joint_key_map[key]
                joint_index = right_joint_name_to_index.get(joint_name)
                if joint_index is not None:
                    new_target = float(
                        np.clip(
                            teleop_state.desired_right_joint_target[joint_index] + delta,
                            lower_limits[joint_index],
                            upper_limits[joint_index],
                        )
                    )
                    teleop_state.desired_right_joint_target[joint_index] = new_target
                    last_action_xyz = np.zeros(3, dtype=np.float64)
            elif key == "r":
                print("[record] reset current episode without saving")
                reset_episode()
            elif key == "p":
                left_drift, gripper_drift = _enforce_fixed_subsets(
                    scene,
                    left_indices=left_indices,
                    left_initial=fixed_left_arm_qpos,
                    gripper_indices=gripper_indices,
                    gripper_initial=fixed_gripper_qpos,
                    frame=frame,
                    warning_state=warning_state,
                    quiet_warnings=quiet_warnings,
                )
                _print_state(
                    scene,
                    frame=frame,
                    desired_ee_position=teleop_state.desired_ee_position,
                    gripper_qpos=_read_joint_positions(scene, gripper_indices),
                    right_joint_names=right_joint_names,
                    right_joint_positions=_read_joint_positions(scene, right_indices),
                    left_arm_drift=left_drift,
                    gripper_qpos_drift=gripper_drift,
                    paused=teleop_state.paused,
                    status=teleop_state.status,
                    control_mode=args.control_mode,
                    ik_fail_count=teleop_state.ik_fail_count,
                    last_ik_error=teleop_state.last_ik_error,
                )
            elif key == "x":
                teleop_state.success_count += 1
                teleop_state.status = "success"
                finalize_episode(success=True)
            elif key == "f":
                teleop_state.failure_count += 1
                teleop_state.status = "failure"
                finalize_episode(success=False)
            elif key == "escape":
                teleop_state.exit_requested = True

        reset_episode()
        while scene._app.is_running() and not teleop_state.exit_requested:
            frame += 1
            keys = active_input_reader.poll() if active_input_reader is not None else []
            for key in keys:
                handle_key(key)
                if teleop_state.exit_requested:
                    break

            if frame % control_interval_frames == 0 and control_available:
                if args.control_mode == "ee":
                    teleop_state.filtered_ee_position = (
                        float(args.alpha) * teleop_state.desired_ee_position
                        + (1.0 - float(args.alpha)) * teleop_state.filtered_ee_position
                    )
                    desired_pose = Pose(
                        position=np.array(teleop_state.filtered_ee_position, dtype=np.float64),
                        rotation=np.array(teleop_state.desired_ee_rotation, dtype=np.float64),
                    )
                    ik_solution, ik_success = scene.solve_ik(
                        desired_pose,
                        warm_start=teleop_state.right_joint_target,
                        position_tolerance=0.01,
                        orientation_tolerance=0.15,
                    )
                    if ik_success and ik_solution is not None and np.all(np.isfinite(ik_solution)):
                        teleop_state.desired_right_joint_target = np.clip(
                            np.array(ik_solution, dtype=np.float64),
                            lower_limits,
                            upper_limits,
                        )
                    else:
                        teleop_state.ik_fail_count += 1
                        teleop_state.last_ik_error = (
                            f"failed_count={teleop_state.ik_fail_count} "
                            f"desired_ee_position={np.round(teleop_state.filtered_ee_position, 4).tolist()}"
                        )
                        _emit_warning(
                            warning_state,
                            "ik_failure",
                            frame=frame,
                            message=f"[ik] {teleop_state.last_ik_error}",
                            interval_frames=IK_WARNING_INTERVAL_FRAMES,
                            quiet_warnings=quiet_warnings,
                        )
                        teleop_state.last_ik_failure_frame = frame
                teleop_state.right_joint_target = _filter_joint_target(
                    current_command=teleop_state.right_joint_target,
                    desired_target=teleop_state.desired_right_joint_target,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    joint_alpha=float(args.joint_alpha),
                    max_joint_delta=float(args.max_joint_delta),
                )
                sample_due = True

            full_target = _compose_full_target(
                initial_full_positions=initial_full_positions,
                right_indices=right_indices,
                right_target=teleop_state.right_joint_target,
                left_indices=left_indices,
                left_initial=fixed_left_arm_qpos,
                gripper_indices=gripper_indices,
                gripper_initial=fixed_gripper_qpos,
            )
            _apply_full_target(scene, full_target)
            scene.step_world(steps=1)
            left_drift, gripper_drift = _enforce_fixed_subsets(
                scene,
                left_indices=left_indices,
                left_initial=fixed_left_arm_qpos,
                gripper_indices=gripper_indices,
                gripper_initial=fixed_gripper_qpos,
                frame=frame,
                warning_state=warning_state,
                quiet_warnings=quiet_warnings,
            )

            ee_pose = scene.get_end_effector_pose()
            ee_position = np.array(ee_pose.position, dtype=np.float64)
            cube_xy, target_xy, _ = _cube_and_target_xy(scene)
            cube_to_target_dist = float(np.linalg.norm(target_xy - cube_xy))
            if frame % status_every == 0:
                _print_periodic_status(
                    scene,
                    frame=frame,
                    cube_xy=cube_xy,
                    target_xy=target_xy,
                    cube_to_target_dist=cube_to_target_dist,
                    ee_position=ee_position,
                    desired_ee_position=teleop_state.desired_ee_position,
                    gripper_drift=gripper_drift,
                    left_drift=left_drift,
                    paused=False,
                    control_mode=args.control_mode,
                    ik_fail_count=teleop_state.ik_fail_count,
                    success_margin=float(args.success_margin),
                    verbose=bool(args.verbose),
                )
            if sample_due and frame % control_interval_frames == 0:
                frame_data = _capture_frame(
                    scene,
                    ee_position=ee_position,
                    right_indices=right_indices,
                    left_indices=left_indices,
                    gripper_indices=gripper_indices,
                    right_joint_target=teleop_state.right_joint_target,
                    last_key=last_key,
                    raw_planar_delta_xy=last_action_xyz,
                    target_size_xy=target_size_xy,
                )
                episode_buffer.append(**frame_data)
                sample_due = False
                last_key = ""
                last_action_xyz = np.zeros(3, dtype=np.float64)
    except KeyboardInterrupt:
        print("[record] Ctrl+C received; exiting without saving the partial episode")
    finally:
        if active_input_reader is not None and isinstance(active_input_reader, TerminalKeyReader):
            active_input_reader.close()
        if viewer_keys is not None:
            viewer_keys.close()
        if scene_bundle is not None:
            scene_bundle.scene.close()
            close_simulation_app(scene_bundle.scene._app)


if __name__ == "__main__":
    main()
