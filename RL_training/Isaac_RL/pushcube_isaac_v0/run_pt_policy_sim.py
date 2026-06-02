"""Run a trained PushCube ``model.pt`` policy in Isaac Sim.

This is a lightweight viewer-oriented inference entrypoint. It loads the
behavior-cloning checkpoint produced by ``train_pushcube_bc.py``, builds the
same 13-D low-dimensional observation used for training, predicts a normalized
2-D planar action, and converts it to right-arm end-effector motion through IK.
"""

from __future__ import annotations

import argparse
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
    _reset_scene,
    _resolve_layout_override,
    _right_joint_limits,
)
from pushcube_isaac_v0.pushcube_dataset_utils import (  # noqa: E402
    ACTION_DIM,
    DEFAULT_ACTION_SCALE,
    OBS_DIM,
    build_low_dim_observation,
    cube_yaw_from_rotation,
    optional_import,
)
from rl_train.check_pose import _parse_bool_arg  # noqa: E402
from rl_train.pose_math import Pose  # noqa: E402


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control the PushCube Isaac scene with a trained model.pt policy.")
    parser.add_argument("--checkpoint", type=str, default="data/simulation/isaac_rl/runs/pushcube_bc_aug100_v1/model.pt")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--headless", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--hold-open", action="store_true", help="Keep the viewer open after rollout completion.")
    parser.add_argument("--show-ranges", action="store_true")
    parser.add_argument("--disable-lula", action="store_true")
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
    parser.add_argument("--target-size", type=float, default=0.22)
    parser.add_argument("--success-margin", type=float, default=0.02)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--alpha", type=float, default=0.2, help="End-effector target low-pass factor.")
    parser.add_argument("--joint-alpha", type=float, default=0.2, help="Joint target low-pass factor.")
    parser.add_argument("--max-joint-delta", type=float, default=0.005)
    parser.add_argument("--expand-right-arm-limits", type=float, default=0.5)
    parser.add_argument("--initial-pose-preset", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--action-scale", type=float, default=None, help="Meters per normalized policy action.")
    parser.add_argument("--device", type=str, default="cpu", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--status-every", type=int, default=15)
    parser.add_argument("--quiet-warnings", type=_parse_bool_arg, nargs="?", const=True, default=True)
    parser.add_argument("--suppress-isaac-warnings", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--summary-json", type=str, default=None)
    return parser


def _checkpoint_architecture(checkpoint: dict, state_dict: dict) -> list[int]:
    architecture = checkpoint.get("architecture")
    if architecture is not None:
        return [int(value) for value in architecture]

    weight_keys = [
        key
        for key in state_dict.keys()
        if key.startswith("net.") and key.endswith(".weight")
    ]
    weight_keys.sort(key=lambda key: int(key.split(".")[1]))
    if not weight_keys:
        raise RuntimeError("Cannot infer MLP architecture from checkpoint state_dict.")

    inferred: list[int] = []
    for key in weight_keys:
        weight = state_dict[key]
        out_dim, in_dim = tuple(int(dim) for dim in weight.shape)
        if not inferred:
            inferred.append(in_dim)
        inferred.append(out_dim)
    return inferred


def _build_mlp(torch, architecture: list[int]):
    if len(architecture) < 2:
        raise RuntimeError(f"Invalid MLP architecture: {architecture}")

    class PolicyMLP(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            layers = []
            for index, (in_dim, out_dim) in enumerate(zip(architecture[:-1], architecture[1:], strict=True)):
                layers.append(torch.nn.Linear(int(in_dim), int(out_dim)))
                if index < len(architecture) - 2:
                    layers.append(torch.nn.ReLU())
            self.net = torch.nn.Sequential(*layers)

        def forward(self, obs):
            return self.net(obs)

    return PolicyMLP()


def _load_policy(torch, checkpoint_path: Path, device):
    try:
        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    except TypeError:
        checkpoint = torch.load(checkpoint_path, map_location=device)
    if not isinstance(checkpoint, dict) or "model_state_dict" not in checkpoint:
        raise RuntimeError(
            f"{checkpoint_path} is not a train_pushcube_bc.py checkpoint; expected a dict with model_state_dict."
        )

    state_dict = checkpoint["model_state_dict"]
    architecture = _checkpoint_architecture(checkpoint, state_dict)
    if architecture[0] != OBS_DIM or architecture[-1] != ACTION_DIM:
        raise RuntimeError(
            f"Checkpoint architecture {architecture} does not match PushCube obs/action dims "
            f"({OBS_DIM}, {ACTION_DIM})."
        )

    model = _build_mlp(torch, architecture).to(device)
    model.load_state_dict(state_dict)
    model.eval()

    obs_mean = torch.as_tensor(checkpoint.get("obs_mean", np.zeros(OBS_DIM)), dtype=torch.float32, device=device)
    obs_std = torch.as_tensor(checkpoint.get("obs_std", np.ones(OBS_DIM)), dtype=torch.float32, device=device).clamp_min(1e-6)
    metadata = checkpoint.get("metadata", {}) if isinstance(checkpoint.get("metadata", {}), dict) else {}
    action_scale = float(metadata.get("action_scale", DEFAULT_ACTION_SCALE))
    return model, obs_mean, obs_std, action_scale, architecture


def _resolve_input_path(path_value: str) -> Path:
    path = Path(path_value).expanduser()
    if path.is_absolute() or path.exists():
        return path
    project_path = PROJECT_ROOT / path
    if project_path.exists():
        return project_path
    return path


def _make_observation(scene, target_size: float) -> np.ndarray:
    ee_pose = scene.get_end_effector_pose()
    cube_pose = scene.get_brick_pose()
    target_center = np.array((scene.pushcube_layout or {}).get("target_center", [0.0, 0.0, 0.0]), dtype=np.float64)
    target_size_xy = np.array(
        (scene.pushcube_layout or {}).get("target_size", [target_size, target_size, 0.002]),
        dtype=np.float64,
    )[:2]
    return build_low_dim_observation(
        ee_xy=np.array(ee_pose.position[:2], dtype=np.float64),
        cube_xy=np.array(cube_pose.position[:2], dtype=np.float64),
        cube_yaw=cube_yaw_from_rotation(np.array(cube_pose.rotation, dtype=np.float64)),
        target_xy=np.array(target_center[:2], dtype=np.float64),
        target_size_xy=target_size_xy,
    )


def _predict_action(torch, model, obs_mean, obs_std, observation: np.ndarray, device) -> np.ndarray:
    with torch.no_grad():
        obs_tensor = torch.as_tensor(observation, dtype=torch.float32, device=device).unsqueeze(0)
        normalized = (obs_tensor - obs_mean) / obs_std
        action = model(normalized).squeeze(0).detach().cpu().numpy()
    if action.shape != (ACTION_DIM,):
        raise RuntimeError(f"Expected policy action shape {(ACTION_DIM,)}, got {action.shape}")
    return np.clip(np.array(action, dtype=np.float64), -1.0, 1.0)


def _rounded_xy(values: np.ndarray) -> list[float]:
    return [round(float(value), 4) for value in np.array(values, dtype=np.float64)[:2]]


def main() -> None:
    args = build_arg_parser().parse_args()
    torch = optional_import("torch", "pip install torch")
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        if args.device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested with --device cuda, but torch.cuda.is_available() is false.")
        device = torch.device(args.device)

    checkpoint_path = _resolve_input_path(args.checkpoint)
    model, obs_mean, obs_std, checkpoint_action_scale, architecture = _load_policy(torch, checkpoint_path, device)
    action_scale = float(args.action_scale) if args.action_scale is not None else checkpoint_action_scale
    _apply_warning_filters(bool(args.suppress_isaac_warnings))
    print(
        f"[policy] checkpoint={checkpoint_path} device={device} architecture={architecture} "
        f"action_scale={action_scale:.6f}"
    )

    scene_bundle = None
    results: list[dict[str, object]] = []
    try:
        scene_bundle = create_check_pose_pushcube_scene(
            headless=bool(args.headless),
            show_ranges=bool(args.show_ranges),
            disable_lula=bool(args.disable_lula),
            hold_open=bool(args.hold_open),
        )
        scene = scene_bundle.scene
        if args.disable_lula or scene.kinematics is None:
            raise RuntimeError("PT policy control requires Lula/IK; run without --disable-lula.")

        resolved_preset, resolved_table_center_xy, resolved_table_top_z, _ = _resolve_layout_override(args, scene)
        actual_table_top_z = _apply_table_layout_override(
            scene_bundle,
            layout_preset=resolved_preset,
            table_center_xy=resolved_table_center_xy,
            table_top_z=resolved_table_top_z,
            target_size_xy=np.array([float(args.target_size), float(args.target_size)], dtype=np.float64),
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
        lower_limits, upper_limits = _right_joint_limits(
            scene_bundle,
            right_joint_names,
            expand_by=float(args.expand_right_arm_limits),
        )

        physics_hz = 1.0 / float(scene.scene_config.physics_dt)
        control_interval_frames = max(1, int(round(physics_hz / float(args.control_hz))))
        print(f"[sim] physics_hz={physics_hz:.2f} control_hz={float(args.control_hz):.2f} interval_frames={control_interval_frames}")

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

            success = False
            failure_reason = "timeout"
            warning_state: dict[str, dict[str, int]] = {}
            last_action = np.zeros(ACTION_DIM, dtype=np.float64)
            for step in range(int(args.max_steps)):
                observation = _make_observation(scene, float(args.target_size))
                action = _predict_action(torch, model, obs_mean, obs_std, observation, device)
                if not np.all(np.isfinite(action)):
                    failure_reason = "invalid_action"
                    break
                last_action = action

                desired_ee_position[:2] = desired_ee_position[:2] + (action[:2] * action_scale)
                desired_ee_position = _clamp_desired_ee_position(desired_ee_position, actual_table_top_z)
                filtered_ee_position = (
                    float(args.alpha) * desired_ee_position
                    + (1.0 - float(args.alpha)) * filtered_ee_position
                )
                desired_pose = Pose(
                    position=np.array(filtered_ee_position, dtype=np.float64),
                    rotation=np.array(desired_ee_rotation, dtype=np.float64),
                )
                ik_solution, ik_success = scene.solve_ik(
                    desired_pose,
                    warm_start=right_joint_target,
                    position_tolerance=0.01,
                    orientation_tolerance=0.15,
                )
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
                        warning_state=warning_state,
                        quiet_warnings=bool(args.quiet_warnings),
                    )

                cube_xy, target_xy, _ = _cube_and_target_xy(scene)
                final_distance = float(np.linalg.norm(target_xy - cube_xy))
                if int(args.status_every) > 0 and (step % int(args.status_every) == 0):
                    print(
                        "[policy-step] "
                        f"episode={episode_index} step={step} action={_rounded_xy(action)} "
                        f"cube_xy={_rounded_xy(cube_xy)} target_xy={_rounded_xy(target_xy)} "
                        f"distance={final_distance:.4f}"
                    )
                if _cube_inside_target(scene, float(args.success_margin)):
                    success = True
                    failure_reason = ""
                    break

            cube_xy, target_xy, _ = _cube_and_target_xy(scene)
            final_distance = float(np.linalg.norm(target_xy - cube_xy))
            result = {
                "episode_index": episode_index,
                "success": success,
                "failure_reason": failure_reason or None,
                "final_distance": final_distance,
                "last_action": np.round(last_action, 8).tolist(),
            }
            results.append(result)
            print(
                "[episode] "
                f"index={episode_index} success={success} "
                f"failure_reason={failure_reason or 'none'} final_distance={final_distance:.4f}"
            )

        summary = {
            "checkpoint": str(checkpoint_path),
            "episodes": int(args.episodes),
            "success_rate": float(np.mean([entry["success"] for entry in results])) if results else 0.0,
            "mean_final_distance": float(np.mean([entry["final_distance"] for entry in results])) if results else 0.0,
            "per_episode": results,
        }
        if args.summary_json is not None:
            summary_path = Path(args.summary_json)
            summary_path.parent.mkdir(parents=True, exist_ok=True)
            with summary_path.open("w", encoding="utf-8") as handle:
                json.dump(summary, handle, indent=2, sort_keys=True)
            print(f"[summary] wrote {summary_path}")

        if bool(args.hold_open) and not bool(args.headless):
            print("[viewer] hold-open enabled; press Ctrl+C in the terminal to close.")
            try:
                while scene._app.is_running():
                    scene.step_world(steps=1)
            except KeyboardInterrupt:
                print("[viewer] Ctrl+C received; closing viewer.")
    except KeyboardInterrupt:
        print("[lifecycle] Ctrl+C received, exiting policy rollout.")
    finally:
        if scene_bundle is not None:
            scene_bundle.scene.close()
            close_simulation_app(scene_bundle.scene._app)


if __name__ == "__main__":
    main()
