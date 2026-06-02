"""PushCube-ready Isaac scene inspection entrypoint."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .brick_pick_demo_support import HumanoidBrickPickDemoScene, PushCubeSceneOptions
from .config import RobotTrainingConfig, load_robot_training_config
from pushcube_isaac_v0.isaac_lifecycle import close_simulation_app, hold_viewer_open

ARM_JOINT_NAMES = [
    "right_base_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_base_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

SETTLE_STEPS = 30


@dataclass(frozen=True)
class CheckPoseSceneConfig:
    pushcube_layout: bool = True
    show_ranges: bool = False
    cube_margin: float = 0.08
    target_margin: float = 0.06
    cube_offset_from_table_center: tuple[float, float] = (-0.08, -0.03)
    target_offset_from_table_center: tuple[float, float] = (0.08, 0.06)
    target_size_xy: tuple[float, float] = (0.16, 0.16)


@dataclass(frozen=True)
class CheckPoseRuntimeOptions:
    headless: bool = False
    show_ranges: bool = False
    disable_lula: bool = False
    hold_open: bool = False


def _parse_bool_arg(value: str | bool | None) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return True
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected one of true/false/1/0/yes/no, got: {value}")


def _default_robot_description_path() -> Path:
    return Path(__file__).resolve().parents[1] / "assets" / "robot_description"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect the PushCube-ready Isaac scene.")
    parser.add_argument("--headless", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--show-ranges", action="store_true")
    parser.add_argument("--disable-lula", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    return parser


def load_robot() -> RobotTrainingConfig:
    return load_robot_training_config(
        repo_root=Path(__file__).resolve().parents[1],
        robot_description_path=_default_robot_description_path().resolve(),
    )


def create_scene(
    training_config: RobotTrainingConfig,
    runtime_options: CheckPoseRuntimeOptions,
    scene_config: CheckPoseSceneConfig,
) -> HumanoidBrickPickDemoScene:
    pushcube_options = PushCubeSceneOptions(
        enabled=scene_config.pushcube_layout,
        show_ranges=scene_config.show_ranges,
        presentation=False,
        cube_margin=scene_config.cube_margin,
        target_margin=scene_config.target_margin,
        cube_offset_from_table_center=scene_config.cube_offset_from_table_center,
        target_offset_from_table_center=scene_config.target_offset_from_table_center,
        target_size_xy=scene_config.target_size_xy,
    )
    return HumanoidBrickPickDemoScene(
        training_config=training_config,
        headless=runtime_options.headless,
        setup_cameras=False,
        enable_lula=not runtime_options.disable_lula,
        pushcube_options=pushcube_options,
    )


def _print_startup_logs(runtime_options: CheckPoseRuntimeOptions, scene_config: CheckPoseSceneConfig) -> None:
    print(f"[check_pose] headless={runtime_options.headless}")
    print(f"[check_pose] disable_lula={runtime_options.disable_lula}")
    print(f"[check_pose] hold_open={runtime_options.hold_open}")
    print(f"[check_pose] show_ranges={runtime_options.show_ranges}")
    print(f"[check_pose] pushcube_layout={scene_config.pushcube_layout}")


def _print_layout_logs(scene: HumanoidBrickPickDemoScene, runtime_options: CheckPoseRuntimeOptions) -> None:
    layout = scene.pushcube_layout or {}
    visibility_state = scene.layout_visibility_state
    cube_xy = [round(float(value), 4) for value in layout.get("cube_position", [0.0, 0.0])[:2]]
    target_xy = [round(float(value), 4) for value in layout.get("target_center", [0.0, 0.0])[:2]]
    cube_margin_min = round(
        float(dict(layout.get("cube_margin_to_table_edges", {"min": 0.0})).get("min", 0.0)),
        4,
    )
    target_margin_min = round(
        float(dict(layout.get("target_margin_to_table_edges", {"min": 0.0})).get("min", 0.0)),
        4,
    )
    print(f"[pushcube-layout] cube_xy={cube_xy}")
    print(f"[pushcube-layout] target_xy={target_xy}")
    print(f"[pushcube-layout] cube_margin_min={cube_margin_min}")
    print(f"[pushcube-layout] target_margin_min={target_margin_min}")
    print(f"[pushcube-layout] show_ranges_arg={runtime_options.show_ranges}")
    print(f"[pushcube-layout] target_visible={visibility_state['target_visible']}")
    print(f"[pushcube-layout] cube_spawn_range_visible={visibility_state['cube_spawn_range_visible']}")
    print(f"[pushcube-layout] target_range_visible={visibility_state['target_range_visible']}")
    print(f"[pushcube-layout] ranges_visible={visibility_state['ranges_visible']}")


def _rounded_joint_list(values: np.ndarray) -> list[float]:
    return [round(float(value), 6) for value in np.array(values, dtype=np.float64).tolist()]


def _print_gripper_logs(scene: HumanoidBrickPickDemoScene) -> None:
    joint_names = [joint_name for joint_name in scene.training_config.gripper_joints if joint_name in scene.dof_names]
    qpos_before = np.array(scene.layout_gripper_qpos_before, dtype=np.float64)
    qpos_after = np.array(scene.layout_gripper_qpos_after, dtype=np.float64)
    max_abs_change = float(scene.layout_gripper_max_abs_qpos_change)
    print("[gripper] fixed=True")
    print("[gripper] controlled=False")
    print(f"[gripper] joint_names={joint_names}")
    print(f"[gripper] qpos_before={_rounded_joint_list(qpos_before)}")
    print(f"[gripper] qpos_after={_rounded_joint_list(qpos_after)}")
    print(f"[gripper] max_abs_qpos_change={max_abs_change}")
    if scene.layout_gripper_qpos_restored:
        print("[warning] pushcube layout changed gripper qpos; restored original values")
    assert max_abs_change <= 1e-6, f"PushCube layout changed gripper qpos by {max_abs_change}"


def _apply_original_check_pose_initial_pose(
    scene: HumanoidBrickPickDemoScene,
    training_config: RobotTrainingConfig,
) -> None:
    print("[check_pose] using original robot initial pose")
    joint_indices = []
    target_positions = []
    for joint_name in ARM_JOINT_NAMES:
        if joint_name not in scene.dof_names:
            continue
        joint_indices.append(scene.dof_names.index(joint_name))
        target_positions.append(float(training_config.home_joint_positions.get(joint_name, 0.0)))
    if not joint_indices:
        return
    target_positions_array = np.array(target_positions, dtype=np.float64)
    joint_indices_array = np.array(joint_indices, dtype=np.int32)
    scene.articulation.set_joint_positions(target_positions_array, joint_indices=joint_indices_array)
    scene.articulation.set_joint_velocities(
        np.zeros(len(target_positions_array), dtype=np.float64),
        joint_indices=joint_indices_array,
    )
    scene.step_world(steps=SETTLE_STEPS)
    scene.articulation.set_joint_positions(target_positions_array, joint_indices=joint_indices_array)
    scene.articulation.set_joint_velocities(
        np.zeros(len(target_positions_array), dtype=np.float64),
        joint_indices=joint_indices_array,
    )


def _show_scene(scene: HumanoidBrickPickDemoScene, runtime_options: CheckPoseRuntimeOptions) -> None:
    warmup_steps = 120 if not runtime_options.headless else 15
    scene.step_world(steps=warmup_steps)
    if runtime_options.hold_open:
        hold_viewer_open(scene, scene._app, render=not runtime_options.headless)


def run_check_pose(runtime_options: CheckPoseRuntimeOptions) -> None:
    scene: HumanoidBrickPickDemoScene | None = None
    scene_config = CheckPoseSceneConfig(show_ranges=runtime_options.show_ranges)
    _print_startup_logs(runtime_options, scene_config)
    try:
        training_config = load_robot()
        scene = create_scene(training_config=training_config, runtime_options=runtime_options, scene_config=scene_config)
        _print_gripper_logs(scene)
        _apply_original_check_pose_initial_pose(scene, training_config)
        _print_layout_logs(scene, runtime_options)
        print("[lifecycle] check_pose scene ready")
        print(f"[lifecycle] hold_open={runtime_options.hold_open}")
        _show_scene(scene, runtime_options)
    finally:
        if scene is not None:
            scene.close()
            print("[lifecycle] closing SimulationApp")
            close_simulation_app(scene._app)


def main() -> None:
    args = build_arg_parser().parse_args()
    print(f"[check_pose] headless={args.headless}")
    print(f"[check_pose] show_ranges={args.show_ranges}")
    runtime_options = CheckPoseRuntimeOptions(
        headless=args.headless,
        show_ranges=args.show_ranges,
        disable_lula=args.disable_lula,
        hold_open=args.hold_open,
    )
    run_check_pose(runtime_options)


if __name__ == "__main__":
    main()
