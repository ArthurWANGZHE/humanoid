"""Helpers for reusing the check_pose PushCube scene setup."""

from __future__ import annotations

from dataclasses import dataclass

from rl_train.check_pose import (
    CheckPoseRuntimeOptions,
    CheckPoseSceneConfig,
    _apply_original_check_pose_initial_pose,
    create_scene,
    load_robot,
)


LEFT_ARM_JOINT_NAMES = [
    "left_base_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]


@dataclass(frozen=True)
class CheckPosePushCubeSceneBundle:
    training_config: object
    runtime_options: CheckPoseRuntimeOptions
    scene_config: CheckPoseSceneConfig
    scene: object


def create_check_pose_pushcube_scene(
    *,
    headless: bool,
    show_ranges: bool,
    disable_lula: bool,
    hold_open: bool = False,
) -> CheckPosePushCubeSceneBundle:
    runtime_options = CheckPoseRuntimeOptions(
        headless=headless,
        show_ranges=show_ranges,
        disable_lula=disable_lula,
        hold_open=hold_open,
    )
    scene_config = CheckPoseSceneConfig(show_ranges=show_ranges)
    training_config = load_robot()
    scene = create_scene(
        training_config=training_config,
        runtime_options=runtime_options,
        scene_config=scene_config,
    )
    _apply_original_check_pose_initial_pose(scene, training_config)
    return CheckPosePushCubeSceneBundle(
        training_config=training_config,
        runtime_options=runtime_options,
        scene_config=scene_config,
        scene=scene,
    )


def get_pushcube_object_handles(scene_bundle: CheckPosePushCubeSceneBundle) -> dict[str, object | None]:
    scene = scene_bundle.scene
    target_path = scene.layout_visual_prim_paths.get("target")
    cube_spawn_range_path = scene.layout_visual_prim_paths.get("cube_spawn_range")
    target_range_path = scene.layout_visual_prim_paths.get("target_range")
    return {
        "robot_articulation": scene.articulation,
        "demo_brick": scene.brick,
        "pushcube_target": scene._get_prim_at_path(target_path) if target_path else None,
        "demo_table": scene.table,
        "cube_spawn_range": scene._get_prim_at_path(cube_spawn_range_path) if cube_spawn_range_path else None,
        "target_range": scene._get_prim_at_path(target_range_path) if target_range_path else None,
    }


def get_robot_articulation(scene_bundle: CheckPosePushCubeSceneBundle):
    return scene_bundle.scene.articulation


def get_right_arm_joint_names(scene_bundle: CheckPosePushCubeSceneBundle) -> list[str]:
    return [
        joint_name
        for joint_name in scene_bundle.training_config.arm_joints
        if joint_name in scene_bundle.scene.dof_names
    ]


def get_left_arm_joint_names(scene_bundle: CheckPosePushCubeSceneBundle) -> list[str]:
    return [
        joint_name
        for joint_name in LEFT_ARM_JOINT_NAMES
        if joint_name in scene_bundle.scene.dof_names
    ]


def get_gripper_joint_names(scene_bundle: CheckPosePushCubeSceneBundle) -> list[str]:
    return [
        joint_name
        for joint_name in scene_bundle.training_config.gripper_joints
        if joint_name in scene_bundle.scene.dof_names
    ]
