"""Debug PushCube motion script using the check_pose scene setup."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.check_pose_scene import (  # noqa: E402
    CheckPosePushCubeSceneBundle,
    create_check_pose_pushcube_scene,
    get_gripper_joint_names,
    get_left_arm_joint_names,
    get_pushcube_object_handles,
    get_right_arm_joint_names,
)
from pushcube_isaac_v0.isaac_lifecycle import close_simulation_app, hold_viewer_open  # noqa: E402
from rl_train.check_pose import _parse_bool_arg, _print_layout_logs  # noqa: E402
from rl_train.pose_math import Pose, normalize  # noqa: E402


RIGHT_GRIPPER_CONTACT_LINK_NAMES = ["right_gripper1_link", "right_gripper2_link"]
RIGHT_GRIPPER_CONTACT_FALLBACK_LINK = "right_wrist_yaw_link"
TABLE_SWEEP_CANDIDATES = [0.30, 0.35, 0.40, 0.45]
PUSH_WAYPOINT_MARKERS = {
    "behind": ("/World/pushcube_waypoint_behind", (0.98, 0.72, 0.14)),
    "contact": ("/World/pushcube_waypoint_contact", (0.98, 0.34, 0.18)),
    "push_goal": ("/World/pushcube_waypoint_goal", (0.16, 0.86, 0.88)),
}
SAFE_DEBUG_JOINT_LIMITS = {
    "right_elbow_pitch_joint": (1.0, 1.5),
    "right_wrist_pitch_joint": (-0.6, 0.8),
    "right_wrist_yaw_joint": (-0.8, 0.8),
    "right_shoulder_roll_joint": (-0.8, 0.1),
}
PER_JOINT_STEP_LIMITS = {
    "right_elbow_pitch_joint": 0.04,
    "right_wrist_pitch_joint": 0.05,
    "right_wrist_yaw_joint": 0.05,
}
GRIPPER_DRIFT_TOLERANCE = 1e-5
LEFT_ARM_DRIFT_TOLERANCE = 1e-5
GRIPPER_DRIFT_WARN_THRESHOLD = 0.03
LEFT_ARM_DRIFT_WARN_THRESHOLD = 0.01
WARNING_INTERVAL_STEPS = 60
CONTROL_VERIFY_STEPS = 60
CONTROL_VERIFY_DELTA = 0.02
COORDINATE_PUSH_JOINT_STEP_LIMIT = 0.02
COORDINATE_PUSH_CART_STEP_LIMIT = 0.005


@dataclass(frozen=True)
class JointPhaseSpec:
    name: str
    duration: int
    right_target: np.ndarray


@dataclass(frozen=True)
class CoordinatePushGeometry:
    cube_center: np.ndarray
    target_center: np.ndarray
    target_size_xy: np.ndarray
    table_top_z: float
    cube_size: float
    push_dir: np.ndarray
    behind_point: np.ndarray
    contact_point: np.ndarray
    push_goal_point: np.ndarray
    desired_contact_z: float
    gripper_contact_ref: np.ndarray
    contact_offset_in_ee: np.ndarray
    ee_reference_rotation: np.ndarray
    selected_contact_link: str


@dataclass
class CoordinatePushState:
    phase_name: str
    phase_index: int
    step_in_phase: int
    phase_duration: int
    phase_start_contact: np.ndarray
    phase_end_contact: np.ndarray
    desired_contact_point: np.ndarray
    last_gripper_to_cube_dist: float
    min_gripper_to_cube_dist: float
    repeated_ik_failures: int
    max_measured_right_movement: float
    ever_cube_gripper_contact: bool
    ik_success_count: int
    ik_fail_count: int
    safety_reject_count: int
    moved_robot: bool
    stopped_reason: str | None
    nonapproach_count: int


@dataclass(frozen=True)
class TableSweepResult:
    table_top_z: float
    cube_center_z: float
    desired_contact_z: float
    gripper_ref_z: float
    initial_z_error: float
    ik_success: bool
    final_contact_error: float | None
    gripper_to_cube_dist: float
    collision_warning: bool


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Debug PushCube right-arm motion with a fixed gripper.")
    parser.add_argument("--headless", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--show-ranges", action="store_true")
    parser.add_argument("--disable-lula", action="store_true")
    parser.add_argument("--hold-open", action="store_true")
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument(
        "--motion-mode",
        choices=["coordinate_push", "joint_debug", "scripted_push", "none"],
        default=None,
    )
    parser.add_argument("--joint-scale", type=float, default=1.0)
    parser.add_argument("--max-joint-delta-per-step", type=float, default=0.02)
    parser.add_argument("--max-total-joint-delta", type=float, default=0.15)
    parser.add_argument("--direct-cartesian-debug", action="store_true")
    parser.add_argument("--table-top-z", type=float, default=None)
    parser.add_argument("--sweep-table-height", action="store_true")
    parser.add_argument("--show-push-waypoints", action="store_true")
    parser.add_argument("--ik-test-only", action="store_true")
    return parser


def _rounded_list(values: np.ndarray) -> list[float]:
    return [round(float(value), 6) for value in np.array(values, dtype=np.float64).tolist()]


def _rounded_named_values(names: list[str], values: np.ndarray) -> dict[str, float]:
    return {
        joint_name: round(float(value), 6)
        for joint_name, value in zip(names, np.array(values, dtype=np.float64).tolist(), strict=False)
    }


def _joint_indices(scene, joint_names: list[str]) -> np.ndarray:
    return np.array([scene.dof_names.index(joint_name) for joint_name in joint_names], dtype=np.int32)


def _read_joint_positions(scene, joint_indices: np.ndarray) -> np.ndarray:
    joint_positions = np.array(scene.articulation.get_joint_positions(), dtype=np.float64)
    return np.array(joint_positions[joint_indices], dtype=np.float64)


def _apply_full_target(scene, full_target: np.ndarray) -> None:
    scene.articulation.apply_action(scene._ArticulationAction(joint_positions=np.array(full_target, dtype=np.float64)))


def _restore_joint_subset(scene, joint_indices: np.ndarray, target_positions: np.ndarray) -> None:
    if joint_indices.size == 0:
        return
    scene.articulation.set_joint_positions(
        np.array(target_positions, dtype=np.float64),
        joint_indices=np.array(joint_indices, dtype=np.int32),
    )
    scene.articulation.set_joint_velocities(
        np.zeros(len(joint_indices), dtype=np.float64),
        joint_indices=np.array(joint_indices, dtype=np.int32),
    )


def _compose_full_target(
    initial_full_positions: np.ndarray,
    right_indices: np.ndarray,
    right_target: np.ndarray,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
) -> np.ndarray:
    full_target = np.array(initial_full_positions, dtype=np.float64)
    full_target[right_indices] = np.array(right_target, dtype=np.float64)
    full_target[left_indices] = np.array(left_initial, dtype=np.float64)
    full_target[gripper_indices] = np.array(gripper_initial, dtype=np.float64)
    return full_target


def _right_joint_limits(scene_bundle: CheckPosePushCubeSceneBundle, right_joint_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    lower = []
    upper = []
    for joint_name in right_joint_names:
        limit = scene_bundle.training_config.joint_limits.get(joint_name, {})
        lower.append(float(limit.get("min_position", -3.14)))
        upper.append(float(limit.get("max_position", 3.14)))
    return np.array(lower, dtype=np.float64), np.array(upper, dtype=np.float64)


def _clip_total_delta(
    initial_positions: np.ndarray,
    target_positions: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    max_total_joint_delta: float,
) -> np.ndarray:
    max_delta = abs(float(max_total_joint_delta))
    delta = np.array(target_positions, dtype=np.float64) - np.array(initial_positions, dtype=np.float64)
    delta = np.clip(delta, -max_delta, max_delta)
    return np.clip(np.array(initial_positions, dtype=np.float64) + delta, lower_limits, upper_limits)


def _step_toward_target(
    current_command: np.ndarray,
    desired_target: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    max_joint_delta_per_step: float,
) -> np.ndarray:
    max_step = abs(float(max_joint_delta_per_step))
    step_delta = np.array(desired_target, dtype=np.float64) - np.array(current_command, dtype=np.float64)
    step_delta = np.clip(step_delta, -max_step, max_step)
    return np.clip(np.array(current_command, dtype=np.float64) + step_delta, lower_limits, upper_limits)


def _resolve_motion_mode(args: argparse.Namespace) -> str:
    if args.motion_mode is not None:
        return args.motion_mode
    return "coordinate_push"


def _build_joint_debug_plan(
    right_initial: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    joint_scale: float,
    max_total_joint_delta: float,
) -> list[JointPhaseSpec]:
    joint_scale = float(joint_scale)
    approach_offsets = np.array([0.08, -0.03, 0.08, -0.10, 0.08, 0.00], dtype=np.float64) * joint_scale
    push_offsets = np.array([0.12, -0.05, 0.12, -0.14, 0.10, 0.00], dtype=np.float64) * joint_scale
    approach_target = _clip_total_delta(
        right_initial,
        right_initial + approach_offsets,
        lower_limits,
        upper_limits,
        max_total_joint_delta,
    )
    push_target = _clip_total_delta(
        right_initial,
        right_initial + push_offsets,
        lower_limits,
        upper_limits,
        max_total_joint_delta,
    )
    return [
        JointPhaseSpec(name="hold_initial", duration=60, right_target=np.array(right_initial, dtype=np.float64)),
        JointPhaseSpec(name="approach", duration=120, right_target=approach_target),
        JointPhaseSpec(name="push", duration=120, right_target=push_target),
        JointPhaseSpec(name="hold_final", duration=120, right_target=push_target),
    ]


def _select_verify_target(
    initial_value: float,
    lower_limit: float,
    upper_limit: float,
    target_delta: float,
) -> float:
    preferred = initial_value + target_delta
    if preferred <= upper_limit:
        return preferred
    alternate = initial_value - target_delta
    if alternate >= lower_limit:
        return alternate
    return float(np.clip(preferred, lower_limit, upper_limit))


def _restore_full_pose(scene, full_positions: np.ndarray, steps: int = 30) -> None:
    scene.articulation.set_joint_positions(np.array(full_positions, dtype=np.float64))
    scene.articulation.set_joint_velocities(np.zeros(len(full_positions), dtype=np.float64))
    _apply_full_target(scene, np.array(full_positions, dtype=np.float64))
    scene.step_world(steps=steps)


def _rate_limited_warning(warning_state: dict[str, int], key: str, step: int, message: str) -> None:
    last_step = warning_state.get(key, -WARNING_INTERVAL_STEPS)
    if step - last_step < WARNING_INTERVAL_STEPS:
        return
    warning_state[key] = step
    print(message)


def _enforce_fixed_subsets(
    scene,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
    *,
    step: int,
    warning_state: dict[str, int],
) -> tuple[float, float]:
    left_current = _read_joint_positions(scene, left_indices)
    gripper_current = _read_joint_positions(scene, gripper_indices)
    left_drift = float(np.max(np.abs(left_current - left_initial))) if left_current.size else 0.0
    gripper_drift = float(np.max(np.abs(gripper_current - gripper_initial))) if gripper_current.size else 0.0
    if left_drift > LEFT_ARM_DRIFT_TOLERANCE:
        if left_drift > LEFT_ARM_DRIFT_WARN_THRESHOLD:
            _rate_limited_warning(
                warning_state,
                "left-arm-drift",
                step,
                f"[warning] left arm drift detected; restoring fixed joints (max_abs_drift={left_drift:.6f})",
            )
        _restore_joint_subset(scene, left_indices, left_initial)
        left_current = _read_joint_positions(scene, left_indices)
        left_drift = float(np.max(np.abs(left_current - left_initial))) if left_current.size else 0.0
    if gripper_drift > GRIPPER_DRIFT_TOLERANCE:
        if gripper_drift > GRIPPER_DRIFT_WARN_THRESHOLD:
            _rate_limited_warning(
                warning_state,
                "gripper-drift",
                step,
                f"[warning] gripper drift detected; restoring fixed joints (max_abs_drift={gripper_drift:.6f})",
            )
        _restore_joint_subset(scene, gripper_indices, gripper_initial)
        gripper_current = _read_joint_positions(scene, gripper_indices)
        gripper_drift = float(np.max(np.abs(gripper_current - gripper_initial))) if gripper_current.size else 0.0
    assert gripper_drift <= GRIPPER_DRIFT_TOLERANCE, f"Gripper drift exceeded tolerance: {gripper_drift}"
    return left_drift, gripper_drift


def _verify_right_arm_control(
    scene_bundle: CheckPosePushCubeSceneBundle,
    initial_full_positions: np.ndarray,
    right_joint_names: list[str],
    right_indices: np.ndarray,
    right_initial: np.ndarray,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
) -> bool:
    scene = scene_bundle.scene
    warning_state: dict[str, int] = {}
    if not right_joint_names:
        print("[articulation] right arm does not respond to joint targets")
        return False
    verify_target = np.array(right_initial, dtype=np.float64)
    verify_target[0] = _select_verify_target(
        initial_value=float(right_initial[0]),
        lower_limit=float(lower_limits[0]),
        upper_limit=float(upper_limits[0]),
        target_delta=CONTROL_VERIFY_DELTA,
    )
    for step in range(CONTROL_VERIFY_STEPS):
        full_target = _compose_full_target(
            initial_full_positions=initial_full_positions,
            right_indices=right_indices,
            right_target=verify_target,
            left_indices=left_indices,
            left_initial=left_initial,
            gripper_indices=gripper_indices,
            gripper_initial=gripper_initial,
        )
        _apply_full_target(scene, full_target)
        scene.step_world(steps=1)
        _enforce_fixed_subsets(
            scene,
            left_indices,
            left_initial,
            gripper_indices,
            gripper_initial,
            step=step,
            warning_state=warning_state,
        )
    measured_positions = _read_joint_positions(scene, right_indices)
    measured_delta = measured_positions - right_initial
    measured_change = float(np.max(np.abs(measured_delta))) if measured_delta.size else 0.0
    print(
        "[articulation] verification",
        {
            "joint_name": right_joint_names[0],
            "target_delta": round(float(verify_target[0] - right_initial[0]), 6),
            "measured_delta": round(float(measured_delta[0]), 6) if measured_delta.size else 0.0,
            "max_abs_measured_delta": round(measured_change, 6),
        },
    )
    _restore_full_pose(scene, initial_full_positions)
    if measured_change < 0.005:
        print("[articulation] right arm does not respond to joint targets")
        return False
    return True


def _get_link_position(scene, link_name: str) -> np.ndarray | None:
    try:
        pose = scene.get_link_pose(link_name)
    except Exception:
        return None
    return np.array(pose.position, dtype=np.float64)


def _get_gripper_contact_reference(scene, cube_center: np.ndarray) -> tuple[str, np.ndarray]:
    candidates: list[tuple[str, np.ndarray]] = []
    for link_name in RIGHT_GRIPPER_CONTACT_LINK_NAMES:
        position = _get_link_position(scene, link_name)
        if position is not None:
            candidates.append((link_name, position))
    if not candidates:
        fallback_position = _get_link_position(scene, RIGHT_GRIPPER_CONTACT_FALLBACK_LINK)
        if fallback_position is not None:
            return RIGHT_GRIPPER_CONTACT_FALLBACK_LINK, fallback_position
        ee_pose = scene.get_end_effector_pose()
        return scene.training_config.end_effector_link, np.array(ee_pose.position, dtype=np.float64)
    return min(candidates, key=lambda item: float(np.linalg.norm(item[1] - cube_center)))


def _compute_contact_offset_in_ee(scene, contact_point_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ee_pose = scene.get_end_effector_pose()
    local_offset = ee_pose.rotation.T @ (np.array(contact_point_world, dtype=np.float64) - ee_pose.position)
    return np.array(local_offset, dtype=np.float64), np.array(ee_pose.rotation, dtype=np.float64)


def _actual_gripper_contact_point_world(scene, contact_offset_in_ee: np.ndarray) -> np.ndarray:
    ee_pose = scene.get_end_effector_pose()
    return ee_pose.transformed(contact_offset_in_ee)


def _desired_ee_pose_for_contact(
    desired_contact_point: np.ndarray,
    contact_offset_in_ee: np.ndarray,
    ee_reference_rotation: np.ndarray,
) -> Pose:
    desired_contact_point = np.array(desired_contact_point, dtype=np.float64)
    desired_ee_position = desired_contact_point - (ee_reference_rotation @ np.array(contact_offset_in_ee, dtype=np.float64))
    return Pose(position=desired_ee_position, rotation=np.array(ee_reference_rotation, dtype=np.float64))


def _set_prim_translate(scene, prim_path: str, center_xyz: np.ndarray) -> None:
    prim = scene._get_prim_at_path(prim_path)
    if prim is None or not prim.IsValid():
        return
    scene._UsdGeom.XformCommonAPI(prim).SetTranslate(tuple(float(value) for value in np.array(center_xyz, dtype=np.float64)))


def _set_prim_scale(scene, prim_path: str, size_xyz: np.ndarray) -> None:
    prim = scene._get_prim_at_path(prim_path)
    if prim is None or not prim.IsValid():
        return
    scene._UsdGeom.XformCommonAPI(prim).SetScale(tuple(float(value) for value in np.array(size_xyz, dtype=np.float64)))


def _current_table_top_z(scene) -> float:
    table_position, _ = scene.table.get_world_pose()
    return float(np.array(table_position, dtype=np.float64)[2] + (scene.scene_config.table_scale[2] / 2.0))


def _apply_table_top_z_override(scene_bundle: CheckPosePushCubeSceneBundle, requested_table_top_z: float | None) -> float:
    scene = scene_bundle.scene
    layout = scene.pushcube_layout or {}
    current_top_z = _current_table_top_z(scene)
    target_top_z = float(current_top_z if requested_table_top_z is None else requested_table_top_z)
    table_position, table_quaternion = scene.table.get_world_pose()
    table_position = np.array(table_position, dtype=np.float64)
    table_position[2] = target_top_z - (scene.scene_config.table_scale[2] / 2.0)
    scene.table.set_world_pose(
        position=np.array(table_position, dtype=np.float32),
        orientation=np.array(table_quaternion, dtype=np.float32),
    )

    cube_size = float(layout.get("cube_size", 0.06))
    cube_position = np.array(layout.get("cube_position", scene.get_brick_pose().position), dtype=np.float64)
    cube_position[2] = target_top_z + (cube_size / 2.0)
    scene.brick.set_world_pose(position=np.array(cube_position, dtype=np.float32))
    scene.brick.set_linear_velocity(np.zeros(3, dtype=np.float32))
    scene.brick.set_angular_velocity(np.zeros(3, dtype=np.float32))

    target_center = np.array(layout.get("target_center", cube_position), dtype=np.float64)
    target_center[2] = target_top_z + 0.002
    spawn_range_center = np.array(layout.get("spawn_range_center", target_center), dtype=np.float64)
    spawn_range_center[2] = target_top_z + 0.003
    target_range_center = np.array(layout.get("target_range_center", target_center), dtype=np.float64)
    target_range_center[2] = target_top_z + 0.004

    layout["cube_position"] = np.array(cube_position, dtype=np.float32)
    layout["target_center"] = np.array(target_center, dtype=np.float32)
    layout["spawn_range_center"] = np.array(spawn_range_center, dtype=np.float32)
    layout["target_range_center"] = np.array(target_range_center, dtype=np.float32)
    layout["table_top_z"] = target_top_z
    layout["overlay_z"] = target_top_z + 0.002
    scene.initial_brick_position = np.array(cube_position, dtype=np.float32)

    target_path = scene.layout_visual_prim_paths.get("target")
    cube_spawn_range_path = scene.layout_visual_prim_paths.get("cube_spawn_range")
    target_range_path = scene.layout_visual_prim_paths.get("target_range")
    if target_path:
        _set_prim_translate(scene, target_path, target_center)
    if cube_spawn_range_path:
        _set_prim_translate(scene, cube_spawn_range_path, spawn_range_center)
    if target_range_path:
        _set_prim_translate(scene, target_range_path, target_range_center)

    actual_table_top_z = _current_table_top_z(scene)
    print(f"[table] requested_table_top_z={round(target_top_z, 6)}")
    print(f"[table] actual_table_top_z={round(actual_table_top_z, 6)}")
    return actual_table_top_z


def _target_contains_cube(geometry: CoordinatePushGeometry, cube_center: np.ndarray) -> bool:
    half_size = geometry.target_size_xy / 2.0
    return bool(
        abs(float(cube_center[0] - geometry.target_center[0])) <= float(half_size[0])
        and abs(float(cube_center[1] - geometry.target_center[1])) <= float(half_size[1])
    )


def _build_coordinate_push_geometry(scene_bundle: CheckPosePushCubeSceneBundle) -> CoordinatePushGeometry:
    scene = scene_bundle.scene
    layout = scene.pushcube_layout or {}
    cube_center = np.array(scene.get_brick_pose().position, dtype=np.float64)
    target_center = np.array(layout.get("target_center", cube_center), dtype=np.float64)
    target_size_full = np.array(layout.get("target_size", [0.16, 0.16, 0.002]), dtype=np.float64)
    target_size_xy = np.array(target_size_full[:2], dtype=np.float64)
    table_top_z = float(layout.get("table_top_z", scene.scene_config.table_height))
    cube_size = float(layout.get("cube_size", 0.06))
    cube_xy = np.array(cube_center[:2], dtype=np.float64)
    target_xy = np.array(target_center[:2], dtype=np.float64)
    push_dir = normalize(target_xy - cube_xy)
    behind_xy = cube_xy - (push_dir * 0.08)
    contact_xy = cube_xy - (push_dir * 0.035)
    push_goal_xy = target_xy - (push_dir * 0.02)
    desired_contact_z = float(cube_center[2] + 0.005)
    selected_contact_link, gripper_contact_ref = _get_gripper_contact_reference(scene, cube_center)
    contact_offset_in_ee, ee_reference_rotation = _compute_contact_offset_in_ee(scene, gripper_contact_ref)
    initial_contact_z_error = float(gripper_contact_ref[2] - desired_contact_z)

    print(f"[cfg] cube_center={np.round(cube_center, 4).tolist()}")
    print(f"[cfg] target_center={np.round(target_center, 4).tolist()}")
    print(f"[cfg] table_top_z={round(table_top_z, 6)}")
    print(f"[cfg] cube_size={round(cube_size, 6)}")
    print(f"[cfg] gripper_contact_ref={np.round(gripper_contact_ref, 4).tolist()}")
    print(f"[cfg] cube_center_z={round(float(cube_center[2]), 6)}")
    print(f"[cfg] desired_contact_z={round(desired_contact_z, 6)}")
    print(f"[cfg] gripper_contact_ref_z={round(float(gripper_contact_ref[2]), 6)}")
    print(f"[cfg] initial_contact_z_error={round(initial_contact_z_error, 6)}")

    return CoordinatePushGeometry(
        cube_center=cube_center,
        target_center=target_center,
        target_size_xy=target_size_xy,
        table_top_z=table_top_z,
        cube_size=cube_size,
        push_dir=np.array(push_dir, dtype=np.float64),
        behind_point=np.array([behind_xy[0], behind_xy[1], desired_contact_z], dtype=np.float64),
        contact_point=np.array([contact_xy[0], contact_xy[1], desired_contact_z], dtype=np.float64),
        push_goal_point=np.array([push_goal_xy[0], push_goal_xy[1], desired_contact_z], dtype=np.float64),
        desired_contact_z=desired_contact_z,
        gripper_contact_ref=np.array(gripper_contact_ref, dtype=np.float64),
        contact_offset_in_ee=np.array(contact_offset_in_ee, dtype=np.float64),
        ee_reference_rotation=np.array(ee_reference_rotation, dtype=np.float64),
        selected_contact_link=selected_contact_link,
    )


def _cube_contact_pairs_if_available() -> list[str] | None:
    return None


def _cube_diagnostics(
    scene,
    cube_start_position: np.ndarray,
    geometry: CoordinatePushGeometry,
    contact_offset_in_ee: np.ndarray,
) -> dict[str, object]:
    cube_center = np.array(scene.get_brick_pose().position, dtype=np.float64)
    actual_contact_point = _actual_gripper_contact_point_world(scene, contact_offset_in_ee)
    cube_delta = cube_center - cube_start_position
    cube_delta_xy = cube_delta[:2]
    cube_delta_norm = float(np.linalg.norm(cube_delta_xy))
    cube_to_target_dist = float(np.linalg.norm(cube_center[:2] - geometry.target_center[:2]))
    gripper_to_cube_dist = float(np.linalg.norm(actual_contact_point - cube_center))
    contact_z_error = float(actual_contact_point[2] - cube_center[2])
    cube_table_contact = bool((cube_center[2] - (geometry.cube_size / 2.0)) <= (geometry.table_top_z + 0.01))
    cube_gripper_contact = bool(gripper_to_cube_dist < 0.035)
    return {
        "cube_center": cube_center,
        "cube_delta_from_start": cube_delta,
        "cube_delta_norm": cube_delta_norm,
        "cube_to_target_dist": cube_to_target_dist,
        "actual_gripper_contact_point": actual_contact_point,
        "gripper_to_cube_dist": gripper_to_cube_dist,
        "contact_z_error": contact_z_error,
        "cube_table_contact": cube_table_contact,
        "cube_gripper_contact": cube_gripper_contact,
        "contact_pairs": _cube_contact_pairs_if_available(),
        "cube_in_target": _target_contains_cube(geometry, cube_center),
    }


def _print_fixed_joint_logs(label: str, joint_names: list[str], qpos_initial: np.ndarray, controlled: bool) -> None:
    print(f"[{label}] fixed=True")
    print(f"[{label}] joint_names={joint_names}")
    print(f"[{label}] qpos_initial={_rounded_list(qpos_initial)}")
    print(f"[{label}] controlled={controlled}")


def _safe_joint_bounds(
    right_joint_names: list[str],
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    safe_lower = np.array(lower_limits, dtype=np.float64)
    safe_upper = np.array(upper_limits, dtype=np.float64)
    for index, joint_name in enumerate(right_joint_names):
        if joint_name not in SAFE_DEBUG_JOINT_LIMITS:
            continue
        extra_lower, extra_upper = SAFE_DEBUG_JOINT_LIMITS[joint_name]
        safe_lower[index] = max(float(safe_lower[index]), float(extra_lower))
        safe_upper[index] = min(float(safe_upper[index]), float(extra_upper))
    return safe_lower, safe_upper


def _validate_ik_solution(
    right_joint_names: list[str],
    current_right: np.ndarray,
    solved_joint_positions: np.ndarray | None,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
) -> bool:
    if solved_joint_positions is None:
        print("[safety] IK rejected, command not applied")
        return False
    solved_joint_positions = np.array(solved_joint_positions, dtype=np.float64)
    if not np.all(np.isfinite(solved_joint_positions)):
        print("[safety] IK rejected, command not applied")
        return False
    safe_lower, safe_upper = _safe_joint_bounds(right_joint_names, lower_limits, upper_limits)
    rejected = False
    for index, joint_name in enumerate(right_joint_names):
        joint_value = float(solved_joint_positions[index])
        if joint_value < float(safe_lower[index]) or joint_value > float(safe_upper[index]):
            print(
                "[safety] joint_limit_reject",
                {
                    "joint": joint_name,
                    "value": round(joint_value, 6),
                    "safe_min": round(float(safe_lower[index]), 6),
                    "safe_max": round(float(safe_upper[index]), 6),
                },
            )
            rejected = True
    delta = solved_joint_positions - np.array(current_right, dtype=np.float64)
    for index, joint_name in enumerate(right_joint_names):
        limit = PER_JOINT_STEP_LIMITS.get(joint_name, 0.08)
        if abs(float(delta[index])) > float(limit):
            print(
                "[safety] delta_reject",
                {
                    "joint": joint_name,
                    "delta": round(float(delta[index]), 6),
                    "max_delta": round(float(limit), 6),
                },
            )
            rejected = True
    if rejected:
        print("[safety] IK rejected, command not applied")
        return False
    return True


def _update_push_waypoint_markers(scene_bundle: CheckPosePushCubeSceneBundle, geometry: CoordinatePushGeometry) -> None:
    scene = scene_bundle.scene
    marker_points = {
        "behind": geometry.behind_point,
        "contact": geometry.contact_point,
        "push_goal": geometry.push_goal_point,
    }
    marker_size = np.array([0.015, 0.015, 0.004], dtype=np.float32)
    for key, center in marker_points.items():
        prim_path, color = PUSH_WAYPOINT_MARKERS[key]
        prim = scene._get_prim_at_path(prim_path)
        if prim is None or not prim.IsValid():
            scene._create_visual_box(
                prim_path,
                np.array(center, dtype=np.float32),
                marker_size,
                color,
                0.55,
            )
        else:
            _set_prim_translate(scene, prim_path, center)
            _set_prim_scale(scene, prim_path, marker_size)


def _log_coordinate_push_status(
    *,
    step: int,
    phase_name: str,
    geometry: CoordinatePushGeometry,
    desired_contact_point: np.ndarray,
    diagnostics: dict[str, object],
    left_drift: float,
    gripper_drift: float,
) -> None:
    print(
        "[coordinate-push]",
        {
            "step": step,
            "phase": phase_name,
            "cube_center": np.round(np.array(diagnostics["cube_center"], dtype=np.float64), 4).tolist(),
            "target_center": np.round(geometry.target_center, 4).tolist(),
            "push_dir": np.round(geometry.push_dir, 4).tolist(),
            "desired_contact_point": np.round(np.array(desired_contact_point, dtype=np.float64), 4).tolist(),
            "actual_gripper_contact_point": np.round(
                np.array(diagnostics["actual_gripper_contact_point"], dtype=np.float64),
                4,
            ).tolist(),
            "gripper_to_cube_dist": round(float(diagnostics["gripper_to_cube_dist"]), 6),
            "contact_z_error": round(float(diagnostics["contact_z_error"]), 6),
            "cube_delta_from_start": np.round(
                np.array(diagnostics["cube_delta_from_start"], dtype=np.float64),
                4,
            ).tolist(),
            "cube_to_target_dist": round(float(diagnostics["cube_to_target_dist"]), 6),
            "cube_gripper_contact": bool(diagnostics["cube_gripper_contact"]),
            "left_arm_drift": round(float(left_drift), 6),
            "gripper_qpos_drift": round(float(gripper_drift), 6),
        },
    )
    print(f"[contact] cube_table_contact={diagnostics['cube_table_contact']}")
    print(f"[contact] cube_gripper_contact={diagnostics['cube_gripper_contact']}")
    print(f"[push] cube_delta_norm={float(diagnostics['cube_delta_norm']):.6f}")
    print(f"[push] pushed_success={float(diagnostics['cube_delta_norm']) > 0.02}")


def _build_joint_debug_phase_plan(
    motion_mode: str,
    right_initial: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    args: argparse.Namespace,
) -> tuple[str, list[JointPhaseSpec]]:
    if motion_mode == "none":
        return motion_mode, [
            JointPhaseSpec(
                name="hold_initial",
                duration=max(1, int(args.max_steps)),
                right_target=np.array(right_initial, dtype=np.float64),
            )
        ]
    if motion_mode == "scripted_push":
        print("[motion] scripted_push currently aliases coordinate_push")
        return "coordinate_push", []
    return motion_mode, _build_joint_debug_plan(
        right_initial=right_initial,
        lower_limits=lower_limits,
        upper_limits=upper_limits,
        joint_scale=args.joint_scale,
        max_total_joint_delta=args.max_total_joint_delta,
    )


def _run_joint_debug_loop(
    scene_bundle: CheckPosePushCubeSceneBundle,
    args: argparse.Namespace,
    phase_plan: list[JointPhaseSpec],
    initial_full_positions: np.ndarray,
    right_joint_names: list[str],
    right_indices: np.ndarray,
    right_initial: np.ndarray,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
) -> dict[str, object]:
    scene = scene_bundle.scene
    geometry = _build_coordinate_push_geometry(scene_bundle)
    commanded_right = np.array(right_initial, dtype=np.float64)
    cube_start_position = np.array(scene.get_brick_pose().position, dtype=np.float64)
    warning_state: dict[str, int] = {}
    max_measured_right_movement = 0.0
    phase_index = 0
    phase_remaining = phase_plan[0].duration if phase_plan else 0
    phase_name = phase_plan[0].name if phase_plan else "idle"
    print("[lifecycle] entering motion loop")
    for step in range(max(0, int(args.max_steps))):
        phase = phase_plan[min(phase_index, len(phase_plan) - 1)]
        phase_name = phase.name
        commanded_right = _step_toward_target(
            current_command=commanded_right,
            desired_target=phase.right_target,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            max_joint_delta_per_step=min(float(args.max_joint_delta_per_step), COORDINATE_PUSH_JOINT_STEP_LIMIT),
        )
        full_target = _compose_full_target(
            initial_full_positions=initial_full_positions,
            right_indices=right_indices,
            right_target=commanded_right,
            left_indices=left_indices,
            left_initial=left_initial,
            gripper_indices=gripper_indices,
            gripper_initial=gripper_initial,
        )
        _apply_full_target(scene, full_target)
        scene.step_world(steps=1)
        left_drift, gripper_drift = _enforce_fixed_subsets(
            scene,
            left_indices,
            left_initial,
            gripper_indices,
            gripper_initial,
            step=step,
            warning_state=warning_state,
        )
        right_current = _read_joint_positions(scene, right_indices)
        previous_gripper_to_cube_dist = float(state.last_gripper_to_cube_dist)
        diagnostics = _cube_diagnostics(
            scene,
            cube_start_position=cube_start_position,
            geometry=geometry,
            contact_offset_in_ee=geometry.contact_offset_in_ee,
        )
        max_measured_right_movement = max(
            max_measured_right_movement,
            float(np.max(np.abs(right_current - right_initial))) if right_current.size else 0.0,
        )
        if step % 20 == 0:
            print(
                "[motion] status",
                {
                    "step": step,
                    "phase": phase_name,
                    "right_joint_current_positions": _rounded_named_values(right_joint_names, right_current),
                    "right_joint_target_positions": _rounded_named_values(right_joint_names, commanded_right),
                    "max_abs_target_delta": round(
                        float(np.max(np.abs(commanded_right - right_initial))) if commanded_right.size else 0.0,
                        6,
                    ),
                    "measured_joint_movement": round(max_measured_right_movement, 6),
                    "gripper_qpos_drift": round(gripper_drift, 6),
                    "left_arm_qpos_drift": round(left_drift, 6),
                    "cube_pose": np.round(np.array(diagnostics["cube_center"], dtype=np.float64), 4).tolist(),
                    "cube_delta_from_start": np.round(
                        np.array(diagnostics["cube_delta_from_start"], dtype=np.float64),
                        4,
                    ).tolist(),
                    "cube_to_target_dist": round(float(diagnostics["cube_to_target_dist"]), 6),
                    "cube_contact_pairs": diagnostics["contact_pairs"] if diagnostics["contact_pairs"] is not None else "unavailable",
                },
            )
            print(f"[contact] cube_table_contact={diagnostics['cube_table_contact']}")
            print(f"[contact] cube_gripper_contact={diagnostics['cube_gripper_contact']}")
            print(f"[push] cube_delta_norm={float(diagnostics['cube_delta_norm']):.6f}")
            print(f"[push] pushed_success={float(diagnostics['cube_delta_norm']) > 0.02}")
        phase_remaining -= 1
        if phase_remaining <= 0 and phase_index < len(phase_plan) - 1:
            phase_index += 1
            phase_remaining = phase_plan[phase_index].duration
    print("[lifecycle] motion loop ended")
    return {
        "motion_mode": "joint_debug",
        "ik_success_count": 0,
        "ik_fail_count": 0,
        "safety_reject_count": 0,
        "moved_robot": bool(max_measured_right_movement > 1e-6),
        "cube_delta_norm": 0.0,
        "stopped_reason": "joint_debug_completed",
    }


def _transition_coordinate_phase(
    state: CoordinatePushState,
    actual_contact_point: np.ndarray,
    geometry: CoordinatePushGeometry,
    diagnostics: dict[str, object],
) -> CoordinatePushState:
    next_index = state.phase_index + 1
    if state.phase_name == "push_forward" and bool(diagnostics["cube_in_target"]):
        next_index = 3
    phase_names = ["move_to_behind", "approach_cube", "push_forward", "hold_final"]
    phase_durations = [120, 80, 180, 60]
    phase_targets = [
        geometry.behind_point,
        geometry.contact_point,
        geometry.push_goal_point,
        geometry.push_goal_point,
    ]
    if next_index >= len(phase_names):
        return CoordinatePushState(
            phase_name="done",
            phase_index=len(phase_names),
            step_in_phase=0,
            phase_duration=0,
            phase_start_contact=np.array(
                [actual_contact_point[0], actual_contact_point[1], geometry.desired_contact_z],
                dtype=np.float64,
            ),
            phase_end_contact=np.array(actual_contact_point, dtype=np.float64),
            desired_contact_point=np.array(
                [actual_contact_point[0], actual_contact_point[1], geometry.desired_contact_z],
                dtype=np.float64,
            ),
            last_gripper_to_cube_dist=float(state.last_gripper_to_cube_dist),
            min_gripper_to_cube_dist=float(state.min_gripper_to_cube_dist),
            repeated_ik_failures=state.repeated_ik_failures,
            max_measured_right_movement=state.max_measured_right_movement,
            ever_cube_gripper_contact=state.ever_cube_gripper_contact,
            ik_success_count=state.ik_success_count,
            ik_fail_count=state.ik_fail_count,
            safety_reject_count=state.safety_reject_count,
            moved_robot=state.moved_robot,
            stopped_reason=state.stopped_reason,
            nonapproach_count=0,
        )
    return CoordinatePushState(
        phase_name=phase_names[next_index],
        phase_index=next_index,
        step_in_phase=0,
        phase_duration=phase_durations[next_index],
        phase_start_contact=np.array(
            [actual_contact_point[0], actual_contact_point[1], geometry.desired_contact_z],
            dtype=np.float64,
        ),
        phase_end_contact=np.array(phase_targets[next_index], dtype=np.float64),
        desired_contact_point=np.array(
            [actual_contact_point[0], actual_contact_point[1], geometry.desired_contact_z],
            dtype=np.float64,
        ),
        last_gripper_to_cube_dist=float(state.last_gripper_to_cube_dist),
        min_gripper_to_cube_dist=float(state.min_gripper_to_cube_dist),
        repeated_ik_failures=state.repeated_ik_failures,
        max_measured_right_movement=state.max_measured_right_movement,
        ever_cube_gripper_contact=state.ever_cube_gripper_contact,
        ik_success_count=state.ik_success_count,
        ik_fail_count=state.ik_fail_count,
        safety_reject_count=state.safety_reject_count,
        moved_robot=state.moved_robot,
        stopped_reason=state.stopped_reason,
        nonapproach_count=0,
    )


def _run_coordinate_push_loop(
    scene_bundle: CheckPosePushCubeSceneBundle,
    args: argparse.Namespace,
    initial_full_positions: np.ndarray,
    right_indices: np.ndarray,
    right_initial: np.ndarray,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
) -> dict[str, object]:
    scene = scene_bundle.scene
    right_joint_names = get_right_arm_joint_names(scene_bundle)
    geometry = _build_coordinate_push_geometry(scene_bundle)
    if args.show_push_waypoints:
        _update_push_waypoint_markers(scene_bundle, geometry)
    cube_start_position = np.array(scene.get_brick_pose().position, dtype=np.float64)
    initial_contact_point = _actual_gripper_contact_point_world(scene, geometry.contact_offset_in_ee)
    state = CoordinatePushState(
        phase_name="move_to_behind",
        phase_index=0,
        step_in_phase=0,
        phase_duration=120,
        phase_start_contact=np.array(
            [initial_contact_point[0], initial_contact_point[1], geometry.desired_contact_z],
            dtype=np.float64,
        ),
        phase_end_contact=np.array(geometry.behind_point, dtype=np.float64),
        desired_contact_point=np.array(
            [initial_contact_point[0], initial_contact_point[1], geometry.desired_contact_z],
            dtype=np.float64,
        ),
        last_gripper_to_cube_dist=float(np.linalg.norm(initial_contact_point - geometry.cube_center)),
        min_gripper_to_cube_dist=float(np.linalg.norm(initial_contact_point - geometry.cube_center)),
        repeated_ik_failures=0,
        max_measured_right_movement=0.0,
        ever_cube_gripper_contact=False,
        ik_success_count=0,
        ik_fail_count=0,
        safety_reject_count=0,
        moved_robot=False,
        stopped_reason=None,
        nonapproach_count=0,
    )
    commanded_right = np.array(right_initial, dtype=np.float64)
    warning_state: dict[str, int] = {}
    print("[lifecycle] entering motion loop")
    for step in range(max(0, int(args.max_steps))):
        if state.phase_name == "done":
            break
        current_right = _read_joint_positions(scene, right_indices)
        commanded_right = np.array(current_right, dtype=np.float64)
        progress = min(1.0, float(state.step_in_phase + 1) / float(max(1, state.phase_duration)))
        desired_contact_point = state.phase_start_contact + progress * (state.phase_end_contact - state.phase_start_contact)
        contact_step = desired_contact_point - state.desired_contact_point
        contact_step_norm = float(np.linalg.norm(contact_step))
        if contact_step_norm > COORDINATE_PUSH_CART_STEP_LIMIT:
            desired_contact_point = state.desired_contact_point + (
                contact_step / max(contact_step_norm, 1e-9) * COORDINATE_PUSH_CART_STEP_LIMIT
            )
        desired_ee_pose = _desired_ee_pose_for_contact(
            desired_contact_point=desired_contact_point,
            contact_offset_in_ee=geometry.contact_offset_in_ee,
            ee_reference_rotation=geometry.ee_reference_rotation,
        )
        ik_solution, ik_success = scene.solve_ik(
            desired_ee_pose,
            warm_start=commanded_right,
            position_tolerance=0.01,
            orientation_tolerance=0.15,
        )
        solution_valid = ik_success and _validate_ik_solution(
            right_joint_names=right_joint_names,
            current_right=current_right,
            solved_joint_positions=ik_solution,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )
        if solution_valid:
            state.repeated_ik_failures = 0
            state.ik_success_count += 1
            commanded_right = _step_toward_target(
                current_command=current_right,
                desired_target=np.array(ik_solution, dtype=np.float64),
                lower_limits=lower_limits,
                upper_limits=upper_limits,
                max_joint_delta_per_step=min(float(args.max_joint_delta_per_step), COORDINATE_PUSH_JOINT_STEP_LIMIT),
            )
            state.moved_robot = state.moved_robot or bool(
                float(np.max(np.abs(commanded_right - current_right))) > 1e-6
            )
        else:
            if not ik_success:
                print("[safety] IK rejected, command not applied")
            state.repeated_ik_failures += 1
            state.ik_fail_count += 1
            state.safety_reject_count += 1
            commanded_right = np.array(current_right, dtype=np.float64)
            if state.repeated_ik_failures >= 3:
                state.stopped_reason = "ik_fail_count_exceeded"
                break
        full_target = _compose_full_target(
            initial_full_positions=initial_full_positions,
            right_indices=right_indices,
            right_target=commanded_right,
            left_indices=left_indices,
            left_initial=left_initial,
            gripper_indices=gripper_indices,
            gripper_initial=gripper_initial,
        )
        _apply_full_target(scene, full_target)
        scene.step_world(steps=1)
        left_drift, gripper_drift = _enforce_fixed_subsets(
            scene,
            left_indices,
            left_initial,
            gripper_indices,
            gripper_initial,
            step=step,
            warning_state=warning_state,
        )
        right_current = _read_joint_positions(scene, right_indices)
        state.max_measured_right_movement = max(
            state.max_measured_right_movement,
            float(np.max(np.abs(right_current - right_initial))) if right_current.size else 0.0,
        )
        diagnostics = _cube_diagnostics(
            scene,
            cube_start_position=cube_start_position,
            geometry=geometry,
            contact_offset_in_ee=geometry.contact_offset_in_ee,
        )
        actual_contact_point = np.array(diagnostics["actual_gripper_contact_point"], dtype=np.float64)
        state.desired_contact_point = np.array(desired_contact_point, dtype=np.float64)
        state.last_gripper_to_cube_dist = float(diagnostics["gripper_to_cube_dist"])
        state.min_gripper_to_cube_dist = min(state.min_gripper_to_cube_dist, state.last_gripper_to_cube_dist)
        state.ever_cube_gripper_contact = state.ever_cube_gripper_contact or bool(diagnostics["cube_gripper_contact"])

        print(
            "[ik]",
            {
                "step": step,
                "phase": state.phase_name,
                "target_contact_point": np.round(desired_contact_point, 4).tolist(),
                "solved_joint_positions": _rounded_list(ik_solution) if ik_solution is not None else None,
                "success": bool(ik_success),
                "actual_gripper_contact_point": np.round(actual_contact_point, 4).tolist(),
                "distance_contact_to_cube": round(float(diagnostics["gripper_to_cube_dist"]), 6),
            },
        )

        if step % 20 == 0:
            _log_coordinate_push_status(
                step=step,
                phase_name=state.phase_name,
                geometry=geometry,
                desired_contact_point=desired_contact_point,
                diagnostics=diagnostics,
                left_drift=left_drift,
                gripper_drift=gripper_drift,
            )
            if abs(float(diagnostics["contact_z_error"])) > 0.03:
                _rate_limited_warning(
                    warning_state,
                    "contact-height",
                    step,
                    "[warning] contact height wrong",
                )
            if (
                state.phase_name == "move_to_behind"
                and step > 0
                and float(diagnostics["gripper_to_cube_dist"]) >= (previous_gripper_to_cube_dist - 0.001)
            ):
                state.nonapproach_count += 1
                _rate_limited_warning(
                    warning_state,
                    "move-to-behind-progress",
                    step,
                    "[warning] gripper not moving toward cube",
                )
                if state.nonapproach_count > 10:
                    print("[safety] gripper not approaching cube; aborting coordinate_push")
                    state.stopped_reason = "gripper_not_approaching_cube"
                    break
            elif state.phase_name == "move_to_behind":
                state.nonapproach_count = 0

        advance_phase = False
        if state.phase_name == "approach_cube" and (
            float(diagnostics["gripper_to_cube_dist"]) < 0.035 or bool(diagnostics["cube_gripper_contact"])
        ):
            advance_phase = True
        elif state.phase_name == "push_forward" and bool(diagnostics["cube_in_target"]):
            advance_phase = True
        elif state.step_in_phase + 1 >= state.phase_duration:
            advance_phase = True

        if advance_phase:
            state = _transition_coordinate_phase(
                state=state,
                actual_contact_point=actual_contact_point,
                geometry=geometry,
                diagnostics=diagnostics,
            )
            if state.phase_name == "done":
                state.stopped_reason = "completed"
                break
        else:
            state.step_in_phase += 1

    final_diagnostics = _cube_diagnostics(
        scene,
        cube_start_position=cube_start_position,
        geometry=geometry,
        contact_offset_in_ee=geometry.contact_offset_in_ee,
    )
    print("[lifecycle] motion loop ended")
    print(f"[push] cube_delta_norm={float(final_diagnostics['cube_delta_norm']):.6f}")
    print(f"[push] pushed_success={float(final_diagnostics['cube_delta_norm']) > 0.02}")
    if float(final_diagnostics["cube_delta_norm"]) <= 0.02:
        reasons = []
        if state.max_measured_right_movement < 0.01:
            reasons.append("right arm did not move")
        if state.min_gripper_to_cube_dist > 0.04:
            reasons.append("gripper never reached cube")
        if abs(float(final_diagnostics["contact_z_error"])) > 0.03:
            reasons.append("gripper too high/low")
        if not state.ever_cube_gripper_contact:
            reasons.append("no cube-gripper contact")
        if not final_diagnostics["cube_table_contact"]:
            reasons.append("cube collision/mass/friction issue")
        elif not reasons:
            reasons.append("cube collision/mass/friction issue")
        print("[push] diagnosis", reasons)
    return {
        "motion_mode": "coordinate_push",
        "ik_success_count": state.ik_success_count,
        "ik_fail_count": state.ik_fail_count,
        "safety_reject_count": state.safety_reject_count,
        "moved_robot": state.moved_robot,
        "cube_delta_norm": float(final_diagnostics["cube_delta_norm"]),
        "min_gripper_to_cube_dist": state.min_gripper_to_cube_dist,
        "stopped_reason": state.stopped_reason or "max_steps_reached",
    }


def _run_table_height_sweep(
    scene_bundle: CheckPosePushCubeSceneBundle,
    args: argparse.Namespace,
    initial_full_positions: np.ndarray,
    right_initial: np.ndarray,
    right_indices: np.ndarray,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
) -> list[TableSweepResult]:
    scene = scene_bundle.scene
    results: list[TableSweepResult] = []
    for table_top_z in TABLE_SWEEP_CANDIDATES:
        _restore_full_pose(scene, initial_full_positions)
        _apply_table_top_z_override(scene_bundle, table_top_z)
        geometry = _build_coordinate_push_geometry(scene_bundle)
        if args.show_push_waypoints:
            _update_push_waypoint_markers(scene_bundle, geometry)
        desired_ee_pose = _desired_ee_pose_for_contact(
            desired_contact_point=geometry.behind_point,
            contact_offset_in_ee=geometry.contact_offset_in_ee,
            ee_reference_rotation=geometry.ee_reference_rotation,
        )
        ik_solution, ik_success = scene.solve_ik(
            desired_ee_pose,
            warm_start=right_initial,
            position_tolerance=0.01,
            orientation_tolerance=0.15,
        )
        final_contact_error = None
        gripper_to_cube_dist = float(np.linalg.norm(geometry.gripper_contact_ref - geometry.cube_center))
        collision_warning = False
        if ik_success and ik_solution is not None:
            full_target = _compose_full_target(
                initial_full_positions=initial_full_positions,
                right_indices=right_indices,
                right_target=np.array(ik_solution, dtype=np.float64),
                left_indices=left_indices,
                left_initial=left_initial,
                gripper_indices=gripper_indices,
                gripper_initial=gripper_initial,
            )
            _apply_full_target(scene, full_target)
            scene.step_world(steps=30)
            actual_contact = _actual_gripper_contact_point_world(scene, geometry.contact_offset_in_ee)
            final_contact_error = float(np.linalg.norm(actual_contact - geometry.behind_point))
            gripper_to_cube_dist = float(np.linalg.norm(actual_contact - geometry.cube_center))
            collision_warning = bool(actual_contact[2] < (geometry.table_top_z + 0.002))
        result = TableSweepResult(
            table_top_z=float(table_top_z),
            cube_center_z=float(geometry.cube_center[2]),
            desired_contact_z=float(geometry.desired_contact_z),
            gripper_ref_z=float(geometry.gripper_contact_ref[2]),
            initial_z_error=float(geometry.gripper_contact_ref[2] - geometry.desired_contact_z),
            ik_success=bool(ik_success),
            final_contact_error=final_contact_error,
            gripper_to_cube_dist=float(gripper_to_cube_dist),
            collision_warning=collision_warning,
        )
        results.append(result)
        print(
            "[table-sweep]",
            {
                "table_top_z": round(result.table_top_z, 6),
                "cube_center_z": round(result.cube_center_z, 6),
                "desired_contact_z": round(result.desired_contact_z, 6),
                "gripper_ref_z": round(result.gripper_ref_z, 6),
                "initial_z_error": round(result.initial_z_error, 6),
                "ik_success": result.ik_success,
                "final_contact_error": None if result.final_contact_error is None else round(result.final_contact_error, 6),
                "gripper_to_cube_dist": round(result.gripper_to_cube_dist, 6),
                "collision_warning": result.collision_warning,
            },
        )
    return results


def _select_best_table_height(results: list[TableSweepResult]) -> tuple[TableSweepResult, str]:
    successful = [result for result in results if result.ik_success and not result.collision_warning]
    if successful:
        best = min(successful, key=lambda result: result.table_top_z)
        return best, "smallest table height with successful behind-point IK and no obvious collision risk"
    best = min(results, key=lambda result: abs(result.initial_z_error))
    return best, "minimum absolute initial contact z error"


def _run_ik_test_only(
    scene_bundle: CheckPosePushCubeSceneBundle,
    args: argparse.Namespace,
    right_initial: np.ndarray,
) -> dict[str, object]:
    geometry = _build_coordinate_push_geometry(scene_bundle)
    if args.show_push_waypoints:
        _update_push_waypoint_markers(scene_bundle, geometry)
    scene = scene_bundle.scene
    right_joint_names = get_right_arm_joint_names(scene_bundle)
    lower_limits, upper_limits = _right_joint_limits(scene_bundle, right_joint_names)
    reachable = 0
    total = 0
    for waypoint_name, contact_point in [
        ("behind", geometry.behind_point),
        ("contact", geometry.contact_point),
        ("push_goal", geometry.push_goal_point),
    ]:
        total += 1
        desired_ee_pose = _desired_ee_pose_for_contact(
            desired_contact_point=contact_point,
            contact_offset_in_ee=geometry.contact_offset_in_ee,
            ee_reference_rotation=geometry.ee_reference_rotation,
        )
        ik_solution, ik_success = scene.solve_ik(
            desired_ee_pose,
            warm_start=right_initial,
            position_tolerance=0.01,
            orientation_tolerance=0.15,
        )
        validated = ik_success and _validate_ik_solution(
            right_joint_names=right_joint_names,
            current_right=right_initial,
            solved_joint_positions=ik_solution,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )
        if validated:
            reachable += 1
        print(
            "[ik-test]",
            {
                "waypoint": waypoint_name,
                "target_contact_point": np.round(np.array(contact_point, dtype=np.float64), 4).tolist(),
                "reachable": bool(validated),
                "solved_joint_positions": None if ik_solution is None else _rounded_list(np.array(ik_solution, dtype=np.float64)),
            },
        )
    return {
        "ik_success_count": reachable,
        "ik_fail_count": total - reachable,
        "safety_reject_count": 0,
        "moved_robot": False,
        "cube_delta_norm": 0.0,
        "stopped_reason": "ik_test_only",
    }


def _print_summary(summary: dict[str, object]) -> None:
    print(f"[summary] ik_success_count={summary.get('ik_success_count', 0)}")
    print(f"[summary] ik_fail_count={summary.get('ik_fail_count', 0)}")
    print(f"[summary] safety_reject_count={summary.get('safety_reject_count', 0)}")
    print(f"[summary] moved_robot={summary.get('moved_robot', False)}")
    print(f"[summary] cube_delta_norm={summary.get('cube_delta_norm', 0.0)}")
    print(f"[summary] stopped_reason={summary.get('stopped_reason', 'unknown')}")


def run_push_cube_debug(args: argparse.Namespace) -> None:
    scene_bundle = None
    summary: dict[str, object] = {
        "ik_success_count": 0,
        "ik_fail_count": 0,
        "safety_reject_count": 0,
        "moved_robot": False,
        "cube_delta_norm": 0.0,
        "stopped_reason": "not_started",
    }
    try:
        scene_bundle = create_check_pose_pushcube_scene(
            headless=args.headless,
            show_ranges=args.show_ranges,
            disable_lula=args.disable_lula,
            hold_open=args.hold_open,
        )
        scene = scene_bundle.scene
        object_handles = get_pushcube_object_handles(scene_bundle)
        print("[lifecycle] scene created")
        print("[scene] object_handles", {name: handle is not None for name, handle in object_handles.items()})
        _print_layout_logs(scene, scene_bundle.runtime_options)

        right_joint_names = get_right_arm_joint_names(scene_bundle)
        left_joint_names = get_left_arm_joint_names(scene_bundle)
        gripper_joint_names = get_gripper_joint_names(scene_bundle)
        right_indices = _joint_indices(scene, right_joint_names)
        left_indices = _joint_indices(scene, left_joint_names)
        gripper_indices = _joint_indices(scene, gripper_joint_names)
        initial_full_positions = np.array(scene.articulation.get_joint_positions(), dtype=np.float64)
        right_initial = np.array(initial_full_positions[right_indices], dtype=np.float64)
        left_initial = np.array(initial_full_positions[left_indices], dtype=np.float64)
        gripper_initial = np.array(initial_full_positions[gripper_indices], dtype=np.float64)
        _apply_table_top_z_override(scene_bundle, args.table_top_z)

        _print_fixed_joint_logs("gripper", gripper_joint_names, gripper_initial, controlled=False)
        _print_fixed_joint_logs("left-arm", left_joint_names, left_initial, controlled=False)

        lower_limits, upper_limits = _right_joint_limits(scene_bundle, right_joint_names)
        motion_mode = _resolve_motion_mode(args)
        if motion_mode == "scripted_push":
            print("[motion] scripted_push currently aliases coordinate_push")
            motion_mode = "coordinate_push"
        print(f"[motion] requested_mode={motion_mode}")

        if args.sweep_table_height:
            if args.disable_lula or scene.kinematics is None:
                print(
                    "[motion] coordinate_push requires IK/Lula or a valid Cartesian controller. "
                    "Disable --disable-lula or provide --direct-cartesian-debug."
                )
                summary["stopped_reason"] = "coordinate_push_unavailable"
            else:
                results = _run_table_height_sweep(
                    scene_bundle=scene_bundle,
                    args=args,
                    initial_full_positions=initial_full_positions,
                    right_initial=right_initial,
                    right_indices=right_indices,
                    left_indices=left_indices,
                    left_initial=left_initial,
                    gripper_indices=gripper_indices,
                    gripper_initial=gripper_initial,
                )
                best_result, reason = _select_best_table_height(results)
                print(
                    "[table-sweep] recommendation",
                    {
                        "best_table_top_z": round(best_result.table_top_z, 6),
                        "reason": reason,
                    },
                )
                _restore_full_pose(scene, initial_full_positions)
                _apply_table_top_z_override(scene_bundle, best_result.table_top_z)
                if args.show_push_waypoints:
                    _update_push_waypoint_markers(scene_bundle, _build_coordinate_push_geometry(scene_bundle))
                summary["stopped_reason"] = "table_height_sweep_completed"
        elif motion_mode == "coordinate_push" and (args.disable_lula or scene.kinematics is None):
            geometry = _build_coordinate_push_geometry(scene_bundle)
            if args.show_push_waypoints:
                _update_push_waypoint_markers(scene_bundle, geometry)
            print(
                "[motion] coordinate_push requires IK/Lula or a valid Cartesian controller. "
                "Disable --disable-lula or provide --direct-cartesian-debug."
            )
            summary["stopped_reason"] = "coordinate_push_unavailable"
        elif args.ik_test_only:
            if args.disable_lula or scene.kinematics is None:
                geometry = _build_coordinate_push_geometry(scene_bundle)
                if args.show_push_waypoints:
                    _update_push_waypoint_markers(scene_bundle, geometry)
                print(
                    "[motion] coordinate_push requires IK/Lula or a valid Cartesian controller. "
                    "Disable --disable-lula or provide --direct-cartesian-debug."
                )
                summary["stopped_reason"] = "coordinate_push_unavailable"
            else:
                summary = _run_ik_test_only(
                    scene_bundle=scene_bundle,
                    args=args,
                    right_initial=right_initial,
                )
        elif motion_mode != "none":
            articulation_ok = _verify_right_arm_control(
                scene_bundle=scene_bundle,
                initial_full_positions=initial_full_positions,
                right_joint_names=right_joint_names,
                right_indices=right_indices,
                right_initial=right_initial,
                left_indices=left_indices,
                left_initial=left_initial,
                gripper_indices=gripper_indices,
                gripper_initial=gripper_initial,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
            )
            if not articulation_ok:
                print("[motion] push attempt skipped because articulation verification failed")
                summary["stopped_reason"] = "articulation_verification_failed"
            elif motion_mode == "coordinate_push":
                summary = _run_coordinate_push_loop(
                    scene_bundle=scene_bundle,
                    args=args,
                    initial_full_positions=initial_full_positions,
                    right_indices=right_indices,
                    right_initial=right_initial,
                    left_indices=left_indices,
                    left_initial=left_initial,
                    gripper_indices=gripper_indices,
                    gripper_initial=gripper_initial,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                )
            else:
                motion_mode, phase_plan = _build_joint_debug_phase_plan(
                    motion_mode=motion_mode,
                    right_initial=right_initial,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    args=args,
                )
                if motion_mode == "coordinate_push":
                    summary = _run_coordinate_push_loop(
                        scene_bundle=scene_bundle,
                        args=args,
                        initial_full_positions=initial_full_positions,
                        right_indices=right_indices,
                        right_initial=right_initial,
                        left_indices=left_indices,
                        left_initial=left_initial,
                        gripper_indices=gripper_indices,
                        gripper_initial=gripper_initial,
                        lower_limits=lower_limits,
                        upper_limits=upper_limits,
                    )
                else:
                    summary = _run_joint_debug_loop(
                        scene_bundle=scene_bundle,
                        args=args,
                        phase_plan=phase_plan,
                        initial_full_positions=initial_full_positions,
                        right_joint_names=right_joint_names,
                        right_indices=right_indices,
                        right_initial=right_initial,
                        left_indices=left_indices,
                        left_initial=left_initial,
                        gripper_indices=gripper_indices,
                        gripper_initial=gripper_initial,
                        lower_limits=lower_limits,
                        upper_limits=upper_limits,
                    )
        else:
            summary["stopped_reason"] = "motion_mode_none"

        if args.hold_open:
            hold_viewer_open(scene, scene._app, render=not args.headless)
    finally:
        _print_summary(summary)
        if scene_bundle is not None:
            scene_bundle.scene.close()
            print("[lifecycle] closing SimulationApp")
            close_simulation_app(scene_bundle.scene._app)


def main() -> None:
    args = build_arg_parser().parse_args()
    print(f"[push-cube-debug] headless={args.headless}")
    print(f"[push-cube-debug] show_ranges={args.show_ranges}")
    print(f"[push-cube-debug] disable_lula={args.disable_lula}")
    print(f"[push-cube-debug] hold_open={args.hold_open}")
    print(f"[push-cube-debug] max_steps={args.max_steps}")
    run_push_cube_debug(args)


if __name__ == "__main__":
    main()
