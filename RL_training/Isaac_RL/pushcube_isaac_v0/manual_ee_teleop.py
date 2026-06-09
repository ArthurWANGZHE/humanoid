"""Manual end-effector teleoperation for PushCube in Isaac Sim."""

from __future__ import annotations

import atexit
import argparse
import logging
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import select
import sys
import time
import warnings

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from pushcube_isaac_v0.check_pose_scene import (  # noqa: E402
    create_check_pose_pushcube_scene,
    get_gripper_joint_names,
    get_left_arm_joint_names,
    get_pushcube_object_handles,
    get_right_arm_joint_names,
)
from pushcube_isaac_v0.isaac_lifecycle import close_simulation_app  # noqa: E402
from rl_train.check_pose import _parse_bool_arg, _print_layout_logs  # noqa: E402
from rl_train.pose_math import Pose  # noqa: E402


KEY_DELTAS = {
    "q": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    "e": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    "a": np.array([0.0, 1.0, 0.0], dtype=np.float64),
    "d": np.array([0.0, -1.0, 0.0], dtype=np.float64),
    "w": np.array([0.0, 0.0, 1.0], dtype=np.float64),
    "s": np.array([0.0, 0.0, -1.0], dtype=np.float64),
}
GRIPPER_DRIFT_TOLERANCE = 1e-5
LEFT_ARM_DRIFT_TOLERANCE = 1e-5
CONTACT_REPORT_INTERVAL_FRAMES = 30
STATUS_INTERVAL_FRAMES = 120
WARNING_SUMMARY_INTERVAL_FRAMES = 120
LEFT_ARM_WARNING_INTERVAL_FRAMES = 120
GRIPPER_WARNING_INTERVAL_FRAMES = 120
IK_WARNING_INTERVAL_FRAMES = 60
CONTACT_WARNING_INTERVAL_FRAMES = 60
FIXED_SUBSET_RESTORE_DELTA = 0.005
FIXED_SUBSET_EMERGENCY_DRIFT_THRESHOLD = 0.05
INITIAL_POSE_TRANSITION_FRAMES = 120
INITIAL_POSE_PRESET_3_ELBOW_TARGET = 1.68
RIGHT_GRIPPER_LINK_NAMES = ["right_gripper1_link", "right_gripper2_link"]
RIGHT_WRIST_LINK_NAMES = ["right_wrist_yaw_link", "right_wrist_pitch_link"]
LAYOUT_PRESET_DEFAULT = "default"
LAYOUT_PRESET_RIGHT_ARM_CLOSE = "right_arm_close"
LAYOUT_PRESET_RIGHT_ARM_VERY_CLOSE = "right_arm_very_close"
LAYOUT_PRESET_OPPOSITE_EDGES = "opposite_edges"
LAYOUT_CUBE_OFFSET_FROM_TABLE_CENTER = np.array([-0.08, -0.03], dtype=np.float64)
LAYOUT_TARGET_OFFSET_FROM_TABLE_CENTER = np.array([0.08, 0.06], dtype=np.float64)
LAYOUT_EDGE_MARGIN = 0.04
LAYOUT_OPPOSITE_EDGES_SPAWN_SIZE = np.array([0.18, 0.14], dtype=np.float64)
LAYOUT_PRESETS = {
    LAYOUT_PRESET_RIGHT_ARM_CLOSE: {
        "table_center_xy": np.array([0.48, -0.30], dtype=np.float64),
        "table_top_z": 0.40,
    },
    LAYOUT_PRESET_RIGHT_ARM_VERY_CLOSE: {
        "table_center_xy": np.array([0.48, -0.28], dtype=np.float64),
        "table_top_z": 0.40,
    },
    LAYOUT_PRESET_OPPOSITE_EDGES: {
        "table_center_xy": np.array([0.48, -0.30], dtype=np.float64),
        "table_top_z": 0.40,
    },
}


@dataclass
class TeleopState:
    desired_ee_position: np.ndarray
    filtered_ee_position: np.ndarray
    desired_ee_rotation: np.ndarray
    desired_right_joint_target: np.ndarray
    right_joint_target: np.ndarray
    paused: bool
    success_count: int
    failure_count: int
    exit_requested: bool
    status: str
    ik_fail_count: int
    last_ik_failure_frame: int
    last_ik_error: str
    contact_warning_count: int


class TerminalKeyReader:
    def __init__(self, backend: str = "terminal") -> None:
        self.backend = backend
        self._fd = None
        self._old_settings = None
        self.available = False
        self.source = "terminal"
        self._atexit_registered = False

    def setup(self) -> "TerminalKeyReader":
        if self.backend == "line":
            self.available = True
            self.source = "line"
            return self
        if not sys.stdin.isatty():
            self.available = True
            self.source = "line"
            print("[teleop] raw terminal unavailable, using line-input fallback")
            return self
        try:
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self.available = True
            self.source = "terminal"
            if not self._atexit_registered:
                atexit.register(self.close)
                self._atexit_registered = True
        except Exception:
            self.available = True
            self.source = "line"
            print("[teleop] raw terminal unavailable, using line-input fallback")
        return self

    def close(self) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        self._fd = None
        self._old_settings = None

    def _normalize_line_key(self, line: str) -> str | None:
        token = line.strip().lower()
        if not token:
            return None
        if token in {"esc", "escape"}:
            return "escape"
        if token == "space":
            return "space"
        return token[0]

    def poll(self) -> list[str]:
        if not self.available:
            return []
        keys: list[str] = []
        if self.source == "line":
            while True:
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    break
                line = sys.stdin.readline()
                if not line:
                    break
                normalized = self._normalize_line_key(line)
                if normalized is not None:
                    keys.append(normalized)
            return keys
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                break
            key = sys.stdin.read(1)
            if not key:
                break
            if key == "\x1b":
                keys.append("escape")
            elif key == " ":
                keys.append("space")
            else:
                keys.append(key.lower())
        return keys


class ViewerKeyReader:
    def __init__(self) -> None:
        self._queue: deque[str] = deque()
        self._subscription = None
        self.available = False
        try:
            import carb.input  # type: ignore
            import omni.appwindow  # type: ignore

            app_window = omni.appwindow.get_default_app_window()
            if app_window is None:
                return
            keyboard = app_window.get_keyboard()
            if keyboard is None:
                return
            input_iface = carb.input.acquire_input_interface()
            event_type = carb.input.KeyboardEventType
            key_enum = carb.input.KeyboardInput
            key_map = {
                key_enum.KEY_0: "0",
                key_enum.KEY_1: "1",
                key_enum.KEY_2: "2",
                key_enum.KEY_3: "3",
                key_enum.KEY_4: "4",
                key_enum.KEY_5: "5",
                key_enum.KEY_6: "6",
                key_enum.W: "w",
                key_enum.S: "s",
                key_enum.A: "a",
                key_enum.D: "d",
                key_enum.Q: "q",
                key_enum.E: "e",
                key_enum.R: "r",
                key_enum.T: "t",
                key_enum.Y: "y",
                key_enum.Z: "z",
                key_enum.P: "p",
                key_enum.X: "x",
                key_enum.F: "f",
                key_enum.SPACE: "space",
                key_enum.ESCAPE: "escape",
            }

            def _on_keyboard_event(event) -> bool:
                if event.type != event_type.KEY_PRESS:
                    return True
                key_name = key_map.get(event.input)
                if key_name is not None:
                    self._queue.append(key_name)
                return True

            self._subscription = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
            self.available = True
        except Exception:
            self.available = False

    def close(self) -> None:
        self._subscription = None

    def poll(self) -> list[str]:
        keys = list(self._queue)
        self._queue.clear()
        return keys


class ManualEpisodeRecorder:
    def __init__(self, enabled: bool, output_path: Path, episode_name: str) -> None:
        self.enabled = enabled
        self.output_path = output_path
        self.episode_name_base = episode_name
        self.episode_index = 0
        self.reset()

    def reset(self) -> None:
        self.obs: list[np.ndarray] = []
        self.action_xy: list[np.ndarray] = []
        self.ee_desired_position: list[np.ndarray] = []
        self.ee_actual_position: list[np.ndarray] = []
        self.right_arm_joint_pos: list[np.ndarray] = []
        self.right_arm_joint_vel: list[np.ndarray] = []
        self.gripper_qpos: list[np.ndarray] = []
        self.cube_pose: list[np.ndarray] = []
        self.target_pose: list[np.ndarray] = []
        self.key_pressed: list[str] = []
        self.timestamp: list[float] = []

    def record_step(
        self,
        *,
        obs: np.ndarray,
        action_xy: np.ndarray,
        ee_desired_position: np.ndarray,
        ee_actual_position: np.ndarray,
        right_arm_joint_pos: np.ndarray,
        right_arm_joint_vel: np.ndarray,
        gripper_qpos: np.ndarray,
        cube_pose: np.ndarray,
        target_pose: np.ndarray,
        key_pressed: str,
        timestamp: float,
    ) -> None:
        if not self.enabled:
            return
        self.obs.append(np.array(obs, dtype=np.float32))
        self.action_xy.append(np.array(action_xy, dtype=np.float32))
        self.ee_desired_position.append(np.array(ee_desired_position, dtype=np.float32))
        self.ee_actual_position.append(np.array(ee_actual_position, dtype=np.float32))
        self.right_arm_joint_pos.append(np.array(right_arm_joint_pos, dtype=np.float32))
        self.right_arm_joint_vel.append(np.array(right_arm_joint_vel, dtype=np.float32))
        self.gripper_qpos.append(np.array(gripper_qpos, dtype=np.float32))
        self.cube_pose.append(np.array(cube_pose, dtype=np.float32))
        self.target_pose.append(np.array(target_pose, dtype=np.float32))
        self.key_pressed.append(key_pressed)
        self.timestamp.append(float(timestamp))

    def _episode_name(self) -> str:
        if self.episode_index == 0:
            return self.episode_name_base
        return f"{self.episode_name_base}_{self.episode_index:06d}"

    def save(self, label: str) -> None:
        if not self.enabled or not self.obs:
            return
        import h5py  # type: ignore

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        episode_name = self._episode_name()
        with h5py.File(self.output_path, "a") as h5_file:
            if episode_name in h5_file:
                del h5_file[episode_name]
            group = h5_file.create_group(episode_name)
            group.create_dataset("obs", data=np.stack(self.obs, axis=0))
            group.create_dataset("action", data=np.stack(self.action_xy, axis=0))
            group.create_dataset("ee_desired_position", data=np.stack(self.ee_desired_position, axis=0))
            group.create_dataset("ee_actual_position", data=np.stack(self.ee_actual_position, axis=0))
            group.create_dataset("right_arm_joint_pos", data=np.stack(self.right_arm_joint_pos, axis=0))
            group.create_dataset("right_arm_joint_vel", data=np.stack(self.right_arm_joint_vel, axis=0))
            group.create_dataset("gripper_qpos", data=np.stack(self.gripper_qpos, axis=0))
            group.create_dataset("cube_pose", data=np.stack(self.cube_pose, axis=0))
            group.create_dataset("target_pose", data=np.stack(self.target_pose, axis=0))
            group.create_dataset("key_pressed", data=np.array(self.key_pressed, dtype="S16"))
            group.create_dataset("timestamp", data=np.array(self.timestamp, dtype=np.float64))
            group.attrs["label"] = label
        print(f"[record] saved episode={episode_name} label={label} output={self.output_path}")
        self.episode_index += 1
        self.reset()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manual end-effector teleoperation for PushCube.")
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
    parser.add_argument("--max-joint-delta", type=float, default=0.005)
    parser.add_argument("--expand-right-arm-limits", type=float, default=0.5)
    parser.add_argument("--initial-pose-preset", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--target-size", type=float, default=0.22)
    parser.add_argument("--success-margin", type=float, default=0.02)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--input-backend", choices=["terminal", "viewer", "auto", "line"], default="terminal")
    parser.add_argument("--control-mode", choices=["joint", "ee"], default="joint")
    parser.add_argument("--inspect-collisions", action="store_true")
    parser.add_argument("--print-contact-pairs", action="store_true")
    parser.add_argument("--quiet-warnings", type=_parse_bool_arg, nargs="?", const=True, default=True)
    parser.add_argument("--verbose-warnings", action="store_true")
    parser.add_argument("--suppress-isaac-warnings", type=_parse_bool_arg, nargs="?", const=True, default=False)
    parser.add_argument("--status-every", type=int, default=STATUS_INTERVAL_FRAMES)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--output", type=str, default="data/simulation/isaac_rl/datasets/manual_pushcube_demo.hdf5")
    parser.add_argument("--episode-name", type=str, default="demo_000000")
    parser.add_argument("--save-failed", action="store_true")
    parser.add_argument("--save-video", type=str, default=None)
    return parser


def _rounded_list(values: np.ndarray) -> list[float]:
    return [round(float(value), 6) for value in np.array(values, dtype=np.float64).tolist()]


def _rounded_xy_list(values: np.ndarray, digits: int = 2) -> list[float]:
    return [round(float(value), digits) for value in np.array(values, dtype=np.float64).tolist()]


def _rounded_named_values(names: list[str], values: np.ndarray) -> dict[str, float]:
    return {
        joint_name: round(float(value), 6)
        for joint_name, value in zip(names, np.array(values, dtype=np.float64).tolist(), strict=False)
    }


def _apply_warning_filters(suppress_isaac_warnings: bool) -> None:
    if not suppress_isaac_warnings:
        return
    for logger_name in ("omni", "omni.isaac", "omni.replicator", "isaacsim", "pxr"):
        logging.getLogger(logger_name).setLevel(logging.ERROR)
    warning_patterns = (
        r".*deprecated omni\.isaac.*",
        r".*Replicator.*",
        r".*pxr\.Semantics.*deprecated.*",
        r".*gpu\.foundation\.plugin.*memory budget.*",
    )
    for pattern in warning_patterns:
        warnings.filterwarnings("ignore", message=pattern)


def _joint_indices(scene, joint_names: list[str]) -> np.ndarray:
    return np.array([scene.dof_names.index(joint_name) for joint_name in joint_names], dtype=np.int32)


def _read_joint_positions(scene, joint_indices: np.ndarray) -> np.ndarray:
    joint_positions = np.array(scene.articulation.get_joint_positions(), dtype=np.float64)
    return np.array(joint_positions[joint_indices], dtype=np.float64)


def _read_joint_velocities(scene, joint_indices: np.ndarray) -> np.ndarray:
    joint_velocities = np.array(scene.articulation.get_joint_velocities(), dtype=np.float64)
    return np.array(joint_velocities[joint_indices], dtype=np.float64)


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


def _rate_limited_restore_subset(
    scene,
    joint_indices: np.ndarray,
    current_positions: np.ndarray,
    target_positions: np.ndarray,
    *,
    max_delta: float,
) -> None:
    if joint_indices.size == 0:
        return
    corrected_positions = np.array(current_positions, dtype=np.float64) + np.clip(
        np.array(target_positions, dtype=np.float64) - np.array(current_positions, dtype=np.float64),
        -abs(float(max_delta)),
        abs(float(max_delta)),
    )
    scene.articulation.set_joint_positions(
        np.array(corrected_positions, dtype=np.float64),
        joint_indices=np.array(joint_indices, dtype=np.int32),
    )
    scene.articulation.set_joint_velocities(
        np.zeros(len(joint_indices), dtype=np.float64),
        joint_indices=np.array(joint_indices, dtype=np.int32),
    )


def _warning_bucket(warning_state: dict[str, dict[str, int]], key: str) -> dict[str, int]:
    bucket = warning_state.get(key)
    if bucket is None:
        bucket = {"last_frame": -1, "count": 0}
        warning_state[key] = bucket
    return bucket


def _emit_warning(
    warning_state: dict[str, dict[str, int]],
    key: str,
    *,
    frame: int,
    message: str,
    interval_frames: int,
    quiet_warnings: bool,
) -> None:
    _ = quiet_warnings
    bucket = _warning_bucket(warning_state, key)
    bucket["count"] += 1
    last_frame = int(bucket.get("last_frame", -interval_frames))
    if frame - last_frame < interval_frames:
        return
    bucket["last_frame"] = frame
    print(message)


def _warning_count(warning_state: dict[str, dict[str, int]], key: str) -> int:
    return int(_warning_bucket(warning_state, key).get("count", 0))


def _enforce_fixed_subsets(
    scene,
    *,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
    frame: int,
    warning_state: dict[str, dict[str, int]],
    quiet_warnings: bool,
) -> tuple[float, float]:
    left_current = _read_joint_positions(scene, left_indices)
    gripper_current = _read_joint_positions(scene, gripper_indices)
    left_drift = float(np.max(np.abs(left_current - left_initial))) if left_current.size else 0.0
    gripper_drift = float(np.max(np.abs(gripper_current - gripper_initial))) if gripper_current.size else 0.0
    if left_drift > FIXED_SUBSET_EMERGENCY_DRIFT_THRESHOLD:
        _emit_warning(
            warning_state,
            "left_arm_emergency",
            frame=frame,
            message=f"[left-arm] ERROR fixed left arm drift too large: max_abs_drift={left_drift:.6f}",
            interval_frames=LEFT_ARM_WARNING_INTERVAL_FRAMES,
            quiet_warnings=False,
        )
        _rate_limited_restore_subset(
            scene,
            left_indices,
            current_positions=left_current,
            target_positions=left_initial,
            max_delta=FIXED_SUBSET_RESTORE_DELTA,
        )
        left_drift = float(np.max(np.abs(_read_joint_positions(scene, left_indices) - left_initial))) if left_initial.size else 0.0
    elif not quiet_warnings and left_drift > LEFT_ARM_DRIFT_TOLERANCE:
        _emit_warning(
            warning_state,
            "left_arm_drift",
            frame=frame,
            message=f"[warning] left arm drift detected; target hold active (max_abs_drift={left_drift:.6f})",
            interval_frames=LEFT_ARM_WARNING_INTERVAL_FRAMES,
            quiet_warnings=False,
        )
    if gripper_drift > FIXED_SUBSET_EMERGENCY_DRIFT_THRESHOLD:
        _emit_warning(
            warning_state,
            "gripper_emergency",
            frame=frame,
            message=f"[gripper] ERROR fixed gripper drift too large: max_abs_drift={gripper_drift:.6f}",
            interval_frames=GRIPPER_WARNING_INTERVAL_FRAMES,
            quiet_warnings=False,
        )
        _rate_limited_restore_subset(
            scene,
            gripper_indices,
            current_positions=gripper_current,
            target_positions=gripper_initial,
            max_delta=FIXED_SUBSET_RESTORE_DELTA,
        )
        gripper_drift = float(
            np.max(np.abs(_read_joint_positions(scene, gripper_indices) - gripper_initial))
        ) if gripper_initial.size else 0.0
    elif not quiet_warnings and gripper_drift > GRIPPER_DRIFT_TOLERANCE:
        _emit_warning(
            warning_state,
            "gripper_drift",
            frame=frame,
            message=f"[warning] gripper drift detected; target hold active (max_abs_drift={gripper_drift:.6f})",
            interval_frames=GRIPPER_WARNING_INTERVAL_FRAMES,
            quiet_warnings=False,
        )
    return left_drift, gripper_drift


def _right_joint_limits(
    scene_bundle,
    right_joint_names: list[str],
    *,
    expand_by: float,
) -> tuple[np.ndarray, np.ndarray]:
    lower = []
    upper = []
    original_limits: dict[str, dict[str, float]] = {}
    expanded_limits: dict[str, dict[str, float]] = {}
    active_limits: dict[str, dict[str, float]] = {}
    urdf_clamped = False
    for joint_name in right_joint_names:
        limit = scene_bundle.training_config.joint_limits.get(joint_name, {})
        original_lower = float(limit.get("min_position", -3.14))
        original_upper = float(limit.get("max_position", 3.14))
        requested_lower = original_lower - abs(float(expand_by))
        requested_upper = original_upper + abs(float(expand_by))
        urdf_lower = float(limit.get("urdf_min_position", requested_lower))
        urdf_upper = float(limit.get("urdf_max_position", requested_upper))
        active_lower = max(requested_lower, urdf_lower)
        active_upper = min(requested_upper, urdf_upper)
        lower.append(active_lower)
        upper.append(active_upper)
        original_limits[joint_name] = {"min": round(original_lower, 4), "max": round(original_upper, 4)}
        expanded_limits[joint_name] = {"min": round(requested_lower, 4), "max": round(requested_upper, 4)}
        active_limits[joint_name] = {"min": round(active_lower, 4), "max": round(active_upper, 4)}
        urdf_clamped = urdf_clamped or not (
            np.isclose(active_lower, requested_lower) and np.isclose(active_upper, requested_upper)
        )
    print(f"[joint-limits] original={original_limits}")
    print(f"[joint-limits] expanded={expanded_limits}")
    print(f"[joint-limits] urdf_clamped={urdf_clamped}")
    print(f"[joint-limits] active_limits={active_limits}")
    return np.array(lower, dtype=np.float64), np.array(upper, dtype=np.float64)


def _filter_joint_target(
    current_command: np.ndarray,
    desired_target: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    joint_alpha: float,
    max_joint_delta: float,
) -> np.ndarray:
    current_command = np.array(current_command, dtype=np.float64)
    desired_target = np.clip(np.array(desired_target, dtype=np.float64), lower_limits, upper_limits)
    blended_target = (float(joint_alpha) * desired_target) + ((1.0 - float(joint_alpha)) * current_command)
    step_delta = blended_target - current_command
    step_delta = np.clip(step_delta, -abs(float(max_joint_delta)), abs(float(max_joint_delta)))
    return np.clip(current_command + step_delta, lower_limits, upper_limits)


def _cube_and_target_xy(scene) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cube_pose = scene.get_brick_pose()
    cube_xy = np.array(cube_pose.position[:2], dtype=np.float64)
    target_center = np.array((scene.pushcube_layout or {}).get("target_center", [0.0, 0.0, 0.0]), dtype=np.float64)
    target_xy = np.array(target_center[:2], dtype=np.float64)
    target_size = np.array((scene.pushcube_layout or {}).get("target_size", [0.16, 0.16, 0.002]), dtype=np.float64)
    return cube_xy, target_xy, target_size[:2]


def _cube_yaw(scene) -> float:
    cube_pose = scene.get_brick_pose()
    rotation = np.array(cube_pose.rotation, dtype=np.float64)
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def _current_table_top_z(scene) -> float:
    table_position, _ = scene.table.get_world_pose()
    return float(np.array(table_position, dtype=np.float64)[2] + (scene.scene_config.table_scale[2] / 2.0))


def _clamp_desired_ee_position(position: np.ndarray, table_top_z: float) -> np.ndarray:
    clamped = np.array(position, dtype=np.float64)
    clamped[0] = np.clip(clamped[0], 0.25, 0.70)
    clamped[1] = np.clip(clamped[1], -0.35, 0.20)
    clamped[2] = np.clip(clamped[2], table_top_z + 0.04, table_top_z + 0.30)
    return clamped


def _cube_gripper_contact(scene) -> bool:
    cube_position = np.array(scene.get_brick_pose().position, dtype=np.float64)
    ee_position = np.array(scene.get_end_effector_pose().position, dtype=np.float64)
    return bool(np.linalg.norm(ee_position - cube_position) < 0.08)


def _compute_observation(scene, ee_position: np.ndarray) -> np.ndarray:
    cube_xy, target_xy, target_size_xy = _cube_and_target_xy(scene)
    rel_ee_to_cube = cube_xy - np.array(ee_position[:2], dtype=np.float64)
    rel_cube_to_target = target_xy - cube_xy
    return np.array(
        [
            float(ee_position[0]),
            float(ee_position[1]),
            float(cube_xy[0]),
            float(cube_xy[1]),
            _cube_yaw(scene),
            float(target_xy[0]),
            float(target_xy[1]),
            float(target_size_xy[0]),
            float(target_size_xy[1]),
            float(rel_ee_to_cube[0]),
            float(rel_ee_to_cube[1]),
            float(rel_cube_to_target[0]),
            float(rel_cube_to_target[1]),
        ],
        dtype=np.float32,
    )


def _print_state(
    scene,
    *,
    frame: int,
    desired_ee_position: np.ndarray,
    gripper_qpos: np.ndarray,
    right_joint_names: list[str],
    right_joint_positions: np.ndarray,
    left_arm_drift: float,
    gripper_qpos_drift: float,
    paused: bool,
    status: str,
    control_mode: str,
    ik_fail_count: int,
    last_ik_error: str,
) -> None:
    cube_pose = scene.get_brick_pose()
    target_center = np.array((scene.pushcube_layout or {}).get("target_center", [0.0, 0.0, 0.0]), dtype=np.float64)
    ee_pose = scene.get_end_effector_pose()
    cube_xy = np.array(cube_pose.position[:2], dtype=np.float64)
    target_xy = np.array(target_center[:2], dtype=np.float64)
    cube_to_target_dist = float(np.linalg.norm(target_xy - cube_xy))
    print(
        "[teleop-state]",
        {
            "frame": frame,
            "cube_pose": np.round(np.array(cube_pose.position, dtype=np.float64), 4).tolist(),
            "target_pose": np.round(target_center, 4).tolist(),
            "ee_pose": np.round(np.array(ee_pose.position, dtype=np.float64), 4).tolist(),
            "desired_ee_position": np.round(np.array(desired_ee_position, dtype=np.float64), 4).tolist(),
            "right_arm_joint_positions": _rounded_named_values(right_joint_names, right_joint_positions),
            "gripper_joint_positions": _rounded_list(gripper_qpos),
            "table_top_z": round(_current_table_top_z(scene), 6),
            "cube_to_target_dist": round(cube_to_target_dist, 6),
            "left_arm_drift": round(left_arm_drift, 6),
            "gripper_qpos_drift": round(gripper_qpos_drift, 6),
            "paused": paused,
            "status": status,
            "control_mode": control_mode,
            "ik_fail_count": ik_fail_count,
            "last_ik_error": last_ik_error,
        },
    )


def _resolve_input_backend(requested_backend: str, viewer_reader: ViewerKeyReader) -> TerminalKeyReader | ViewerKeyReader:
    if requested_backend == "viewer":
        if viewer_reader.available:
            return viewer_reader
        print("[teleop] viewer backend unavailable, falling back to terminal")
        return TerminalKeyReader("terminal").setup()
    if requested_backend == "line":
        return TerminalKeyReader("line").setup()
    if requested_backend == "auto":
        if sys.stdin.isatty():
            return TerminalKeyReader("terminal").setup()
        if viewer_reader.available:
            return viewer_reader
        return TerminalKeyReader("line").setup()
    return TerminalKeyReader("terminal").setup()


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


def _clamp_center_to_table(
    preferred_xy: np.ndarray,
    size_xy: np.ndarray,
    table_center_xy: np.ndarray,
    table_size_xy: np.ndarray,
) -> tuple[np.ndarray, bool]:
    half_table = np.array(table_size_xy, dtype=np.float64) / 2.0
    half_size = np.array(size_xy, dtype=np.float64) / 2.0
    min_xy = np.array(table_center_xy, dtype=np.float64) - half_table + half_size
    max_xy = np.array(table_center_xy, dtype=np.float64) + half_table - half_size
    clamped_xy = np.clip(np.array(preferred_xy, dtype=np.float64), min_xy, max_xy)
    return clamped_xy, bool(not np.allclose(clamped_xy, preferred_xy))


def _margin_to_table(
    center_xy: np.ndarray,
    size_xy: np.ndarray,
    table_center_xy: np.ndarray,
    table_size_xy: np.ndarray,
) -> float:
    center_xy = np.array(center_xy, dtype=np.float64)
    size_xy = np.array(size_xy, dtype=np.float64)
    table_center_xy = np.array(table_center_xy, dtype=np.float64)
    table_size_xy = np.array(table_size_xy, dtype=np.float64)
    table_min = table_center_xy - (table_size_xy / 2.0)
    table_max = table_center_xy + (table_size_xy / 2.0)
    obj_min = center_xy - (size_xy / 2.0)
    obj_max = center_xy + (size_xy / 2.0)
    margins = np.array(
        [
            obj_min[0] - table_min[0],
            table_max[0] - obj_max[0],
            obj_min[1] - table_min[1],
            table_max[1] - obj_max[1],
        ],
        dtype=np.float64,
    )
    return float(np.min(margins))


def _margin_to_table_edges(
    center_xy: np.ndarray,
    size_xy: np.ndarray,
    table_center_xy: np.ndarray,
    table_size_xy: np.ndarray,
) -> dict[str, float]:
    center_xy = np.array(center_xy, dtype=np.float64)
    size_xy = np.array(size_xy, dtype=np.float64)
    table_center_xy = np.array(table_center_xy, dtype=np.float64)
    table_size_xy = np.array(table_size_xy, dtype=np.float64)
    table_min = table_center_xy - (table_size_xy / 2.0)
    table_max = table_center_xy + (table_size_xy / 2.0)
    rect_min = center_xy - (size_xy / 2.0)
    rect_max = center_xy + (size_xy / 2.0)
    margins = {
        "x_min": float(rect_min[0] - table_min[0]),
        "x_max": float(table_max[0] - rect_max[0]),
        "y_min": float(rect_min[1] - table_min[1]),
        "y_max": float(table_max[1] - rect_max[1]),
    }
    margins["min"] = float(min(margins.values()))
    return margins


def _rectangles_overlap(center_a: np.ndarray, size_a: np.ndarray, center_b: np.ndarray, size_b: np.ndarray) -> bool:
    center_a = np.array(center_a, dtype=np.float64)
    size_a = np.array(size_a, dtype=np.float64)
    center_b = np.array(center_b, dtype=np.float64)
    size_b = np.array(size_b, dtype=np.float64)
    return bool(
        abs(float(center_a[0] - center_b[0])) < float((size_a[0] + size_b[0]) / 2.0)
        and abs(float(center_a[1] - center_b[1])) < float((size_a[1] + size_b[1]) / 2.0)
    )


def _table_bounds_dict(table_center_xy: np.ndarray, table_size_xy: np.ndarray) -> dict[str, float]:
    table_center_xy = np.array(table_center_xy, dtype=np.float64)
    table_size_xy = np.array(table_size_xy, dtype=np.float64)
    return {
        "x_min": float(table_center_xy[0] - (table_size_xy[0] / 2.0)),
        "x_max": float(table_center_xy[0] + (table_size_xy[0] / 2.0)),
        "y_min": float(table_center_xy[1] - (table_size_xy[1] / 2.0)),
        "y_max": float(table_center_xy[1] + (table_size_xy[1] / 2.0)),
    }


def _table_bounds_log(table_bounds: dict[str, float]) -> dict[str, list[float]]:
    return {
        "x": [round(float(table_bounds["x_min"]), 6), round(float(table_bounds["x_max"]), 6)],
        "y": [round(float(table_bounds["y_min"]), 6), round(float(table_bounds["y_max"]), 6)],
    }


def _extract_bound_material_prim(scene, prim):
    if prim is None or not prim.IsValid():
        return None
    try:
        material, _ = scene._UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        if material:
            return material.GetPrim()
    except Exception:
        pass
    try:
        relationship = prim.GetRelationship("material:binding:physics")
        targets = relationship.GetTargets() if relationship else []
        if targets:
            material_prim = scene.world.stage.GetPrimAtPath(targets[0])
            if material_prim and material_prim.IsValid():
                return material_prim
    except Exception:
        return None
    return None


def _material_properties(scene, prim) -> dict[str, object]:
    material_prim = _extract_bound_material_prim(scene, prim)
    if material_prim is None or not material_prim.IsValid():
        return {
            "material_path": None,
            "static_friction": None,
            "dynamic_friction": None,
            "restitution": None,
            "density": None,
        }
    material_api = scene._UsdPhysics.MaterialAPI(material_prim)
    return {
        "material_path": str(material_prim.GetPath()),
        "static_friction": material_api.GetStaticFrictionAttr().Get() if material_api else None,
        "dynamic_friction": material_api.GetDynamicFrictionAttr().Get() if material_api else None,
        "restitution": material_api.GetRestitutionAttr().Get() if material_api else None,
        "density": material_api.GetDensityAttr().Get() if material_api else None,
    }


def _nearest_rigid_body_prim(scene, prim):
    current = prim
    while current is not None and current.IsValid():
        if current.HasAPI(scene._UsdPhysics.RigidBodyAPI) or current.HasAPI(scene._UsdPhysics.MassAPI):
            return current
        current = current.GetParent()
    return prim


def _safe_attr_value(attr):
    if attr is None:
        return None
    try:
        return attr.Get()
    except Exception:
        return None


def _prim_world_pose_log(scene, prim) -> dict[str, object]:
    try:
        pose = scene.get_prim_pose(prim)
        return {
            "position": np.round(np.array(pose.position, dtype=np.float64), 4).tolist(),
            "rotation": np.round(np.array(pose.rotation, dtype=np.float64), 4).tolist(),
        }
    except Exception:
        return {"position": None, "rotation": None}


def _approx_prim_size(scene, prim) -> list[float] | None:
    prim_path = str(prim.GetPath())
    if prim_path == str(scene.table.prim.GetPath()):
        return np.round(np.array(scene.scene_config.table_scale, dtype=np.float64), 4).tolist()
    if prim_path == str(scene.brick.prim.GetPath()):
        return np.round(np.array(scene.pushcube_layout["cube_scale"], dtype=np.float64), 4).tolist()
    extent_attr = prim.GetAttribute("extent")
    scale_attr = prim.GetAttribute("xformOp:scale")
    extent = _safe_attr_value(extent_attr)
    scale = _safe_attr_value(scale_attr)
    if extent is None:
        return None
    extent_array = np.array(extent, dtype=np.float64)
    if extent_array.shape != (2, 3):
        return None
    size = extent_array[1] - extent_array[0]
    if scale is not None:
        size = size * np.array(scale, dtype=np.float64)
    return np.round(np.abs(size), 4).tolist()


def _collision_approximation(scene, prim) -> str | None:
    if prim.IsA(scene._UsdGeom.Mesh):
        try:
            mesh_collision_api = scene._UsdPhysics.MeshCollisionAPI(prim)
            approximation = mesh_collision_api.GetApproximationAttr().Get()
            if approximation:
                return str(approximation)
        except Exception:
            pass
    prim_type = prim.GetTypeName()
    if prim_type.lower() in {"cube", "box"}:
        return "box"
    if prim_type:
        return str(prim_type)
    return None


def _collision_enabled(scene, prim) -> bool | None:
    if prim is None or not prim.IsValid():
        return None
    if not prim.HasAPI(scene._UsdPhysics.CollisionAPI):
        return bool(scene._prim_has_collision(prim))
    return bool(scene._UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())


def _rigid_body_enabled(scene, prim) -> bool | None:
    body_prim = _nearest_rigid_body_prim(scene, prim)
    if body_prim is None or not body_prim.IsValid() or not body_prim.HasAPI(scene._UsdPhysics.RigidBodyAPI):
        return False
    return bool(scene._UsdPhysics.RigidBodyAPI(body_prim).GetRigidBodyEnabledAttr().Get())


def _physics_entry(scene, label: str, prim) -> dict[str, object]:
    body_prim = _nearest_rigid_body_prim(scene, prim)
    mass_api = scene._UsdPhysics.MassAPI(body_prim) if body_prim and body_prim.HasAPI(scene._UsdPhysics.MassAPI) else None
    physx_rigid_body_api = (
        scene._PhysxSchema.PhysxRigidBodyAPI(body_prim)
        if body_prim and body_prim.HasAPI(scene._PhysxSchema.PhysxRigidBodyAPI)
        else None
    )
    physx_collision_api = (
        scene._PhysxSchema.PhysxCollisionAPI(prim)
        if prim.HasAPI(scene._PhysxSchema.PhysxCollisionAPI)
        else None
    )
    material_props = _material_properties(scene, prim)
    return {
        "label": label,
        "prim_path": str(prim.GetPath()),
        "world_pose": _prim_world_pose_log(scene, prim),
        "collision_enabled": _collision_enabled(scene, prim),
        "rigid_body_enabled": _rigid_body_enabled(scene, prim),
        "mass": _safe_attr_value(mass_api.GetMassAttr()) if mass_api else None,
        "density": material_props["density"] if material_props["density"] is not None else (_safe_attr_value(mass_api.GetDensityAttr()) if mass_api else None),
        "friction_material": material_props["material_path"],
        "static_friction": material_props["static_friction"],
        "dynamic_friction": material_props["dynamic_friction"],
        "restitution": material_props["restitution"],
        "linear_damping": _safe_attr_value(physx_rigid_body_api.GetLinearDampingAttr()) if physx_rigid_body_api else None,
        "angular_damping": _safe_attr_value(physx_rigid_body_api.GetAngularDampingAttr()) if physx_rigid_body_api else None,
        "contact_offset": _safe_attr_value(physx_collision_api.GetContactOffsetAttr()) if physx_collision_api else None,
        "rest_offset": _safe_attr_value(physx_collision_api.GetRestOffsetAttr()) if physx_collision_api else None,
        "solver_position_iteration_count": _safe_attr_value(physx_rigid_body_api.GetSolverPositionIterationCountAttr()) if physx_rigid_body_api else None,
        "solver_velocity_iteration_count": _safe_attr_value(physx_rigid_body_api.GetSolverVelocityIterationCountAttr()) if physx_rigid_body_api else None,
        "approx_size": _approx_prim_size(scene, prim),
        "collision_approximation": _collision_approximation(scene, prim),
    }


def _robot_collision_prims(scene, link_names: list[str]) -> list[object]:
    collision_prims: list[object] = []
    seen_paths: set[str] = set()
    for link_name in link_names:
        link_prim = scene._get_prim_at_path(f"{scene.robot_prim_path}/{link_name}")
        if link_prim is None or not link_prim.IsValid():
            continue
        for prim in scene._Usd.PrimRange(link_prim):
            if not prim.HasAPI(scene._UsdPhysics.CollisionAPI):
                continue
            prim_path = str(prim.GetPath())
            if prim_path in seen_paths:
                continue
            seen_paths.add(prim_path)
            collision_prims.append(prim)
    return collision_prims


def inspect_pushcube_physics(scene) -> None:
    entries: list[tuple[str, object]] = [
        ("cube_body", scene.brick.prim),
        ("table_body", scene.table.prim),
    ]
    for link_name in RIGHT_GRIPPER_LINK_NAMES + RIGHT_WRIST_LINK_NAMES:
        link_prim = scene._get_prim_at_path(f"{scene.robot_prim_path}/{link_name}")
        if link_prim is not None and link_prim.IsValid():
            entries.append((f"{link_name}_body", link_prim))
    for collision_prim in _robot_collision_prims(scene, RIGHT_GRIPPER_LINK_NAMES + RIGHT_WRIST_LINK_NAMES):
        entries.append((f"collision::{collision_prim.GetName()}", collision_prim))
    seen_paths: set[str] = set()
    for label, prim in entries:
        prim_path = str(prim.GetPath())
        if prim_path in seen_paths:
            continue
        seen_paths.add(prim_path)
        print("[physics-inspect]", _physics_entry(scene, label, prim))


def _enable_contact_reports(scene) -> bool:
    try:
        for prim in [scene.brick.prim, scene.robot_prim]:
            if prim is None or not prim.IsValid():
                continue
            contact_report_api = (
                scene._PhysxSchema.PhysxContactReportAPI(prim)
                if prim.HasAPI(scene._PhysxSchema.PhysxContactReportAPI)
                else scene._PhysxSchema.PhysxContactReportAPI.Apply(prim)
            )
            contact_report_api.CreateThresholdAttr().Set(0.0)
        return True
    except Exception as exc:
        print(f"[contact-pairs] unavailable: {exc}")
        return False


def _contact_pair_logs(scene) -> list[dict[str, object]] | None:
    try:
        import omni.physx  # type: ignore
    except Exception:
        return None
    try:
        contact_headers, contact_data = omni.physx.get_physx_simulation_interface().get_contact_report()
    except Exception:
        return None
    relevant_prefixes = [str(scene.brick.prim.GetPath()), str(scene.table.prim.GetPath()), str(scene.robot_prim.GetPath())]
    reports: list[dict[str, object]] = []
    for header in contact_headers:
        body_a = str(scene._PhysicsSchemaTools.intToSdfPath(header.actor0))
        body_b = str(scene._PhysicsSchemaTools.intToSdfPath(header.actor1))
        collider_a = str(scene._PhysicsSchemaTools.intToSdfPath(header.collider0))
        collider_b = str(scene._PhysicsSchemaTools.intToSdfPath(header.collider1))
        if not any(path.startswith(prefix) for path in (body_a, body_b, collider_a, collider_b) for prefix in relevant_prefixes):
            continue
        first_contact = None
        if int(header.num_contact_data) > 0:
            first_contact = contact_data[int(header.contact_data_offset)]
        reports.append(
            {
                "body_a": body_a,
                "body_b": body_b,
                "collider_a": collider_a,
                "collider_b": collider_b,
                "contact_count": int(header.num_contact_data),
                "contact_point": np.round(np.array(first_contact.position, dtype=np.float64), 4).tolist() if first_contact else None,
                "contact_normal": np.round(np.array(first_contact.normal, dtype=np.float64), 4).tolist() if first_contact else None,
                "normal_force_or_impulse": (
                    round(float(np.linalg.norm(np.array(first_contact.impulse, dtype=np.float64))), 6) if first_contact else None
                ),
                "relative_velocity": "unavailable",
            }
        )
    return reports


def _cube_inside_target(scene, success_margin: float) -> bool:
    cube_xy, target_xy, target_size_xy = _cube_and_target_xy(scene)
    half_size = (np.array(target_size_xy, dtype=np.float64) / 2.0) + abs(float(success_margin))
    return bool(
        abs(float(cube_xy[0] - target_xy[0])) <= float(half_size[0])
        and abs(float(cube_xy[1] - target_xy[1])) <= float(half_size[1])
    )


def _resolve_layout_override(args, scene) -> tuple[str, np.ndarray, float, bool]:
    current_table_position, _ = scene.table.get_world_pose()
    current_table_position = np.array(current_table_position, dtype=np.float64)
    current_table_top_z = _current_table_top_z(scene)
    manual_override_requested = any(
        value is not None
        for value in (
            args.table_center_x,
            args.table_center_y,
            args.table_top_z,
        )
    )
    if args.layout_preset == LAYOUT_PRESET_DEFAULT and not manual_override_requested:
        return (
            LAYOUT_PRESET_DEFAULT,
            np.array(current_table_position[:2], dtype=np.float64),
            float(current_table_top_z),
            False,
        )
    if args.layout_preset == LAYOUT_PRESET_DEFAULT:
        resolved_table_center_xy = np.array(current_table_position[:2], dtype=np.float64)
        resolved_table_top_z = float(current_table_top_z)
    else:
        preset = LAYOUT_PRESETS[args.layout_preset]
        resolved_table_center_xy = np.array(preset["table_center_xy"], dtype=np.float64)
        resolved_table_top_z = float(preset["table_top_z"])
    if args.table_center_x is not None:
        resolved_table_center_xy[0] = float(args.table_center_x)
    if args.table_center_y is not None:
        resolved_table_center_xy[1] = float(args.table_center_y)
    if args.table_top_z is not None:
        resolved_table_top_z = float(args.table_top_z)
    moved_table = bool(
        not np.allclose(resolved_table_center_xy, current_table_position[:2])
        or not np.isclose(resolved_table_top_z, current_table_top_z)
    )
    return (
        str(args.layout_preset),
        resolved_table_center_xy,
        resolved_table_top_z,
        moved_table,
    )


def _apply_table_layout_override(
    scene_bundle,
    *,
    layout_preset: str,
    table_center_xy: np.ndarray,
    table_top_z: float,
    target_size_xy: np.ndarray,
) -> float:
    scene = scene_bundle.scene
    layout = scene.pushcube_layout or {}
    table_position, table_quaternion = scene.table.get_world_pose()
    table_position = np.array(table_position, dtype=np.float64)
    table_position[0] = float(table_center_xy[0])
    table_position[1] = float(table_center_xy[1])
    table_position[2] = float(table_top_z) - (scene.scene_config.table_scale[2] / 2.0)
    scene.table.set_world_pose(
        position=np.array(table_position, dtype=np.float32),
        orientation=np.array(table_quaternion, dtype=np.float32),
    )

    table_size_xy = np.array(scene.scene_config.table_scale[:2], dtype=np.float64)
    cube_size = float(layout.get("cube_size", 0.06))
    cube_size_xy = np.array([cube_size, cube_size], dtype=np.float64)
    target_size = np.array(layout.get("target_size", [0.16, 0.16, 0.002]), dtype=np.float64)
    target_size_xy = np.array(target_size_xy[:2], dtype=np.float64)
    target_size[0] = float(target_size_xy[0])
    target_size[1] = float(target_size_xy[1])
    spawn_range_size = np.array(layout.get("spawn_range_size", [0.18, 0.14, 0.002]), dtype=np.float64)
    spawn_range_size_xy = np.array(spawn_range_size[:2], dtype=np.float64)
    target_range_size = np.array(layout.get("target_range_size", [0.22, 0.18, 0.002]), dtype=np.float64)
    target_range_size_xy = np.maximum(np.array(target_range_size[:2], dtype=np.float64), target_size_xy)
    target_range_size[0] = float(target_range_size_xy[0])
    target_range_size[1] = float(target_range_size_xy[1])

    if layout_preset == LAYOUT_PRESET_OPPOSITE_EDGES:
        spawn_range_size_xy = np.array(LAYOUT_OPPOSITE_EDGES_SPAWN_SIZE, dtype=np.float64)
        spawn_range_size[0] = float(spawn_range_size_xy[0])
        spawn_range_size[1] = float(spawn_range_size_xy[1])
        target_range_size_xy = np.array(target_size_xy, dtype=np.float64)
        target_range_size[0] = float(target_range_size_xy[0])
        target_range_size[1] = float(target_range_size_xy[1])
        table_bounds = _table_bounds_dict(table_center_xy, table_size_xy)
        spawn_preferred_xy = np.array(
            [
                float(table_center_xy[0] - 0.08),
                float(table_bounds["y_min"] + LAYOUT_EDGE_MARGIN + (spawn_range_size_xy[1] / 2.0)),
            ],
            dtype=np.float64,
        )
        target_preferred_xy = np.array(
            [
                float(spawn_preferred_xy[0]),
                float(table_bounds["y_max"] - LAYOUT_EDGE_MARGIN - (target_size_xy[1] / 2.0)),
            ],
            dtype=np.float64,
        )
        spawn_range_center_xy, spawn_range_clamped = _clamp_center_to_table(
            spawn_preferred_xy,
            spawn_range_size_xy,
            table_center_xy,
            table_size_xy,
        )
        target_center_xy, target_clamped = _clamp_center_to_table(
            target_preferred_xy,
            target_size_xy,
            table_center_xy,
            table_size_xy,
        )
        layout_adjusted = False
        if _rectangles_overlap(spawn_range_center_xy, spawn_range_size_xy, target_center_xy, target_size_xy):
            available_spawn_height = max(
                0.06,
                (float(target_center_xy[1]) - (float(target_size_xy[1]) / 2.0)) - (float(table_bounds["y_min"]) + LAYOUT_EDGE_MARGIN),
            )
            adjusted_spawn_height = min(float(spawn_range_size_xy[1]), max(0.06, (2.0 * available_spawn_height) - 1e-4))
            if adjusted_spawn_height < float(spawn_range_size_xy[1]):
                spawn_range_size_xy[1] = adjusted_spawn_height
                spawn_range_size[1] = adjusted_spawn_height
                spawn_preferred_xy[1] = float(table_bounds["y_min"] + LAYOUT_EDGE_MARGIN + (spawn_range_size_xy[1] / 2.0))
                spawn_range_center_xy, spawn_range_clamped = _clamp_center_to_table(
                    spawn_preferred_xy,
                    spawn_range_size_xy,
                    table_center_xy,
                    table_size_xy,
                )
                layout_adjusted = True
        spawn_target_overlap = _rectangles_overlap(spawn_range_center_xy, spawn_range_size_xy, target_center_xy, target_size_xy)
        cube_preferred_xy = np.array(spawn_range_center_xy, dtype=np.float64)
        cube_center_xy, cube_clamped = _clamp_center_to_table(cube_preferred_xy, cube_size_xy, table_center_xy, table_size_xy)
        target_range_center_xy = np.array(target_center_xy, dtype=np.float64)
        target_range_clamped = bool(target_clamped)
    else:
        cube_preferred_xy = np.array(table_center_xy, dtype=np.float64) + LAYOUT_CUBE_OFFSET_FROM_TABLE_CENTER
        target_preferred_xy = np.array(table_center_xy, dtype=np.float64) + LAYOUT_TARGET_OFFSET_FROM_TABLE_CENTER
        cube_center_xy, cube_clamped = _clamp_center_to_table(cube_preferred_xy, cube_size_xy, table_center_xy, table_size_xy)
        target_center_xy, target_clamped = _clamp_center_to_table(target_preferred_xy, target_size_xy, table_center_xy, table_size_xy)
        spawn_range_center_xy, spawn_range_clamped = _clamp_center_to_table(
            cube_center_xy,
            spawn_range_size_xy,
            table_center_xy,
            table_size_xy,
        )
        target_range_center_xy, target_range_clamped = _clamp_center_to_table(
            target_center_xy,
            target_range_size_xy,
            table_center_xy,
            table_size_xy,
        )
        spawn_target_overlap = _rectangles_overlap(spawn_range_center_xy, spawn_range_size_xy, target_center_xy, target_size_xy)
        layout_adjusted = False

    cube_size = float(layout.get("cube_size", 0.06))
    cube_position = np.array(layout.get("cube_position", scene.get_brick_pose().position), dtype=np.float64)
    cube_position[0] = float(cube_center_xy[0])
    cube_position[1] = float(cube_center_xy[1])
    cube_position[2] = float(table_top_z) + (cube_size / 2.0)
    scene.brick.set_world_pose(position=np.array(cube_position, dtype=np.float32))
    scene.brick.set_linear_velocity(np.zeros(3, dtype=np.float32))
    scene.brick.set_angular_velocity(np.zeros(3, dtype=np.float32))

    target_center = np.array(layout.get("target_center", cube_position), dtype=np.float64)
    target_center[0] = float(target_center_xy[0])
    target_center[1] = float(target_center_xy[1])
    target_center[2] = float(table_top_z) + 0.002
    spawn_range_center = np.array(layout.get("spawn_range_center", target_center), dtype=np.float64)
    spawn_range_center[0] = float(spawn_range_center_xy[0])
    spawn_range_center[1] = float(spawn_range_center_xy[1])
    spawn_range_center[2] = float(table_top_z) + 0.003
    target_range_center = np.array(layout.get("target_range_center", target_center), dtype=np.float64)
    target_range_center[0] = float(target_range_center_xy[0])
    target_range_center[1] = float(target_range_center_xy[1])
    target_range_center[2] = float(table_top_z) + 0.004

    layout["cube_position"] = np.array(cube_position, dtype=np.float32)
    layout["target_center"] = np.array(target_center, dtype=np.float32)
    layout["target_size"] = np.array(target_size, dtype=np.float32)
    layout["spawn_range_center"] = np.array(spawn_range_center, dtype=np.float32)
    layout["spawn_range_size"] = np.array(spawn_range_size, dtype=np.float32)
    layout["target_range_center"] = np.array(target_range_center, dtype=np.float32)
    layout["target_range_size"] = np.array(target_range_size, dtype=np.float32)
    layout["table_top_z"] = float(table_top_z)
    layout["overlay_z"] = float(table_top_z) + 0.002
    layout["table_center_xy"] = np.array(table_center_xy, dtype=np.float32)
    layout["table_size_xy"] = np.array(table_size_xy, dtype=np.float32)
    table_bounds = _table_bounds_dict(table_center_xy, table_size_xy)
    layout["table_bounds"] = table_bounds
    layout["cube_preferred_xy"] = np.array(cube_preferred_xy, dtype=np.float32)
    layout["target_preferred_xy"] = np.array(target_preferred_xy, dtype=np.float32)
    layout["cube_margin_to_table_edges"] = _margin_to_table_edges(
        cube_center_xy,
        cube_size_xy,
        table_center_xy,
        table_size_xy,
    )
    layout["target_margin_to_table_edges"] = _margin_to_table_edges(
        target_center_xy,
        target_size_xy,
        table_center_xy,
        table_size_xy,
    )
    layout["cube_clamped"] = bool(cube_clamped)
    layout["target_clamped"] = bool(target_clamped)
    layout["spawn_range_clamped"] = bool(spawn_range_clamped)
    layout["target_range_clamped"] = bool(target_range_clamped)
    scene.initial_brick_position = np.array(cube_position, dtype=np.float32)

    target_path = scene.layout_visual_prim_paths.get("target")
    cube_spawn_range_path = scene.layout_visual_prim_paths.get("cube_spawn_range")
    target_range_path = scene.layout_visual_prim_paths.get("target_range")
    if target_path:
        _set_prim_translate(scene, target_path, target_center)
        _set_prim_scale(scene, target_path, target_size)
    if cube_spawn_range_path:
        _set_prim_translate(scene, cube_spawn_range_path, spawn_range_center)
        _set_prim_scale(scene, cube_spawn_range_path, spawn_range_size)
    if target_range_path:
        _set_prim_translate(scene, target_range_path, target_range_center)
        _set_prim_scale(scene, target_range_path, target_range_size)

    actual_table_top_z = _current_table_top_z(scene)
    table_bounds = _table_bounds_dict(table_center_xy, table_size_xy)
    table_bounds_log = _table_bounds_log(table_bounds)
    cube_margin_min = _margin_to_table(cube_center_xy, cube_size_xy, table_center_xy, table_size_xy)
    target_margin_min = _margin_to_table(target_center_xy, target_size_xy, table_center_xy, table_size_xy)
    spawn_margin = _margin_to_table_edges(spawn_range_center_xy, spawn_range_size_xy, table_center_xy, table_size_xy)
    target_margin = _margin_to_table_edges(target_center_xy, target_size_xy, table_center_xy, table_size_xy)
    print(f"[layout] preset={layout_preset}")
    print(f"[table] center={_rounded_xy_list(table_center_xy)}")
    print(f"[table] bounds={table_bounds_log}")
    print(f"[spawn] center={_rounded_xy_list(spawn_range_center_xy)}")
    print(f"[spawn] size={_rounded_xy_list(spawn_range_size_xy)}")
    print(f"[target] center={_rounded_xy_list(target_center_xy)}")
    print(f"[target] size={_rounded_xy_list(target_size_xy)}")
    if layout_adjusted:
        print("[layout] adjusted regions to avoid overlap")
    print(f"[layout] spawn_target_overlap={spawn_target_overlap}")
    print(f"[layout] spawn_margin_to_edges={spawn_margin}")
    print(f"[layout] target_margin_to_edges={target_margin}")
    print(
        "[clamp]",
        {
            "cube": bool(cube_clamped),
            "target": bool(target_clamped),
            "cube_spawn_range": bool(spawn_range_clamped),
            "target_range": bool(target_range_clamped),
        },
    )
    print(f"[cube] xy={_rounded_xy_list(cube_center_xy)}")
    print(f"[target] xy={_rounded_xy_list(target_center_xy)}")
    print(f"[target] target_size_xy={_rounded_xy_list(target_size_xy)}")
    print(f"[layout] cube_margin_min={round(cube_margin_min, 6)}")
    print(f"[layout] target_margin_min={round(target_margin_min, 6)}")
    print(f"[layout] cube_spawn_range={np.round(spawn_range_center_xy, 4).tolist()}")
    print(f"[layout] target_range={np.round(target_range_center_xy, 4).tolist()}")
    print(f"[table] cube_center_z={round(float(cube_position[2]), 6)}")
    print(f"[table] target_z={round(float(target_center[2]), 6)}")
    print(f"[table] ranges_z={[round(float(spawn_range_center[2]), 6), round(float(target_range_center[2]), 6)]}")
    print(f"[table] top_z={actual_table_top_z:.2f}")
    return actual_table_top_z


def _reset_scene(
    scene,
    *,
    initial_full_positions: np.ndarray,
    initial_brick_position: np.ndarray,
    right_indices: np.ndarray,
    left_indices: np.ndarray,
    gripper_indices: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    scene.articulation.set_joint_positions(np.array(initial_full_positions, dtype=np.float64))
    scene.articulation.set_joint_velocities(np.zeros(len(initial_full_positions), dtype=np.float64))
    _apply_full_target(scene, np.array(initial_full_positions, dtype=np.float64))
    scene.brick.set_world_pose(position=np.array(initial_brick_position, dtype=np.float32))
    scene.brick.set_linear_velocity(np.zeros(3, dtype=np.float32))
    scene.brick.set_angular_velocity(np.zeros(3, dtype=np.float32))
    scene.step_world(steps=30)
    ee_pose = scene.get_end_effector_pose()
    desired_ee_position = np.array(ee_pose.position, dtype=np.float64)
    desired_ee_rotation = np.array(ee_pose.rotation, dtype=np.float64)
    right_joint_target = _read_joint_positions(scene, right_indices)
    return desired_ee_position, desired_ee_position.copy(), desired_ee_rotation, right_joint_target


def _build_initial_pose_target(
    current_right_qpos: np.ndarray,
    right_joint_names: list[str],
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    preset_id: int,
) -> np.ndarray:
    target = np.array(current_right_qpos, dtype=np.float64)
    elbow_index = right_joint_names.index("right_elbow_pitch_joint") if "right_elbow_pitch_joint" in right_joint_names else None
    if elbow_index is None:
        return np.clip(target, lower_limits, upper_limits)
    if preset_id == 2:
        target[elbow_index] = float(target[elbow_index]) + 0.05
    elif preset_id == 3:
        target[elbow_index] = INITIAL_POSE_PRESET_3_ELBOW_TARGET
    return np.clip(target, lower_limits, upper_limits)


def _apply_initial_pose_preset(
    scene,
    *,
    preset_id: int,
    right_joint_names: list[str],
    right_indices: np.ndarray,
    left_indices: np.ndarray,
    left_initial: np.ndarray,
    gripper_indices: np.ndarray,
    gripper_initial: np.ndarray,
    initial_full_positions: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    max_joint_delta: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    current_right_qpos = _read_joint_positions(scene, right_indices)
    target_right_qpos = _build_initial_pose_target(
        current_right_qpos,
        right_joint_names,
        lower_limits,
        upper_limits,
        preset_id,
    )
    applied_smoothly = bool(preset_id != 0 and not np.allclose(current_right_qpos, target_right_qpos))
    command_qpos = np.array(current_right_qpos, dtype=np.float64)
    if applied_smoothly:
        step_limit = min(abs(float(max_joint_delta)), FIXED_SUBSET_RESTORE_DELTA)
        for step in range(INITIAL_POSE_TRANSITION_FRAMES):
            blend = float(step + 1) / float(INITIAL_POSE_TRANSITION_FRAMES)
            interpolated_target = current_right_qpos + ((target_right_qpos - current_right_qpos) * blend)
            command_qpos = _filter_joint_target(
                current_command=command_qpos,
                desired_target=interpolated_target,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
                joint_alpha=1.0,
                max_joint_delta=step_limit,
            )
            full_target = _compose_full_target(
                initial_full_positions=initial_full_positions,
                right_indices=right_indices,
                right_target=command_qpos,
                left_indices=left_indices,
                left_initial=left_initial,
                gripper_indices=gripper_indices,
                gripper_initial=gripper_initial,
            )
            _apply_full_target(scene, full_target)
            scene.step_world(steps=1)
    else:
        command_qpos = np.array(target_right_qpos, dtype=np.float64)
    ee_pose = scene.get_end_effector_pose()
    desired_ee_position = np.array(ee_pose.position, dtype=np.float64)
    desired_ee_rotation = np.array(ee_pose.rotation, dtype=np.float64)
    final_right_qpos = _read_joint_positions(scene, right_indices)
    print(f"[initial-pose] preset={preset_id}")
    print(f"[initial-pose] right_arm_target={_rounded_named_values(right_joint_names, target_right_qpos)}")
    print(f"[initial-pose] applied_smoothly={applied_smoothly}")
    print(f"[initial-pose] final_right_arm_qpos={_rounded_named_values(right_joint_names, final_right_qpos)}")
    print(f"[initial-pose] desired_ee_position={_rounded_list(desired_ee_position)}")
    return desired_ee_position, desired_ee_position.copy(), desired_ee_rotation, final_right_qpos


def _print_preset_candidate(
    scene,
    *,
    right_joint_names: list[str],
    right_indices: np.ndarray,
    desired_ee_position: np.ndarray,
) -> None:
    right_qpos = _read_joint_positions(scene, right_indices)
    print("[preset-candidate]")
    for joint_name, joint_value in zip(right_joint_names, right_qpos.tolist(), strict=False):
        print(f"{joint_name}={float(joint_value):.6f}")
    print(f"desired_ee_position={_rounded_list(desired_ee_position)}")


def _print_warning_summary(warning_state: dict[str, dict[str, int]]) -> None:
    print(
        "[warning-summary] "
        f"left_arm_drift_count={_warning_count(warning_state, 'left_arm_drift')} "
        f"gripper_drift_count={_warning_count(warning_state, 'gripper_drift')} "
        f"ik_failure_count={_warning_count(warning_state, 'ik_failure')} "
        f"contact_warning_count={_warning_count(warning_state, 'contact_warning')}"
    )


def _print_periodic_status(
    scene,
    *,
    frame: int,
    cube_xy: np.ndarray,
    target_xy: np.ndarray,
    cube_to_target_dist: float,
    ee_position: np.ndarray,
    desired_ee_position: np.ndarray,
    gripper_drift: float,
    left_drift: float,
    paused: bool,
    control_mode: str,
    ik_fail_count: int,
    success_margin: float,
    verbose: bool,
) -> None:
    if verbose:
        print(
            "[teleop-status]",
            {
                "frame": frame,
                "cube_xy": np.round(cube_xy, 4).tolist(),
                "target_xy": np.round(target_xy, 4).tolist(),
                "cube_to_target_dist": round(cube_to_target_dist, 6),
                "ee_position": np.round(ee_position, 4).tolist(),
                "desired_ee_position": np.round(desired_ee_position, 4).tolist(),
                "gripper_qpos_drift": round(gripper_drift, 6),
                "left_arm_qpos_drift": round(left_drift, 6),
                "cube_gripper_contact": _cube_gripper_contact(scene),
                "cube_in_target": _cube_inside_target(scene, success_margin),
                "paused": paused,
                "control_mode": control_mode,
                "ik_fail_count": ik_fail_count,
            },
        )
        return
    print(
        "[teleop-status] "
        f"frame={frame} "
        f"cube_xy={_rounded_list(cube_xy)} "
        f"target_xy={_rounded_list(target_xy)} "
        f"dist={cube_to_target_dist:.6f} "
        f"ee={_rounded_list(ee_position)} "
        f"mode={control_mode}"
    )


def main() -> None:
    args = build_arg_parser().parse_args()
    quiet_warnings = bool(args.quiet_warnings) and not bool(args.verbose_warnings)
    status_every = max(1, int(args.status_every))
    _apply_warning_filters(bool(args.suppress_isaac_warnings))
    print(f"[manual-teleop] headless={args.headless}")
    print(f"[manual-teleop] show_ranges={args.show_ranges}")
    print(f"[manual-teleop] disable_lula={args.disable_lula}")
    print(f"[manual-teleop] hold_open={args.hold_open}")
    print(f"[manual-teleop] layout_preset={args.layout_preset}")
    print(f"[manual-teleop] table_center_x={args.table_center_x if args.table_center_x is not None else 'auto'}")
    print(f"[manual-teleop] table_center_y={args.table_center_y if args.table_center_y is not None else 'auto'}")
    print(f"[manual-teleop] table_top_z={args.table_top_z if args.table_top_z is not None else 'auto'}")
    print(f"[manual-teleop] step_size={args.step_size}")
    print(f"[manual-teleop] joint_step={args.joint_step}")
    print(f"[manual-teleop] alpha={args.alpha}")
    print(f"[manual-teleop] joint_alpha={args.joint_alpha}")
    print(f"[manual-teleop] max_joint_delta={args.max_joint_delta}")
    print(f"[manual-teleop] expand_right_arm_limits={args.expand_right_arm_limits}")
    print(f"[manual-teleop] initial_pose_preset={args.initial_pose_preset}")
    print(f"[manual-teleop] target_size={args.target_size}")
    print(f"[manual-teleop] success_margin={args.success_margin}")
    print(f"[manual-teleop] control_hz={args.control_hz}")
    print(f"[teleop] input_backend={args.input_backend}")
    print(f"[teleop] control_mode={args.control_mode}")
    print(f"[teleop] inspect_collisions={args.inspect_collisions}")
    print(f"[teleop] print_contact_pairs={args.print_contact_pairs}")
    print(f"[teleop] quiet_warnings={quiet_warnings}")
    print(f"[teleop] verbose_warnings={args.verbose_warnings}")
    print(f"[teleop] suppress_isaac_warnings={args.suppress_isaac_warnings}")
    print(f"[teleop] status_every={status_every}")
    print(f"[teleop] verbose={args.verbose}")
    if args.save_video:
        print(f"[video] save-video requested at {args.save_video}, but video capture is not implemented in this script yet")

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
        resolved_preset, resolved_table_center_xy, resolved_table_top_z, moved_table_closer = _resolve_layout_override(
            args,
            scene,
        )
        actual_table_top_z = _apply_table_layout_override(
            scene_bundle,
            layout_preset=resolved_preset,
            table_center_xy=resolved_table_center_xy,
            table_top_z=resolved_table_top_z,
            target_size_xy=np.array([float(args.target_size), float(args.target_size)], dtype=np.float64),
        )
        object_handles = get_pushcube_object_handles(scene_bundle)
        print("[lifecycle] scene created")
        print("[scene] object_handles", {name: handle is not None for name, handle in object_handles.items()})
        _print_layout_logs(scene, scene_bundle.runtime_options)
        table_bounds = dict((scene.pushcube_layout or {}).get("table_bounds", {}))
        layout = scene.pushcube_layout or {}
        cube_xy, target_xy, _ = _cube_and_target_xy(scene)
        print(f"[layout-preset] {resolved_preset}")
        print(f"[table] center={_rounded_xy_list(resolved_table_center_xy)}")
        print(f"[table] top_z={actual_table_top_z:.2f}")
        if table_bounds:
            print(f"[table] bounds={_table_bounds_log(table_bounds)}")
        print(f"[cube] xy={_rounded_xy_list(cube_xy)}")
        print(f"[target] xy={_rounded_xy_list(target_xy)}")
        print(
            f"[target] target_size_xy={_rounded_xy_list(np.array(layout.get('target_size', [args.target_size, args.target_size])[:2], dtype=np.float64))}"
        )
        print(f"[target] target_visible={scene.layout_visibility_state['target_visible']}")
        print(f"[target] margin_to_table_edges={dict(layout.get('target_margin_to_table_edges', {}))}")
        print(f"[workspace] moved_table_closer_to_right_arm={moved_table_closer}")
        print(f"[control] max_joint_delta={float(args.max_joint_delta):.6f}")
        print(f"[control] joint_alpha={float(args.joint_alpha):.3f}")
        print("[control] hard_restore=False")
        print("[control] fixed_joint_restore_method=rate_limited_reset")
        if args.inspect_collisions:
            inspect_pushcube_physics(scene)
        print("[teleop] controls:")
        if args.control_mode == "ee":
            print("[teleop] EE control mapping:")
            print("  q/e: X +/-")
            print("  a/d: Y +/-")
            print("  w/s: Z +/-")
        else:
            print("[teleop] Joint control mapping:")
            print("  1/q: right_base_pitch_joint +/-")
            print("  2/w: right_shoulder_yaw_joint +/-")
            print("  3/e: right_elbow_pitch_joint +/-")
            print("  4/r: right_shoulder_roll_joint +/-")
            print("  5/t: right_wrist_pitch_joint +/-")
            print("  6/y: right_wrist_yaw_joint +/-")
        print("  0 print preset candidate")
        print("  p print")
        print("  z reset")
        print("  x success")
        print("  f failure")
        print("  esc exit")
        print("[teleop] run from a terminal, then focus the terminal window and press the control keys there")
        print("[teleop] terminal backend keeps Isaac rendering while keyboard input comes from the terminal")
        print(f"[teleop] viewer_keyboard_available={bool(viewer_keys.available)}")
        if active_input_reader is not None:
            print(f"[teleop] active_input_backend={getattr(active_input_reader, 'source', 'viewer')}")

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

        print("[gripper] fixed=True")
        print(f"[gripper] joint_names={gripper_joint_names}")
        print(f"[gripper] qpos_initial={_rounded_list(fixed_gripper_qpos)}")
        print(f"[gripper] warnings={'quiet' if quiet_warnings else 'verbose'}")
        print("[gripper] controlled=False")
        print("[left-arm] fixed=True")
        print(f"[left-arm] joint_names={left_joint_names}")
        print(f"[left-arm] qpos_initial={_rounded_list(fixed_left_arm_qpos)}")
        print(f"[left-arm] warnings={'quiet' if quiet_warnings else 'verbose'}")

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
            right_joint_target=right_joint_target,
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
        recorder = ManualEpisodeRecorder(
            enabled=args.record,
            output_path=Path(args.output),
            episode_name=args.episode_name,
        )
        warning_state: dict[str, dict[str, int]] = {}
        if args.control_mode == "ee" and (args.disable_lula or scene.kinematics is None):
            print(
                "[teleop] Cartesian end-effector teleop requires Lula/IK. "
                "Remove --disable-lula or use --control-mode joint."
            )
            teleop_available = False
        else:
            teleop_available = True

        physics_hz = 1.0 / float(scene.scene_config.physics_dt)
        control_interval_frames = max(1, int(round(physics_hz / float(args.control_hz))))
        frame = 0
        last_action_xyz = np.zeros(3, dtype=np.float64)
        last_key = ""
        contact_report_enabled = _enable_contact_reports(scene) if args.print_contact_pairs else False
        joint_key_map = {
            "1": ("right_base_pitch_joint", float(args.joint_step), "+"),
            "q": ("right_base_pitch_joint", -float(args.joint_step), "-"),
            "2": ("right_shoulder_yaw_joint", float(args.joint_step), "+"),
            "w": ("right_shoulder_yaw_joint", -float(args.joint_step), "-"),
            "3": ("right_elbow_pitch_joint", float(args.joint_step), "+"),
            "e": ("right_elbow_pitch_joint", -float(args.joint_step), "-"),
            "4": ("right_shoulder_roll_joint", float(args.joint_step), "+"),
            "r": ("right_shoulder_roll_joint", -float(args.joint_step), "-"),
            "5": ("right_wrist_pitch_joint", float(args.joint_step), "+"),
            "t": ("right_wrist_pitch_joint", -float(args.joint_step), "-"),
            "6": ("right_wrist_yaw_joint", float(args.joint_step), "+"),
            "y": ("right_wrist_yaw_joint", -float(args.joint_step), "-"),
        }
        right_joint_name_to_index = {joint_name: index for index, joint_name in enumerate(right_joint_names)}

        def handle_key(key: str, source: str) -> None:
            nonlocal desired_ee_position
            nonlocal filtered_ee_position
            nonlocal desired_ee_rotation
            nonlocal right_joint_target
            nonlocal last_action_xyz
            nonlocal last_key
            before_position = np.array(teleop_state.desired_ee_position, dtype=np.float64)
            print(f"[teleop-key] source={source} key={key}")
            print(f"[teleop-key] key={key}")
            if key in KEY_DELTAS or (args.control_mode == "joint" and key in joint_key_map):
                last_key = key
                if args.control_mode == "ee":
                    axis_name = "X" if key in {"q", "e"} else ("Y" if key in {"a", "d"} else "Z")
                    delta_sign = "+" if key in {"q", "a", "w"} else "-"
                    unclamped_position = teleop_state.desired_ee_position + (KEY_DELTAS[key] * float(args.step_size))
                    teleop_state.desired_ee_position = _clamp_desired_ee_position(
                        unclamped_position,
                        _current_table_top_z(scene),
                    )
                    print(f"[teleop-key] key={key} axis={axis_name} delta={delta_sign}{float(args.step_size)}")
                    if not np.allclose(unclamped_position, teleop_state.desired_ee_position):
                        print("[teleop] clamped desired_ee_position")
                    last_action_xyz = KEY_DELTAS[key] * float(args.step_size)
                else:
                    joint_name, delta, direction = joint_key_map[key]
                    joint_index = right_joint_name_to_index.get(joint_name)
                    if joint_index is not None:
                        old_value = float(teleop_state.desired_right_joint_target[joint_index])
                        new_value = float(np.clip(old_value + delta, lower_limits[joint_index], upper_limits[joint_index]))
                        teleop_state.desired_right_joint_target[joint_index] = new_value
                        last_action_xyz = np.zeros(3, dtype=np.float64)
                        current_value = float(_read_joint_positions(scene, right_indices)[joint_index])
                        print(
                            "[joint-teleop] "
                            f"key={key} joint={joint_name} direction={direction} "
                            f"old_target={old_value:.6f} new_target={new_value:.6f} "
                            f"command={float(teleop_state.right_joint_target[joint_index]):.6f} current={current_value:.6f}"
                        )
            elif key == "space":
                teleop_state.paused = not teleop_state.paused
                print(f"[teleop] paused={teleop_state.paused}")
            elif key == "z":
                if args.save_failed and args.record and recorder.obs:
                    recorder.save("reset")
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
                teleop_state.right_joint_target = right_joint_target
                teleop_state.status = "reset"
                teleop_state.ik_fail_count = 0
                teleop_state.last_ik_failure_frame = -30
                teleop_state.last_ik_error = ""
                teleop_state.contact_warning_count = 0
                recorder.reset()
                print("[teleop] scene reset")
            elif key == "0":
                _print_preset_candidate(
                    scene,
                    right_joint_names=right_joint_names,
                    right_indices=right_indices,
                    desired_ee_position=teleop_state.desired_ee_position,
                )
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
                recorder.save("success")
                print("[teleop] episode marked success")
                if not args.hold_open:
                    teleop_state.exit_requested = True
            elif key == "f":
                teleop_state.failure_count += 1
                teleop_state.status = "failure"
                recorder.save("failure")
                print("[teleop] episode marked failure")
                if not args.hold_open:
                    teleop_state.exit_requested = True
            elif key == "escape":
                teleop_state.exit_requested = True
            print(f"[teleop] desired_ee_position before={np.round(before_position, 4).tolist()}")
            print(f"[teleop] desired_ee_position after={np.round(teleop_state.desired_ee_position, 4).tolist()}")

        while scene._app.is_running() and not teleop_state.exit_requested:
            frame += 1
            keys = []
            if active_input_reader is not None:
                keys = active_input_reader.poll()
            for key in keys:
                handle_key(key, getattr(active_input_reader, "source", "viewer"))

            if not teleop_state.paused and teleop_available and frame % control_interval_frames == 0:
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
                            f"failed_count={teleop_state.ik_fail_count} desired_ee_position="
                            f"{np.round(teleop_state.filtered_ee_position, 4).tolist()}"
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
                right_joint_target = np.array(teleop_state.right_joint_target, dtype=np.float64)

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
            cube_xy, target_xy, target_size_xy = _cube_and_target_xy(scene)
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
                    paused=teleop_state.paused,
                    control_mode=args.control_mode,
                    ik_fail_count=teleop_state.ik_fail_count,
                    success_margin=float(args.success_margin),
                    verbose=bool(args.verbose),
                )
            if bool(args.verbose_warnings) and frame % WARNING_SUMMARY_INTERVAL_FRAMES == 0:
                _print_warning_summary(warning_state)
            if frame % CONTACT_REPORT_INTERVAL_FRAMES == 0:
                if args.print_contact_pairs:
                    contact_reports = _contact_pair_logs(scene) if contact_report_enabled else None
                    print(f"[contact-pairs] frame={frame} available={contact_reports is not None}")
                    if contact_reports is None:
                        teleop_state.contact_warning_count += 1
                        _emit_warning(
                            warning_state,
                            "contact_warning",
                            frame=frame,
                            message="[contact-warning] contact reports unavailable",
                            interval_frames=CONTACT_WARNING_INTERVAL_FRAMES,
                            quiet_warnings=quiet_warnings,
                        )
                    for report in contact_reports or []:
                        print("[contact-pair]", report)
            if args.record:
                recorder.record_step(
                    obs=_compute_observation(scene, ee_position),
                    action_xy=np.array(last_action_xyz[:2], dtype=np.float32),
                    ee_desired_position=teleop_state.desired_ee_position,
                    ee_actual_position=ee_position,
                    right_arm_joint_pos=_read_joint_positions(scene, right_indices),
                    right_arm_joint_vel=_read_joint_velocities(scene, right_indices),
                    gripper_qpos=_read_joint_positions(scene, gripper_indices),
                    cube_pose=np.array(scene.get_brick_pose().position, dtype=np.float32),
                    target_pose=np.array((scene.pushcube_layout or {}).get("target_center", [0.0, 0.0, 0.0]), dtype=np.float32),
                    key_pressed=last_key,
                    timestamp=time.time(),
                )
            last_action_xyz = np.zeros(3, dtype=np.float64)
            last_key = ""
    except KeyboardInterrupt:
        print("[lifecycle] Ctrl+C received, exiting teleop loop")
    finally:
        if active_input_reader is not None and isinstance(active_input_reader, TerminalKeyReader):
            active_input_reader.close()
        if viewer_keys is not None:
            viewer_keys.close()
        if scene_bundle is not None:
            scene_bundle.scene.close()
            print("[lifecycle] closing SimulationApp")
            close_simulation_app(scene_bundle.scene._app)


if __name__ == "__main__":
    main()
