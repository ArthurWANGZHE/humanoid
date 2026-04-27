#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from robot_imitation_pipeline.io_utils import list_episode_dirs, load_episode_arrays, read_json

try:
    from PIL import Image
except ImportError:
    Image = None


def _rate(times: np.ndarray) -> float:
    if len(times) < 2:
        return 0.0
    duration = float(times[-1] - times[0])
    return 0.0 if duration <= 0 else float((len(times) - 1) / duration)


def _status_level(issues: List[str], warnings: List[str]) -> str:
    if issues:
        return "ERROR"
    if warnings:
        return "LEGACY/WARN"
    return "OK"


def _check_images(camera_dir: Path, expected_count: int) -> Tuple[List[str], List[str], int]:
    issues: List[str] = []
    warnings: List[str] = []
    if not camera_dir.exists():
        if expected_count == 0:
            return issues, warnings, 0
        issues.append(f"missing image directory {camera_dir}")
        return issues, warnings, 0
    image_paths = sorted(camera_dir.glob("*.jpg"))
    if len(image_paths) != expected_count:
        issues.append(f"image count {len(image_paths)} != timestamps length {expected_count}")
    if Image is None:
        warnings.append("Pillow not installed; skipped image open checks")
        return issues, warnings, len(image_paths)
    for image_path in image_paths:
        try:
            with Image.open(image_path) as img:
                img.verify()
        except Exception as exc:
            issues.append(f"failed to open image {image_path.name}: {exc}")
            break
    return issues, warnings, len(image_paths)


def validate_episode(episode_dir: Path, rate_tolerance_hz: float) -> Dict[str, object]:
    report: Dict[str, object] = {
        "episode": episode_dir.name,
        "issues": [],
        "warnings": [],
    }
    issues: List[str] = report["issues"]  # type: ignore[assignment]
    warnings: List[str] = report["warnings"]  # type: ignore[assignment]
    meta_path = episode_dir / "meta.json"
    success_path = episode_dir / "success.json"
    arrays = load_episode_arrays(episode_dir)

    if not meta_path.exists():
        issues.append("missing meta.json")
        report["status"] = _status_level(issues, warnings)
        return report
    meta = read_json(meta_path)
    report["meta"] = meta

    is_strict = all((episode_dir / name).exists() for name in ["timestamps.npy", "robot_state.npy", "action.npy"])
    if not is_strict:
        warnings.append("legacy or incomplete episode: strict Diffusion-Policy-ready files are missing")

    if not success_path.exists():
        warnings.append("missing success.json")
        success = None
        valid_for_training = None
    else:
        success_data = read_json(success_path)
        success = success_data.get("success")
        valid_for_training = success_data.get("valid_for_training")
        if "valid_for_training" not in success_data:
            warnings.append("success.json missing valid_for_training")
    report["success"] = success
    report["valid_for_training"] = valid_for_training

    timestamps = arrays.get("timestamps", np.zeros((0,), dtype=np.float64))
    robot_state = arrays.get("robot_state", np.zeros((0, 0), dtype=np.float64))
    action = arrays.get("action", np.zeros((0, 0), dtype=np.float64))
    camera_name = str(meta.get("camera_name") or next(iter(meta.get("camera_topics", {"unknown": ""}))))
    camera_dir = episode_dir / "obs" / camera_name

    for required in ["timestamps.npy", "robot_state.npy", "action.npy", "meta.json", "success.json"]:
        if not (episode_dir / required).exists():
            if is_strict or required == "meta.json":
                issues.append(f"missing {required}")
            else:
                warnings.append(f"missing {required}")
    if not camera_dir.exists():
        legacy_camera_dir = episode_dir / camera_name
        if legacy_camera_dir.exists():
            warnings.append(f"legacy image directory layout detected: {legacy_camera_dir.name}/")
            camera_dir = legacy_camera_dir
        else:
            if is_strict:
                issues.append(f"missing obs/{camera_name}/")
            else:
                warnings.append(f"missing obs/{camera_name}/")

    report["frames"] = int(len(timestamps))
    report["robot_state_dim"] = int(robot_state.shape[1]) if robot_state.ndim == 2 else 0
    report["action_dim"] = int(action.shape[1]) if action.ndim == 2 else 0
    if is_strict and len(timestamps) == 0:
        issues.append("episode contains zero frames")

    if timestamps.ndim != 1:
        issues.append(f"timestamps shape should be [T], got {timestamps.shape}")
    if robot_state.ndim != 2:
        issues.append(f"robot_state shape should be [T, D], got {robot_state.shape}")
    if action.ndim != 2:
        issues.append(f"action shape should be [T, D], got {action.shape}")
    if timestamps.shape[0] != robot_state.shape[0]:
        issues.append("robot_state length does not match timestamps")
    if timestamps.shape[0] != action.shape[0]:
        issues.append("action length does not match timestamps")

    if np.any(~np.isfinite(timestamps)):
        issues.append("timestamps contain NaN/Inf")
    if robot_state.size and np.any(~np.isfinite(robot_state)):
        issues.append("robot_state contains NaN/Inf")
    if action.size and np.any(~np.isfinite(action)):
        issues.append("action contains NaN/Inf")
    if len(timestamps) > 1 and np.any(np.diff(timestamps) <= 0):
        issues.append("timestamps are not strictly increasing")

    avg_hz = _rate(timestamps)
    report["avg_hz"] = avg_hz
    expected_hz = float(meta.get("control_rate_hz", meta.get("sample_rate_hz", 0.0)))
    if expected_hz > 0.0 and len(timestamps) > 1 and abs(avg_hz - expected_hz) > rate_tolerance_hz:
        issues.append(f"avg_hz {avg_hz:.2f} differs from control_rate_hz {expected_hz:.2f}")

    if is_strict and meta.get("robot_state_dim") not in (None, report["robot_state_dim"]):
        issues.append(
            f"meta robot_state_dim {meta.get('robot_state_dim')} != array dim {report['robot_state_dim']}"
        )
    if is_strict and meta.get("action_dim") not in (None, report["action_dim"]):
        issues.append(f"meta action_dim {meta.get('action_dim')} != array dim {report['action_dim']}")
    if is_strict and len(meta.get("right_arm_joint_names", [])) not in (0, 6):
        issues.append("right_arm_joint_names must have length 6")
    if is_strict and meta.get("action_names") and len(meta["action_names"]) != report["action_dim"]:
        issues.append("action_names length does not match action_dim")

    image_issues, image_warnings, image_count = _check_images(camera_dir, len(timestamps))
    issues.extend(image_issues)
    warnings.extend(image_warnings)
    report["images"] = image_count

    duration = 0.0 if len(timestamps) < 2 else float(timestamps[-1] - timestamps[0])
    report["duration_sec"] = duration
    report["status"] = _status_level(issues, warnings)
    return report


def print_report(report: Dict[str, object]) -> None:
    print(f"{report['episode']}:")
    print(f"  frames: {report.get('frames', 0)}")
    print(f"  duration: {report.get('duration_sec', 0.0):.1f}s")
    print(f"  avg_hz: {report.get('avg_hz', 0.0):.1f}")
    print(f"  robot_state_dim: {report.get('robot_state_dim', 0)}")
    print(f"  action_dim: {report.get('action_dim', 0)}")
    print(f"  images: {report.get('images', 0)}")
    print(f"  success: {report.get('success')}")
    print(f"  valid_for_training: {report.get('valid_for_training')}")
    print(f"  status: {report.get('status')}")
    issues = report.get("issues", [])
    warnings = report.get("warnings", [])
    if warnings:
        print("  warnings:")
        for warning in warnings:
            print(f"    - {warning}")
    if issues:
        print("  issues:")
        for issue in issues:
            print(f"    - {issue}")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Validate recorded imitation episodes.")
    parser.add_argument("path", type=Path, nargs="?", default=None, help="Episode directory or raw dataset root.")
    parser.add_argument("--rate-tolerance-hz", type=float, default=2.0)
    parser.add_argument("--episode", type=Path, default=None, help="Alias for one episode path.")
    args = parser.parse_args(argv)

    target = args.episode or args.path
    if target is None:
        raise SystemExit("Provide either a dataset path or --episode.")
    episodes = list_episode_dirs(target)
    if not episodes:
        raise SystemExit(f"No episode directories found under {target}")

    failed = False
    for episode in episodes:
        report = validate_episode(episode, args.rate_tolerance_hz)
        print_report(report)
        if report["issues"]:
            failed = True
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
