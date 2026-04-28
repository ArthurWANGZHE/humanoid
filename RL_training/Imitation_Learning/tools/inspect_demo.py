#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "src" / "robot_imitation_pipeline"))

from robot_imitation_pipeline.io_utils import load_episode_arrays, read_json  # noqa: E402


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Inspect one raw imitation episode.")
    parser.add_argument("--episode", type=Path, required=True)
    args = parser.parse_args(argv)

    episode = args.episode
    meta = read_json(episode / "meta.json")
    arrays = load_episode_arrays(episode)
    timestamps = arrays.get("timestamps", np.zeros((0,), dtype=np.float64))
    robot_state = arrays.get("robot_state", np.zeros((0, 0), dtype=np.float64))
    action = arrays.get("action", np.zeros((0, 0), dtype=np.float64))
    camera_name = meta.get("camera_name", "unknown")
    image_dir = episode / "obs" / camera_name
    image_paths = sorted(image_dir.glob("*.jpg")) if image_dir.exists() else []

    print(f"episode: {episode}")
    print(f"task_name: {meta.get('task_name')}")
    print(f"robot_name: {meta.get('robot_name')}")
    print(f"camera_name: {camera_name}")
    print(f"control_mode: {meta.get('control_mode')}")
    print(f"control_rate_hz: {meta.get('control_rate_hz')}")
    if len(timestamps):
        print(f"first_timestamp: {timestamps[0]:.6f}")
        print(f"last_timestamp: {timestamps[-1]:.6f}")
    else:
        print("first_timestamp: n/a")
        print("last_timestamp: n/a")
    print(f"first_robot_state: {robot_state[0].tolist() if len(robot_state) else []}")
    print(f"first_action: {action[0].tolist() if len(action) else []}")
    if len(action):
        print(f"action_min: {np.min(action, axis=0).tolist()}")
        print(f"action_max: {np.max(action, axis=0).tolist()}")
    else:
        print("action_min: []")
        print("action_max: []")
    print(f"image_count: {len(image_paths)}")
    print(f"first_image: {image_paths[0] if image_paths else 'n/a'}")
    print(f"last_image: {image_paths[-1] if image_paths else 'n/a'}")


if __name__ == "__main__":
    main()
