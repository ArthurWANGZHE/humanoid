#!/usr/bin/env python3
import argparse
import base64
import shutil
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]

try:
    import sys

    sys.path.insert(0, str(REPO_ROOT / "src" / "robot_imitation_pipeline"))
    from robot_imitation_pipeline.io_utils import write_json
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"Failed to import robot_imitation_pipeline helpers: {exc}") from exc


JPEG_BYTES = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAkGBxAQEBUQEBIVFRUVFRUVFRUVFRUVFRUVFRUXFhUV"
    "FRUYHSggGBolHRUVITEhJSkrLi4uFx8zODMsNygtLisBCgoKDg0OGhAQGi0lHyUtLS0tLS0tLS0t"
    "LS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLf/AABEIAAEAAQMBIgACEQEDEQH/"
    "xAAXAAADAQAAAAAAAAAAAAAAAAAAAQMC/8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAwDAQACEAMQ"
    "AAAB6A//xAAXEAADAQAAAAAAAAAAAAAAAAAAAREx/9oACAEBAAEFAmP/xAAVEQEBAAAAAAAAAAAA"
    "AAAAAAAAEf/aAAgBAwEBPwF//8QAFBEBAAAAAAAAAAAAAAAAAAAAEP/aAAgBAgEBPwB//8QAGBAA"
    "AgMAAAAAAAAAAAAAAAAAAREhMUH/2gAIAQEABj8CYqf/xAAWEAEBAQAAAAAAAAAAAAAAAAABABH/"
    "2gAIAQEAAT8hE2f/2gAMAwEAAgADAAAAED//xAAVEQEBAAAAAAAAAAAAAAAAAAABEP/aAAgBAwEB"
    "PxB//8QAFBEBAAAAAAAAAAAAAAAAAAAAEP/aAAgBAgEBPxB//8QAFxABAQEBAAAAAAAAAAAAAAAA"
    "AREAITFQ/9oACAEBAAE/EF2smV//2Q=="
)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Create a fake strict-format raw demo episode.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=12)
    args = parser.parse_args(argv)

    episode_dir = args.output
    if episode_dir.exists():
        shutil.rmtree(episode_dir)
    image_dir = episode_dir / "obs" / "right_wrist_camera"
    image_dir.mkdir(parents=True, exist_ok=True)

    frames = int(args.frames)
    timestamps = np.arange(frames, dtype=np.float64) * 0.1
    right_pos = np.linspace(0.0, 0.55, frames * 6, dtype=np.float64).reshape(frames, 6)
    right_vel = np.full((frames, 6), 0.05, dtype=np.float64)
    gripper_state = np.linspace(0.0, 1.0, frames, dtype=np.float64).reshape(frames, 1)
    robot_state = np.concatenate([right_pos, right_vel, gripper_state], axis=1)
    action = np.concatenate([right_pos + 0.02, gripper_state], axis=1)

    joint_pos = np.zeros((frames, 16), dtype=np.float64)
    joint_vel = np.zeros((frames, 16), dtype=np.float64)
    joint_pos[:, 6:12] = right_pos
    joint_pos[:, 15:16] = gripper_state
    joint_vel[:, 6:12] = right_vel
    actions = np.zeros((frames, 16), dtype=np.float64)
    actions[:, 6:12] = action[:, :6]
    actions[:, 15:16] = action[:, 6:7]
    action_valid = np.zeros((frames, 16), dtype=bool)
    action_valid[:, 6:12] = True
    action_valid[:, 15:16] = True
    gripper = np.concatenate([np.zeros((frames, 1), dtype=np.float64), gripper_state], axis=1)

    np.save(episode_dir / "timestamps.npy", timestamps)
    np.save(episode_dir / "joint_state_timestamps.npy", timestamps)
    np.save(episode_dir / "robot_state.npy", robot_state)
    np.save(episode_dir / "action.npy", action)
    np.save(episode_dir / "joint_pos.npy", joint_pos)
    np.save(episode_dir / "joint_vel.npy", joint_vel)
    np.save(episode_dir / "actions.npy", actions)
    np.save(episode_dir / "action_valid.npy", action_valid)
    np.save(episode_dir / "gripper.npy", gripper)
    np.save(episode_dir / "right_wrist_camera_timestamps.npy", timestamps)
    np.save(episode_dir / "right_wrist_camera_ros_timestamps.npy", timestamps)

    for idx in range(frames):
        (image_dir / f"{idx:06d}.jpg").write_bytes(JPEG_BYTES)

    write_json(
        episode_dir / "meta.json",
        {
            "schema_version": "0.2.0",
            "format": "robot_imitation_pipeline_raw_episode_v2",
            "control_mode": "joint_target",
            "control_rate_hz": 10.0,
            "robot_state_dim": 13,
            "action_dim": 7,
            "right_arm_joint_names": [
                "right_base_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_pitch_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ],
            "action_names": [
                "right_base_pitch_joint",
                "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint",
                "right_elbow_pitch_joint",
                "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
                "right_gripper_command",
            ],
            "camera_name": "right_wrist_camera",
            "camera_topics": {"right_wrist_camera": "/right_wrist_camera/image_raw"},
            "topic_names": {
                "joint_states": "/joint_states",
                "right_arm_command": "/right_joint_command",
                "right_gripper_command": "/right_gripper_command",
                "camera": "/right_wrist_camera/image_raw",
            },
            "image_size": [224, 224],
            "start_time": "2026-04-24T00:00:00+0800",
            "end_time": "2026-04-24T00:00:01+0800",
            "task_name": "toy_brick_grasp",
            "robot_name": "humanoid",
            "num_samples": frames,
            "alignment_rule": "obs[i], robot_state[i] -> action[i]",
        },
    )
    write_json(episode_dir / "success.json", {"success": True, "valid_for_training": True})
    print(f"Wrote fake episode to {episode_dir}")


if __name__ == "__main__":
    main()
