#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = REPO_ROOT / "RL_training" / "Imitation_Learning" / "config" / "cameras.yaml"


def run_command(cmd: List[str], timeout: float = 5.0) -> str:
    try:
        completed = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except FileNotFoundError as exc:
        raise SystemExit(f"Required command not found: {cmd[0]}") from exc
    except subprocess.TimeoutExpired:
        return ""
    return completed.stdout.strip()


def get_topic_list() -> List[str]:
    output = run_command(["ros2", "topic", "list"])
    return [line.strip() for line in output.splitlines() if line.strip()]


def format_hz(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:.1f}"


def choose_topic(camera_cfg: Dict[str, object], available_topics: List[str]) -> str:
    candidates = [str(camera_cfg["image_topic"])]
    candidates.extend(str(topic) for topic in camera_cfg.get("alternate_image_topics", []))
    for topic in candidates:
        if topic in available_topics:
            return topic
    return candidates[0]


def print_block(camera_name: str, lines: List[str]) -> None:
    print(f"  {camera_name}:")
    for line in lines:
        print(f"    {line}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check camera topics required by the imitation recorder.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Camera config YAML.")
    parser.add_argument("--sample-seconds", type=float, default=3.0, help="Sampling time for frequency estimate.")
    parser.add_argument(
        "--skip-head-camera",
        action="store_true",
        help="Ignore head camera checks even if configured.",
    )
    args = parser.parse_args()

    if not args.config.exists():
        raise SystemExit(f"Camera config not found: {args.config}")

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise SystemExit(
            "ROS 2 Python modules are not available. Source your ROS 2 environment before running "
            "`python3 RL_training/Imitation_Learning/tools/check_camera_topics.py`."
        ) from exc

    class TopicProbe(Node):
        def __init__(self, topic: str, duration_sec: float) -> None:
            super().__init__("camera_topic_probe")
            self.topic = topic
            self.duration_sec = duration_sec
            self.message_times: List[float] = []
            self.subscription = self.create_subscription(Image, topic, self._callback, 10)

        def _callback(self, msg: Image) -> None:
            del msg
            self.message_times.append(time.monotonic())

        def sample(self) -> Dict[str, Optional[float]]:
            deadline = time.monotonic() + self.duration_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            if not self.message_times:
                return {"received": False, "hz": None}
            if len(self.message_times) < 2:
                return {"received": True, "hz": None}

            elapsed = self.message_times[-1] - self.message_times[0]
            hz = None if elapsed <= 0.0 else (len(self.message_times) - 1) / elapsed
            return {"received": True, "hz": hz}

    with args.config.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    available_topics = get_topic_list()

    print("Camera topic diagnostics:")

    rclpy.init(args=None)
    exit_code = 0
    try:
        for camera_name in ("wrist_camera", "head_camera"):
            if camera_name == "head_camera" and args.skip_head_camera:
                continue

            camera_cfg = config.get(camera_name)
            if not isinstance(camera_cfg, dict):
                continue

            required = bool(camera_cfg.get("required", False))
            topic = choose_topic(camera_cfg, available_topics)
            lines = [
                f"required: {'true' if required else 'false'}",
                f"topic: {topic}",
            ]

            if topic not in available_topics:
                lines.append("status: MISSING")
                lines.append(
                    "action: ignored because optional"
                    if not required
                    else "action: start the wrist camera pipeline before recording"
                )
                print_block(camera_name, lines)
                if required:
                    exit_code = 1
                continue

            topic_type = run_command(["ros2", "topic", "type", topic]) or "unknown"
            lines.append(f"type: {topic_type}")
            if topic_type != "sensor_msgs/msg/Image":
                lines.append("status: WRONG_TYPE")
                lines.append("action: expected sensor_msgs/msg/Image")
                print_block(camera_name, lines)
                if required:
                    exit_code = 1
                continue

            probe = TopicProbe(topic, args.sample_seconds)
            try:
                result = probe.sample()
            finally:
                probe.destroy_node()

            if not result["received"]:
                lines.append("status: NO_MESSAGES")
                lines.append(
                    "action: ignored because optional"
                    if not required
                    else "action: camera topic exists but is not publishing frames"
                )
                print_block(camera_name, lines)
                if required:
                    exit_code = 1
                continue

            lines.append("status: OK")
            lines.append(f"hz: {format_hz(result['hz'])}")
            print_block(camera_name, lines)
    finally:
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
