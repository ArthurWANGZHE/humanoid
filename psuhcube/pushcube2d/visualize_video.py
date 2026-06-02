"""Render PushCube2D rollouts or dataset episodes to MP4 video."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict

import numpy as np

from .collect_data import rollout_episode
from .dataset import load_dataset
from .env import PushCube2DEnv
from .scripted_policy import make_policy_from_env
from .visualize_trajectory import episode_from_dataset, rollout_model


def default_out() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("outputs") / "pushcube2d" / f"video_{stamp}.mp4"


def require_imageio():
    try:
        import imageio.v2 as imageio
        import imageio_ffmpeg  # noqa: F401 - imageio uses this backend for MP4.
    except ImportError as exc:
        raise SystemExit(
            "imageio and imageio-ffmpeg are required for MP4 export. "
            "Install them with `pip install imageio imageio-ffmpeg`."
        ) from exc
    return imageio


def env_from_dataset_metadata(dataset_path: Path, render_size: int | None = None) -> PushCube2DEnv:
    data = load_dataset(dataset_path)
    env_config: Dict = dict(data.get("metadata", {}).get("env_config", {}))
    if render_size is not None:
        env_config["render_size"] = render_size
    return PushCube2DEnv(config=env_config if env_config else None)


def render_episode_frames(
    episode,
    env: PushCube2DEnv,
    stride: int = 1,
    repeat_final: int = 12,
) -> list[np.ndarray]:
    states = np.asarray(episode.get("states", []), dtype=np.float32)
    obs = np.asarray(episode.get("obs", []), dtype=np.float32)
    if states.size == 0 and obs.size == 0:
        raise ValueError("episode must contain `states` or `obs` for video rendering")

    source = states if states.size else obs
    stride = max(1, int(stride))
    frames = []
    for state in source[::stride]:
        env.set_state(state)
        frames.append(env.render("rgb_array"))

    if frames and repeat_final > 0:
        frames.extend([frames[-1].copy() for _ in range(int(repeat_final))])
    return frames


def save_mp4(frames: list[np.ndarray], out: Path, fps: int, codec: str, quality: int) -> Path:
    if not frames:
        raise ValueError("No frames to write.")
    imageio = require_imageio()
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        with imageio.get_writer(
            out,
            fps=fps,
            codec=codec,
            quality=quality,
            macro_block_size=1,
        ) as writer:
            for frame in frames:
                writer.append_data(np.asarray(frame, dtype=np.uint8))
    except Exception as exc:
        raise SystemExit(
            "Failed to write MP4. Make sure `imageio-ffmpeg` is installed, "
            "or install system ffmpeg and retry."
        ) from exc
    return out


def make_episode(args):
    if args.source == "scripted":
        env = PushCube2DEnv(render_size=args.render_size, pusher_reset_mode=args.reset_mode)
        policy = make_policy_from_env(env, seed=args.seed)
        try:
            episode = rollout_episode(env, policy, seed=args.seed, render=False)
        finally:
            env.close()
        return episode, PushCube2DEnv(render_size=args.render_size, pusher_reset_mode=args.reset_mode), f"scripted seed={args.seed} reset={args.reset_mode}"

    if args.source == "dataset":
        if args.dataset is None:
            raise SystemExit("--dataset is required for --source dataset")
        episode = episode_from_dataset(args.dataset, args.episode)
        env = env_from_dataset_metadata(args.dataset, render_size=args.render_size)
        return episode, env, f"dataset episode={args.episode}"

    if args.model is None:
        raise SystemExit("--model is required for --source model")
    episode = rollout_model(args.model, args.seed, reset_mode=args.reset_mode)
    return episode, PushCube2DEnv(render_size=args.render_size, pusher_reset_mode=args.reset_mode), f"model seed={args.seed} reset={args.reset_mode}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=["scripted", "dataset", "model"], default="scripted")
    parser.add_argument("--dataset", type=Path, default=None)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--reset-mode", choices=["aligned", "near", "random"], default="near")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--repeat-final", type=int, default=12)
    parser.add_argument("--render-size", type=int, default=512)
    parser.add_argument("--codec", type=str, default="libx264")
    parser.add_argument("--quality", type=int, default=8)
    args = parser.parse_args()

    out = args.out or default_out()
    episode, env, label = make_episode(args)
    try:
        frames = render_episode_frames(episode, env, stride=args.stride, repeat_final=args.repeat_final)
    finally:
        env.close()
    path = save_mp4(frames, out, fps=args.fps, codec=args.codec, quality=args.quality)
    print(f"saved {len(frames)} frames ({label}) to {path}")


if __name__ == "__main__":
    main()
