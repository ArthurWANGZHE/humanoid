"""Collect PushCube2D demonstrations with WASD/arrow-key control."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

from .collect_data import default_output_path
from .dataset import episode_from_lists, save_dataset
from .env import PushCube2DEnv


def _action_from_keys(pygame) -> np.ndarray:
    keys = pygame.key.get_pressed()
    dx = float(keys[pygame.K_RIGHT] or keys[pygame.K_d]) - float(keys[pygame.K_LEFT] or keys[pygame.K_a])
    dy = float(keys[pygame.K_UP] or keys[pygame.K_w]) - float(keys[pygame.K_DOWN] or keys[pygame.K_s])
    action = np.array([dx, dy], dtype=np.float32)
    norm = np.linalg.norm(action)
    if norm > 1.0:
        action = action / norm
    return action


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    try:
        import pygame
    except ImportError as exc:
        raise SystemExit("pygame is required for keyboard collection. Install it with `pip install pygame`.") from exc

    out = args.out or default_output_path()
    env = PushCube2DEnv()
    demos = []
    obs, _ = env.reset(seed=args.seed)
    obs_list, action_list, state_list, reward_list, done_list = [], [], [], [], []

    print("Controls: WASD/arrows move, N starts next episode, Q/Esc quits and saves collected episodes.")
    running = True
    try:
        while running and len(demos) < args.episodes:
            env.render("human")
            manual_next = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_n:
                        manual_next = True

            action = _action_from_keys(pygame)
            state = env.get_state()
            next_obs, reward, terminated, truncated, info = env.step(action)
            done = bool(terminated or truncated or manual_next)

            obs_list.append(obs.copy())
            action_list.append(action.copy())
            state_list.append(state.copy())
            reward_list.append(float(reward))
            done_list.append(done)
            obs = next_obs

            if done:
                success = bool(info.get("success", False))
                demos.append(episode_from_lists(obs_list, action_list, state_list, reward_list, done_list, success))
                print(f"recorded episode {len(demos)}/{args.episodes}: len={len(action_list)} success={int(success)}")
                obs, _ = env.reset(seed=args.seed + len(demos))
                obs_list, action_list, state_list, reward_list, done_list = [], [], [], [], []
    finally:
        env.close()

    if demos:
        metadata = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "env_config": env.config.__dict__,
            "collector": "keyboard",
            "seed": args.seed,
        }
        path = save_dataset(out, demos, metadata)
        print(f"saved {len(demos)} episodes to {path}")
    else:
        print("no completed episodes collected")


if __name__ == "__main__":
    main()

