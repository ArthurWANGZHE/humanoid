"""A small deterministic 2D PushCube environment.

The environment intentionally uses simple contact dynamics so data collection
and policy debugging stay transparent. The pusher is a circle. The cube is an
axis-aligned square by default; yaw is included in the observation/state for
API compatibility, but v1 leaves it fixed unless ``yaw_gain`` is non-zero.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:  # Optional dependency.
    import gymnasium as gym
    from gymnasium import spaces
except Exception:  # pragma: no cover - exercised only when gymnasium is absent.
    gym = None
    spaces = None


OBS_NAMES = [
    "pusher_x",
    "pusher_y",
    "cube_x",
    "cube_y",
    "cube_theta",
    "goal_x",
    "goal_y",
]
ACTION_NAMES = ["dx", "dy"]
STATE_NAMES = OBS_NAMES + ["step_count"]


@dataclass
class PushCube2DConfig:
    table_low: float = 0.0
    table_high: float = 1.0
    cube_size: float = 0.12
    pusher_radius: float = 0.035
    action_scale: float = 0.035
    max_steps: int = 160
    success_threshold: float = 0.055
    push_gain: float = 0.95
    yaw_gain: float = 0.0
    render_size: int = 512
    pusher_reset_mode: str = "near"
    pusher_reset_distance_min: float = 0.12
    pusher_reset_distance_max: float = 0.19
    pusher_reset_angle_noise: float = 1.8

    @property
    def bounds(self) -> Tuple[float, float]:
        return self.table_low, self.table_high


class PushCube2DEnv(gym.Env if gym is not None else object):
    """Simple 2D pushing environment with Gymnasium-like API."""

    metadata = {"render_modes": ["rgb_array", "human"], "render_fps": 30}

    def __init__(self, config: Optional[PushCube2DConfig | Dict[str, Any]] = None, **kwargs: Any):
        if config is None:
            config = PushCube2DConfig()
        elif isinstance(config, dict):
            config = PushCube2DConfig(**config)
        if kwargs:
            cfg = asdict(config)
            cfg.update(kwargs)
            config = PushCube2DConfig(**cfg)

        self.config = config
        self.rng = np.random.default_rng()
        self.pusher_xy = np.zeros(2, dtype=np.float32)
        self.cube_xy = np.zeros(2, dtype=np.float32)
        self.cube_theta = 0.0
        self.goal_xy = np.zeros(2, dtype=np.float32)
        self.step_count = 0
        self._last_contact = False
        self._pygame = None
        self._screen = None
        self._clock = None

        if spaces is not None:
            low = np.array([config.table_low, config.table_low, config.table_low, config.table_low, -np.pi, config.table_low, config.table_low], dtype=np.float32)
            high = np.array([config.table_high, config.table_high, config.table_high, config.table_high, np.pi, config.table_high, config.table_high], dtype=np.float32)
            self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}

        low, high = self.config.bounds
        half = self.config.cube_size / 2.0
        margin = max(half, self.config.pusher_radius) + 0.02

        cube_x_range = options.get("cube_x_range", (0.24, 0.43))
        cube_y_range = options.get("cube_y_range", (0.32, 0.68))
        goal_x_range = options.get("goal_x_range", (0.66, 0.88))
        goal_y_range = options.get("goal_y_range", (0.24, 0.76))

        self.cube_xy = np.array(
            [
                self.rng.uniform(max(low + half, cube_x_range[0]), min(high - half, cube_x_range[1])),
                self.rng.uniform(max(low + half, cube_y_range[0]), min(high - half, cube_y_range[1])),
            ],
            dtype=np.float32,
        )
        self.goal_xy = np.array(
            [
                self.rng.uniform(max(low + half, goal_x_range[0]), min(high - half, goal_x_range[1])),
                self.rng.uniform(max(low + half, goal_y_range[0]), min(high - half, goal_y_range[1])),
            ],
            dtype=np.float32,
        )

        reset_mode = options.get("pusher_reset_mode", self.config.pusher_reset_mode)
        pusher = self._sample_pusher_position(reset_mode)
        self.pusher_xy = self._clip_circle(pusher).astype(np.float32)

        # If clipping accidentally places the pusher in the cube, retry near the
        # same behind point with a larger standoff. This keeps reset deterministic.
        if self._circle_square_intersect(self.pusher_xy, self.cube_xy):
            direction = self._unit(self.goal_xy - self.cube_xy)
            fallback_distance = half + self.config.pusher_radius + 0.08
            self.pusher_xy = self._clip_circle(self.cube_xy - direction * fallback_distance).astype(np.float32)

        self.cube_theta = 0.0
        self.step_count = 0
        self._last_contact = False
        info = self._info()
        return self._obs(), info

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(2)
        action = np.clip(action, -1.0, 1.0)
        delta = action * self.config.action_scale

        old_pusher = self.pusher_xy.copy()
        new_pusher = self._clip_circle(old_pusher + delta)
        actual_delta = new_pusher - old_pusher
        self.pusher_xy = new_pusher.astype(np.float32)

        contact = self._circle_square_intersect(self.pusher_xy, self.cube_xy)
        if contact and np.linalg.norm(actual_delta) > 1e-8:
            cube_delta = actual_delta * self.config.push_gain
            self.cube_xy = self._clip_cube(self.cube_xy + cube_delta).astype(np.float32)
            if self.config.yaw_gain != 0.0:
                offset = self.pusher_xy - self.cube_xy
                torque = offset[0] * cube_delta[1] - offset[1] * cube_delta[0]
                self.cube_theta = float(self._wrap_angle(self.cube_theta + self.config.yaw_gain * torque / max(self.config.cube_size**2, 1e-6)))
        self._last_contact = bool(contact)

        self.step_count += 1
        dist = self.distance_to_goal()
        success = dist < self.config.success_threshold
        terminated = bool(success)
        truncated = bool(self.step_count >= self.config.max_steps)
        reward = float(-dist + (1.0 if success else 0.0))
        return self._obs(), reward, terminated, truncated, self._info()

    def render(self, mode: str = "rgb_array"):
        frame = self._render_rgb()
        if mode == "rgb_array":
            return frame
        if mode == "human":
            return self._render_human(frame)
        raise ValueError(f"Unsupported render mode: {mode!r}")

    def get_state(self) -> np.ndarray:
        return np.array(
            [
                self.pusher_xy[0],
                self.pusher_xy[1],
                self.cube_xy[0],
                self.cube_xy[1],
                self.cube_theta,
                self.goal_xy[0],
                self.goal_xy[1],
                float(self.step_count),
            ],
            dtype=np.float32,
        )

    def set_state(self, state) -> None:
        if isinstance(state, dict):
            self.pusher_xy = np.asarray(state["pusher_xy"], dtype=np.float32).reshape(2)
            self.cube_xy = np.asarray(state["cube_xy"], dtype=np.float32).reshape(2)
            self.cube_theta = float(state.get("cube_theta", 0.0))
            self.goal_xy = np.asarray(state["goal_xy"], dtype=np.float32).reshape(2)
            self.step_count = int(state.get("step_count", 0))
            return

        state = np.asarray(state, dtype=np.float32).reshape(-1)
        if state.shape[0] not in (7, 8):
            raise ValueError("state must have 7 obs values or 8 state values")
        self.pusher_xy = state[0:2].astype(np.float32)
        self.cube_xy = state[2:4].astype(np.float32)
        self.cube_theta = float(state[4])
        self.goal_xy = state[5:7].astype(np.float32)
        self.step_count = int(state[7]) if state.shape[0] == 8 else 0

    def close(self) -> None:
        if self._pygame is not None:
            self._pygame.quit()
        self._pygame = None
        self._screen = None
        self._clock = None

    def distance_to_goal(self) -> float:
        return float(np.linalg.norm(self.cube_xy - self.goal_xy))

    def _obs(self) -> np.ndarray:
        return np.array(
            [
                self.pusher_xy[0],
                self.pusher_xy[1],
                self.cube_xy[0],
                self.cube_xy[1],
                self.cube_theta,
                self.goal_xy[0],
                self.goal_xy[1],
            ],
            dtype=np.float32,
        )

    def _info(self) -> Dict[str, Any]:
        dist = self.distance_to_goal()
        return {
            "distance_to_goal": dist,
            "success": bool(dist < self.config.success_threshold),
            "contact": self._last_contact,
            "step_count": self.step_count,
        }

    def _clip_circle(self, xy: np.ndarray) -> np.ndarray:
        low, high = self.config.bounds
        r = self.config.pusher_radius
        return np.clip(np.asarray(xy, dtype=np.float32), low + r, high - r)

    def _clip_cube(self, xy: np.ndarray) -> np.ndarray:
        low, high = self.config.bounds
        half = self.config.cube_size / 2.0
        return np.clip(np.asarray(xy, dtype=np.float32), low + half, high - half)

    def _circle_square_intersect(self, circle_xy: np.ndarray, square_xy: np.ndarray) -> bool:
        half = self.config.cube_size / 2.0
        closest = np.clip(circle_xy, square_xy - half, square_xy + half)
        return bool(np.linalg.norm(circle_xy - closest) <= self.config.pusher_radius + 1e-6)

    def _sample_pusher_position(self, reset_mode: str) -> np.ndarray:
        half = self.config.cube_size / 2.0
        base_direction = -self._unit(self.goal_xy - self.cube_xy)
        min_distance = max(
            self.config.pusher_reset_distance_min,
            half + self.config.pusher_radius + 0.02,
        )
        max_distance = max(self.config.pusher_reset_distance_max, min_distance + 0.01)

        if reset_mode == "aligned":
            direction = base_direction
            distance = half + self.config.pusher_radius + self.rng.uniform(0.045, 0.075)
            side_offset = self._perp(direction) * self.rng.uniform(-0.025, 0.025)
            return self.cube_xy + direction * distance + side_offset

        if reset_mode == "near":
            angle = self.rng.uniform(-self.config.pusher_reset_angle_noise, self.config.pusher_reset_angle_noise)
            direction = self._rotate(base_direction, angle)
        elif reset_mode == "random":
            angle = self.rng.uniform(-np.pi, np.pi)
            direction = np.array([np.cos(angle), np.sin(angle)], dtype=np.float32)
        else:
            raise ValueError("pusher_reset_mode must be one of: aligned, near, random")

        distance = self.rng.uniform(min_distance, max_distance)
        return self.cube_xy + direction * distance

    @staticmethod
    def _unit(vec: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vec))
        if norm < 1e-8:
            return np.array([1.0, 0.0], dtype=np.float32)
        return (vec / norm).astype(np.float32)

    @staticmethod
    def _perp(vec: np.ndarray) -> np.ndarray:
        return np.array([-vec[1], vec[0]], dtype=np.float32)

    @staticmethod
    def _wrap_angle(theta: float) -> float:
        return float((theta + np.pi) % (2.0 * np.pi) - np.pi)

    @staticmethod
    def _rotate(vec: np.ndarray, angle: float) -> np.ndarray:
        c = float(np.cos(angle))
        s = float(np.sin(angle))
        return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]], dtype=np.float32)

    def _world_to_px(self, xy: np.ndarray) -> Tuple[int, int]:
        low, high = self.config.bounds
        size = self.config.render_size
        normalized = (np.asarray(xy) - low) / (high - low)
        x = int(np.clip(normalized[0] * (size - 1), 0, size - 1))
        y = int(np.clip((1.0 - normalized[1]) * (size - 1), 0, size - 1))
        return x, y

    def _length_to_px(self, length: float) -> int:
        low, high = self.config.bounds
        return max(1, int(round(length / (high - low) * (self.config.render_size - 1))))

    def _render_rgb(self) -> np.ndarray:
        size = self.config.render_size
        frame = np.full((size, size, 3), [246, 244, 238], dtype=np.uint8)
        frame[[0, -1], :, :] = [70, 74, 84]
        frame[:, [0, -1], :] = [70, 74, 84]

        # Draw goal region.
        gx, gy = self._world_to_px(self.goal_xy)
        goal_r = self._length_to_px(self.config.success_threshold)
        self._draw_disc(frame, gx, gy, goal_r, color=np.array([120, 190, 120], dtype=np.uint8), alpha=0.35)
        self._draw_ring(frame, gx, gy, goal_r, color=np.array([50, 130, 70], dtype=np.uint8))

        # Draw cube as a filled square. Yaw is not visualized in v1 when fixed.
        cx, cy = self._world_to_px(self.cube_xy)
        half_px = self._length_to_px(self.config.cube_size / 2.0)
        x0, x1 = max(0, cx - half_px), min(size - 1, cx + half_px)
        y0, y1 = max(0, cy - half_px), min(size - 1, cy + half_px)
        frame[y0 : y1 + 1, x0 : x1 + 1, :] = [86, 132, 202]
        frame[y0, x0 : x1 + 1, :] = [33, 70, 130]
        frame[y1, x0 : x1 + 1, :] = [33, 70, 130]
        frame[y0 : y1 + 1, x0, :] = [33, 70, 130]
        frame[y0 : y1 + 1, x1, :] = [33, 70, 130]

        px, py = self._world_to_px(self.pusher_xy)
        pr = self._length_to_px(self.config.pusher_radius)
        self._draw_disc(frame, px, py, pr, color=np.array([218, 96, 77], dtype=np.uint8), alpha=1.0)
        self._draw_ring(frame, px, py, pr, color=np.array([125, 40, 35], dtype=np.uint8))
        return frame

    @staticmethod
    def _draw_disc(frame: np.ndarray, cx: int, cy: int, radius: int, color: np.ndarray, alpha: float) -> None:
        h, w = frame.shape[:2]
        y0, y1 = max(0, cy - radius), min(h - 1, cy + radius)
        x0, x1 = max(0, cx - radius), min(w - 1, cx + radius)
        yy, xx = np.ogrid[y0 : y1 + 1, x0 : x1 + 1]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius**2
        patch = frame[y0 : y1 + 1, x0 : x1 + 1]
        if alpha >= 1.0:
            patch[mask] = color
        else:
            patch[mask] = (patch[mask].astype(np.float32) * (1.0 - alpha) + color.astype(np.float32) * alpha).astype(np.uint8)

    @staticmethod
    def _draw_ring(frame: np.ndarray, cx: int, cy: int, radius: int, color: np.ndarray) -> None:
        h, w = frame.shape[:2]
        thickness = max(1, radius // 12)
        y0, y1 = max(0, cy - radius - thickness), min(h - 1, cy + radius + thickness)
        x0, x1 = max(0, cx - radius - thickness), min(w - 1, cx + radius + thickness)
        yy, xx = np.ogrid[y0 : y1 + 1, x0 : x1 + 1]
        dist2 = (xx - cx) ** 2 + (yy - cy) ** 2
        mask = (radius - thickness) ** 2 <= dist2
        mask &= dist2 <= (radius + thickness) ** 2
        frame[y0 : y1 + 1, x0 : x1 + 1][mask] = color

    def _render_human(self, frame: np.ndarray):
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("pygame is required for human rendering. Install it with `pip install pygame`.") from exc

        if self._pygame is None:
            pygame.init()
            self._pygame = pygame
            self._screen = pygame.display.set_mode((self.config.render_size, self.config.render_size))
            pygame.display.set_caption("PushCube2D")
            self._clock = pygame.time.Clock()

        self._pygame.event.pump()
        if self._pygame.event.peek(self._pygame.QUIT):
            self.close()
            return frame
        surface = self._pygame.surfarray.make_surface(np.transpose(frame, (1, 0, 2)))
        self._screen.blit(surface, (0, 0))
        self._pygame.display.flip()
        self._clock.tick(self.metadata["render_fps"])
        return frame
