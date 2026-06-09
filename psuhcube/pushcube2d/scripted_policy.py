"""Scripted PushCube2D policy for collecting starter demonstrations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ScriptedPolicyConfig:
    cube_size: float = 0.12
    pusher_radius: float = 0.035
    action_scale: float = 0.035
    success_threshold: float = 0.055
    behind_margin: float = 0.045
    contact_slop: float = 0.012
    position_gain: float = 1.8
    push_speed: float = 0.95
    max_action: float = 1.0
    lateral_correction_gain: float = 0.7
    alignment_tolerance: float = 0.08
    clearance_margin: float = 0.08
    orbit_speed: float = 0.85
    radial_gain: float = 1.2
    align_cos_threshold: float = 0.93


class ScriptedPushPolicy:
    """Move behind the cube, approach contact, then push toward the goal.

    If the pusher is already behind the cube, the policy pushes directly. If it
    starts elsewhere, it first backs out to a clearance radius and orbits around
    the cube to the behind position before approaching contact.
    """

    def __init__(self, config: ScriptedPolicyConfig | None = None, noise_std: float = 0.0, seed: int | None = None):
        self.config = config or ScriptedPolicyConfig()
        self.noise_std = float(noise_std)
        self.rng = np.random.default_rng(seed)

    def reset(self) -> None:
        pass

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32)
        pusher = obs[0:2]
        cube = obs[2:4]
        goal = obs[5:7]

        to_goal = goal - cube
        goal_dist = float(np.linalg.norm(to_goal))
        if goal_dist < self.config.success_threshold:
            return np.zeros(2, dtype=np.float32)

        direction = _unit(to_goal)
        lateral = _perp(direction)
        half = self.config.cube_size / 2.0
        contact_distance = half + self.config.pusher_radius - self.config.contact_slop
        behind_distance = half + self.config.pusher_radius + self.config.behind_margin
        clearance_distance = half + self.config.pusher_radius + self.config.clearance_margin

        behind = cube - direction * behind_distance
        contact_point = cube - direction * contact_distance

        relative = pusher - cube
        radial_distance = float(np.linalg.norm(relative))
        radial = _unit(relative) if radial_distance > 1e-8 else -direction
        behind_radial = -direction
        forward_from_cube = float(np.dot(relative, direction))
        lateral_from_cube = float(np.dot(relative, lateral))
        aligned_behind = float(np.dot(radial, behind_radial)) > self.config.align_cos_threshold

        if forward_from_cube < -0.02 and abs(lateral_from_cube) < self.config.alignment_tolerance:
            desired_delta = direction * (self.config.push_speed * self.config.action_scale)
        elif not aligned_behind:
            if radial_distance < clearance_distance - 0.01:
                desired_point = cube + radial * clearance_distance
                desired_delta = (desired_point - pusher) * self.config.position_gain
            else:
                cross = radial[0] * behind_radial[1] - radial[1] * behind_radial[0]
                tangent = np.sign(cross) * _perp(radial)
                if np.linalg.norm(tangent) < 1e-8:
                    tangent = _perp(radial)
                radial_error = clearance_distance - radial_distance
                desired_delta = tangent * (self.config.orbit_speed * self.config.action_scale)
                desired_delta += radial * (self.config.radial_gain * radial_error)
        elif np.linalg.norm(pusher - behind) > 0.025:
            desired_delta = (behind - pusher) * self.config.position_gain
        else:
            desired_delta = (contact_point - pusher) * self.config.position_gain

        action = desired_delta / max(self.config.action_scale, 1e-8)
        action_norm = float(np.linalg.norm(action))
        if action_norm > self.config.max_action:
            action = action / action_norm * self.config.max_action
        if self.noise_std > 0.0:
            action = action + self.rng.normal(0.0, self.noise_std, size=2)
            action_norm = float(np.linalg.norm(action))
            if action_norm > self.config.max_action:
                action = action / action_norm * self.config.max_action
        return np.clip(action, -self.config.max_action, self.config.max_action).astype(np.float32)


def make_policy_from_env(env, noise_std: float = 0.0, seed: int | None = None) -> ScriptedPushPolicy:
    cfg = env.config
    return ScriptedPushPolicy(
        ScriptedPolicyConfig(
            cube_size=cfg.cube_size,
            pusher_radius=cfg.pusher_radius,
            action_scale=cfg.action_scale,
            success_threshold=cfg.success_threshold,
        ),
        noise_std=noise_std,
        seed=seed,
    )


def _unit(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-8:
        return np.array([1.0, 0.0], dtype=np.float32)
    return (vec / norm).astype(np.float32)


def _perp(vec: np.ndarray) -> np.ndarray:
    return np.array([-vec[1], vec[0]], dtype=np.float32)
