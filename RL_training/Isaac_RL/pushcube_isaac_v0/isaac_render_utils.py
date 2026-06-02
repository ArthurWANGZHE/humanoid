"""Isaac-side viewport camera and frame capture helpers."""

from __future__ import annotations

import asyncio
from pathlib import Path

import numpy as np


CAMERA_PRESET_NAMES = ["presentation", "top_oblique", "side", "wrist_optional"]


def camera_pose_for_preset(camera_name: str, table_center_xy: np.ndarray, table_top_z: float) -> tuple[np.ndarray, np.ndarray]:
    center = np.array([float(table_center_xy[0]), float(table_center_xy[1]), float(table_top_z)], dtype=np.float64)
    target = center + np.array([0.0, 0.02, 0.08], dtype=np.float64)
    presets = {
        "presentation": center + np.array([-0.40, -0.78, 0.42], dtype=np.float64),
        "top_oblique": center + np.array([0.02, -0.56, 0.78], dtype=np.float64),
        "side": center + np.array([-0.68, -0.10, 0.26], dtype=np.float64),
        "wrist_optional": center + np.array([-0.18, -0.22, 0.22], dtype=np.float64),
    }
    if camera_name not in presets:
        raise ValueError(f"Unknown camera preset: {camera_name}")
    return presets[camera_name], target


def configure_active_viewport_camera(camera_name: str, table_center_xy: np.ndarray, table_top_z: float, resolution: tuple[int, int]):
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport

    eye, target = camera_pose_for_preset(camera_name, table_center_xy, table_top_z)
    viewport_api = get_active_viewport()
    if viewport_api is None:
        raise RuntimeError("Could not access the active viewport.")
    viewport_api.resolution = resolution
    set_camera_view(eye=eye.tolist(), target=target.tolist(), camera_prim_path="/OmniverseKit_Persp")
    viewport_api.camera_path = "/OmniverseKit_Persp"
    return viewport_api


def capture_viewport_rgb(viewport_api) -> np.ndarray:
    from isaacsim.test.utils.image_capture import capture_viewport_annotator_data_async

    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    frame = loop.run_until_complete(capture_viewport_annotator_data_async(viewport_api, annotator_name="rgb"))
    return np.asarray(frame, dtype=np.uint8)[..., :3]


def write_mp4(output_path: Path, frames: list[np.ndarray], fps: int) -> None:
    imageio = __import__("imageio.v2", fromlist=["mimsave"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(output_path, frames, fps=fps)

