"""Shared Isaac Sim viewer lifecycle helpers."""

from __future__ import annotations


def _step_context(world_or_context, render: bool) -> None:
    if hasattr(world_or_context, "step_world"):
        world_or_context.step_world(steps=1)
        return
    if hasattr(world_or_context, "world") and hasattr(world_or_context.world, "step"):
        world_or_context.world.step(render=render)
        return
    if hasattr(world_or_context, "step"):
        world_or_context.step(render=render)
        return
    raise TypeError("world_or_context must expose step_world(), world.step(), or step()")


def hold_viewer_open(world_or_context, simulation_app, render: bool = True) -> None:
    print("[lifecycle] entering hold-open loop")
    frame_count = 0
    try:
        while simulation_app.is_running():
            _step_context(world_or_context, render=render)
            frame_count += 1
            if frame_count == 1 or frame_count % 60 == 0:
                print(f"[lifecycle] hold-open frame {frame_count}")
    except KeyboardInterrupt:
        pass
    print("[lifecycle] hold-open loop ended")


def close_simulation_app(simulation_app) -> None:
    if simulation_app is None:
        return
    if hasattr(simulation_app, "close"):
        simulation_app.close()
