"""Isaac Lab PhysX backend (T02: Franka software chain).

The scene/worker imports pull in Isaac Lab and are therefore lazy: protocol and
command-loop tests can import ``robotics_backends.isaaclab_physx.server`` without
launching Kit.
"""

from typing import Any

__all__ = ["FrankaSceneCfg", "build_franka_scene", "IsaacLabWorker"]


def __getattr__(name: str) -> Any:
    if name in {"FrankaSceneCfg", "build_franka_scene"}:
        from .scene import FrankaSceneCfg, build_franka_scene

        return {"FrankaSceneCfg": FrankaSceneCfg, "build_franka_scene": build_franka_scene}[name]
    if name == "IsaacLabWorker":
        from .worker import IsaacLabWorker

        return IsaacLabWorker
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
