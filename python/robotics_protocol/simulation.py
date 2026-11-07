"""Simulation launch specification and the fixed Worker profile allowlist."""

from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from .base import BackendId, ProfileId, RobotId, SceneId, StrictModel
from .capabilities import BackendKind


class PhysicsConfig(StrictModel):
    engine: Literal["physx", "newton"]
    dt: float = Field(gt=0.0, le=0.1)
    decimation: int = Field(ge=1, le=32)
    solver_iterations: int = Field(ge=1, le=64)


class CameraConfig(StrictModel):
    name: str = Field(min_length=1, max_length=64)
    width: int = Field(ge=16, le=4096)
    height: int = Field(ge=16, le=4096)
    update_period_s: float = Field(gt=0.0, le=10.0)
    data_types: list[Literal["rgb", "depth"]] = Field(min_length=1, max_length=2)


class RendererConfig(StrictModel):
    enabled: bool
    headless: bool
    backend: str | None = Field(default=None, min_length=1, max_length=64)
    cameras: list[CameraConfig] = Field(default_factory=list, max_length=8)

    @model_validator(mode="after")
    def _camera_requires_renderer(self) -> "RendererConfig":
        if self.cameras and not self.enabled:
            raise ValueError("cameras require the renderer")
        if self.enabled and self.backend is None:
            raise ValueError("an enabled renderer requires a backend")
        return self


class SimulationBudget(StrictModel):
    max_sim_time_s: float = Field(gt=0.0, le=86_400.0)
    max_wall_time_s: float = Field(gt=0.0, le=86_400.0)


class WorkerProfile(StrictModel):
    """A launch profile the Gateway resolves; clients never pass an executable."""

    profile_id: ProfileId
    backend_id: BackendId
    backend_kind: BackendKind
    robot_id: RobotId
    scene_id: SceneId
    python_executable: str = Field(min_length=1, max_length=1024)
    entry_module: str = Field(pattern=r"^[a-zA-Z_][a-zA-Z0-9_.]*$")
    allowed_imports: list[str] = Field(default_factory=list, max_length=64)
    max_envs: int = Field(ge=1, le=64)
    description: str = Field(min_length=1, max_length=256)


class SimulationSpec(StrictModel):
    profile_id: ProfileId
    backend_kind: BackendKind
    robot_id: RobotId
    scene_id: SceneId
    physics: PhysicsConfig
    renderer: RendererConfig
    seed: int = Field(ge=0, le=2**31 - 1)
    num_envs: int = Field(ge=1, le=64)
    budget: SimulationBudget

    @model_validator(mode="after")
    def _engine_matches_backend(self) -> "SimulationSpec":
        expected = {
            BackendKind.ISAACLAB_PHYSX: "physx",
            BackendKind.ISAACLAB_NEWTON: "newton",
        }.get(self.backend_kind)
        if expected is not None and self.physics.engine != expected:
            raise ValueError(f"{self.backend_kind.value} requires physics engine {expected}")
        return self


__all__ = [
    "PhysicsConfig",
    "CameraConfig",
    "RendererConfig",
    "SimulationBudget",
    "WorkerProfile",
    "SimulationSpec",
]
