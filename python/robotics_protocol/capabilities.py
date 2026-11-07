"""Backend capability records.

A combination is only ``verified`` after a real backend run produced evidence;
``installed`` and ``verified`` are deliberately separate.  A mock backend can
never be marked verified.
"""

from __future__ import annotations

from enum import Enum
from typing import Literal

from pydantic import Field, model_validator

from .base import BackendId, RobotId, SceneId, StrictModel, Timestamp
from .errors import ErrorInfo


class BackendKind(str, Enum):
    MOCK = "mock"
    ISAACLAB_PHYSX = "isaaclab_physx"
    ISAACLAB_NEWTON = "isaaclab_newton"


class CapabilityStatus(str, Enum):
    INSTALLED = "installed"
    VERIFIED = "verified"
    FAILED = "failed"
    UNAVAILABLE = "unavailable"


class PhysicsInfo(StrictModel):
    engine: str = Field(min_length=1, max_length=64)
    version: str = Field(min_length=1, max_length=64)
    deterministic: bool


class RendererInfo(StrictModel):
    enabled: bool
    backend: str | None = Field(default=None, min_length=1, max_length=64)
    version: str | None = Field(default=None, min_length=1, max_length=64)

    @model_validator(mode="after")
    def _enabled_has_backend(self) -> "RendererInfo":
        if self.enabled and self.backend is None:
            raise ValueError("an enabled renderer requires a backend")
        return self


class VisualizerInfo(StrictModel):
    enabled: bool
    kind: str | None = Field(default=None, min_length=1, max_length=64)

    @model_validator(mode="after")
    def _enabled_has_kind(self) -> "VisualizerInfo":
        if self.enabled and self.kind is None:
            raise ValueError("an enabled visualizer requires a kind")
        return self


class CapabilityCombination(StrictModel):
    combination_id: str = Field(pattern=r"^combo_[a-z0-9_]{3,48}$")
    robot_id: RobotId
    task_id: str = Field(pattern=r"^[a-z][a-z0-9_]{2,63}$")
    scene_id: SceneId
    backend_kind: BackendKind
    status: CapabilityStatus
    evidence: list[str] = Field(default_factory=list, max_length=64)
    checked_at: Timestamp | None = None
    load_error: ErrorInfo | None = None

    @model_validator(mode="after")
    def _status_has_proof(self) -> "CapabilityCombination":
        if self.status is CapabilityStatus.VERIFIED:
            if not self.evidence:
                raise ValueError("a verified combination requires evidence")
            if self.checked_at is None:
                raise ValueError("a verified combination requires checked_at")
        if self.status in (CapabilityStatus.FAILED, CapabilityStatus.UNAVAILABLE):
            if not self.evidence and self.load_error is None:
                raise ValueError("a failed or unavailable combination must record evidence or load_error")
        return self


class BackendCapabilities(StrictModel):
    backend_id: BackendId
    backend_kind: BackendKind
    schema_version: Literal["0.1"] = "0.1"
    protocol_version: Literal["0.1"] = "0.1"
    generated_at: Timestamp
    updated_at: Timestamp
    physics: PhysicsInfo
    renderer: RendererInfo
    visualizer: VisualizerInfo
    installed: bool
    verified: bool
    evidence: list[str] = Field(default_factory=list, max_length=64)
    combinations: list[CapabilityCombination] = Field(default_factory=list, max_length=128)

    @model_validator(mode="after")
    def _honest_status(self) -> "BackendCapabilities":
        if self.verified and not self.installed:
            raise ValueError("verified implies installed")
        if self.verified and not self.evidence:
            raise ValueError("a verified backend requires evidence")
        if self.backend_kind is BackendKind.MOCK and self.verified:
            raise ValueError("a mock backend cannot be verified")
        for combination in self.combinations:
            if combination.backend_kind is not self.backend_kind:
                raise ValueError("combination backend_kind must match the backend")
        return self


__all__ = [
    "BackendKind",
    "CapabilityStatus",
    "PhysicsInfo",
    "RendererInfo",
    "VisualizerInfo",
    "CapabilityCombination",
    "BackendCapabilities",
]
