"""Backend adapter contract shared by the fake and real Isaac Workers."""

from __future__ import annotations

from abc import ABC, abstractmethod

from robotics_protocol import (
    BackendCapabilities,
    BackendKind,
    JobEvent,
    JobId,
    SimulationSpec,
    SkillInvocation,
    WorldState,
)


class SimulationBackend(ABC):
    """One controlled simulation instance.

    The Worker owns the main thread and calls these methods; the Gateway never
    touches physics directly.
    """

    backend_id: str
    backend_kind: BackendKind

    @abstractmethod
    def describe_capabilities(self) -> BackendCapabilities:
        """Return installed/verified capability records with evidence."""

    @abstractmethod
    def start(self, spec: SimulationSpec) -> WorldState:
        """Start one simulation and return the first observable state."""

    @abstractmethod
    def observe(self) -> WorldState:
        """Return the current state snapshot without advancing physics."""

    @abstractmethod
    def reset(self, seed: int | None = None) -> WorldState:
        """Start a new episode; existing jobs must be canceled first."""

    @abstractmethod
    def submit_skill(self, invocation: SkillInvocation) -> JobId:
        """Accept one bounded skill and return its job id."""

    @abstractmethod
    def cancel_skill(self, job_id: str) -> bool:
        """Request cancellation; True only when the Worker acknowledged it."""

    @abstractmethod
    def drain_events(self) -> list[JobEvent]:
        """Return pending ordered events and clear the local queue."""

    @abstractmethod
    def close(self) -> None:
        """Release all owned resources; later calls must fail fast."""


__all__ = ["SimulationBackend"]
