"""Policy protocol and scripted stubs (C1).

A ``Policy`` turns an observation into an **action chunk**; the worker executes
the chunk on the simulation and asks again.  ``vla_stub`` runs these scripted
policies so replacing ``act()`` with a real VLA is the only integration step.

Action chunk::

    {"mode": "joint",    "targets": [[j0..j5], ...], "done": bool, "progress": float}
    {"mode": "ee_delta", "deltas":  [[dx, dy, dz], ...], "done": bool, "progress": float}

Observation (built by the worker)::

    {"joints": [...], "ee": [x, y, z], "ee_quat": [...], "gripper": {...},
     "objects": {"red_block": [...], "target_box": [...]},
     "sim_step": int, "state_version": int, "episode_id": str,
     "rgb": np.ndarray | None, "frame_id": str | None}

``PolicyFailure`` carries the worker-facing error code (``POLICY_TIMEOUT``,
``NO_PROGRESS``) so the Gateway job records a specific failure instead of a
generic ``INVALID_REQUEST``.
"""

from __future__ import annotations

import math
from typing import Any, Protocol, runtime_checkable


class PolicyFailure(Exception):
    """Raised by the worker when a policy run violates its budget/tracking."""

    def __init__(self, code: str, message: str, detail: dict[str, Any] | None = None) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message
        self.detail = detail or {}


@runtime_checkable
class Policy(Protocol):
    name: str

    def reset(self, context: dict[str, Any] | None = None) -> None: ...

    def act(self, obs: dict[str, Any]) -> dict[str, Any]: ...

    def progress(self) -> float: ...


class StubWaypointPolicy:
    """Joint-space waypoint policy: interpolate toward one waypoint per chunk.

    The waypoints are the same fixed SO-101 poses the tuned ``pick`` skill uses,
    so the stub performs a real approach/grasp/lift without a model.
    """

    name = "vla_stub/waypoint"

    def __init__(
        self,
        waypoints: list[Any],
        *,
        chunk_steps: int = 10,
        steps_per_waypoint: int = 60,
        tolerance_rad: float = 0.03,
        fail: str | None = None,
    ) -> None:
        if not waypoints:
            raise ValueError("StubWaypointPolicy requires at least one waypoint")
        # Each entry is either [j0..j5] (uses steps_per_waypoint) or
        # {"joints": [...], "steps": N} for a per-waypoint duration.
        self._waypoints: list[tuple[list[float], int]] = []
        for entry in waypoints:
            if isinstance(entry, dict):
                joints = [float(value) for value in entry["joints"]]
                steps = int(entry.get("steps", steps_per_waypoint))
            else:
                joints = [float(value) for value in entry]
                steps = int(steps_per_waypoint)
            if not joints:
                raise ValueError("waypoint has no joints")
            self._waypoints.append((joints, max(1, steps)))
        self._chunk_steps = max(1, int(chunk_steps))
        self._tolerance = float(tolerance_rad)
        self._fail = fail
        self._index = 0
        self._steps_at_waypoint = 0
        self._waypoint_start: list[float] | None = None
        self._chunks = 0

    def reset(self, context: dict[str, Any] | None = None) -> None:
        self._index = 0
        self._steps_at_waypoint = 0
        self._waypoint_start = None
        self._chunks = 0

    def progress(self) -> float:
        return min(1.0, self._index / float(len(self._waypoints)))

    def act(self, obs: dict[str, Any]) -> dict[str, Any]:
        joints = [float(value) for value in obs.get("joints") or []]
        if not joints:
            raise ValueError("observation has no joints")
        if self._fail == "no_progress":
            return {"mode": "joint", "targets": [list(joints) for _ in range(self._chunk_steps)], "done": False, "progress": 0.0}
        if self._fail == "timeout":
            # Keep moving a little so the worker's no-progress guard is not what
            # fires first; the wall-clock budget must produce POLICY_TIMEOUT.
            targets = []
            for step in range(self._chunk_steps):
                target = list(joints)
                target[0] += 0.08 * math.sin(0.35 * (self._chunks * self._chunk_steps + step))
                targets.append(target)
            self._chunks += 1
            return {"mode": "joint", "targets": targets, "done": False, "progress": 0.0}

        while self._index < len(self._waypoints):
            goal, total_steps = self._waypoints[self._index]
            if self._waypoint_start is None:
                self._waypoint_start = list(joints)
            error = max(abs(g - c) for g, c in zip(goal, joints))
            if error <= self._tolerance or self._steps_at_waypoint >= total_steps:
                self._index += 1
                self._steps_at_waypoint = 0
                self._waypoint_start = None
                continue
            targets = []
            start = self._waypoint_start
            for step in range(1, self._chunk_steps + 1):
                alpha = min(1.0, (self._steps_at_waypoint + step) / float(total_steps))
                targets.append([s + (g - s) * alpha for s, g in zip(start, goal)])
            self._steps_at_waypoint += self._chunk_steps
            self._chunks += 1
            return {"mode": "joint", "targets": targets, "done": False, "progress": self.progress()}
        return {"mode": "joint", "targets": [], "done": True, "progress": 1.0}


class StubDeltaPolicy:
    """EE-delta policy: the worker converts deltas to joint targets with DLS."""

    name = "vla_stub/ee_delta"

    def __init__(self, deltas: list[list[float]], *, repeat: int = 2, fail: str | None = None) -> None:
        if not deltas:
            raise ValueError("StubDeltaPolicy requires at least one delta")
        self._deltas = [[float(value) for value in delta] for delta in deltas]
        self._repeat = max(1, int(repeat))
        self._fail = fail
        self._round = 0

    def reset(self, context: dict[str, Any] | None = None) -> None:
        self._round = 0

    def progress(self) -> float:
        return min(1.0, self._round / float(self._repeat))

    def act(self, obs: dict[str, Any]) -> dict[str, Any]:
        if self._fail == "no_progress":
            return {"mode": "ee_delta", "deltas": [[0.0, 0.0, 0.0]] * len(self._deltas), "done": False, "progress": 0.0}
        if self._fail == "timeout":
            self._round += 1
            return {"mode": "ee_delta", "deltas": self._deltas, "done": False, "progress": 0.0}
        if self._round >= self._repeat:
            return {"mode": "ee_delta", "deltas": [], "done": True, "progress": 1.0}
        self._round += 1
        return {"mode": "ee_delta", "deltas": self._deltas, "done": False, "progress": self.progress()}


__all__ = ["Policy", "PolicyFailure", "StubWaypointPolicy", "StubDeltaPolicy"]
