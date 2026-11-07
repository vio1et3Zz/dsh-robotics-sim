"""Fixed Worker profiles.

Clients send a ``SimulationSpec`` that only names a profile.  The executable,
entry module and environment come from this registry, never from HTTP input.
"""

from __future__ import annotations

import os
import sys
from datetime import datetime, timezone

from robotics_protocol import (
    RETRYABLE_CODES,
    BackendKind,
    ErrorCode,
    ErrorInfo,
    ProtocolViolation,
    SimulationSpec,
    WorkerProfile,
)

DEV_FRANKA = "profile_dev_franka_physx"
MOCK_FRANKA = "profile_mock_franka"
SO101 = "profile_so101_physx"

# Interpreter that runs the Isaac workers.  The Gateway passes its own choice
# to SubprocessEndpoint; this value only documents the profile.
PYTHON_EXECUTABLE = os.environ.get("DSH_ISAAC_PYTHON") or sys.executable

PROFILES: dict[str, WorkerProfile] = {
    DEV_FRANKA: WorkerProfile(
        profile_id=DEV_FRANKA,
        backend_id="backend_isaac_physx",
        backend_kind=BackendKind.ISAACLAB_PHYSX,
        robot_id="franka",
        scene_id="table_top",
        python_executable=PYTHON_EXECUTABLE,
        entry_module="robotics_backends.isaaclab_physx.entry",
        allowed_imports=["isaaclab", "isaacsim"],
        max_envs=1,
        description="Franka development profile for the T02-T04 software chain.",
    ),
    MOCK_FRANKA: WorkerProfile(
        profile_id=MOCK_FRANKA,
        backend_id="backend_mock",
        backend_kind=BackendKind.MOCK,
        robot_id="franka",
        scene_id="table_top",
        python_executable=PYTHON_EXECUTABLE,
        entry_module="robotics_backends.fake",
        allowed_imports=[],
        max_envs=1,
        description="Protocol and concurrency tests only; never counted as simulation verification.",
    ),
    SO101: WorkerProfile(
        profile_id=SO101,
        backend_id="backend_isaac_physx",
        backend_kind=BackendKind.ISAACLAB_PHYSX,
        robot_id="so101",
        scene_id="table_pick_place",
        python_executable=PYTHON_EXECUTABLE,
        entry_module="robotics_backends.isaaclab_physx.so101_server_entry",
        allowed_imports=["isaaclab", "isaacsim"],
        max_envs=1,
        description="SO-ARM101 pick/place worker; T06 physical verification, T07 agent recovery.",
    ),
}


def violation(code: ErrorCode, message: str, **details: object) -> ProtocolViolation:
    return ProtocolViolation(
        ErrorInfo(
            code=code,
            message=message,
            retryable=code in RETRYABLE_CODES,
            details=dict(details),
            occurred_at=datetime.now(timezone.utc),
        )
    )


def get_profile(profile_id: str) -> WorkerProfile:
    profile = PROFILES.get(profile_id)
    if profile is None:
        raise violation(
            ErrorCode.UNSUPPORTED_CAPABILITY,
            f"unknown worker profile: {profile_id}",
            allowed=sorted(PROFILES),
        )
    return profile


def validate_spec(spec: SimulationSpec) -> WorkerProfile:
    profile = get_profile(spec.profile_id)
    if spec.backend_kind is not profile.backend_kind:
        raise violation(
            ErrorCode.UNSUPPORTED_BACKEND,
            f"profile {profile.profile_id} runs {profile.backend_kind.value}, not {spec.backend_kind.value}",
        )
    if spec.robot_id != profile.robot_id:
        raise violation(
            ErrorCode.UNSUPPORTED_CAPABILITY,
            f"profile {profile.profile_id} is fixed to robot {profile.robot_id}",
        )
    if spec.scene_id != profile.scene_id:
        raise violation(
            ErrorCode.UNSUPPORTED_CAPABILITY,
            f"profile {profile.profile_id} is fixed to scene {profile.scene_id}",
        )
    if spec.num_envs > profile.max_envs:
        raise violation(
            ErrorCode.UNSUPPORTED_CAPABILITY,
            f"profile {profile.profile_id} allows at most {profile.max_envs} environments",
        )
    return profile


__all__ = ["PROFILES", "DEV_FRANKA", "MOCK_FRANKA", "SO101", "get_profile", "validate_spec", "violation"]
