"""Shared primitives for the DSH Robotics protocol (v0.1).

The protocol fixes units and conventions so the Gateway, Worker and DSH tools
never have to guess:

* lengths in metres, angles in radians, time in seconds;
* quaternions are ``xyzw`` and must be normalized;
* every pose names its frame explicitly;
* every model forbids unknown fields and non-finite numbers.

IDs are branded by prefix and validated at the schema boundary.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Annotated, Any, Literal, TypeAlias

from pydantic import (
    AfterValidator,
    AwareDatetime,
    BaseModel,
    ConfigDict,
    Field,
    model_validator,
)

SCHEMA_VERSION = "0.1"
PROTOCOL_VERSION = "0.1"


class StrictModel(BaseModel):
    """Base model: strict fields, finite floats, stripped strings."""

    model_config = ConfigDict(
        extra="forbid",
        validate_assignment=True,
        allow_inf_nan=False,
        str_strip_whitespace=True,
    )


Timestamp: TypeAlias = AwareDatetime


def _id_type(prefix: str) -> Any:
    return Annotated[str, Field(pattern=rf"^{prefix}_[0-9a-f-]{{8,36}}$")]


SimulationId = _id_type("sim")
EpisodeId = _id_type("ep")
JobId = _id_type("job")
TaskId = _id_type("task")
PlanId = _id_type("plan")
RequestId = _id_type("req")
TraceId = _id_type("trace")

RuntimeId = Annotated[str, Field(pattern=r"^runtime_[a-z0-9_-]{3,48}$")]
BackendId = Annotated[str, Field(pattern=r"^backend_[a-z0-9_-]{3,48}$")]

ObjectId = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]{1,31}$")]
RobotId = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]{1,31}$")]
SceneId = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]{1,63}$")]
ProfileId = Annotated[str, Field(pattern=r"^profile_[a-z0-9_]{3,48}$")]
SkillName = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]{2,40}$")]
StepId = Annotated[str, Field(pattern=r"^step_[a-z0-9_]{1,40}$")]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]


def _check_quaternion(value: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in value))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"quaternion must be normalized (xyzw), got norm {norm:.6f}")
    return value


Vec3 = Annotated[
    tuple[float, float, float],
    Field(description="3-D vector in metres, frame declared by the containing pose."),
]
QuatXYZW = Annotated[
    tuple[float, float, float, float],
    AfterValidator(_check_quaternion),
    Field(description="Normalized quaternion in xyzw order."),
]


class Frame(str, Enum):
    WORLD = "world"
    ROBOT_BASE = "robot_base"
    TOOL0 = "tool0"
    CAMERA = "camera"


class Pose(StrictModel):
    frame: Frame
    frame_id: str | None = Field(default=None, min_length=1, max_length=64)
    position: Vec3
    orientation_xyzw: QuatXYZW


class Message(StrictModel):
    """Common envelope for protocol messages crossing the HTTP/WS boundary."""

    schema_version: Literal["0.1"] = "0.1"
    request_id: RequestId | None = None
    trace_id: TraceId | None = None


class Empty(StrictModel):
    """Explicit empty payload; used where a schema still needs an object."""


def _ensure_unique(values: list[str], what: str) -> None:
    if len(values) != len(set(values)):
        raise ValueError(f"{what} must be unique")


def _check_region(region: Any) -> None:
    for axis, low, high in zip(("x", "y", "z"), region.min, region.max):
        if not low < high:
            raise ValueError(f"region min must be < max on {axis} axis")


__all__ = [
    "SCHEMA_VERSION",
    "PROTOCOL_VERSION",
    "StrictModel",
    "Timestamp",
    "SimulationId",
    "EpisodeId",
    "JobId",
    "TaskId",
    "PlanId",
    "RuntimeId",
    "BackendId",
    "RequestId",
    "TraceId",
    "ObjectId",
    "RobotId",
    "SceneId",
    "ProfileId",
    "SkillName",
    "StepId",
    "Sha256",
    "Vec3",
    "QuatXYZW",
    "Frame",
    "Pose",
    "Message",
    "Empty",
]
