"""Observable world state returned by a Worker."""

from __future__ import annotations

from enum import Enum
from typing import Literal

from pydantic import Field, model_validator

from .base import (
    EpisodeId,
    Frame,
    ObjectId,
    Pose,
    RobotId,
    SimulationId,
    SkillName,
    StrictModel,
    Timestamp,
    Vec3,
)


class ObservationSource(str, Enum):
    SIM_GROUND_TRUTH = "sim_ground_truth"
    VISION = "vision"
    ESTIMATOR = "estimator"
    MIXED = "mixed"


class GripperState(StrictModel):
    width_m: float = Field(ge=0.0, le=1.0)
    closed: bool


class RobotState(StrictModel):
    robot_id: RobotId
    dof: int = Field(ge=1, le=64)
    joint_positions: list[float] = Field(min_length=1, max_length=64)
    joint_velocities: list[float] = Field(min_length=1, max_length=64)
    ee_pose: Pose
    gripper: GripperState

    @model_validator(mode="after")
    def _joint_arrays_match_dof(self) -> "RobotState":
        if len(self.joint_positions) != self.dof:
            raise ValueError(f"joint_positions has {len(self.joint_positions)} values, expected {self.dof}")
        if len(self.joint_velocities) != self.dof:
            raise ValueError(f"joint_velocities has {len(self.joint_velocities)} values, expected {self.dof}")
        return self


class ObjectState(StrictModel):
    object_id: ObjectId
    pose: Pose
    size_m: Vec3
    grasped_by: RobotId | None = None
    visible: bool = True

    @model_validator(mode="after")
    def _size_is_positive(self) -> "ObjectState":
        if any(extent <= 0.0 for extent in self.size_m):
            raise ValueError("object size_m extents must be positive")
        return self


class SkillPhase(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    VERIFYING = "verifying"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


class SkillStatus(StrictModel):
    skill: SkillName
    phase: SkillPhase = SkillPhase.IDLE
    progress: float = Field(ge=0.0, le=1.0)
    message: str | None = Field(default=None, max_length=512)


class WorldState(StrictModel):
    simulation_id: SimulationId
    episode_id: EpisodeId
    env_id: int = Field(ge=0, le=63)
    state_version: int = Field(ge=0)
    event_seq: int = Field(ge=0)
    sim_step: int = Field(ge=0)
    sim_time_s: float = Field(ge=0.0)
    wall_time: Timestamp
    source: ObservationSource
    frame: Frame = Frame.WORLD
    robots: list[RobotState] = Field(min_length=1, max_length=8)
    objects: list[ObjectState] = Field(default_factory=list, max_length=256)
    skills: list[SkillStatus] = Field(default_factory=list, max_length=64)

    @model_validator(mode="after")
    def _state_is_consistent(self) -> "WorldState":
        robot_ids = [robot.robot_id for robot in self.robots]
        if len(robot_ids) != len(set(robot_ids)):
            raise ValueError("robot ids must be unique")
        object_ids = [obj.object_id for obj in self.objects]
        if len(object_ids) != len(set(object_ids)):
            raise ValueError("object ids must be unique")
        for obj in self.objects:
            if obj.grasped_by is not None and obj.grasped_by not in robot_ids:
                raise ValueError(f"object {obj.object_id} is grasped by an undeclared robot")
        skills = [status.skill for status in self.skills]
        if len(skills) != len(set(skills)):
            raise ValueError("skill statuses must be unique")
        return self


__all__ = [
    "ObservationSource",
    "GripperState",
    "RobotState",
    "ObjectState",
    "SkillPhase",
    "SkillStatus",
    "WorldState",
]
