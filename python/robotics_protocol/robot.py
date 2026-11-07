"""Robot embodiment specification and joint calibration."""

from __future__ import annotations

from enum import Enum
from typing import Literal

from pydantic import Field, model_validator

from .base import RobotId, SkillName, StrictModel, Vec3


class JointType(str, Enum):
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    FIXED = "fixed"


class ControlMode(str, Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"


class JointSpec(StrictModel):
    name: str = Field(min_length=1, max_length=96)
    logical_name: str = Field(pattern=r"^[a-z][a-z0-9_]{1,48}$")
    type: JointType
    unit: Literal["rad", "m"]
    lower: float
    upper: float
    home: float
    velocity_limit: float = Field(gt=0)

    @model_validator(mode="after")
    def _limits_are_sane(self) -> "JointSpec":
        if not self.lower <= self.home <= self.upper:
            raise ValueError(f"joint {self.name}: home must lie within [lower, upper]")
        if self.lower >= self.upper:
            raise ValueError(f"joint {self.name}: lower must be < upper")
        if self.type is JointType.REVOLUTE and self.unit != "rad":
            raise ValueError(f"joint {self.name}: revolute joints use radians")
        if self.type is JointType.PRISMATIC and self.unit != "m":
            raise ValueError(f"joint {self.name}: prismatic joints use metres")
        return self


class EndEffectorSpec(StrictModel):
    name: str = Field(min_length=1, max_length=64)
    parent_joint: str = Field(min_length=1, max_length=96)
    offset_m: Vec3 = (0.0, 0.0, 0.0)


class GripperSpec(StrictModel):
    kind: Literal["parallel", "single", "none"] = "none"
    joint_names: list[str] = Field(default_factory=list, max_length=8)
    open_value: float = 0.0
    close_value: float = 0.0
    max_width_m: float = Field(default=0.0, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def _gripper_is_consistent(self) -> "GripperSpec":
        if self.kind != "none":
            if not self.joint_names:
                raise ValueError("a gripper requires joint_names")
            if self.open_value == self.close_value:
                raise ValueError("open_value and close_value must differ")
            if self.max_width_m <= 0:
                raise ValueError("a gripper requires max_width_m > 0")
        return self


class RobotAsset(StrictModel):
    usd_path: str = Field(min_length=1, max_length=2048)
    sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    license: str = Field(min_length=1, max_length=128)
    source: str = Field(min_length=1, max_length=1024)


class RobotSpec(StrictModel):
    robot_id: RobotId
    model: str = Field(min_length=1, max_length=96)
    dof: int = Field(ge=1, le=64)
    joints: list[JointSpec] = Field(min_length=1, max_length=64)
    end_effector: EndEffectorSpec
    gripper: GripperSpec
    control_modes: list[ControlMode] = Field(min_length=1, max_length=3)
    skills: list[SkillName] = Field(min_length=1, max_length=64)
    asset: RobotAsset

    @model_validator(mode="after")
    def _robot_is_consistent(self) -> "RobotSpec":
        if self.dof != len(self.joints):
            raise ValueError(f"dof={self.dof} does not match {len(self.joints)} joints")
        names = [joint.name for joint in self.joints]
        if len(names) != len(set(names)):
            raise ValueError("joint names must be unique")
        logical = [joint.logical_name for joint in self.joints]
        if len(logical) != len(set(logical)):
            raise ValueError("logical joint names must be unique")
        if self.end_effector.parent_joint not in names:
            raise ValueError("end_effector.parent_joint must reference a declared joint")
        missing = [name for name in self.gripper.joint_names if name not in names]
        if missing:
            raise ValueError(f"gripper joints not declared: {missing}")
        if len(self.skills) != len(set(self.skills)):
            raise ValueError("skills must be unique")
        return self


__all__ = [
    "JointType",
    "ControlMode",
    "JointSpec",
    "EndEffectorSpec",
    "GripperSpec",
    "RobotAsset",
    "RobotSpec",
]
