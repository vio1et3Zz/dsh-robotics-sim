"""Skill catalog entries and one invocation request."""

from __future__ import annotations

from typing import Any

from pydantic import Field, model_validator

from .base import Frame, Message, ObjectId, RobotId, SkillName, StrictModel


class SkillSpec(StrictModel):
    name: SkillName
    robot_id: RobotId
    description: str = Field(min_length=1, max_length=512)
    parameters_schema: dict[str, Any] = Field(default_factory=dict)
    timeout_s: float = Field(gt=0.0, le=600.0)
    preconditions: list[str] = Field(default_factory=list, max_length=32)

    @model_validator(mode="after")
    def _parameters_are_an_object_schema(self) -> "SkillSpec":
        schema_type = self.parameters_schema.get("type")
        if schema_type not in (None, "object"):
            raise ValueError("parameters_schema must describe an object")
        return self


class SkillInvocation(Message):
    skill: SkillName
    parameters: dict[str, Any] = Field(default_factory=dict)
    target_object_id: ObjectId | None = None
    frame: Frame = Frame.WORLD
    timeout_s: float = Field(gt=0.0, le=600.0)
    expected_duration_s: float = Field(gt=0.0, le=600.0)

    @model_validator(mode="after")
    def _timeout_covers_expected_duration(self) -> "SkillInvocation":
        if self.expected_duration_s > self.timeout_s:
            raise ValueError("expected_duration_s must not exceed timeout_s")
        return self


__all__ = ["SkillSpec", "SkillInvocation"]
