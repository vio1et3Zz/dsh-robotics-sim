"""Durable job events shared by the Gateway, Worker and DSH tools."""

from __future__ import annotations

from enum import Enum
from typing import Any

from pydantic import Field, model_validator

from .base import JobId, Message, SkillName, Timestamp


class JobStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    CANCELING = "canceling"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"


TERMINAL_STATUSES = {JobStatus.SUCCEEDED, JobStatus.FAILED, JobStatus.CANCELED}


class JobEventType(str, Enum):
    CREATED = "created"
    STARTED = "started"
    PROGRESS = "progress"
    SKILL_STARTED = "skill_started"
    SKILL_FINISHED = "skill_finished"
    VERIFYING = "verifying"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCEL_REQUESTED = "cancel_requested"
    CANCELED = "canceled"
    LOG = "log"
    FRAME = "frame"


_EXPECTED_STATUS = {
    JobEventType.CREATED: JobStatus.PENDING,
    JobEventType.STARTED: JobStatus.RUNNING,
    JobEventType.PROGRESS: JobStatus.RUNNING,
    JobEventType.SKILL_STARTED: JobStatus.RUNNING,
    JobEventType.SKILL_FINISHED: JobStatus.RUNNING,
    JobEventType.VERIFYING: JobStatus.RUNNING,
    JobEventType.SUCCEEDED: JobStatus.SUCCEEDED,
    JobEventType.FAILED: JobStatus.FAILED,
    JobEventType.CANCEL_REQUESTED: JobStatus.CANCELING,
    JobEventType.CANCELED: JobStatus.CANCELED,
}


class JobEvent(Message):
    job_id: JobId
    event_seq: int = Field(ge=0)
    at: Timestamp
    type: JobEventType
    status: JobStatus
    skill: SkillName | None = None
    message: str | None = Field(default=None, max_length=2000)
    progress: float | None = Field(default=None, ge=0.0, le=1.0)
    data: dict[str, Any] = Field(default_factory=dict)
    evidence: list[str] = Field(default_factory=list, max_length=64)

    @model_validator(mode="after")
    def _event_matches_status(self) -> "JobEvent":
        expected = _EXPECTED_STATUS.get(self.type)
        if expected is not None and self.status is not expected:
            raise ValueError(f"{self.type.value} event requires status {expected.value}")
        if self.type in (JobEventType.SKILL_STARTED, JobEventType.SKILL_FINISHED, JobEventType.VERIFYING):
            if self.skill is None:
                raise ValueError(f"{self.type.value} requires a skill")
        if self.type is JobEventType.FAILED and "error_code" not in self.data:
            raise ValueError("a failed event must carry data.error_code")
        return self


__all__ = ["JobStatus", "TERMINAL_STATUSES", "JobEventType", "JobEvent"]
