"""Structured protocol errors."""

from __future__ import annotations

from enum import Enum
from typing import Any

from pydantic import Field, model_validator

from .base import Message, StrictModel, Timestamp


class ErrorCode(str, Enum):
    INVALID_REQUEST = "INVALID_REQUEST"
    UNSUPPORTED_BACKEND = "UNSUPPORTED_BACKEND"
    UNSUPPORTED_CAPABILITY = "UNSUPPORTED_CAPABILITY"
    ASSET_NOT_READY = "ASSET_NOT_READY"
    UNREACHABLE = "UNREACHABLE"
    GRASP_FAILED = "GRASP_FAILED"
    PLACEMENT_FAILED = "PLACEMENT_FAILED"
    STALE_WORLD_STATE = "STALE_WORLD_STATE"
    CONTROL_BUSY = "CONTROL_BUSY"
    LIMIT_VIOLATION = "LIMIT_VIOLATION"
    WORKER_LOST = "WORKER_LOST"
    TIMEOUT = "TIMEOUT"
    MOCK_NOT_VERIFIED = "MOCK_NOT_VERIFIED"
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"


# Codes a caller may safely retry without changing the request.
RETRYABLE_CODES = {
    ErrorCode.STALE_WORLD_STATE,
    ErrorCode.CONTROL_BUSY,
    ErrorCode.WORKER_LOST,
    ErrorCode.TIMEOUT,
}


class ErrorInfo(StrictModel):
    code: ErrorCode
    message: str = Field(min_length=1, max_length=2000)
    retryable: bool = False
    details: dict[str, Any] = Field(default_factory=dict)
    evidence: list[str] = Field(default_factory=list, max_length=64)
    occurred_at: Timestamp

    @model_validator(mode="after")
    def _retryability_matches_code(self) -> "ErrorInfo":
        if self.retryable != (self.code in RETRYABLE_CODES):
            raise ValueError(f"retryable must be {self.code in RETRYABLE_CODES} for {self.code.value}")
        return self


class ProtocolViolation(ValueError):
    """Raised when a request is rejected before it reaches a Worker."""

    def __init__(self, info: ErrorInfo):
        super().__init__(f"{info.code.value}: {info.message}")
        self.info = info


class ErrorMessage(Message):
    error: ErrorInfo


__all__ = ["ErrorCode", "RETRYABLE_CODES", "ErrorInfo", "ProtocolViolation", "ErrorMessage"]
