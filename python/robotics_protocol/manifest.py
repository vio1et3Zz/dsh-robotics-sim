"""Runtime manifest: the exact software identity behind one Worker run."""

from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from .base import RuntimeId, Sha256, StrictModel, Timestamp


class PythonRuntime(StrictModel):
    version: str = Field(min_length=3, max_length=32)
    executable: str = Field(min_length=1, max_length=1024)
    platform: str = Field(min_length=1, max_length=256)


class SourceRevision(StrictModel):
    repo: str = Field(min_length=1, max_length=1024)
    commit: str = Field(pattern=r"^[0-9a-f]{40}$")
    describe: str = Field(min_length=1, max_length=128)
    dirty: bool
    diff_sha256: Sha256 | None = None

    @model_validator(mode="after")
    def _dirty_has_diff_hash(self) -> "SourceRevision":
        if self.dirty and self.diff_sha256 is None:
            raise ValueError("a dirty source revision requires diff_sha256")
        return self


class GpuInfo(StrictModel):
    name: str = Field(min_length=1, max_length=128)
    memory_mib: int = Field(ge=1, le=1_048_576)
    driver: str = Field(min_length=1, max_length=64)


class WorkerRuntime(StrictModel):
    cwd: str = Field(min_length=1, max_length=1024)
    log_dir: str = Field(min_length=1, max_length=1024)
    num_envs: int = Field(ge=1, le=64)
    device: str = Field(pattern=r"^(cpu|cuda(:\d+)?)$")


class RuntimeManifest(StrictModel):
    runtime_id: RuntimeId
    schema_version: Literal["0.1"] = "0.1"
    created_at: Timestamp
    protocol_version: Literal["0.1"] = "0.1"
    adapter_version: str = Field(min_length=1, max_length=32)
    python: PythonRuntime
    packages: dict[str, str] = Field(default_factory=dict)
    source: SourceRevision
    config_sha256: Sha256
    assets: dict[str, Sha256] = Field(default_factory=dict)
    gpu: GpuInfo
    worker: WorkerRuntime
    notes: list[str] = Field(default_factory=list, max_length=32)

    @model_validator(mode="after")
    def _package_versions_are_pinned(self) -> "RuntimeManifest":
        for name, version in self.packages.items():
            if not name or not version:
                raise ValueError("package names and versions must be non-empty")
        return self


__all__ = [
    "PythonRuntime",
    "SourceRevision",
    "GpuInfo",
    "WorkerRuntime",
    "RuntimeManifest",
]
