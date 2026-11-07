"""Task descriptions, agent-generated plans and plan hashing."""

from __future__ import annotations

import hashlib
import json
from enum import Enum
from typing import Any

from pydantic import Field, model_validator

from .base import (
    EpisodeId,
    Message,
    ObjectId,
    PlanId,
    RobotId,
    SceneId,
    Sha256,
    SkillName,
    StepId,
    StrictModel,
    TaskId,
    Timestamp,
    Vec3,
)


class Region3D(StrictModel):
    min: Vec3
    max: Vec3

    @model_validator(mode="after")
    def _min_below_max(self) -> "Region3D":
        for axis, low, high in zip(("x", "y", "z"), self.min, self.max):
            if not low < high:
                raise ValueError(f"region min must be < max on {axis} axis")
        return self


class PredicateKind(str, Enum):
    OBJECT_IN_REGION = "object_in_region"
    OBJECT_HEIGHT = "object_height"
    ROBOT_RELEASED = "robot_released"
    GRIPPER_WIDTH = "gripper_width"
    OBJECT_POSE = "object_pose"
    CUSTOM = "custom"


class SuccessPredicate(StrictModel):
    predicate_id: str = Field(pattern=r"^pred_[a-z0-9_]{1,40}$")
    kind: PredicateKind
    object_id: ObjectId | None = None
    region: Region3D | None = None
    params: dict[str, Any] = Field(default_factory=dict)
    tolerance_m: float = Field(default=0.005, ge=0.0, le=0.5)

    @model_validator(mode="after")
    def _kind_has_required_inputs(self) -> "SuccessPredicate":
        if self.kind is PredicateKind.OBJECT_IN_REGION:
            if self.object_id is None or self.region is None:
                raise ValueError("object_in_region requires object_id and region")
        if self.kind in (PredicateKind.OBJECT_HEIGHT, PredicateKind.OBJECT_POSE) and self.object_id is None:
            raise ValueError(f"{self.kind.value} requires object_id")
        if self.kind is PredicateKind.CUSTOM and not self.params:
            raise ValueError("custom predicates require params")
        return self


class TaskBudget(StrictModel):
    sim_time_s: float = Field(gt=0.0, le=86_400.0)
    wall_time_s: float = Field(gt=0.0, le=86_400.0)
    max_skill_calls: int = Field(ge=1, le=1000)
    max_retries: int = Field(ge=0, le=100)


class TaskSpec(Message):
    task_id: TaskId
    goal: str = Field(min_length=1, max_length=2000)
    robot_id: RobotId
    scene_id: SceneId
    success_predicates: list[SuccessPredicate] = Field(min_length=1, max_length=32)
    budgets: TaskBudget
    constraints: list[str] = Field(default_factory=list, max_length=32)

    @model_validator(mode="after")
    def _predicates_are_unique(self) -> "TaskSpec":
        ids = [predicate.predicate_id for predicate in self.success_predicates]
        if len(ids) != len(set(ids)):
            raise ValueError("predicate ids must be unique")
        return self


class PlanStep(StrictModel):
    step_id: StepId
    skill: SkillName
    parameters: dict[str, Any] = Field(default_factory=dict)
    depends_on: list[StepId] = Field(default_factory=list, max_length=32)
    expected_duration_s: float = Field(gt=0.0, le=600.0)

    @model_validator(mode="after")
    def _no_self_dependency(self) -> "PlanStep":
        if self.step_id in self.depends_on:
            raise ValueError(f"step {self.step_id} depends on itself")
        return self


class TaskPlan(Message):
    plan_id: PlanId
    task_id: TaskId
    episode_id: EpisodeId
    base_state_version: int = Field(ge=0)
    created_at: Timestamp
    steps: list[PlanStep] = Field(min_length=1, max_length=64)
    plan_hash: Sha256 | None = None
    budget: TaskBudget

    @model_validator(mode="after")
    def _plan_is_a_dag(self) -> "TaskPlan":
        ids = [step.step_id for step in self.steps]
        if len(ids) != len(set(ids)):
            raise ValueError("step ids must be unique")
        known = set(ids)
        for step in self.steps:
            unknown = [dep for dep in step.depends_on if dep not in known]
            if unknown:
                raise ValueError(f"step {step.step_id} depends on unknown steps: {unknown}")
        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(node: str) -> None:
            if node in visiting:
                raise ValueError(f"dependency cycle at {node}")
            if node in visited:
                return
            visiting.add(node)
            for dependency in by_id[node].depends_on:
                visit(dependency)
            visiting.remove(node)
            visited.add(node)

        by_id = {step.step_id: step for step in self.steps}
        for step in self.steps:
            visit(step.step_id)
        if self.plan_hash is not None and self.plan_hash != compute_plan_hash(
            task_id=self.task_id,
            episode_id=self.episode_id,
            base_state_version=self.base_state_version,
            steps=self.steps,
        ):
            raise ValueError("plan_hash does not match the plan content")
        return self


def compute_plan_hash(
    *,
    task_id: str,
    episode_id: str,
    base_state_version: int,
    steps: list[PlanStep],
) -> str:
    """Canonical hash the Gateway returns and later re-checks before execution."""

    payload = {
        "task_id": task_id,
        "episode_id": episode_id,
        "base_state_version": base_state_version,
        "steps": [step.model_dump(mode="json") for step in steps],
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


__all__ = [
    "Region3D",
    "PredicateKind",
    "SuccessPredicate",
    "TaskBudget",
    "TaskSpec",
    "PlanStep",
    "TaskPlan",
    "compute_plan_hash",
]
