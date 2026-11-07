"""In-memory fake backend for protocol, idempotency and cancellation tests.

It is deliberately honest: ``backend_kind=mock``, ``verified=False`` and every
event/evidence string says it is not a simulation.  Nothing produced here may
be counted as a real Isaac run.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import datetime, timezone

from robotics_protocol import (
    BackendCapabilities,
    BackendKind,
    CapabilityCombination,
    CapabilityStatus,
    ErrorCode,
    Frame,
    GripperState,
    JobEvent,
    JobEventType,
    JobId,
    JobStatus,
    ObjectState,
    ObservationSource,
    PhysicsInfo,
    Pose,
    RendererInfo,
    RobotState,
    SimulationSpec,
    SkillInvocation,
    SkillName,
    SkillPhase,
    SkillStatus,
    VisualizerInfo,
    WorldState,
)

from .base import SimulationBackend
from .profiles import violation, validate_spec

SUPPORTED_SKILLS = {"observe_scene", "home", "pick", "place"}
ROBOT_DOF = {"franka": 7, "so101": 6}


@dataclass
class _FakeJob:
    job_id: str
    skill: str
    total_steps: int
    remaining_steps: int
    status: JobStatus = JobStatus.RUNNING


class FakeBackend(SimulationBackend):
    backend_id = "backend_mock"
    backend_kind = BackendKind.MOCK

    def __init__(self) -> None:
        self._closed = False
        self._started = False
        self._spec: SimulationSpec | None = None
        self._simulation_id: str | None = None
        self._episode_id: str | None = None
        self._state: WorldState | None = None
        self._state_version = 0
        self._event_seq = 0
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._events: list[JobEvent] = []
        self._job: _FakeJob | None = None
        self._robot_id = "franka"

    # -- lifecycle -----------------------------------------------------

    def describe_capabilities(self) -> BackendCapabilities:
        now = datetime.now(timezone.utc)
        return BackendCapabilities(
            backend_id=self.backend_id,
            backend_kind=self.backend_kind,
            generated_at=now,
            updated_at=now,
            physics=PhysicsInfo(engine="none", version="0", deterministic=True),
            renderer=RendererInfo(enabled=False),
            visualizer=VisualizerInfo(enabled=False),
            installed=True,
            verified=False,
            evidence=["fake backend; protocol tests only; never a simulation verification"],
            combinations=[
                CapabilityCombination(
                    combination_id="combo_mock_franka_pick_place",
                    robot_id="franka",
                    task_id="pick_place",
                    scene_id="table_top",
                    backend_kind=BackendKind.MOCK,
                    status=CapabilityStatus.INSTALLED,
                    evidence=["fake backend; protocol tests only"],
                    checked_at=now,
                )
            ],
        )

    def start(self, spec: SimulationSpec) -> WorldState:
        self._require_open()
        if self._started:
            raise violation(ErrorCode.INVALID_REQUEST, "fake backend already started")
        profile = validate_spec(spec)
        if profile.backend_kind is not BackendKind.MOCK:
            raise violation(
                ErrorCode.UNSUPPORTED_BACKEND,
                "fake backend cannot run a non-mock profile",
                profile=profile.profile_id,
            )
        self._spec = spec
        self._robot_id = spec.robot_id
        self._simulation_id = f"sim_{uuid.uuid4().hex}"
        self._episode_id = f"ep_{uuid.uuid4().hex}"
        self._state_version = 0
        self._event_seq = 0
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._job = None
        self._started = True
        self._state = self._build_state()
        return self.observe()

    def observe(self) -> WorldState:
        self._require_open()
        self._require_started()
        assert self._state is not None
        return self._state.model_copy(
            update={
                "wall_time": datetime.now(timezone.utc),
                "event_seq": self._event_seq,
            }
        )

    def reset(self, seed: int | None = None) -> WorldState:
        self._require_open()
        self._require_started()
        if self._job is not None and self._job.status is JobStatus.RUNNING:
            self.cancel_skill(self._job.job_id)
        self._episode_id = f"ep_{uuid.uuid4().hex}"
        self._state_version += 1
        self._sim_step = 0
        self._sim_time_s = 0.0
        self._state = self._build_state()
        return self.observe()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._started = False
        self._job = None
        self._state = None

    # -- skills --------------------------------------------------------

    def submit_skill(self, invocation: SkillInvocation) -> JobId:
        self._require_open()
        self._require_started()
        if self._job is not None and self._job.status is JobStatus.RUNNING:
            raise violation(ErrorCode.CONTROL_BUSY, "a skill is already running")
        if invocation.skill not in SUPPORTED_SKILLS:
            raise violation(
                ErrorCode.UNSUPPORTED_CAPABILITY,
                f"unsupported skill: {invocation.skill}",
                supported=sorted(SUPPORTED_SKILLS),
            )
        object_ids = {obj.object_id for obj in (self._state.objects if self._state else [])}
        if invocation.target_object_id is not None and invocation.target_object_id not in object_ids:
            raise violation(
                ErrorCode.INVALID_REQUEST,
                f"unknown target object: {invocation.target_object_id}",
                objects=sorted(object_ids),
            )
        job_id = f"job_{uuid.uuid4().hex}"
        steps = 1 if invocation.skill in {"observe_scene", "home"} else 3
        self._job = _FakeJob(job_id=job_id, skill=invocation.skill, total_steps=steps, remaining_steps=steps)
        self._emit(JobEventType.CREATED, JobStatus.PENDING, skill=invocation.skill)
        self._emit(JobEventType.STARTED, JobStatus.RUNNING, skill=invocation.skill)
        return job_id

    def cancel_skill(self, job_id: str) -> bool:
        self._require_open()
        if self._job is None or self._job.job_id != job_id:
            return False
        if self._job.status is not JobStatus.RUNNING:
            return False
        self._emit(JobEventType.CANCEL_REQUESTED, JobStatus.CANCELING, skill=self._job.skill)
        self._job.status = JobStatus.CANCELED
        self._emit(JobEventType.CANCELED, JobStatus.CANCELED, skill=self._job.skill, message="worker stop acknowledged")
        self._job = None
        return True

    def tick(self) -> None:
        """Advance the active fake skill by one step (tests only)."""

        self._require_open()
        if self._job is None or self._job.status is not JobStatus.RUNNING:
            return
        self._job.remaining_steps -= 1
        progress = 1.0 - (self._job.remaining_steps / self._job.total_steps)
        if self._job.remaining_steps > 0:
            self._emit(JobEventType.PROGRESS, JobStatus.RUNNING, skill=self._job.skill, progress=progress)
            return
        self._finish_job(self._job, progress=1.0)

    def _finish_job(self, job: _FakeJob, progress: float) -> None:
        assert self._state is not None
        objects = []
        for obj in self._state.objects:
            update = {}
            if job.skill == "pick" and obj.object_id == "red_block":
                update = {"grasped_by": self._robot_id}
            if job.skill == "place" and obj.object_id == "red_block":
                update = {
                    "grasped_by": None,
                    "pose": obj.pose.model_copy(update={"position": (0.6, 0.3, 0.06)}),
                }
            objects.append(obj.model_copy(update=update))
        self._state_version += 1
        self._sim_step += 1
        self._sim_time_s += 1.0
        self._state = self._state.model_copy(
            update={
                "state_version": self._state_version,
                "sim_step": self._sim_step,
                "sim_time_s": self._sim_time_s,
                "objects": objects,
                "skills": [
                    SkillStatus(
                        skill=job.skill,
                        phase=SkillPhase.SUCCEEDED,
                        progress=progress,
                        message="fake skill completed",
                    )
                ],
            }
        )
        self._emit(JobEventType.SUCCEEDED, JobStatus.SUCCEEDED, skill=job.skill, progress=progress)
        self._job = None

    def drain_events(self) -> list[JobEvent]:
        events = self._events
        self._events = []
        return events

    # -- helpers -------------------------------------------------------

    def _build_state(self) -> WorldState:
        dof = ROBOT_DOF.get(self._robot_id, 7)
        robot = RobotState(
            robot_id=self._robot_id,
            dof=dof,
            joint_positions=[0.0] * dof,
            joint_velocities=[0.0] * dof,
            ee_pose=Pose(
                frame=Frame.WORLD,
                position=(0.4, 0.0, 0.25),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
            gripper=GripperState(width_m=0.04, closed=False),
        )
        objects = [
            ObjectState(
                object_id="red_block",
                pose=Pose(
                    frame=Frame.WORLD,
                    position=(0.5, 0.0, 0.02),
                    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                ),
                size_m=(0.04, 0.04, 0.04),
            ),
            ObjectState(
                object_id="target_box",
                pose=Pose(
                    frame=Frame.WORLD,
                    position=(0.6, 0.3, 0.04),
                    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                ),
                size_m=(0.12, 0.12, 0.08),
            ),
        ]
        return WorldState(
            simulation_id=self._simulation_id,
            episode_id=self._episode_id,
            env_id=0,
            state_version=self._state_version,
            event_seq=self._event_seq,
            sim_step=self._sim_step,
            sim_time_s=self._sim_time_s,
            wall_time=datetime.now(timezone.utc),
            source=ObservationSource.SIM_GROUND_TRUTH,
            frame=Frame.WORLD,
            robots=[robot],
            objects=objects,
            skills=[],
        )

    def _emit(
        self,
        event_type: JobEventType,
        status: JobStatus,
        *,
        skill: str | None = None,
        message: str | None = None,
        progress: float | None = None,
    ) -> None:
        assert self._job is not None
        event = JobEvent(
            job_id=self._job.job_id,
            event_seq=self._event_seq,
            at=datetime.now(timezone.utc),
            type=event_type,
            status=status,
            skill=skill,
            message=message,
            progress=progress,
            evidence=["fake backend"],
        )
        self._event_seq += 1
        self._events.append(event)

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("fake backend is closed")

    def _require_started(self) -> None:
        if not self._started:
            raise violation(ErrorCode.INVALID_REQUEST, "simulation has not been started")


__all__ = ["FakeBackend", "SUPPORTED_SKILLS"]
