"""Gateway orchestration: one simulation, durable jobs, confirmed cancellation."""

from __future__ import annotations

import math
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from typing import Any

from pydantic import ValidationError
from robotics_protocol import (
    EpisodeId,
    JobEvent,
    JobEventType,
    JobStatus,
    ProtocolViolation,
    SimulationId,
    SimulationSpec,
    SkillInvocation,
    TaskPlan,
    WorldState,
    compute_plan_hash,
)

from .endpoint import WorkerEndpoint
from .store import Store

TERMINAL = {JobStatus.SUCCEEDED.value, JobStatus.FAILED.value, JobStatus.CANCELED.value}
LIVE = {JobStatus.PENDING.value, JobStatus.RUNNING.value, JobStatus.CANCELING.value}


class GatewayError(Exception):
    def __init__(self, status_code: int, code: str, message: str, retryable: bool = False) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.code = code
        self.message = message
        self.retryable = retryable


def _job_id() -> str:
    return f"job_{uuid.uuid4().hex}"


class GatewayService:
    def __init__(self, store: Store, endpoint: WorkerEndpoint) -> None:
        self.store = store
        self.endpoint = endpoint
        self._lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="gateway")
        self._stop = threading.Event()
        self._event_seq: dict[str, int] = {}
        # A1: the worker now runs the skill on its own thread, so its first
        # events can arrive before submit_skill stores worker_job_id.  Buffer
        # such events by worker job id and replay them once the row is known.
        self._pending_events: dict[str, list[JobEvent]] = {}
        self._poller = threading.Thread(target=self._poll_loop, name="job-poller", daemon=True)
        self._poller.start()
        lost = self.store.mark_live_jobs_lost()
        self.startup_lost_jobs = lost

    # -- helpers -------------------------------------------------------

    def _live_jobs(self, simulation_id: str) -> list[dict[str, Any]]:
        return [job for job in self.store.list_jobs(simulation_id) if job["status"] in LIVE]

    def _require_simulation(self, simulation_id: str) -> dict[str, Any]:
        simulation = self.store.get_simulation(simulation_id)
        if simulation is None or simulation["closed"]:
            raise GatewayError(404, "UNKNOWN_SIMULATION", f"unknown or closed simulation {simulation_id}")
        return simulation

    def _error_from_exception(self, error: Exception) -> dict[str, Any]:
        if isinstance(error, ProtocolViolation):
            info = error.info
            return {"code": info.code.value, "message": info.message, "retryable": info.retryable}
        if isinstance(error, ValidationError):
            return {"code": "INVALID_REQUEST", "message": str(error), "retryable": False}
        message = str(error)
        if message.startswith("WORKER_BUSY"):
            detail = message.split(":", 1)[-1].strip() or "worker is busy"
            return {"code": "WORKER_BUSY", "message": detail, "retryable": True}
        return {"code": "WORKER_LOST", "message": f"{type(error).__name__}: {error}", "retryable": False}

    def _endpoint_call(self, call: Any) -> Any:
        """Run one worker-bound call and map its error codes to Gateway errors."""
        try:
            return call()
        except GatewayError:
            raise
        except ProtocolViolation:
            raise
        except Exception as error:
            info = self._error_from_exception(error)
            if info["code"] == "WORKER_BUSY":
                raise GatewayError(409, "WORKER_BUSY", info["message"], retryable=True) from error
            raise

    def _append_synthetic(self, job: dict[str, Any], event_type: JobEventType, status: JobStatus, **data: Any) -> None:
        seq = self._event_seq.get(job["job_id"], 0)
        payload = {
            "job_id": job["job_id"],
            "event_seq": seq,
            "at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "type": event_type.value,
            "status": status.value,
            "data": data,
            "evidence": [],
        }
        self._event_seq[job["job_id"]] = seq + 1
        self.store.append_event(simulation_id=job["simulation_id"], job_id=job["job_id"], payload=payload)

    # -- job polling ---------------------------------------------------

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                events = self.endpoint.poll_events()
            except Exception:
                events = []
            for event in events:
                try:
                    self._apply_event(event)
                except Exception:
                    pass
            try:
                self._check_worker_alive()
            except Exception:
                pass
            time.sleep(0.01)

    def _check_worker_alive(self) -> None:
        if self.endpoint.alive():
            return
        live = [
            job
            for job in self.store.list_jobs()
            if job["worker_job_id"] and job["status"] in LIVE
        ]
        for job in live:
            import json

            self.store.update_job(
                job["job_id"],
                status=JobStatus.FAILED.value,
                error_json=json.dumps(
                    {"code": "WORKER_LOST", "message": "worker exited while the job was live"}
                ),
            )

    def _apply_event(self, event: JobEvent) -> None:
        with self._lock:
            job = self.store.get_job_by_worker_id(event.job_id)
            if job is None:
                bucket = self._pending_events.setdefault(event.job_id, [])
                if len(bucket) < 64 and len(self._pending_events) <= 128:
                    bucket.append(event)
                return
            self._apply_event_locked(job, event)

    def _apply_event_locked(self, job: dict[str, Any], event: JobEvent) -> None:
        seq = self._event_seq.get(job["job_id"], 0)
        payload = event.model_dump(mode="json")
        payload["event_seq"] = seq
        self._event_seq[job["job_id"]] = seq + 1
        self.store.append_event(simulation_id=job["simulation_id"], job_id=job["job_id"], payload=payload)
        if job["status"] in TERMINAL:
            return
        fields: dict[str, Any] = {"status": event.status.value}
        if event.progress is not None:
            fields["progress"] = event.progress
        if event.status.value == JobStatus.FAILED.value:
            fields["error_json"] = __import__("json").dumps(
                {
                    "code": event.data.get("error_code", "WORKER_LOST"),
                    "message": event.data.get("message", event.message or "worker reported failure"),
                }
            )
        self.store.update_job(job["job_id"], **fields)

    # -- simulation lifecycle -----------------------------------------

    def start_simulation(self, spec: SimulationSpec, idempotency_key: str | None) -> dict[str, Any]:
        if idempotency_key:
            existing = self.store.get_idempotent(idempotency_key)
            if existing:
                job = self.store.get_job(existing)
                if job:
                    return job
        if self.store.list_jobs():
            active = [job for job in self.store.list_jobs() if job["status"] in LIVE]
            if active:
                raise GatewayError(409, "CONTROL_BUSY", "another simulation or job is in progress", retryable=True)
        job = self.store.create_job(job_id=_job_id(), kind="start_simulation")
        if idempotency_key:
            self.store.put_idempotent(idempotency_key, job["job_id"])
        self._append_synthetic(job, JobEventType.CREATED, JobStatus.PENDING)
        self._executor.submit(self._run_start, job["job_id"], spec)
        return self.store.get_job(job["job_id"])  # type: ignore[return-value]

    def _run_start(self, job_id: str, spec: SimulationSpec) -> None:
        self.store.update_job(job_id, status=JobStatus.RUNNING.value)
        try:
            state = self.endpoint.start(spec)
            self.store.save_simulation(
                simulation_id=state.simulation_id,
                episode_id=state.episode_id,
                state=state.model_dump(mode="json"),
            )
            self.store.update_job(
                job_id,
                status=JobStatus.SUCCEEDED.value,
                progress=1.0,
                simulation_id=state.simulation_id,
            )
            job = self.store.get_job(job_id)
            if job:
                self._append_synthetic(job, JobEventType.SUCCEEDED, JobStatus.SUCCEEDED, simulation_id=state.simulation_id)
        except Exception as error:
            info = self._error_from_exception(error)
            __import__("json")
            import json

            self.store.update_job(
                job_id,
                status=JobStatus.FAILED.value,
                error_json=json.dumps(info),
            )

    def get_state(self, simulation_id: str) -> WorldState:
        self._require_simulation(simulation_id)
        with self._lock:
            state = self._endpoint_call(self.endpoint.observe)
            self.store.save_simulation(
                simulation_id=state.simulation_id,
                episode_id=state.episode_id,
                state=state.model_dump(mode="json"),
            )
        return state

    def reset(self, simulation_id: str) -> WorldState:
        self._require_simulation(simulation_id)
        if self._live_jobs(simulation_id):
            raise GatewayError(409, "CONTROL_BUSY", "cancel live jobs before reset", retryable=True)
        with self._lock:
            state = self._endpoint_call(self.endpoint.reset)
            self.store.save_simulation(
                simulation_id=state.simulation_id,
                episode_id=state.episode_id,
                state=state.model_dump(mode="json"),
            )
        return state

    def close_simulation(self, simulation_id: str) -> None:
        self._require_simulation(simulation_id)
        if self._live_jobs(simulation_id):
            raise GatewayError(409, "CONTROL_BUSY", "cancel live jobs before close", retryable=True)
        with self._lock:
            self.endpoint.close()
            self.store.close_simulation(simulation_id)

    # -- camera frames --------------------------------------------------

    def capture_frame(self, simulation_id: str) -> dict[str, Any]:
        self._require_simulation(simulation_id)
        if not hasattr(self.endpoint, "capture"):
            raise GatewayError(400, "UNSUPPORTED_CAPABILITY", "this worker does not provide frames")
        metadata = self._endpoint_call(self.endpoint.capture)
        if metadata.get("simulation_id") != simulation_id:
            raise GatewayError(
                409, "STALE_WORLD_STATE", "frame belongs to a different simulation", retryable=True
            )
        self.store.save_frame(frame_id=metadata["frame_id"], simulation_id=simulation_id, metadata=metadata)
        return metadata

    def get_frame(self, frame_id: str) -> dict[str, Any]:
        frame = self.store.get_frame(frame_id)
        if frame is None:
            raise GatewayError(404, "UNKNOWN_FRAME", f"unknown frame {frame_id}")
        return frame

    def latest_frame(self, simulation_id: str) -> dict[str, Any]:
        """Newest stored frame for a simulation (A3: the Agent can poll this)."""
        self._require_simulation(simulation_id)
        frames = self.store.list_frames(simulation_id, 1)
        if not frames:
            raise GatewayError(404, "UNKNOWN_FRAME", f"no frames stored for {simulation_id}")
        frame = self.store.get_frame(frames[0]["frame_id"])
        if frame is None:
            raise GatewayError(404, "UNKNOWN_FRAME", f"frame {frames[0]['frame_id']} vanished")
        return frame

    # -- skills and jobs ----------------------------------------------

    def submit_skill(
        self,
        invocation: SkillInvocation,
        *,
        simulation_id: str,
        idempotency_key: str | None = None,
        expected_episode: str | None = None,
        expected_state_version: int | None = None,
    ) -> dict[str, Any]:
        self._require_simulation(simulation_id)
        if idempotency_key:
            existing = self.store.get_idempotent(idempotency_key)
            if existing:
                job = self.store.get_job(existing)
                if job:
                    return job
        if invocation.skill not in self.endpoint.skills():
            raise GatewayError(400, "UNSUPPORTED_CAPABILITY", f"unsupported skill {invocation.skill}")
        with self._lock:
            state = self.endpoint.observe()
            if state.simulation_id != simulation_id:
                raise GatewayError(409, "STALE_WORLD_STATE", "simulation id changed", retryable=True)
            if expected_episode and state.episode_id != expected_episode:
                raise GatewayError(409, "STALE_WORLD_STATE", "episode changed", retryable=True)
            if expected_state_version is not None and state.state_version != expected_state_version:
                raise GatewayError(409, "STALE_WORLD_STATE", "state_version changed", retryable=True)
            job = self.store.create_job(
                job_id=_job_id(),
                kind="skill",
                simulation_id=simulation_id,
                skill=invocation.skill,
            )
            self._append_synthetic(job, JobEventType.CREATED, JobStatus.PENDING)
            try:
                worker_job_id = self.endpoint.submit(invocation)
            except Exception as error:
                info = self._error_from_exception(error)
                import json

                self.store.update_job(job["job_id"], status=JobStatus.FAILED.value, error_json=json.dumps(info))
                status_code = 409 if info["code"] == "WORKER_BUSY" else 400
                raise GatewayError(status_code, info["code"], info["message"], info.get("retryable", False)) from error
            self.store.update_job(
                job["job_id"],
                status=JobStatus.RUNNING.value,
                worker_job_id=worker_job_id,
            )
            # Replay events the poller saw before the worker id was stored
            # (a fast skill can finish before this update lands).
            for buffered in self._pending_events.pop(worker_job_id, []):
                row = self.store.get_job(job["job_id"])
                if row is not None:
                    self._apply_event_locked(row, buffered)
        if idempotency_key:
            self.store.put_idempotent(idempotency_key, job["job_id"])
        return self.store.get_job(job["job_id"])  # type: ignore[return-value]

    def get_job(self, job_id: str) -> dict[str, Any]:
        job = self.store.get_job(job_id)
        if job is None:
            raise GatewayError(404, "UNKNOWN_JOB", f"unknown job {job_id}")
        return job

    def cancel_job(self, job_id: str) -> dict[str, Any]:
        job = self.get_job(job_id)
        if job["status"] in TERMINAL:
            return job
        if not job.get("worker_job_id"):
            raise GatewayError(409, "CANNOT_CANCEL", "job has no worker acknowledgement target")
        self.store.update_job(job_id, status=JobStatus.CANCELING.value, cancel_requested=1)
        with self._lock:
            self.endpoint.cancel(job["worker_job_id"])
        # CANCELED is only written when the Worker's canceled event arrives.
        return self.get_job(job_id)

    def wait_job(self, job_id: str, timeout_s: float = 10.0) -> dict[str, Any]:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            job = self.get_job(job_id)
            if job["status"] in TERMINAL:
                return job
            time.sleep(0.02)
        return self.get_job(job_id)

    # -- tasks and plans ----------------------------------------------

    def create_task(self, spec: Any) -> dict[str, Any]:
        self.store.save_task(task_id=spec.task_id, spec=spec.model_dump(mode="json"))
        return self.store.get_task(spec.task_id)  # type: ignore[return-value]

    OBJECT_MOVE_TOLERANCE_M = 0.005

    def validate_plan(self, task_id: str, plan: TaskPlan) -> str:
        task = self.store.get_task(task_id)
        if task is None:
            raise GatewayError(404, "UNKNOWN_TASK", f"unknown task {task_id}")
        if plan.task_id != task_id:
            raise GatewayError(400, "INVALID_REQUEST", "plan.task_id does not match the task")
        unknown = [step.skill for step in plan.steps if step.skill not in self.endpoint.skills()]
        if unknown:
            raise GatewayError(400, "UNSUPPORTED_CAPABILITY", f"unsupported skills: {unknown}")
        budget = task["spec"]["budgets"]
        if len(plan.steps) > budget["max_skill_calls"]:
            raise GatewayError(400, "INVALID_REQUEST", "plan exceeds max_skill_calls")
        plan_hash = compute_plan_hash(
            task_id=plan.task_id,
            episode_id=plan.episode_id,
            base_state_version=plan.base_state_version,
            steps=plan.steps,
        )
        state = self.endpoint.observe()
        snapshot = {
            "episode_id": state.episode_id,
            "state_version": state.state_version,
            "objects": {
                obj.object_id: [float(v) for v in obj.pose.position] for obj in state.objects
            },
        }
        self.store.save_plan(
            task_id=task_id, plan=plan.model_dump(mode="json"), plan_hash=plan_hash, snapshot=snapshot
        )
        return plan_hash

    def execute_plan(
        self,
        task_id: str,
        *,
        simulation_id: str,
        plan_hash: str,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        if idempotency_key:
            existing = self.store.get_idempotent(idempotency_key)
            if existing:
                job = self.store.get_job(existing)
                if job:
                    return job
        task = self.store.get_task(task_id)
        if task is None or task["plan"] is None:
            raise GatewayError(404, "UNKNOWN_TASK", "task or validated plan not found")
        if task["plan_hash"] != plan_hash:
            raise GatewayError(409, "STALE_PLAN", "plan_hash does not match the validated plan")
        self._require_simulation(simulation_id)
        plan = TaskPlan.model_validate(task["plan"])
        state = self.endpoint.observe()
        if state.episode_id != plan.episode_id or state.state_version != plan.base_state_version:
            raise GatewayError(409, "STALE_WORLD_STATE", "plan episode/state_version is stale", retryable=True)
        snapshot = task.get("snapshot") or {}
        expected_objects = snapshot.get("objects") or {}
        actual_objects = {
            obj.object_id: [float(v) for v in obj.pose.position] for obj in state.objects
        }
        moved: dict[str, float] = {}
        for object_id, expected in expected_objects.items():
            actual = actual_objects.get(object_id)
            if actual is None:
                moved[object_id] = float("inf")
                continue
            distance = math.dist(expected, actual)
            if distance > self.OBJECT_MOVE_TOLERANCE_M:
                moved[object_id] = distance
        if set(actual_objects) != set(expected_objects):
            moved["__object_set__"] = float("inf")
        if moved:
            raise GatewayError(
                409,
                "STALE_WORLD_STATE",
                f"objects moved since plan validation: {moved}",
                retryable=True,
            )
        job = self.store.create_job(job_id=_job_id(), kind="task", simulation_id=simulation_id, task_id=task_id)
        if idempotency_key:
            self.store.put_idempotent(idempotency_key, job["job_id"])
        self._append_synthetic(job, JobEventType.CREATED, JobStatus.PENDING)
        self._executor.submit(self._run_plan, job["job_id"], plan, simulation_id)
        return self.store.get_job(job["job_id"])  # type: ignore[return-value]

    def _run_plan(self, job_id: str, plan: TaskPlan, simulation_id: str) -> None:
        self.store.update_job(job_id, status=JobStatus.RUNNING.value)
        current_child: str | None = None
        try:
            for step in plan.steps:
                parent = self.store.get_job(job_id)
                if parent and parent["cancel_requested"]:
                    if current_child:
                        self.cancel_job(current_child)
                    self.store.update_job(job_id, status=JobStatus.CANCELED.value)
                    job = self.store.get_job(job_id)
                    if job:
                        self._append_synthetic(job, JobEventType.CANCELED, JobStatus.CANCELED)
                    return
                child = self.submit_skill(
                    SkillInvocation(
                        skill=step.skill,
                        parameters=step.parameters,
                        timeout_s=max(step.expected_duration_s * 2.0, 5.0),
                        expected_duration_s=step.expected_duration_s,
                    ),
                    simulation_id=simulation_id,
                )
                current_child = child["job_id"]
                child = self.wait_job(child["job_id"], timeout_s=step.expected_duration_s * 2.0 + 10.0)
                if child["status"] == JobStatus.FAILED.value:
                    import json

                    self.store.update_job(
                        job_id,
                        status=JobStatus.FAILED.value,
                        error_json=json.dumps(child.get("error") or {"code": "SKILL_FAILED"}),
                    )
                    return
                if child["status"] == JobStatus.CANCELED.value:
                    self.store.update_job(job_id, status=JobStatus.CANCELED.value)
                    job = self.store.get_job(job_id)
                    if job:
                        self._append_synthetic(job, JobEventType.CANCELED, JobStatus.CANCELED)
                    return
                current_child = None
            self.store.update_job(job_id, status=JobStatus.SUCCEEDED.value, progress=1.0)
            job = self.store.get_job(job_id)
            if job:
                self._append_synthetic(job, JobEventType.SUCCEEDED, JobStatus.SUCCEEDED)
        except Exception as error:
            import json

            info = self._error_from_exception(error)
            self.store.update_job(job_id, status=JobStatus.FAILED.value, error_json=json.dumps(info))

    def close(self) -> None:
        self._stop.set()
        self._poller.join(timeout=1.0)
        with self._lock:
            self._pending_events.clear()
        self._executor.shutdown(wait=False)
        self.store.close()
