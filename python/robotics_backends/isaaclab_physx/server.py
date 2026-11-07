"""Long-running Isaac Worker protocol: loopback TCP JSON lines.

The command loop answers ``observe``/``reset``/``capture``/``submit``/``close``.
A skill runs synchronously on the simulation thread, but between control steps it
calls back into the loop (``_hook``): the loop drains queued commands and answers
``observe`` from the latest snapshot and ``capture`` from the monitor's frame
buffer.  This keeps the renderer/PhysX on one thread (Isaac is not thread-safe:
a worker-thread ``camera.update`` hung in the T09 A1 probe) while the Gateway
still gets fresh state and frames during a long skill.

``cancel`` is read by a separate reader thread and is only acknowledged after
the controller has actively stopped and held the pose.
"""

from __future__ import annotations

import json
import os
import queue
import socket
import sys
import threading
import time
import traceback
import uuid
from datetime import datetime, timezone
from typing import Any

from robotics_protocol import (
    JobEventType,
    JobStatus,
    SkillInvocation,
)

from .stub_policy import PolicyFailure
from .worker import IsaacLabWorker, SkillCanceled

# A long skill (policy loop / VLA) must not flood SQLite with progress rows.
MAX_EVENTS_PER_JOB = 200
TERMINAL_EVENTS = {JobEventType.SUCCEEDED, JobEventType.FAILED, JobEventType.CANCELED}

_DEBUG = os.environ.get("DSH_ROBOTICS_DEBUG") == "1"


def _debug(message: str) -> None:
    if _DEBUG:
        print(f"[server] {message}", file=sys.stderr, flush=True)


class WorkerServer:
    def __init__(self, worker: IsaacLabWorker, sock: socket.socket) -> None:
        self._worker = worker
        self._sock = sock
        self._file = sock.makefile("r", encoding="utf-8")
        self._send_lock = threading.Lock()
        self._commands: queue.Queue[dict[str, Any]] = queue.Queue()
        self._cancel = threading.Event()
        self._cancel_request_id: str | None = None
        self._running = False
        self._active_job_id: str | None = None
        self._active_skill = "calibrate_joint"
        self._event_seq = 0
        self._event_count = 0
        self._progress_bucket = -1
        self._ticks = 0
        self._last_phase: str | None = None
        self._last_phase_at = 0.0
        self._close_requested = False
        self._close_request_id: str | None = None
        self._state_lock = threading.Lock()
        self._monitor = getattr(worker, "monitor", None)
        self._reader = threading.Thread(target=self._read_loop, name="worker-reader", daemon=True)

    # -- wire ----------------------------------------------------------

    def send(self, payload: dict[str, Any]) -> None:
        line = json.dumps(payload, ensure_ascii=False) + "\n"
        with self._send_lock:
            self._sock.sendall(line.encode("utf-8"))

    def respond(self, request_id: str, **fields: Any) -> None:
        self.send({"id": request_id, "ok": True, **fields})

    def fail(self, request_id: str, message: str, code: str = "INVALID_REQUEST") -> None:
        self.send({"id": request_id, "ok": False, "error": {"code": code, "message": message}})

    def emit(
        self,
        event_type: JobEventType,
        status: JobStatus,
        *,
        progress: float | None = None,
        message: str | None = None,
        data: dict[str, Any] | None = None,
    ) -> None:
        if self._active_job_id is None:
            return
        terminal = event_type in TERMINAL_EVENTS
        with self._state_lock:
            if not terminal and self._event_count >= MAX_EVENTS_PER_JOB:
                return
            self._event_count += 1
            seq = self._event_seq
            self._event_seq = seq + 1
        self.send(
            {
                "type": "event",
                "event": {
                    "job_id": self._active_job_id,
                    "event_seq": seq,
                    "at": datetime.now(timezone.utc).isoformat(),
                    "type": event_type.value,
                    "status": status.value,
                    "skill": self._active_skill,
                    "message": message,
                    "progress": progress,
                    "data": data or {},
                    "evidence": ["isaaclab_physx"],
                },
            }
        )

    # -- loops ---------------------------------------------------------

    def run(self) -> None:
        self._reader.start()
        self.send({"type": "ready", "robot_spec": self._worker.robot_spec().model_dump(mode="json")})
        while True:
            command = self._commands.get()
            op = command.get("op")
            request_id = command.get("id", "unknown")
            if op == "close":
                self.respond(request_id, closed=True)
                break
            try:
                if op == "observe":
                    self.respond(request_id, state=self._observe().model_dump(mode="json"))
                elif op == "reset":
                    if self._is_running():
                        self.fail(request_id, "a skill is running; cancel it before reset", "WORKER_BUSY")
                    else:
                        self.respond(request_id, state=self._worker.reset().model_dump(mode="json"))
                elif op == "capture":
                    self._capture(request_id)
                elif op == "submit":
                    self._run_skill(request_id, command)
                    if self._close_requested:
                        if self._close_request_id is not None:
                            self.respond(self._close_request_id, closed=True)
                        break
                else:
                    self.fail(request_id, f"unknown operation {op}")
            except Exception as error:
                self.fail(request_id, f"{type(error).__name__}: {error}")

    def _read_loop(self) -> None:
        try:
            for line in self._file:
                if not line.strip():
                    continue
                message = json.loads(line)
                if message.get("op") == "cancel":
                    with self._state_lock:
                        running = self._running
                        if running:
                            self._cancel_request_id = message.get("id")
                    if running:
                        self._cancel.set()
                    else:
                        self.respond(message.get("id", "unknown"), canceled=False)
                else:
                    self._commands.put(message)
        except Exception:
            pass
        finally:
            self._commands.put({"id": "eof", "op": "close"})

    # -- command handlers ---------------------------------------------

    def _is_running(self) -> bool:
        with self._state_lock:
            return self._running

    def _observe(self):
        if self._is_running():
            snapshot = getattr(self._worker, "observe_snapshot", None)
            if snapshot is not None:
                state = snapshot()
                if state is not None:
                    return state
        return self._worker.observe()

    def _capture(self, request_id: str) -> None:
        if not hasattr(self._worker, "capture_frame"):
            self.fail(request_id, "worker does not implement capture_frame", "UNSUPPORTED_CAPABILITY")
            return
        running = self._is_running()
        started = time.perf_counter()
        try:
            frame = self._worker.capture_frame(fresh=not running)
        except RuntimeError as error:
            if "FRAME_NOT_READY" in str(error):
                # No buffered frame yet: the command loop owns the simulation
                # thread, so rendering here is safe even mid-skill.
                frame = self._worker.capture_frame(fresh=True)
            else:
                raise
        elapsed = time.perf_counter() - started
        if elapsed > 0.2:
            _debug(f"capture took {1000 * elapsed:.0f} ms (running={running})")
        self.respond(request_id, frame=frame)

    def _publish_now(self) -> None:
        publish = getattr(self._worker, "publish_snapshot", None)
        if publish is not None:
            try:
                publish()
            except Exception:
                pass

    def _seed_frame(self) -> None:
        # Render one frame before the skill starts so the first capture during
        # the skill never has to wait for a monitor tick.
        seed = getattr(self._worker, "publish_frame", None)
        if seed is not None:
            try:
                seed()
            except Exception:
                pass

    # -- cooperative scheduling ----------------------------------------

    def _tick(self) -> None:
        self._ticks += 1
        if _DEBUG and self._ticks <= 3:
            _debug(f"tick #{self._ticks}")
        if self._monitor is not None:
            self._monitor.tick()

    def _hook(self) -> None:
        """Called by the skill after every control step (simulation thread)."""
        self._tick()
        self._service_commands()

    def _service_commands(self) -> None:
        """Answer queued commands between control steps; never start a skill."""
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            op = command.get("op")
            request_id = command.get("id", "unknown")
            if op == "observe":
                started = time.perf_counter()
                state = self._observe()
                elapsed = time.perf_counter() - started
                if elapsed > 0.2:
                    _debug(f"observe took {1000 * elapsed:.0f} ms")
                self.respond(request_id, state=state.model_dump(mode="json"))
            elif op == "capture":
                self._capture(request_id)
            elif op in {"submit", "reset"}:
                self.fail(request_id, "a skill is running; cancel it first", "WORKER_BUSY")
            elif op == "close":
                self._close_requested = True
                self._close_request_id = request_id
                self._cancel.set()
            else:
                self.fail(request_id, f"unknown operation {op}")

    def _phase(self, phase: str, **data: Any) -> None:
        """Policy phase events (policy/execute/verify), throttled to 4 Hz."""
        now = time.time()
        if phase == self._last_phase and now - self._last_phase_at < 0.25:
            return
        self._last_phase = phase
        self._last_phase_at = now
        progress = data.pop("progress", None)
        self.emit(JobEventType.PROGRESS, JobStatus.RUNNING, progress=progress, data={"phase": phase, **data})

    def _progress(self, value: float) -> None:
        # One progress event per 10% bucket; a per-step check would emit the
        # same bucket ~40 times for a 4000-step hold.
        bucket = min(100, int(value * 100)) // 10 * 10
        if bucket == self._progress_bucket:
            return
        self._progress_bucket = bucket
        self.emit(JobEventType.PROGRESS, JobStatus.RUNNING, progress=bucket / 100.0)

    # -- skill execution -----------------------------------------------

    def _run_skill(self, request_id: str, command: dict[str, Any]) -> None:
        invocation = SkillInvocation.model_validate(command["invocation"])
        if invocation.skill not in self._supported_skills():
            self.fail(request_id, f"worker does not implement {invocation.skill}", "UNSUPPORTED_CAPABILITY")
            return
        with self._state_lock:
            if self._running:
                self.fail(request_id, "another skill is already running", "WORKER_BUSY")
                return
            self._active_skill = invocation.skill
            self._active_job_id = f"job_{uuid.uuid4().hex}"
            self._event_seq = 0
            self._event_count = 0
            self._progress_bucket = -1
            self._ticks = 0
            self._last_phase = None
            self._last_phase_at = 0.0
            self._cancel.clear()
            self._running = True
        if self._monitor is not None:
            self._monitor.start()
        started = time.perf_counter()
        self._publish_now()
        self._seed_frame()
        _debug(f"submit {invocation.skill} seeded in {1000 * (time.perf_counter() - started):.1f} ms")
        self.emit(JobEventType.CREATED, JobStatus.PENDING)
        self.emit(JobEventType.STARTED, JobStatus.RUNNING, progress=0.0)
        # The job id is handed to the Gateway before the first control step.
        self.respond(request_id, job_id=self._active_job_id)
        self._skill_body(invocation)

    def _skill_body(self, invocation: SkillInvocation) -> None:
        parameters = invocation.parameters
        started = time.perf_counter()
        _debug(f"skill {invocation.skill} started")
        try:
            if hasattr(self._worker, "run_skill"):
                result = self._worker.run_skill(
                    invocation.skill,
                    parameters,
                    cancel_check=self._cancel.is_set,
                    progress_cb=self._progress,
                    step_cb=self._hook,
                    phase_cb=self._phase,
                )
                self._emit_result(result)
            elif invocation.skill == "hold_pose":
                result = self._worker.hold_pose(
                    duration_s=float(parameters.get("duration_s", 10.0)),
                    cancel_check=self._cancel.is_set,
                    progress_cb=self._progress,
                    step_cb=self._hook,
                )
                self.emit(JobEventType.SUCCEEDED, JobStatus.SUCCEEDED, progress=1.0, data={"result": result})
            else:
                result = self._worker.calibrate_joint(
                    joint_name=str(parameters.get("joint_name", "panda_joint1")),
                    delta_rad=float(parameters.get("delta_rad", 0.05)),
                    hold_s=float(parameters.get("hold_s", 0.25)),
                    settle_s=float(parameters.get("settle_s", 0.20)),
                    cancel_check=self._cancel.is_set,
                    progress_cb=self._progress,
                    step_cb=self._hook,
                )
                self.emit(JobEventType.SUCCEEDED, JobStatus.SUCCEEDED, progress=1.0, data={"result": result})
        except PolicyFailure as failure:
            _debug(f"skill {invocation.skill} policy failure {failure.code} after {time.perf_counter() - started:.2f} s")
            self.emit(
                JobEventType.FAILED,
                JobStatus.FAILED,
                message=failure.code,
                data={"error_code": failure.code, "message": failure.message, "detail": failure.detail},
            )
        except SkillCanceled:
            _debug(f"skill {invocation.skill} canceled after {time.perf_counter() - started:.2f} s")
            self.emit(JobEventType.CANCELED, JobStatus.CANCELED, progress=0.0, message="cancel acknowledged")
            self._ack_cancel(True)
        except Exception as error:
            _debug(f"skill {invocation.skill} raised {type(error).__name__}: {error}")
            if _DEBUG:
                traceback.print_exc(file=sys.stderr)
            self.emit(
                JobEventType.FAILED,
                JobStatus.FAILED,
                data={"error_code": "INVALID_REQUEST", "message": f"{type(error).__name__}: {error}"},
            )
        finally:
            _debug(f"skill {invocation.skill} finished after {time.perf_counter() - started:.2f} s")
            self._skill_finished()

    def _emit_result(self, result: dict[str, Any]) -> None:
        if result.get("passed") is False:
            failure = str(result.get("failure") or "SKILL_FAILED")
            self.emit(
                JobEventType.FAILED,
                JobStatus.FAILED,
                progress=1.0,
                message=failure,
                data={"error_code": failure, "message": failure, "result": result},
            )
        else:
            self.emit(JobEventType.SUCCEEDED, JobStatus.SUCCEEDED, progress=1.0, data={"result": result})

    def _ack_cancel(self, canceled: bool) -> None:
        with self._state_lock:
            request_id = self._cancel_request_id
            self._cancel_request_id = None
        if request_id is not None:
            self.respond(request_id, canceled=canceled)

    def _skill_finished(self) -> None:
        if self._monitor is not None:
            self._monitor.stop()
        with self._state_lock:
            self._running = False
            self._cancel.clear()
            request_id = self._cancel_request_id
            self._cancel_request_id = None
            self._active_job_id = None
        if request_id is not None:
            self.respond(request_id, canceled=False)
        self._publish_now()

    def _supported_skills(self) -> set[str]:
        if hasattr(self._worker, "run_skill"):
            return set(self._worker.robot_spec().skills)
        return {"calibrate_joint", "hold_pose"}

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass


__all__ = ["WorkerServer"]
