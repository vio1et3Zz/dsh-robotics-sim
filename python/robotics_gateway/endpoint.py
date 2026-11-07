"""Worker endpoints used by the Gateway.

The loopback endpoint is deterministic and exists for protocol/concurrency
tests; the subprocess endpoint runs the real Isaac Worker over a loopback TCP
JSON-lines protocol.
"""

from __future__ import annotations

import json
import os
import queue
import socket
import subprocess
import threading
import time
import uuid
from abc import ABC, abstractmethod
from pathlib import Path

from robotics_backends.fake import SUPPORTED_SKILLS, FakeBackend
from robotics_backends.profiles import validate_spec, violation
from robotics_protocol import (
    BackendKind,
    ErrorCode,
    JobEvent,
    RobotSpec,
    SimulationSpec,
    SkillInvocation,
    WorldState,
)

ROOT = Path(__file__).resolve().parents[2]


class WorkerEndpoint(ABC):
    @abstractmethod
    def start(self, spec: SimulationSpec) -> WorldState: ...

    @abstractmethod
    def observe(self) -> WorldState: ...

    @abstractmethod
    def reset(self) -> WorldState: ...

    @abstractmethod
    def submit(self, invocation: SkillInvocation) -> str: ...

    @abstractmethod
    def cancel(self, worker_job_id: str) -> bool: ...

    @abstractmethod
    def poll_events(self) -> list[JobEvent]: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def alive(self) -> bool: ...

    @abstractmethod
    def skills(self) -> set[str]: ...

    @property
    @abstractmethod
    def kind(self) -> str: ...


class LoopbackEndpoint(WorkerEndpoint):
    """FakeBackend behind a thread; mock evidence only, never simulation proof."""

    def __init__(self, tick_s: float = 0.05) -> None:
        self._backend = FakeBackend()
        self._lock = threading.RLock()
        self._tick_s = tick_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._active: str | None = None

    @property
    def kind(self) -> str:
        return "mock"

    def start(self, spec: SimulationSpec) -> WorldState:
        with self._lock:
            return self._backend.start(spec)

    def observe(self) -> WorldState:
        with self._lock:
            return self._backend.observe()

    def reset(self) -> WorldState:
        with self._lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            self._thread = None
            self._active = None
            return self._backend.reset()

    def submit(self, invocation: SkillInvocation) -> str:
        with self._lock:
            job_id = self._backend.submit_skill(invocation)
            self._active = job_id
            self._stop.clear()
            self._thread = threading.Thread(target=self._drive, name="fake-skill", daemon=True)
            self._thread.start()
            return job_id

    def _drive(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                try:
                    self._backend.tick()
                except Exception:
                    return
            time.sleep(self._tick_s)

    def cancel(self, worker_job_id: str) -> bool:
        with self._lock:
            if self._active != worker_job_id:
                return False
            stopped = self._backend.cancel_skill(worker_job_id)
            if stopped:
                self._active = None
                self._stop.set()
            return stopped

    def poll_events(self) -> list[JobEvent]:
        with self._lock:
            return self._backend.drain_events()

    def close(self) -> None:
        with self._lock:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
                self._thread = None
            self._backend.close()

    def alive(self) -> bool:
        return True

    def skills(self) -> set[str]:
        return set(SUPPORTED_SKILLS)


class SubprocessEndpoint(WorkerEndpoint):
    """Real Isaac Worker over a loopback TCP JSON-lines protocol.

    The worker entry is selected from the fixed ``WorkerProfile`` of the
    ``SimulationSpec`` (never from HTTP input).  A different profile restarts the
    worker process; the Gateway still owns exactly one live worker at a time.
    """

    def __init__(
        self,
        *,
        python: str,
        output_dir: Path,
        entry: Path | None = None,
        device: str = "cuda:0",
        startup_timeout_s: float = 240.0,
    ) -> None:
        self._python = python
        self._entry = entry
        self._device = device
        self._output_dir = output_dir
        self._startup_timeout_s = startup_timeout_s
        self._process: subprocess.Popen[bytes] | None = None
        self._listener: socket.socket | None = None
        self._sock: socket.socket | None = None
        self._file: object | None = None
        self._send_lock = threading.Lock()
        self._pending: dict[str, tuple[threading.Event, dict]] = {}
        self._events: queue.Queue[JobEvent] = queue.Queue()
        self._ready = threading.Event()
        self._robot_spec: RobotSpec | None = None
        self._reader: threading.Thread | None = None
        self._stdout = None
        self._stderr = None
        self._closed = False
        self._connection_lost = False
        self._launch_index = 0

    @property
    def kind(self) -> str:
        return "isaaclab_physx"

    # -- profile -> worker entry ---------------------------------------

    @staticmethod
    def entry_for_profile(profile: object) -> Path:
        module = str(getattr(profile, "entry_module", "")).replace(".", "/")
        if not module:
            raise violation(ErrorCode.UNSUPPORTED_CAPABILITY, "profile has no entry module")
        path = ROOT / "python" / f"{module}.py"
        if not path.is_file():
            raise violation(ErrorCode.ASSET_NOT_READY, f"worker entry not found for profile: {path}")
        return path

    # -- lifecycle ------------------------------------------------------

    def _ensure_started(self, entry: Path | None) -> None:
        selected = entry or self._entry
        if selected is None:
            raise RuntimeError("no worker entry selected")
        if self._sock is not None and self._entry == selected and self._ready.is_set() and not self._connection_lost:
            return
        if self._process is not None or self._sock is not None:
            self._shutdown_worker()
        self._entry = selected
        self._launch_index += 1
        self._output_dir.mkdir(parents=True, exist_ok=True)
        launch_dir = self._output_dir / f"worker_{self._launch_index}"
        self._stdout = (self._output_dir / f"stdout_{self._launch_index}.log").open("wb")
        self._stderr = (self._output_dir / f"stderr_{self._launch_index}.log").open("wb")
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(1)
        self._listener.settimeout(self._startup_timeout_s)
        port = int(self._listener.getsockname()[1])

        env = os.environ.copy()
        project_root = Path(__file__).resolve().parents[2]
        pythonpath = [str(project_root / "python")]
        # Optional Isaac Lab source tree (only needed when it is not installed
        # into the worker interpreter).  Enable with DSH_ISAACLAB_SOURCE.
        default_source = Path.home() / "isaac" / "IsaacLab" / "source" / "isaaclab"
        isaaclab_source = os.environ.get("DSH_ISAACLAB_SOURCE") or (
            str(default_source) if default_source.is_dir() else ""
        )
        if isaaclab_source:
            pythonpath.append(isaaclab_source)
        env["PYTHONPATH"] = os.pathsep.join(pythonpath)
        for name in ("ISAAC_PATH", "EXP_PATH", "CARB_APP_PATH", "PYTHONHOME"):
            env[name] = ""
        command = [
            self._python,
            "-u",
            str(selected),
            "--headless",
            "--device",
            self._device,
            "--gateway-host",
            "127.0.0.1",
            "--gateway-port",
            str(port),
            "--output_dir",
            str(launch_dir),
        ]
        self._process = subprocess.Popen(
            command,
            cwd=str(project_root),
            env=env,
            stdout=self._stdout,
            stderr=self._stderr,
            stdin=subprocess.DEVNULL,
        )
        try:
            self._sock, _ = self._listener.accept()
        except socket.timeout as error:
            raise RuntimeError(f"worker did not connect within {self._startup_timeout_s}s") from error
        finally:
            self._listener.close()
            self._listener = None
        self._sock.settimeout(None)
        self._file = self._sock.makefile("r", encoding="utf-8")
        self._reader = threading.Thread(target=self._read_loop, name="worker-endpoint-reader", daemon=True)
        self._reader.start()
        if not self._ready.wait(self._startup_timeout_s):
            raise RuntimeError("worker did not send ready")
        if self._robot_spec is None:
            raise RuntimeError("worker ready message had no robot_spec")

    def _shutdown_worker(self) -> None:
        try:
            if self._process is not None and self._process.poll() is None and self._sock is not None:
                self._request("close", {}, timeout_s=10.0)
        except Exception:
            pass
        try:
            if self._sock is not None:
                self._sock.close()
        except Exception:
            pass
        self._sock = None
        self._file = None
        if self._process is not None:
            try:
                self._process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
        self._process = None
        for handle in (self._stdout, self._stderr):
            if handle is not None:
                handle.close()
        self._stdout = None
        self._stderr = None
        self._ready.clear()
        self._robot_spec = None
        self._connection_lost = False
        self._entry = None

    def _read_loop(self) -> None:
        assert self._file is not None
        try:
            for line in self._file:
                if not line.strip():
                    continue
                message = json.loads(line)
                if message.get("type") == "event":
                    self._events.put(JobEvent.model_validate(message["event"]))
                    continue
                if message.get("type") == "ready":
                    self._robot_spec = RobotSpec.model_validate(message["robot_spec"])
                    self._ready.set()
                    continue
                request_id = message.get("id")
                if request_id and request_id in self._pending:
                    event, slot = self._pending.pop(request_id)
                    slot.update(message)
                    event.set()
        except Exception:
            pass
        finally:
            self._connection_lost = True
            self._ready.set()
            for event, slot in list(self._pending.values()):
                slot["ok"] = False
                slot["error"] = {"code": "WORKER_LOST", "message": "worker connection ended"}
                event.set()
            self._pending.clear()

    def _request(self, op: str, payload: dict, timeout_s: float = 120.0, entry: Path | None = None) -> dict:
        self._ensure_started(entry)
        assert self._sock is not None
        request_id = f"req_{uuid.uuid4().hex}"
        event = threading.Event()
        slot: dict = {}
        self._pending[request_id] = (event, slot)
        line = json.dumps({"id": request_id, "op": op, **payload}, ensure_ascii=False) + "\n"
        with self._send_lock:
            self._sock.sendall(line.encode("utf-8"))
        if not event.wait(timeout_s):
            raise RuntimeError(f"worker request {op} timed out")
        if not slot.get("ok"):
            error = slot.get("error", {})
            raise RuntimeError(f"{error.get('code', 'WORKER_LOST')}: {error.get('message', 'request failed')}")
        return slot

    def start(self, spec: SimulationSpec) -> WorldState:
        profile = validate_spec(spec)
        if profile.backend_kind is not BackendKind.ISAACLAB_PHYSX:
            raise violation(ErrorCode.UNSUPPORTED_BACKEND, "subprocess endpoint only runs PhysX profiles")
        entry = self.entry_for_profile(profile)
        response = self._request("observe", {}, entry=entry)
        return WorldState.model_validate(response["state"])

    def observe(self) -> WorldState:
        return WorldState.model_validate(self._request("observe", {})["state"])

    def reset(self) -> WorldState:
        return WorldState.model_validate(self._request("reset", {})["state"])

    def capture(self) -> dict:
        response = self._request("capture", {}, timeout_s=120.0)
        return dict(response["frame"])

    def submit(self, invocation: SkillInvocation) -> str:
        response = self._request("submit", {"invocation": invocation.model_dump(mode="json")})
        return str(response["job_id"])

    def cancel(self, worker_job_id: str) -> bool:
        response = self._request("cancel", {"job_id": worker_job_id}, timeout_s=30.0)
        return bool(response.get("canceled", False))

    def poll_events(self) -> list[JobEvent]:
        events = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                return events

    def close(self) -> None:
        # Reusable: closing a simulation releases the worker but the endpoint
        # stays usable, so a later start_simulation on the same Gateway works
        # (the T09 "restart required after close" limitation).
        self._shutdown_worker()

    def alive(self) -> bool:
        # env_isaaclab\Scripts\python.exe is a shim: the tracked Popen may exit
        # while the real Kit process keeps the accepted TCP connection.
        return self._sock is not None and not self._connection_lost

    def skills(self) -> set[str]:
        if self._robot_spec is None:
            return set()
        return set(self._robot_spec.skills)

