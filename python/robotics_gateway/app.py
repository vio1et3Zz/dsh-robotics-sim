"""FastAPI surface for the Robotics Gateway.

This is the Gateway's own REST/WS protocol, not something Isaac ships.  The
Agent talks to these routes through the DSH plugin tools (T04).
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError
from robotics_protocol import (
    ProtocolViolation,
    Sha256,
    SimulationId,
    SimulationSpec,
    SkillInvocation,
    StrictModel,
    TaskPlan,
    TaskSpec,
)

from .service import GatewayError, GatewayService


class StartSimulationRequest(StrictModel):
    spec: SimulationSpec
    idempotency_key: str | None = None


class ExecutePlanRequest(StrictModel):
    simulation_id: SimulationId
    plan_hash: Sha256
    idempotency_key: str | None = None


class PlanRequest(StrictModel):
    plan: TaskPlan


class TaskRequest(StrictModel):
    spec: TaskSpec


class SubmitSkillRequest(StrictModel):
    invocation: SkillInvocation
    idempotency_key: str | None = None


def frame_payload(metadata: dict[str, Any]) -> dict[str, Any]:
    """Public frame description: no worker filesystem paths, stable URLs."""
    payload = dict(metadata)
    payload.pop("rgb_path", None)
    payload.pop("depth_path", None)
    frame_id = payload.get("frame_id", "")
    payload["frame_url"] = f"/api/v1/frames/{frame_id}"
    payload["rgb_url"] = f"/api/v1/frames/{frame_id}/rgb.png"
    payload["depth_url"] = f"/api/v1/frames/{frame_id}/depth.npy"
    return payload


UI_DIR = Path(__file__).resolve().parent / "ui"


def create_app(service: GatewayService) -> FastAPI:
    app = FastAPI(title="DSH Robotics Gateway", version="0.1.0")

    @app.exception_handler(GatewayError)
    async def _gateway_error(_request: Any, error: GatewayError) -> JSONResponse:
        return JSONResponse(
            status_code=error.status_code,
            content={"error": {"code": error.code, "message": error.message, "retryable": error.retryable}},
        )

    @app.exception_handler(ProtocolViolation)
    async def _protocol_error(_request: Any, error: ProtocolViolation) -> JSONResponse:
        info = error.info
        return JSONResponse(
            status_code=400,
            content={"error": {"code": info.code.value, "message": info.message, "retryable": info.retryable}},
        )

    @app.exception_handler(ValidationError)
    async def _validation_error(_request: Any, error: ValidationError) -> JSONResponse:
        return JSONResponse(
            status_code=400,
            content={"error": {"code": "INVALID_REQUEST", "message": str(error), "retryable": False}},
        )

    @app.get("/api/v1/health")
    async def health() -> dict[str, Any]:
        simulations = [job for job in service.store.list_jobs() if job["kind"] == "start_simulation"]
        ready = any(job["status"] == "succeeded" for job in simulations) and service.endpoint.alive()
        return {
            "gateway": "ok",
            "worker": "alive" if service.endpoint.alive() else "dead",
            "simulation_ready": ready,
            "startup_lost_jobs": service.startup_lost_jobs,
        }

    @app.get("/api/v1/backends")
    async def backends() -> dict[str, Any]:
        return {
            "backends": [
                {
                    "kind": service.endpoint.kind,
                    "alive": service.endpoint.alive(),
                    "skills": sorted(service.endpoint.skills()),
                    "verified": False,
                    "note": "mock endpoints are never simulation verification",
                }
            ]
        }

    @app.get("/api/v1/simulations")
    async def list_simulations(limit: int = Query(default=20, ge=1, le=100)) -> dict[str, Any]:
        return {"simulations": service.store.list_simulations(limit)}

    @app.post("/api/v1/simulations", status_code=202)
    async def start_simulation(request: StartSimulationRequest) -> dict[str, Any]:
        return service.start_simulation(request.spec, request.idempotency_key)

    @app.get("/api/v1/simulations/{simulation_id}")
    async def get_simulation(simulation_id: str) -> dict[str, Any]:
        simulation = service.store.get_simulation(simulation_id)
        if simulation is None:
            raise GatewayError(404, "UNKNOWN_SIMULATION", f"unknown simulation {simulation_id}")
        return simulation

    @app.get("/api/v1/simulations/{simulation_id}/state")
    async def get_state(simulation_id: str) -> dict[str, Any]:
        # A2: worker-bound calls run off the event loop so /health and the WS
        # stay responsive while a skill executes.
        state = await run_in_threadpool(service.get_state, simulation_id)
        return {
            "simulation_id": state.simulation_id,
            "episode_id": state.episode_id,
            "state_version": state.state_version,
            "sim_step": state.sim_step,
            "sim_time_s": state.sim_time_s,
            "state": state.model_dump(mode="json"),
            "skills": sorted(service.endpoint.skills()),
        }

    @app.post("/api/v1/simulations/{simulation_id}/reset", status_code=202)
    async def reset(simulation_id: str) -> dict[str, Any]:
        state = await run_in_threadpool(service.reset, simulation_id)
        return {"simulation_id": state.simulation_id, "episode_id": state.episode_id, "state": state.model_dump(mode="json")}

    @app.delete("/api/v1/simulations/{simulation_id}", status_code=202)
    async def close_simulation(simulation_id: str) -> dict[str, Any]:
        await run_in_threadpool(service.close_simulation, simulation_id)
        return {"simulation_id": simulation_id, "closed": True}

    @app.post("/api/v1/simulations/{simulation_id}/skills", status_code=202)
    async def submit_skill(simulation_id: str, request: SubmitSkillRequest) -> dict[str, Any]:
        return await run_in_threadpool(
            service.submit_skill,
            request.invocation,
            simulation_id=simulation_id,
            idempotency_key=request.idempotency_key,
        )

    @app.post("/api/v1/tasks", status_code=201)
    async def create_task(request: TaskRequest) -> dict[str, Any]:
        return service.create_task(request.spec)

    @app.post("/api/v1/tasks/{task_id}/plans/validate")
    async def validate_plan(task_id: str, request: PlanRequest) -> dict[str, Any]:
        plan_hash = await run_in_threadpool(service.validate_plan, task_id, request.plan)
        return {"task_id": task_id, "plan_hash": plan_hash, "valid": True}

    @app.post("/api/v1/tasks/{task_id}/execute", status_code=202)
    async def execute_plan(task_id: str, request: ExecutePlanRequest) -> dict[str, Any]:
        return await run_in_threadpool(
            service.execute_plan,
            task_id,
            simulation_id=request.simulation_id,
            plan_hash=request.plan_hash,
            idempotency_key=request.idempotency_key,
        )

    @app.post("/api/v1/simulations/{simulation_id}/frames", status_code=201)
    async def capture_frame(simulation_id: str) -> dict[str, Any]:
        metadata = await run_in_threadpool(service.capture_frame, simulation_id)
        return frame_payload(metadata)

    @app.get("/api/v1/simulations/{simulation_id}/frames")
    async def list_frames(
        simulation_id: str, limit: int = Query(default=12, ge=1, le=50)
    ) -> dict[str, Any]:
        if service.store.get_simulation(simulation_id) is None:
            raise GatewayError(404, "UNKNOWN_SIMULATION", f"unknown simulation {simulation_id}")
        frames = []
        for item in service.store.list_frames(simulation_id, limit):
            frame = service.store.get_frame(item["frame_id"])
            if frame is not None:
                frames.append(frame_payload(frame["metadata"]))
        return {"frames": frames}

    @app.get("/api/v1/simulations/{simulation_id}/frames/latest")
    async def latest_frame(simulation_id: str) -> dict[str, Any]:
        frame = service.latest_frame(simulation_id)
        return frame_payload(frame["metadata"])

    @app.get("/api/v1/frames/{frame_id}")
    async def get_frame(frame_id: str) -> dict[str, Any]:
        return frame_payload(service.get_frame(frame_id)["metadata"])

    @app.get("/api/v1/frames/{frame_id}/rgb.png")
    async def get_frame_rgb(frame_id: str) -> FileResponse:
        frame = service.get_frame(frame_id)
        path = Path(frame["metadata"]["rgb_path"])
        if not path.is_file():
            raise GatewayError(404, "FRAME_FILE_MISSING", f"rgb file missing for {frame_id}")
        return FileResponse(path, media_type="image/png", filename=path.name)

    @app.get("/api/v1/frames/{frame_id}/depth.npy")
    async def get_frame_depth(frame_id: str) -> FileResponse:
        frame = service.get_frame(frame_id)
        path = Path(frame["metadata"]["depth_path"])
        if not path.is_file():
            raise GatewayError(404, "FRAME_FILE_MISSING", f"depth file missing for {frame_id}")
        return FileResponse(path, media_type="application/octet-stream", filename=path.name)

    @app.get("/ui", include_in_schema=False)
    async def ui() -> FileResponse:
        index = UI_DIR / "index.html"
        if not index.is_file():
            raise GatewayError(500, "UI_MISSING", "console assets are not installed")
        return FileResponse(index, media_type="text/html; charset=utf-8")

    @app.get("/", include_in_schema=False)
    async def index() -> RedirectResponse:
        return RedirectResponse(url="/ui")

    @app.get("/api/v1/jobs")
    async def list_jobs(
        simulation_id: str | None = Query(default=None),
        limit: int = Query(default=50, ge=1, le=200),
    ) -> dict[str, Any]:
        jobs = service.store.list_jobs(simulation_id)
        jobs.reverse()
        return {"jobs": jobs[:limit]}

    @app.get("/api/v1/jobs/{job_id}")
    async def get_job(job_id: str) -> dict[str, Any]:
        return service.get_job(job_id)

    @app.get("/api/v1/jobs/{job_id}/events")
    async def get_job_events(
        job_id: str,
        after_seq: int = Query(default=0, ge=0),
        limit: int = Query(default=200, ge=1, le=500),
    ) -> dict[str, Any]:
        service.get_job(job_id)  # 404 for unknown jobs
        events = service.store.events_for_job(job_id=job_id, after_seq=after_seq, limit=limit)
        return {
            "job_id": job_id,
            "after_seq": after_seq,
            "events": events,
            "next_seq": events[-1]["seq"] if events else after_seq,
        }

    @app.post("/api/v1/jobs/{job_id}/cancel", status_code=202)
    async def cancel_job(job_id: str) -> dict[str, Any]:
        return service.cancel_job(job_id)

    @app.websocket("/api/v1/events")
    async def events(
        websocket: WebSocket,
        simulation_id: str | None = Query(default=None),
        after_seq: int = Query(default=0, ge=0),
    ) -> None:
        await websocket.accept()
        cursor = after_seq
        try:
            while True:
                batch = service.store.events_after(simulation_id=simulation_id, after_seq=cursor)
                for record in batch:
                    await websocket.send_json(record)
                    cursor = record["seq"]
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            return
        except RuntimeError:
            return
        except Exception:
            return

    if UI_DIR.is_dir():
        app.mount("/ui/assets", StaticFiles(directory=UI_DIR), name="ui-assets")

    return app
