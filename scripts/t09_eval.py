"""T09 fixed evaluation: 10 SO-101 pick/place runs + recovery/cancel tests.

Drives the real Gateway HTTP API (the same surface the DSH tools call).  The
manifest at docs/execution/eval_manifest.json was written before this run.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path

BASE = "http://127.0.0.1:8714"
PLACE = (0.22, 0.05)
SUCCESS_XY_M = 0.05
SUCCESS_MAX_Z = 0.09


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def request(method: str, path: str, body: dict | None = None, timeout: float = 120.0) -> tuple[int, dict]:
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(
        BASE + path, data=data, method=method, headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        raw = error.read().decode("utf-8")
        try:
            payload = json.loads(raw)
        except Exception:
            payload = {"raw": raw}
        return error.code, payload


def wait_job(job_id: str, timeout_s: float = 240.0) -> dict:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status, job = request("GET", f"/api/v1/jobs/{job_id}")
        if status != 200:
            raise RuntimeError(f"job poll failed: {status} {job}")
        if job["status"] in {"succeeded", "failed", "canceled"}:
            return job
        time.sleep(0.2)
    raise RuntimeError(f"job {job_id} did not finish in {timeout_s}s")


def state_of(simulation_id: str) -> dict:
    status, payload = request("GET", f"/api/v1/simulations/{simulation_id}/state")
    if status != 200:
        raise RuntimeError(f"state failed: {status} {payload}")
    return payload


def block_of(state: dict) -> list[float]:
    return next(obj for obj in state["state"]["objects"] if obj["object_id"] == "red_block")["pose"]["position"]


def success_of(state: dict, job: dict) -> tuple[bool, dict]:
    block = block_of(state)
    details = {
        "job_status": job["status"],
        "block": block,
        "xy_distance": ((block[0] - PLACE[0]) ** 2 + (block[1] - PLACE[1]) ** 2) ** 0.5,
        "z": block[2],
    }
    ok = (
        job["status"] == "succeeded"
        and details["xy_distance"] <= SUCCESS_XY_M
        and block[2] <= SUCCESS_MAX_Z
    )
    return ok, details


def task_spec(task_id: str) -> dict:
    return {
        "task_id": task_id,
        "goal": "把红色积木放入目标盒",
        "robot_id": "so101",
        "scene_id": "table_pick_place",
        "success_predicates": [
            {
                "predicate_id": "pred_block_in_box",
                "kind": "object_in_region",
                "object_id": "red_block",
                "region": {"min": [0.19, 0.02, 0.03], "max": [0.25, 0.08, 0.08]},
                "tolerance_m": 0.01,
            }
        ],
        "budgets": {"sim_time_s": 180.0, "wall_time_s": 240.0, "max_skill_calls": 6, "max_retries": 2},
        "constraints": ["top grasp", "place on target box top face"],
    }


def plan_for(task_id: str, episode_id: str, base_state_version: int, plan_id: str, grasp_x: float) -> dict:
    return {
        "plan_id": plan_id,
        "task_id": task_id,
        "episode_id": episode_id,
        "base_state_version": base_state_version,
        "created_at": now(),
        "steps": [
            {
                "step_id": "step_observe",
                "skill": "observe_scene",
                "parameters": {},
                "depends_on": [],
                "expected_duration_s": 1.0,
            },
            {
                "step_id": "step_pick",
                "skill": "pick",
                "parameters": {"place_x": PLACE[0], "place_y": PLACE[1], "grasp_x": grasp_x},
                "depends_on": ["step_observe"],
                "expected_duration_s": 60.0,
            },
        ],
        "budget": {"sim_time_s": 180.0, "wall_time_s": 240.0, "max_skill_calls": 6, "max_retries": 2},
    }


def run_pick_plan(simulation_id: str, run_label: str, grasp_x: float | None = None) -> dict:
    state = state_of(simulation_id)
    block = block_of(state)
    if grasp_x is None:
        grasp_x = 0.18 if block[0] < 0.19 else 0.20
    task_id = f"task_{uuid.uuid4().hex}"
    plan_id = f"plan_{uuid.uuid4().hex}"
    status, created = request("POST", "/api/v1/tasks", {"spec": task_spec(task_id)})
    if status != 201:
        return {"label": run_label, "ok": False, "error": f"create_task {status}: {created}"}
    plan = plan_for(task_id, state["episode_id"], state["state_version"], plan_id, grasp_x)
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan})
    if status != 200:
        return {"label": run_label, "ok": False, "error": f"validate {status}: {validated}"}
    started = time.time()
    status, executed = request(
        "POST",
        f"/api/v1/tasks/{task_id}/execute",
        {"simulation_id": simulation_id, "plan_hash": validated["plan_hash"]},
    )
    if status != 202:
        return {"label": run_label, "ok": False, "error": f"execute {status}: {executed}"}
    job = wait_job(executed["job_id"], 240.0)
    wall = time.time() - started
    final_state = state_of(simulation_id)
    ok, details = success_of(final_state, job)
    return {
        "label": run_label,
        "ok": ok,
        "job": {
            "job_id": job["job_id"],
            "status": job["status"],
            "error": job.get("error"),
        },
        "grasp_x": grasp_x,
        "block_before": block,
        "details": details,
        "wall_s": wall,
        "state": {
            "episode_id": final_state["episode_id"],
            "state_version": final_state["state_version"],
            "sim_time_s": final_state["state"]["sim_time_s"],
            "sim_step": final_state["state"]["sim_step"],
        },
    }


def run_recovery_stale(simulation_id: str, out: dict) -> dict:
    """Validate, move the block 4 mm, stale execute, then replan and succeed."""
    state = state_of(simulation_id)
    task_id = f"task_{uuid.uuid4().hex}"
    request("POST", "/api/v1/tasks", {"spec": task_spec(task_id)})
    plan = plan_for(task_id, state["episode_id"], state["state_version"], f"plan_{uuid.uuid4().hex}", 0.18)
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan})
    record: dict = {"case": "stale_after_object_move", "validate": status}
    if status != 200:
        return record
    move_status, move = request(
        "POST",
        f"/api/v1/simulations/{simulation_id}/skills",
        {
            "invocation": {
                "skill": "move_object",
                "parameters": {"object_id": "red_block", "position": [0.184, 0.0, 0.075]},
                "timeout_s": 30,
                "expected_duration_s": 5,
            }
        },
    )
    record["move_status"] = move_status
    if move_status == 202:
        record["move_job"] = wait_job(move["job_id"], 60.0)["status"]
    stale_status, stale = request(
        "POST",
        f"/api/v1/tasks/{task_id}/execute",
        {"simulation_id": simulation_id, "plan_hash": validated["plan_hash"]},
    )
    record["stale_status"] = stale_status
    record["stale_code"] = (stale.get("error") or {}).get("code")
    record["stale_detected"] = stale_status == 409 and record["stale_code"] == "STALE_WORLD_STATE"
    state = state_of(simulation_id)
    plan2 = plan_for(task_id, state["episode_id"], state["state_version"], f"plan_{uuid.uuid4().hex}", 0.18)
    status, validated2 = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan2})
    record["replan_validate"] = status
    if status == 200:
        status, executed = request(
            "POST",
            f"/api/v1/tasks/{task_id}/execute",
            {"simulation_id": simulation_id, "plan_hash": validated2["plan_hash"]},
        )
        record["replan_execute"] = status
        if status == 202:
            job = wait_job(executed["job_id"], 240.0)
            final = state_of(simulation_id)
            ok, details = success_of(final, job)
            record["replan_job"] = job["status"]
            record["replan_success"] = ok
            record["details"] = details
    return record


def run_recovery_wrong_variant(simulation_id: str, out: dict) -> dict:
    """Move the block to x=0.20, use grasp_x=0.18 first, then replan with 0.20."""
    record: dict = {"case": "first_grasp_wrong_variant"}
    status, reset = request("POST", f"/api/v1/simulations/{simulation_id}/reset")
    record["reset"] = status
    record["block_after_reset"] = block_of(state_of(simulation_id))
    status, move = request(
        "POST",
        f"/api/v1/simulations/{simulation_id}/skills",
        {
            "invocation": {
                "skill": "move_object",
                "parameters": {"object_id": "red_block", "position": [0.2, 0.0, 0.075]},
                "timeout_s": 30,
                "expected_duration_s": 5,
            }
        },
    )
    record["move_status"] = status
    if status == 202:
        record["move_job"] = wait_job(move["job_id"], 60.0)["status"]
    first = run_pick_plan(simulation_id, "recovery_first_grasp", grasp_x=0.18)
    record["first"] = first
    record["first_failed_with_grasp_missed"] = (
        first.get("job", {}).get("status") == "failed"
        and (first.get("job", {}).get("error") or {}).get("code") == "GRASP_MISSED"
    )
    second = run_pick_plan(simulation_id, "recovery_second_grasp", grasp_x=0.20)
    record["second"] = second
    record["recovered"] = bool(second.get("ok"))
    return record


def run_cancel(simulation_id: str) -> dict:
    request("POST", f"/api/v1/simulations/{simulation_id}/reset")
    state_before = state_of(simulation_id)
    status, job = request(
        "POST",
        f"/api/v1/simulations/{simulation_id}/skills",
        {
            "invocation": {
                "skill": "pick",
                "parameters": {"place_x": PLACE[0], "place_y": PLACE[1], "grasp_x": 0.18},
                "timeout_s": 120,
                "expected_duration_s": 60,
            }
        },
    )
    record = {"case": "cancel_pick", "submit_status": status, "job_id": job.get("job_id")}
    time.sleep(1.5)
    request("POST", f"/api/v1/jobs/{job['job_id']}/cancel")
    t0 = time.time()
    final = wait_job(job["job_id"], 120.0)
    record["ack_s"] = time.time() - t0
    record["status"] = final["status"]
    record["worker_ack_canceled"] = final["status"] == "canceled"
    state_after = state_of(simulation_id)
    record["episode_unchanged"] = state_after["episode_id"] == state_before["episode_id"]
    record["state_version_after"] = state_after["state_version"]
    record["block_after_cancel"] = block_of(state_after)
    return record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise SystemExit(f"output exists: {args.out}")
    report: dict = {"started_at": now(), "runs": [], "recoveries": [], "cancel": None}

    def save() -> None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    spec = {
        "profile_id": "profile_so101_physx",
        "backend_kind": "isaaclab_physx",
        "robot_id": "so101",
        "scene_id": "table_pick_place",
        "physics": {"engine": "physx", "dt": 0.005, "decimation": 1, "solver_iterations": 8},
        "renderer": {
            "enabled": True,
            "headless": True,
            "backend": "isaac_rtx",
            "cameras": [
                {"name": "front", "width": 640, "height": 480, "update_period_s": 0.1, "data_types": ["rgb", "depth"]}
            ],
        },
        "seed": 7,
        "num_envs": 1,
        "budget": {"max_sim_time_s": 1800.0, "max_wall_time_s": 3600.0},
    }
    status, started = request("POST", "/api/v1/simulations", {"spec": spec, "idempotency_key": "t09-eval"})
    report["start"] = {"status": status, "response": started}
    save()
    if status != 202:
        print(json.dumps({"error": f"start failed: {status} {started}"}))
        return 1
    start_job = wait_job(started["job_id"], 300.0)
    report["start_job"] = {"status": start_job["status"], "simulation_id": start_job.get("simulation_id")}
    simulation_id = start_job.get("simulation_id")
    if not simulation_id:
        print(json.dumps({"error": "no simulation id"}))
        return 1
    save()

    # camera frame on the first run
    frame_status, frame = request("POST", f"/api/v1/simulations/{simulation_id}/frames")
    report["frame"] = {"status": frame_status, "frame_id": frame.get("frame_id"), "rgb_url": frame.get("rgb_url")}
    save()

    successes = 0
    for index in range(args.runs):
        if index:
            reset_status, _ = request("POST", f"/api/v1/simulations/{simulation_id}/reset")
            report.setdefault("resets", []).append({"run": index + 1, "status": reset_status})
            save()
        entry = run_pick_plan(simulation_id, f"run_{index + 1}")
        report["runs"].append(entry)
        successes += 1 if entry.get("ok") else 0
        save()
        print(
            json.dumps(
                {
                    "run": index + 1,
                    "ok": entry.get("ok"),
                    "job": entry.get("job", {}).get("status"),
                    "error": (entry.get("job", {}).get("error") or {}).get("code"),
                    "wall_s": round(entry.get("wall_s", 0.0), 2),
                    "block": entry.get("details", {}).get("block"),
                }
            ),
            flush=True,
        )

    report["successes"] = successes
    report["runs_completed"] = len(report["runs"])
    report["pass_rate"] = successes / max(len(report["runs"]), 1)
    report["meets_target"] = successes >= 8
    save()

    report["recoveries"].append(run_recovery_stale(simulation_id, report))
    save()
    report["recoveries"].append(run_recovery_wrong_variant(simulation_id, report))
    save()
    report["cancel"] = run_cancel(simulation_id)
    save()

    status, closed = request("DELETE", f"/api/v1/simulations/{simulation_id}")
    report["close"] = {"status": status, "response": closed}
    report["finished_at"] = now()
    save()
    print(
        json.dumps(
            {
                "successes": successes,
                "runs": len(report["runs"]),
                "meets_target": report["meets_target"],
                "recoveries": [
                    {
                        "case": rec.get("case"),
                        "stale_detected": rec.get("stale_detected"),
                        "first_failed_with_grasp_missed": rec.get("first_failed_with_grasp_missed"),
                        "recovered": rec.get("recovered") or rec.get("replan_success"),
                    }
                    for rec in report["recoveries"]
                ],
                "cancel": report["cancel"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
