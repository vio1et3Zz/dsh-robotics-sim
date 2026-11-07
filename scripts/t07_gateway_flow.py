"""T07 gateway flow: SO-101 start, task/plan, stale fault, recovery, grasp failure.

Run against a live Gateway (``python -m robotics_gateway --worker isaac``).
Every request/response and job is written to the output JSON for evidence.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path

BASE = "http://127.0.0.1:8714"
TASK_BUDGET = {"sim_time_s": 180.0, "wall_time_s": 240.0, "max_skill_calls": 6, "max_retries": 2}
PLACE = (0.22, 0.05)
TRANSFER_BUDGET = TASK_BUDGET


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def request(method: str, path: str, body: dict | None = None, timeout: float = 60.0) -> tuple[int, dict]:
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
        time.sleep(0.25)
    raise RuntimeError(f"job {job_id} did not finish in {timeout_s}s")


def run_skill(simulation_id: str, skill: str, parameters: dict, timeout_s: float = 240.0) -> dict:
    status, job = request(
        "POST",
        f"/api/v1/simulations/{simulation_id}/skills",
        {"invocation": {"skill": skill, "parameters": parameters, "timeout_s": timeout_s, "expected_duration_s": 60.0}},
    )
    if status != 202:
        return {"http_status": status, "response": job}
    finished = wait_job(job["job_id"], timeout_s + 30.0)
    return {"http_status": status, "job": finished}


def state_of(simulation_id: str) -> dict:
    status, payload = request("GET", f"/api/v1/simulations/{simulation_id}/state")
    if status != 200:
        raise RuntimeError(f"state failed: {status} {payload}")
    return payload


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
        "budgets": TASK_BUDGET,
        "constraints": ["top grasp", "place on the target box top face"],
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
        "budget": TRANSFER_BUDGET,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise SystemExit(f"output exists: {args.out}")
    report: dict = {"started_at": now(), "events": []}

    def record(label: str, payload: dict) -> None:
        report["events"].append({"at": now(), "label": label, "payload": payload})
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    spec = {
        "profile_id": "profile_so101_physx",
        "backend_kind": "isaaclab_physx",
        "robot_id": "so101",
        "scene_id": "table_pick_place",
        "physics": {"engine": "physx", "dt": 0.005, "decimation": 1, "solver_iterations": 8},
        "renderer": {"enabled": False, "headless": True},
        "seed": 7,
        "num_envs": 1,
        "budget": {"max_sim_time_s": 300.0, "max_wall_time_s": 300.0},
    }
    status, started = request("POST", "/api/v1/simulations", {"spec": spec, "idempotency_key": "t07-start"})
    record("start_simulation", {"status": status, "response": started})
    if status != 202:
        return 1
    start_job = wait_job(started["job_id"], 300.0)
    record("start_job", start_job)
    simulation_id = start_job.get("simulation_id")
    if not simulation_id:
        return 1
    state = state_of(simulation_id)
    record("state_initial", state)

    # 1) validate a plan, then move the block: executing the old plan must be stale
    task_id = f"task_{uuid.uuid4().hex}"
    status, created = request("POST", "/api/v1/tasks", {"spec": task_spec(task_id)})
    record("create_task", {"status": status, "response": created})
    plan1 = plan_for(task_id, state["episode_id"], state["state_version"], f"plan_{uuid.uuid4().hex}", 0.18)
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan1})
    record("validate_plan_1", {"status": status, "response": validated})
    plan_hash = validated.get("plan_hash")

    move = run_skill(simulation_id, "move_object", {"object_id": "red_block", "position": [0.184, 0.0, 0.075]})
    record("fault_move_block", move)
    stale_status, stale = request(
        "POST",
        f"/api/v1/tasks/{task_id}/execute",
        {"simulation_id": simulation_id, "plan_hash": plan_hash},
    )
    record("execute_stale_plan", {"status": stale_status, "response": stale})

    # 2) re-observe, re-plan with the observed state and execute
    state = state_of(simulation_id)
    record("state_after_move", state)
    plan2 = plan_for(task_id, state["episode_id"], state["state_version"], f"plan_{uuid.uuid4().hex}", 0.18)
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan2})
    record("validate_plan_2", {"status": status, "response": validated})
    plan_hash2 = validated.get("plan_hash")
    status, executed = request(
        "POST",
        f"/api/v1/tasks/{task_id}/execute",
        {"simulation_id": simulation_id, "plan_hash": plan_hash2},
    )
    record("execute_plan_2", {"status": status, "response": executed})
    task_job = wait_job(executed["job_id"], 300.0)
    record("task_job_2", task_job)
    state = state_of(simulation_id)
    record("state_after_plan_2", state)
    block_after = next(obj for obj in state["state"]["objects"] if obj["object_id"] == "red_block")["pose"]["position"]
    report["success_after_recovery"] = (
        task_job["status"] == "succeeded"
        and abs(block_after[0] - PLACE[0]) <= 0.05
        and abs(block_after[1] - PLACE[1]) <= 0.05
        and block_after[2] <= 0.09
    )

    # 3) reset, move the block to x=0.20, plan a first grasp with the wrong variant
    status, reset = request("POST", f"/api/v1/simulations/{simulation_id}/reset")
    record("reset", {"status": status, "response": reset})
    state = state_of(simulation_id)
    record("state_after_reset", state)
    move = run_skill(simulation_id, "move_object", {"object_id": "red_block", "position": [0.20, 0.0, 0.075]})
    record("fault_move_block_2", move)
    state = state_of(simulation_id)
    record("state_after_move_2", state)
    plan3 = plan_for(task_id, state["episode_id"], state["state_version"], f"plan_{uuid.uuid4().hex}", 0.18)
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan3})
    record("validate_plan_3_wrong_variant", {"status": status, "response": validated})
    status, executed = request(
        "POST",
        f"/api/v1/tasks/{task_id}/execute",
        {"simulation_id": simulation_id, "plan_hash": validated.get("plan_hash")},
    )
    record("execute_plan_3_wrong_variant", {"status": status, "response": executed})
    failed_job = wait_job(executed["job_id"], 300.0)
    record("task_job_3", failed_job)

    # 4) re-observe and retry with the observed variant
    state = state_of(simulation_id)
    record("state_after_failure", state)
    plan4 = plan_for(task_id, state["episode_id"], state["state_version"], f"plan_{uuid.uuid4().hex}", 0.20)
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan4})
    record("validate_plan_4", {"status": status, "response": validated})
    status, executed = request(
        "POST",
        f"/api/v1/tasks/{task_id}/execute",
        {"simulation_id": simulation_id, "plan_hash": validated.get("plan_hash")},
    )
    record("execute_plan_4", {"status": status, "response": executed})
    task_job = wait_job(executed["job_id"], 300.0)
    record("task_job_4", task_job)
    state = state_of(simulation_id)
    record("state_final", state)
    block_final = next(obj for obj in state["state"]["objects"] if obj["object_id"] == "red_block")["pose"]["position"]
    report["failure_then_recovery"] = (
        failed_job["status"] == "failed"
        and task_job["status"] == "succeeded"
        and abs(block_final[0] - PLACE[0]) <= 0.05
        and abs(block_final[1] - PLACE[1]) <= 0.05
        and block_final[2] <= 0.09
    )
    report["stale_detected"] = stale_status == 409 and (stale.get("error") or {}).get("code") == "STALE_WORLD_STATE"
    report["task_3_failure"] = (failed_job.get("error") or {}).get("code")
    report["block_final"] = block_final
    record("summary", {k: report[k] for k in ("stale_detected", "task_3_failure", "success_after_recovery", "failure_then_recovery", "block_final")})

    status, closed = request("DELETE", f"/api/v1/simulations/{simulation_id}")
    record("close", {"status": status, "response": closed})
    print(json.dumps({k: report[k] for k in ("stale_detected", "task_3_failure", "success_after_recovery", "failure_then_recovery", "block_final")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


