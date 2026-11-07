"""T09 clean-restart spot check: one task/plan pick against a live simulation."""

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
PLACE = (0.22, 0.05)


def request(method: str, path: str, body: dict | None = None, timeout: float = 60.0) -> tuple[int, dict]:
    data = json.dumps(body).encode("utf-8") if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        raw = error.read().decode("utf-8")
        try:
            return error.code, json.loads(raw)
        except Exception:
            return error.code, {"raw": raw}


def wait_job(job_id: str, timeout_s: float = 240.0) -> dict:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        _, job = request("GET", f"/api/v1/jobs/{job_id}")
        if job.get("status") in {"succeeded", "failed", "canceled"}:
            return job
        time.sleep(0.2)
    raise RuntimeError("job timeout")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    _, state = request("GET", f"/api/v1/simulations/{args.sim}/state")
    episode = state["episode_id"]
    version = state["state_version"]
    task_id = f"task_{uuid.uuid4().hex}"
    plan_id = f"plan_{uuid.uuid4().hex}"
    spec = {
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
    status, created = request("POST", "/api/v1/tasks", {"spec": spec})
    if status != 201:
        print(json.dumps({"error": f"create_task {status}: {created}"}))
        return 1
    plan = {
        "plan_id": plan_id,
        "task_id": task_id,
        "episode_id": episode,
        "base_state_version": version,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "steps": [
            {"step_id": "step_observe", "skill": "observe_scene", "parameters": {}, "depends_on": [], "expected_duration_s": 1.0},
            {
                "step_id": "step_pick",
                "skill": "pick",
                "parameters": {"place_x": PLACE[0], "place_y": PLACE[1], "grasp_x": 0.18},
                "depends_on": ["step_observe"],
                "expected_duration_s": 60.0,
            },
        ],
        "budget": {"sim_time_s": 180.0, "wall_time_s": 240.0, "max_skill_calls": 6, "max_retries": 2},
    }
    status, validated = request("POST", f"/api/v1/tasks/{task_id}/plans/validate", {"plan": plan})
    if status != 200:
        print(json.dumps({"error": f"validate {status}: {validated}"}))
        return 1
    started = time.time()
    status, executed = request(
        "POST", f"/api/v1/tasks/{task_id}/execute", {"simulation_id": args.sim, "plan_hash": validated["plan_hash"]}
    )
    if status != 202:
        print(json.dumps({"error": f"execute {status}: {executed}"}))
        return 1
    job = wait_job(executed["job_id"], 240.0)
    wall = time.time() - started
    _, final = request("GET", f"/api/v1/simulations/{args.sim}/state")
    block = next(obj for obj in final["state"]["objects"] if obj["object_id"] == "red_block")["pose"]["position"]
    ok = (
        job["status"] == "succeeded"
        and ((block[0] - PLACE[0]) ** 2 + (block[1] - PLACE[1]) ** 2) ** 0.5 <= 0.05
        and block[2] <= 0.09
    )
    result = {
        "restart": {
            "sim": args.sim,
            "episode": final["episode_id"],
            "task": task_id,
            "plan_hash": validated["plan_hash"],
            "job": job["job_id"],
            "status": job["status"],
            "error": job.get("error"),
            "wall_s": wall,
            "block": block,
            "sim_time_s": final["state"]["sim_time_s"],
            "sim_step": final["state"]["sim_step"],
            "ok": ok,
        }
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result["restart"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
