"""Prove a Gateway can close a simulation and start another one (A1 follow-up).

Before the T09 A1 changes ``SubprocessEndpoint.close()`` latched a terminal
flag, so the next ``start_simulation`` was reported as ``WORKER_LOST`` even
though a new worker had been launched.  This script:

1. finds the newest open simulation, closes it (DELETE);
2. starts a fresh SO-101 simulation on the same Gateway and waits for it;
3. closes that one too and writes ``summary.json``.

Usage (Gateway already listening):

    python scripts/gateway_reuse_check.py --base http://127.0.0.1:8714 \
        --out artifacts/monitor_probe/reuse
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

SO101_SPEC = {
    "profile_id": "profile_so101_physx",
    "backend_kind": "isaaclab_physx",
    "robot_id": "so101",
    "scene_id": "table_pick_place",
    "physics": {"engine": "physx", "dt": 0.005, "decimation": 4, "solver_iterations": 8},
    "renderer": {
        "enabled": True,
        "headless": True,
        "backend": "isaac_rtx",
        "cameras": [
            {"name": "front", "width": 640, "height": 480, "update_period_s": 0.1, "data_types": ["rgb", "depth"]}
        ],
    },
    "seed": 13,
    "num_envs": 1,
    "budget": {"max_sim_time_s": 180.0, "max_wall_time_s": 420.0},
}


def request(base: str, path: str, method: str = "GET", body: dict | None = None, timeout: float = 120.0):
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["content-type"] = "application/json"
    req = urllib.request.Request(base + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()


def json_of(base: str, path: str, method: str = "GET", body: dict | None = None, timeout: float = 120.0) -> dict:
    status, raw = request(base, path, method, body, timeout)
    if status >= 400:
        raise RuntimeError(f"{method} {path} -> {status}: {raw[:300].decode('utf-8', 'replace')}")
    return json.loads(raw)


def wait_job(base: str, job_id: str, timeout_s: float = 300.0) -> tuple[dict, float]:
    started = time.time()
    while time.time() - started < timeout_s:
        job = json_of(base, f"/api/v1/jobs/{job_id}", timeout=30.0)
        if job["status"] in {"succeeded", "failed", "canceled"}:
            return job, time.time() - started
        time.sleep(2.0)
    return {"status": "timeout"}, time.time() - started


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="http://127.0.0.1:8714")
    parser.add_argument("--out", type=Path, default=Path("artifacts/monitor_probe/reuse"))
    args = parser.parse_args()
    base = args.base.rstrip("/")
    args.out.mkdir(parents=True, exist_ok=True)

    summary: dict = {"base": base, "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    health = json_of(base, "/api/v1/health")
    summary["health_before"] = health

    simulations = json_of(base, "/api/v1/simulations?limit=10")["simulations"]
    open_simulations = [item for item in simulations if not item["closed"]]
    if not open_simulations:
        raise SystemExit("no open simulation to close; start one first")
    closed_id = open_simulations[0]["simulation_id"]
    status, raw = request(base, f"/api/v1/simulations/{closed_id}", method="DELETE", timeout=60.0)
    summary["closed"] = {"simulation_id": closed_id, "status": status, "body": json.loads(raw) if raw else None}
    time.sleep(1.0)
    summary["health_after_close"] = json_of(base, "/api/v1/health")

    start = json_of(base, "/api/v1/simulations", "POST", {"spec": SO101_SPEC})
    job, elapsed = wait_job(base, start["job_id"])
    summary["restart"] = {"job_id": start["job_id"], "status": job.get("status"), "wall_s": round(elapsed, 2)}
    if job.get("status") != "succeeded":
        summary["restart"]["error"] = job.get("error")
        summary["verdict"] = {"reused": False}
    else:
        simulation_id = job["simulation_id"]
        summary["restart"]["simulation_id"] = simulation_id
        state = json_of(base, f"/api/v1/simulations/{simulation_id}/state")
        summary["restart"]["state"] = {
            "episode_id": state["episode_id"],
            "sim_step": state["sim_step"],
            "source": state["state"]["source"],
        }
        status, raw = request(base, f"/api/v1/simulations/{simulation_id}", method="DELETE", timeout=60.0)
        summary["restart"]["closed"] = {"status": status, "body": json.loads(raw) if raw else None}
        summary["verdict"] = {"reused": True, "worker_alive_during_restart": summary["health_after_close"]["worker"]}

    summary["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["verdict"].get("reused") else 1


if __name__ == "__main__":
    sys.exit(main())
