"""T03 real end-to-end: start Franka via Gateway, cancel a running skill, close."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import requests


def poll_job(base: str, job_id: str, timeout_s: float) -> dict:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        response = requests.get(f"{base}/api/v1/jobs/{job_id}", timeout=10)
        response.raise_for_status()
        job = response.json()
        if job["status"] in {"succeeded", "failed", "canceled"}:
            return job
        time.sleep(0.05)
    raise TimeoutError(f"job {job_id} did not settle within {timeout_s}s")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="http://127.0.0.1:8712")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    spec = {
        "profile_id": "profile_dev_franka_physx",
        "backend_kind": "isaaclab_physx",
        "robot_id": "franka",
        "scene_id": "table_top",
        "physics": {"engine": "physx", "dt": 0.005, "decimation": 4, "solver_iterations": 8},
        "renderer": {"enabled": False, "headless": True},
        "seed": 11,
        "num_envs": 1,
        "budget": {"max_sim_time_s": 120.0, "max_wall_time_s": 300.0},
    }
    started = time.time()
    health = requests.get(f"{args.base}/api/v1/health", timeout=5).json()

    response = requests.post(
        f"{args.base}/api/v1/simulations",
        json={"spec": spec, "idempotency_key": "t03-real-start"},
        timeout=10,
    )
    response.raise_for_status()
    start_job = response.json()
    start_job = poll_job(args.base, start_job["job_id"], timeout_s=300.0)
    if start_job["status"] != "succeeded":
        raise RuntimeError(f"start failed: {start_job}")
    health_after_start = requests.get(f"{args.base}/api/v1/health", timeout=5).json()
    backends = requests.get(f"{args.base}/api/v1/backends", timeout=5).json()

    state_response = requests.get(f"{args.base}/api/v1/jobs/{start_job['job_id']}", timeout=5)
    state_response.raise_for_status()
    simulation_id = state_response.json().get("simulation_id")
    if simulation_id is None:
        raise RuntimeError("start job did not record a simulation id")

    state_response = requests.get(f"{args.base}/api/v1/simulations/{simulation_id}/state", timeout=10)
    state_response.raise_for_status()
    state_before = state_response.json()

    invocation = {
        "skill": "calibrate_joint",
        "parameters": {"joint_name": "panda_joint1", "delta_rad": 0.10, "hold_s": 1.5, "settle_s": 0.2},
        "frame": "world",
        "timeout_s": 5.0,
        "expected_duration_s": 1.5,
    }
    response = requests.post(
        f"{args.base}/api/v1/simulations/{simulation_id}/skills",
        json={"invocation": invocation, "idempotency_key": "t03-real-calibrate"},
        timeout=10,
    )
    response.raise_for_status()
    job = response.json()

    deadline = time.time() + 10.0
    while time.time() < deadline:
        current = requests.get(f"{args.base}/api/v1/jobs/{job['job_id']}", timeout=5).json()
        if current["status"] == "running":
            break
        time.sleep(0.02)
    time.sleep(0.15)

    cancel_started = time.time()
    cancel_response = requests.post(f"{args.base}/api/v1/jobs/{job['job_id']}/cancel", timeout=10)
    cancel_response.raise_for_status()
    settled = poll_job(args.base, job["job_id"], timeout_s=10.0)
    cancel_elapsed = time.time() - cancel_started

    duplicate = requests.post(f"{args.base}/api/v1/jobs/{job['job_id']}/cancel", timeout=10)
    duplicate.raise_for_status()

    state_after = requests.get(f"{args.base}/api/v1/simulations/{simulation_id}/state", timeout=10).json()
    close_response = requests.delete(f"{args.base}/api/v1/simulations/{simulation_id}", timeout=30)

    summary = {
        "status": "passed"
        if (
            settled["status"] == "canceled"
            and cancel_elapsed < 1.0
            and close_response.status_code == 202
            and state_after["state_version"] >= state_before["state_version"]
        )
        else "failed",
        "health": health,
        "health_after_start": health_after_start,
        "backends": backends,
        "simulation_id": simulation_id,
        "start_job": start_job,
        "state_before": {"episode_id": state_before["episode_id"], "state_version": state_before["state_version"]},
        "skill_job": job,
        "settled_job": settled,
        "cancel_elapsed_s": cancel_elapsed,
        "duplicate_cancel_status": duplicate.json()["status"],
        "state_after": {"episode_id": state_after["episode_id"], "state_version": state_after["state_version"]},
        "close_status": close_response.status_code,
        "total_s": time.time() - started,
    }
    args.output.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
