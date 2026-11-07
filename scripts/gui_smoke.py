"""End-to-end smoke test for the Gateway console (GUI phase A).

Talks to a running Gateway over the same HTTP API the console uses:

* start ``profile_so101_physx`` and wait for the real camera worker;
* read the world state, capture one front frame, download RGB + depth;
* check the frame list, jobs list and static console assets;
* close the simulation and confirm the worker is released.

Usage (with a Gateway already listening, e.g. ``--worker isaac --port 8716``)::

    python scripts/gui_smoke.py --base http://127.0.0.1:8716 --out artifacts/gui_smoke

The script writes ``summary.json`` plus the downloaded frame files into ``--out``.
"""

from __future__ import annotations

import argparse
import hashlib
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
            {
                "name": "front",
                "width": 640,
                "height": 480,
                "update_period_s": 0.1,
                "data_types": ["rgb", "depth"],
            }
        ],
    },
    "seed": 7,
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
            return response.status, response.read(), dict(response.headers)
    except urllib.error.HTTPError as error:
        return error.code, error.read(), dict(error.headers)


def json_of(base: str, path: str, method: str = "GET", body: dict | None = None) -> dict:
    status, raw, _ = request(base, path, method, body)
    if status >= 400:
        raise RuntimeError(f"{method} {path} -> {status}: {raw[:500].decode('utf-8', 'replace')}")
    return json.loads(raw)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="http://127.0.0.1:8716")
    parser.add_argument("--out", type=Path, default=Path("artifacts/gui_smoke"))
    parser.add_argument("--timeout-s", type=float, default=300.0)
    parser.add_argument("--keep-open", action="store_true", help="leave the simulation running after the checks")
    args = parser.parse_args()
    base = args.base.rstrip("/")
    args.out.mkdir(parents=True, exist_ok=True)

    summary: dict = {"base": base, "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}

    health = json_of(base, "/api/v1/health")
    backends = json_of(base, "/api/v1/backends")
    summary["health_before"] = health
    summary["backends"] = backends

    started = time.time()
    start_job = json_of(base, "/api/v1/simulations", "POST", {"spec": SO101_SPEC})
    summary["start_job"] = start_job
    job = start_job
    while job["status"] not in {"succeeded", "failed", "canceled"}:
        if time.time() - started > args.timeout_s:
            raise SystemExit(f"start job did not settle within {args.timeout_s}s: {job}")
        time.sleep(2.0)
        job = json_of(base, f"/api/v1/jobs/{job['job_id']}")
    summary["startup_wall_s"] = round(time.time() - started, 2)
    if job["status"] != "succeeded":
        summary["start_error"] = job.get("error")
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return 1

    simulation_id = job["simulation_id"]
    summary["simulation_id"] = simulation_id

    simulations = json_of(base, "/api/v1/simulations?limit=5")["simulations"]
    summary["simulation_in_list"] = any(item["simulation_id"] == simulation_id for item in simulations)

    state = json_of(base, f"/api/v1/simulations/{simulation_id}/state")
    summary["state"] = {
        "episode_id": state["episode_id"],
        "state_version": state["state_version"],
        "sim_step": state["sim_step"],
        "source": state["state"]["source"],
        "dof": state["state"]["robots"][0]["dof"],
        "objects": [obj["object_id"] for obj in state["state"]["objects"]],
    }

    jobs = json_of(base, "/api/v1/jobs?limit=5")["jobs"]
    summary["jobs_list_ok"] = bool(jobs) and jobs[0]["job_id"] == start_job["job_id"]

    capture_started = time.time()
    frame = json_of(base, f"/api/v1/simulations/{simulation_id}/frames", "POST")
    summary["capture_wall_s"] = round(time.time() - capture_started, 2)
    summary["frame"] = frame

    rgb_status, rgb_bytes, rgb_headers = request(base, frame["rgb_url"], timeout=120.0)
    depth_status, depth_bytes, depth_headers = request(base, frame["depth_url"], timeout=120.0)
    (args.out / "frame_rgb.png").write_bytes(rgb_bytes)
    (args.out / "frame_depth.npy").write_bytes(depth_bytes)
    summary["rgb"] = {
        "status": rgb_status,
        "bytes": len(rgb_bytes),
        "sha256": hashlib.sha256(rgb_bytes).hexdigest(),
        "png": rgb_bytes[:8] == b"\x89PNG\r\n\x1a\n",
        "content_type": rgb_headers.get("content-type"),
    }
    summary["depth"] = {
        "status": depth_status,
        "bytes": len(depth_bytes),
        "sha256": hashlib.sha256(depth_bytes).hexdigest(),
        "npy": depth_bytes[:6] == b"\x93NUMPY",
        "content_type": depth_headers.get("content-type"),
    }

    frames = json_of(base, f"/api/v1/simulations/{simulation_id}/frames?limit=5")["frames"]
    summary["frame_in_list"] = any(item["frame_id"] == frame["frame_id"] for item in frames)
    summary["frames_list_count"] = len(frames)

    ui_status, ui_bytes, _ = request(base, "/ui", timeout=30.0)
    app_status, app_bytes, _ = request(base, "/ui/assets/app.js", timeout=30.0)
    summary["console"] = {
        "ui_status": ui_status,
        "ui_has_title": b"DSH Robotics" in ui_bytes,
        "ui_has_script": b"/ui/assets/app.js" in ui_bytes,
        "app_status": app_status,
        "app_has_connect": b"connectEvents" in app_bytes,
        "app_bytes": len(app_bytes),
    }

    if args.keep_open:
        summary["kept_open"] = True
    else:
        summary["close"] = json_of(base, f"/api/v1/simulations/{simulation_id}", "DELETE")
        time.sleep(1.0)
        summary["health_after_close"] = json_of(base, "/api/v1/health")
    summary["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    (args.out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
