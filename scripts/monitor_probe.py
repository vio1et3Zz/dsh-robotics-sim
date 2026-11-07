"""Measure Agent-visibility during a running skill (checklist A1/A2/A3).

While one skill runs, the probe concurrently hits the same HTTP routes the DSH
plugin tools use:

* ``GET /state`` (``robotics_inspect_scene``) -- target p95 < 200 ms;
* ``POST /frames`` (``robotics_capture_frame``) -- target < 1 s and rising
  ``sim_step``;
* ``POST /jobs/{id}/cancel`` -- target ACK < 1 s;
* ``GET /health`` -- must not freeze while the skill runs;
* ``GET /jobs/{id}/events`` and ``/frames/latest`` -- A3 endpoints (skipped
  cleanly on a Gateway that predates them).

Run against a live Gateway started with ``--worker isaac``::

    python scripts/monitor_probe.py --base http://127.0.0.1:8714 \
        --skill hold_pose --parameters "{\"duration_s\": 20}" \
        --cancel-at 8 --out artifacts/monitor_probe/before

The baseline (pre-A1) result is the "before" directory: ``/state`` blocks until
the skill ends, so the samples are tens of seconds instead of milliseconds.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import threading
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
    "seed": 11,
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


def json_of(base: str, path: str, method: str = "GET", body: dict | None = None, timeout: float = 120.0) -> dict:
    status, raw, _ = request(base, path, method, body, timeout)
    if status >= 400:
        raise RuntimeError(f"{method} {path} -> {status}: {raw[:400].decode('utf-8', 'replace')}")
    return json.loads(raw)


def timed(
    base: str,
    path: str,
    method: str = "GET",
    body: dict | None = None,
    timeout: float = 120.0,
    intended: float | None = None,
):
    started = time.perf_counter()
    delay_ms = max(0.0, (started - intended) * 1000.0) if intended is not None else 0.0
    try:
        status, raw, _ = request(base, path, method, body, timeout)
    except Exception as error:  # noqa: BLE001 - recorded as a sample failure
        return {
            "ms": (time.perf_counter() - started) * 1000.0,
            "delay_ms": round(delay_ms, 1),
            "status": 0,
            "error": f"{type(error).__name__}: {error}",
        }
    elapsed = (time.perf_counter() - started) * 1000.0
    parsed = None
    if status < 400 and raw:
        try:
            parsed = json.loads(raw)
        except ValueError:
            parsed = None
    error = None
    if status >= 400:
        try:
            error = json.loads(raw).get("error")
        except ValueError:
            error = raw[:200].decode("utf-8", "replace")
    return {"ms": elapsed, "delay_ms": round(delay_ms, 1), "status": status, "json": parsed, "error": error}


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


class Sampler(threading.Thread):
    """Poll one route at a fixed period and keep every latency sample."""

    def __init__(self, base: str, path: str, *, period_s: float, timeout_s: float, method: str = "GET", body=None):
        super().__init__(daemon=True)
        self.base = base
        self.path = path
        self.period_s = period_s
        self.timeout_s = timeout_s
        self.method = method
        self.body = body
        self.stop = threading.Event()
        self.samples: list[dict] = []

    def run(self) -> None:
        while not self.stop.is_set():
            started = time.time()
            result = timed(self.base, self.path, self.method, self.body, self.timeout_s)
            result["t"] = round(started - self.t0, 3)
            data = result.get("json") or {}
            if "sim_step" in data:
                result["sim_step"] = data["sim_step"]
            if "state_version" in data:
                result["state_version"] = data["state_version"]
            if "episode_id" in data:
                result["episode_id"] = data["episode_id"]
            if "frame_id" in data:
                result["frame_id"] = data["frame_id"]
                result["buffered"] = data.get("buffered")
            joints = ((data.get("state") or {}).get("robots") or [{}])[0].get("joint_positions")
            if joints:
                result["joints_finite"] = all(math.isfinite(value) for value in joints)
            self.samples.append(result)
            if self.stop.wait(self.period_s):
                return

    def start_at(self, t0: float) -> None:
        self.t0 = t0
        self.start()


def summarise(samples: list[dict]) -> dict:
    latencies = [sample["ms"] for sample in samples]
    errors = [sample for sample in samples if sample["status"] == 0 or sample["status"] >= 400]
    return {
        "count": len(samples),
        "errors": len(errors),
        "p50_ms": round(percentile(latencies, 0.50), 1),
        "p95_ms": round(percentile(latencies, 0.95), 1),
        "max_ms": round(max(latencies), 1) if latencies else 0.0,
        "samples": [
            {key: value for key, value in sample.items() if key != "json"} for sample in samples
        ],
    }


def wait_terminal(base: str, job_id: str, timeout_s: float) -> tuple[dict, float]:
    started = time.perf_counter()
    deadline = started + timeout_s
    job: dict = {}
    while time.perf_counter() < deadline:
        status, raw, _ = request(base, f"/api/v1/jobs/{job_id}", timeout=30.0)
        if status == 200:
            job = json.loads(raw)
            if job.get("status") in {"succeeded", "failed", "canceled"}:
                return job, (time.perf_counter() - started) * 1000.0
        time.sleep(0.05)
    return job or {"status": "timeout"}, (time.perf_counter() - started) * 1000.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="http://127.0.0.1:8714")
    parser.add_argument("--out", type=Path, default=Path("artifacts/monitor_probe/run"))
    parser.add_argument("--skill", default="hold_pose")
    parser.add_argument("--parameters", default='{"duration_s": 20}')
    parser.add_argument("--state-period", type=float, default=0.5)
    parser.add_argument("--frame-period", type=float, default=2.0)
    parser.add_argument("--request-timeout", type=float, default=40.0)
    parser.add_argument("--cancel-at", type=float, default=-1.0, help="seconds after submit; <0 skips the cancel")
    parser.add_argument("--health-at", type=float, default=10.0, help="seconds after submit; <0 skips")
    parser.add_argument("--settle-timeout", type=float, default=120.0)
    parser.add_argument("--simulation-id", default=None, help="reuse an existing simulation instead of starting one")
    parser.add_argument("--reset", action="store_true", help="reset the episode before submitting the skill")
    parser.add_argument("--keep-open", action="store_true")
    args = parser.parse_args()
    base = args.base.rstrip("/")
    args.out.mkdir(parents=True, exist_ok=True)

    summary: dict = {"base": base, "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}

    simulation_id = args.simulation_id
    if simulation_id is None:
        start = json_of(base, "/api/v1/simulations", "POST", {"spec": SO101_SPEC})
        job, _ = wait_terminal(base, start["job_id"], 300.0)
        summary["startup"] = {"job_id": start["job_id"], "status": job.get("status"), "error": job.get("error")}
        if job.get("status") != "succeeded":
            (args.out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
            print(json.dumps(summary, indent=2, ensure_ascii=False))
            return 1
        simulation_id = job["simulation_id"]
    summary["simulation_id"] = simulation_id

    if args.reset:
        reset = json_of(base, f"/api/v1/simulations/{simulation_id}/reset", "POST")
        summary["reset"] = {"episode_id": reset.get("episode_id"), "status": "ok"}

    state_before = json_of(base, f"/api/v1/simulations/{simulation_id}/state")
    summary["state_before"] = {
        "episode_id": state_before["episode_id"],
        "state_version": state_before["state_version"],
        "sim_step": state_before["sim_step"],
    }

    parameters = json.loads(args.parameters)
    submitted = json_of(
        base,
        f"/api/v1/simulations/{simulation_id}/skills",
        "POST",
        {
            "invocation": {
                "skill": args.skill,
                "parameters": parameters,
                "target_object_id": "red_block",
                "frame": "world",
                "timeout_s": 120.0,
                "expected_duration_s": 30.0,
            }
        },
    )
    job_id = submitted["job_id"]
    t0 = time.time()
    summary["job_id"] = job_id
    summary["skill"] = {"name": args.skill, "parameters": parameters}

    state_sampler = Sampler(
        base, f"/api/v1/simulations/{simulation_id}/state", period_s=args.state_period, timeout_s=args.request_timeout
    )
    frame_sampler = Sampler(
        base,
        f"/api/v1/simulations/{simulation_id}/frames",
        period_s=args.frame_period,
        timeout_s=args.request_timeout,
        method="POST",
    )
    state_sampler.start_at(t0)
    frame_sampler.start_at(t0)

    health_sample = None
    if args.health_at >= 0:
        intended = t0 + args.health_at
        remaining = intended - time.time()
        if remaining > 0:
            time.sleep(remaining)
        health_sample = timed(base, "/api/v1/health", timeout=args.request_timeout, intended=intended)
        health_sample["t"] = round(time.time() - t0, 3)
    cancel_sample = None
    if args.cancel_at >= 0:
        intended = t0 + args.cancel_at
        remaining = intended - time.time()
        if remaining > 0:
            time.sleep(remaining)
        cancel_sample = timed(
            base, f"/api/v1/jobs/{job_id}/cancel", "POST", timeout=args.request_timeout, intended=intended
        )
        cancel_sample["t"] = round(time.time() - t0, 3)

    job, settle_ms = wait_terminal(base, job_id, args.settle_timeout)
    time.sleep(0.5)
    state_sampler.stop.set()
    frame_sampler.stop.set()
    state_sampler.join(timeout=5.0)
    frame_sampler.join(timeout=5.0)

    summary["state"] = summarise(state_sampler.samples)
    summary["frames"] = summarise(frame_sampler.samples)
    summary["health"] = health_sample
    summary["cancel"] = {
        "request": cancel_sample,
        "settled_ms": round(settle_ms, 1),
        "final_status": job.get("status"),
        "error": job.get("error"),
    }
    summary["job"] = {
        "status": job.get("status"),
        "progress": job.get("progress"),
        "error": job.get("error"),
    }

    steps = [sample.get("sim_step") for sample in frame_sampler.samples if "sim_step" in sample]
    summary["frame_sim_steps"] = steps
    summary["frame_sim_step_monotonic"] = all(b >= a for a, b in zip(steps, steps[1:])) if len(steps) > 1 else None

    # A3 endpoints: skip cleanly if this Gateway predates them.
    events_status, events_raw, _ = request(base, f"/api/v1/jobs/{job_id}/events?after_seq=0&limit=50", timeout=30.0)
    summary["events_endpoint"] = {"status": events_status}
    if events_status == 200:
        events = json.loads(events_raw)["events"]
        terminal = next(
            (record for record in events if record["event"]["type"] in {"succeeded", "failed", "canceled"}),
            None,
        )
        summary["events_endpoint"].update(
            {
                "count": len(events),
                "types": [event["event"]["type"] for event in events],
                "seq": [event["seq"] for event in events],
                "terminal": terminal["event"] if terminal else None,
            }
        )
    latest_status, latest_raw, _ = request(
        base, f"/api/v1/simulations/{simulation_id}/frames/latest", timeout=30.0
    )
    summary["latest_frame_endpoint"] = {"status": latest_status}
    if latest_status == 200:
        latest = json.loads(latest_raw)
        summary["latest_frame_endpoint"].update(
            {"frame_id": latest.get("frame_id"), "sim_step": latest.get("sim_step")}
        )

    if not args.keep_open:
        try:
            json_of(base, f"/api/v1/simulations/{simulation_id}", "DELETE")
            summary["closed"] = True
        except Exception as error:  # noqa: BLE001
            summary["closed"] = f"{type(error).__name__}: {error}"

    summary["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    cancel = summary["cancel"]["request"]
    summary["verdict"] = {
        "state_p95_under_200ms": summary["state"]["p95_ms"] < 200.0,
        "state_samples_present": summary["state"]["count"] > 0 and summary["state"]["errors"] == 0,
        "frame_max_under_1000ms": summary["frames"]["max_ms"] < 1000.0,
        "frames_increasing": summary["frame_sim_step_monotonic"] is True,
        "cancel_ack_under_1000ms": (
            cancel is not None and cancel["ms"] < 1000.0 and cancel.get("delay_ms", 0.0) < 500.0
        ),
        "cancel_final_canceled": (
            summary["cancel"]["final_status"] == "canceled" if args.cancel_at >= 0 else True
        ),
        "health_under_100ms": (
            health_sample is not None and health_sample["ms"] < 100.0 and health_sample.get("delay_ms", 0.0) < 500.0
        ),
    }
    summary["verdict"]["all_pass"] = all(summary["verdict"].values())

    (args.out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({key: summary[key] for key in ("state", "frames", "health", "cancel", "verdict")}, indent=2, ensure_ascii=False))
    (args.out / "summary.md").write_text(
        "\n".join(
            [
                f"# monitor probe ({summary['skill']['name']})",
                "",
                f"- state: p50 {summary['state']['p50_ms']} ms / p95 {summary['state']['p95_ms']} ms / max {summary['state']['max_ms']} ms ({summary['state']['count']} samples, {summary['state']['errors']} errors)",
                f"- frames: p50 {summary['frames']['p50_ms']} ms / p95 {summary['frames']['p95_ms']} ms / max {summary['frames']['max_ms']} ms ({summary['frames']['count']} samples, {summary['frames']['errors']} errors)",
                f"- health: {json.dumps(health_sample, ensure_ascii=False)}",
                f"- cancel: {json.dumps(summary['cancel'], ensure_ascii=False)}",
                f"- frame sim_step: {steps}",
                f"- events endpoint: {summary['events_endpoint']}",
                f"- terminal event: {json.dumps(summary['events_endpoint'].get('terminal'), ensure_ascii=False)}",
                f"- verdict: {json.dumps(summary['verdict'], ensure_ascii=False)}",
            ]
        ),
        encoding="utf-8",
    )
    return 0 if summary["verdict"]["all_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
