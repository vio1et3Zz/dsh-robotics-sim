"""B1/B2: rollout recorder + Agent evaluation runner.

Runs a task matrix against a live Gateway (the same HTTP surface the DSH tools
use), records one JSONL per episode (state samples, job events, frame ids,
final result and timing) and writes ``report.json`` / ``report.md`` next to
them::

    python scripts/agent_eval.py --base http://127.0.0.1:8714 \
        --matrix configs/agent_eval_tasks.json --out artifacts/agent_eval/baseline_pick_10

The simulation is started once and every episode begins with
``reset_simulation`` (never ``close``, which would release the worker).
During the skill the runner polls the A1 endpoints: ``GET /state`` (2 Hz),
``POST /frames`` (1 Hz, served from the monitor buffer) and
``GET /jobs/{id}/events`` (5 Hz).  Episodes can run ``setup`` skills first
(e.g. ``move_object``) and declare ``expected_failure`` for failure-taxonomy
runs.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path

PLACE = (0.22, 0.05)
SUCCESS_XY_M = 0.05
SUCCESS_MAX_Z = 0.09

PROFILE_SPECS = {
    "profile_so101_physx": {
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
        "num_envs": 1,
        "budget": {"max_sim_time_s": 1800.0, "max_wall_time_s": 3600.0},
    },
}

# B3 taxonomy: worker/Gateway error codes grouped for the report histogram.
FAILURE_CATEGORIES = {
    "GRASP_MISSED": "grasp_offset",
    "BLOCK_MOVED_ON_APPROACH": "grasp_offset",
    "EMPTY_GRASP": "grasp_empty",
    "GRASP_POSE_UNAVAILABLE": "grasp_pose_unavailable",
    "UNREACHABLE_TARGET": "unreachable",
    "OBJECT_OUT_OF_WORKSPACE": "unreachable",
    "OBJECT_NOT_FOUND": "scene",
    "STALE_WORLD_STATE": "stale",
    "STALE_PLAN": "stale",
    "WORKER_BUSY": "busy",
    "CONTROL_BUSY": "busy",
    "CANNOT_CANCEL": "invalid",
    "WORKER_LOST": "worker_lost",
    "POLICY_TIMEOUT": "timeout",
    "NO_PROGRESS": "timeout",
    "SKILL_FAILED": "skill_failed",
    "INVALID_REQUEST": "invalid",
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class Gateway:
    def __init__(self, base: str) -> None:
        self.base = base.rstrip("/")

    def request(self, method: str, path: str, body: dict | None = None, timeout: float = 120.0):
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(
            self.base + path,
            data=data,
            method=method,
            headers={"Content-Type": "application/json"} if data is not None else {},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as response:
                raw = response.read()
                return response.status, json.loads(raw) if raw else {}
        except urllib.error.HTTPError as error:
            raw = error.read()
            try:
                payload = json.loads(raw) if raw else {}
            except ValueError:
                payload = {"raw": raw[:300].decode("utf-8", "replace")}
            return error.code, payload

    def json(self, method: str, path: str, body: dict | None = None, timeout: float = 120.0) -> dict:
        status, payload = self.request(method, path, body, timeout)
        if status >= 400:
            raise RuntimeError(f"{method} {path} -> {status}: {json.dumps(payload)[:300]}")
        return payload

    def wait_job(self, job_id: str, timeout_s: float = 300.0) -> dict:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            status, job = self.request("GET", f"/api/v1/jobs/{job_id}", timeout=30.0)
            if status == 200 and job.get("status") in {"succeeded", "failed", "canceled"}:
                return job
            time.sleep(0.2)
        return {"status": "timeout", "job_id": job_id}


class Recorder:
    """One JSONL per episode; every line is a self-contained record."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = path.open("w", encoding="utf-8")
        self.records = 0

    def write(self, record: dict) -> None:
        self._file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.records += 1

    def close(self) -> None:
        self._file.close()


def block_of(state_payload: dict) -> list[float]:
    for obj in state_payload["state"]["objects"]:
        if obj["object_id"] == "red_block":
            return list(obj["pose"]["position"])
    raise KeyError("red_block missing from state")


def state_view(state_payload: dict) -> dict:
    state = state_payload["state"]
    robot = state["robots"][0]
    return {
        "sim_step": state["sim_step"],
        "sim_time_s": state["sim_time_s"],
        "state_version": state_payload["state_version"],
        "episode_id": state_payload["episode_id"],
        "joints": robot["joint_positions"],
        "ee": robot["ee_pose"]["position"],
        "gripper": robot["gripper"],
        "objects": {obj["object_id"]: obj["pose"]["position"] for obj in state["objects"]},
        "skill": (state.get("skills") or [{}])[0].get("phase") if state.get("skills") else None,
    }


def evaluate(state_payload: dict, job: dict) -> tuple[bool, dict]:
    block = block_of(state_payload)
    xy = math.dist(block[:2], PLACE)
    details = {
        "job_status": job.get("status"),
        "block": block,
        "xy_distance": round(xy, 5),
        "z": round(block[2], 5),
    }
    details["passed"] = bool(job.get("status") == "succeeded" and xy <= SUCCESS_XY_M and block[2] <= SUCCESS_MAX_Z)
    return details["passed"], details


def submit_skill(gw: Gateway, simulation_id: str, skill: str, parameters: dict, expected_s: float = 60.0) -> dict:
    return gw.json(
        "POST",
        f"/api/v1/simulations/{simulation_id}/skills",
        {
            "invocation": {
                "skill": skill,
                "parameters": parameters,
                "target_object_id": "red_block",
                "frame": "world",
                "timeout_s": max(expected_s * 2.0, 30.0),
                "expected_duration_s": expected_s,
            }
        },
    )


def run_episode(
    gw: Gateway,
    simulation_id: str,
    episode: dict,
    index: int,
    out_dir: Path,
    *,
    state_period: float,
    frame_period: float,
    job_timeout: float,
) -> dict:
    path = out_dir / f"episode_{index:02d}.jsonl"
    recorder = Recorder(path)
    started = time.time()
    t0 = started

    def write(record: dict) -> None:
        record.setdefault("t", round(time.time() - t0, 3))
        recorder.write(record)

    summary: dict = {
        "index": index,
        "task_id": episode.get("task_id"),
        "skill": episode["skill"],
        "parameters": episode.get("parameters") or {},
        "seed": episode.get("seed"),
        "expected_failure": episode.get("expected_failure"),
        "jsonl": str(path),
    }
    setup_runs: list[dict] = []
    try:
        reset = gw.json("POST", f"/api/v1/simulations/{simulation_id}/reset")
        write({"kind": "reset", "episode_id": reset["episode_id"], "state_version": reset["state"]["state_version"]})

        for setup in episode.get("setup") or []:
            job = submit_skill(gw, simulation_id, setup["skill"], setup.get("parameters") or {})
            settled = gw.wait_job(job["job_id"], timeout_s=60.0)
            setup_runs.append({"skill": setup["skill"], "job_id": job["job_id"], "status": settled["status"]})
            write({"kind": "setup", "job_id": job["job_id"], "skill": setup["skill"], "status": settled["status"]})

        state_before = gw.json("GET", f"/api/v1/simulations/{simulation_id}/state")
        write({"kind": "episode_start", **{k: summary[k] for k in ("index", "task_id", "skill", "parameters", "seed", "expected_failure")},
               "simulation_id": simulation_id, "episode_id": state_before["episode_id"],
               "state_version": state_before["state_version"]})
        write({"kind": "state", "phase": "before", **state_view(state_before)})
        submit_started = time.time()
        submitted = submit_skill(gw, simulation_id, episode["skill"], episode.get("parameters") or {})
        job_id = submitted["job_id"]
        summary["job_id"] = job_id
        write({"kind": "submit", "job_id": job_id, "worker_job_id": submitted.get("worker_job_id")})

        cursor = 0
        next_state = submit_started
        next_frame = submit_started
        frame_records: list[dict] = []
        event_records = 0
        job = {"status": "running"}
        while True:
            job = gw.json("GET", f"/api/v1/jobs/{job_id}")
            events = gw.json("GET", f"/api/v1/jobs/{job_id}/events?after_seq={cursor}&limit=200")
            for record in events["events"]:
                cursor = record["seq"]
                event_records += 1
                write({"kind": "event", "seq": record["seq"], "event": record["event"]})
            now = time.time()
            if now >= next_state:
                state = gw.json("GET", f"/api/v1/simulations/{simulation_id}/state")
                write({"kind": "state", "phase": "during", **state_view(state)})
                next_state = now + state_period
            if now >= next_frame:
                frame = gw.json("POST", f"/api/v1/simulations/{simulation_id}/frames", timeout=60.0)
                frame_records.append(
                    {"frame_id": frame["frame_id"], "sim_step": frame["sim_step"], "buffered": frame.get("buffered")}
                )
                write(
                    {
                        "kind": "frame",
                        "frame_id": frame["frame_id"],
                        "sim_step": frame["sim_step"],
                        "buffered": frame.get("buffered"),
                        "captured_at": frame.get("captured_at"),
                    }
                )
                next_frame = now + frame_period
            if job["status"] in {"succeeded", "failed", "canceled"}:
                break
            if time.time() - submit_started > job_timeout:
                job = gw.wait_job(job_id, timeout_s=1.0)
                if job.get("status") not in {"succeeded", "failed", "canceled"}:
                    job = {"status": "timeout", "job_id": job_id}
                break
            time.sleep(0.2)

        # Drain the tail of the event stream and record the final state / frame.
        while True:
            events = gw.json("GET", f"/api/v1/jobs/{job_id}/events?after_seq={cursor}&limit=200")
            if not events["events"]:
                break
            for record in events["events"]:
                cursor = record["seq"]
                event_records += 1
                write({"kind": "event", "seq": record["seq"], "event": record["event"]})
        final_state = gw.json("GET", f"/api/v1/simulations/{simulation_id}/state")
        write({"kind": "state", "phase": "after", **state_view(final_state)})
        final_frame = gw.json("POST", f"/api/v1/simulations/{simulation_id}/frames", timeout=60.0)
        frame_records.append(
            {"frame_id": final_frame["frame_id"], "sim_step": final_frame["sim_step"], "buffered": final_frame.get("buffered")}
        )
        write({"kind": "frame", "phase": "final", "frame_id": final_frame["frame_id"], "sim_step": final_frame["sim_step"],
               "buffered": final_frame.get("buffered")})

        passed, details = evaluate(final_state, job)
        error = job.get("error") or {}
        summary.update(
            {
                "status": job.get("status"),
                "error_code": error.get("code"),
                "passed": passed,
                "details": details,
                "wall_s": round(time.time() - submit_started, 2),
                "sim_step_end": final_state["sim_step"],
                "sim_time_s_end": final_state["state"]["sim_time_s"],
                "state_version_end": final_state["state_version"],
                "frames": len(frame_records),
                "frame_steps": [item["sim_step"] for item in frame_records],
                "events": event_records,
                "records": recorder.records + 1,
                "setup": setup_runs,
                "final_frame": final_frame["frame_id"],
            }
        )
        expected = episode.get("expected_failure")
        summary["expected_met"] = (summary["error_code"] == expected) if expected else passed
        write({"kind": "episode_end", **{k: summary[k] for k in ("index", "status", "error_code", "passed", "expected_failure", "expected_met", "wall_s", "sim_step_end", "frames", "events")}})
    except Exception as error:  # noqa: BLE001 - an episode failure must not kill the run
        summary.update({"status": "error", "error_code": type(error).__name__, "error": str(error), "passed": False})
        write({"kind": "episode_error", "error": f"{type(error).__name__}: {error}"})
    finally:
        recorder.close()
    summary["jsonl"] = str(path)
    return summary


def build_spec(matrix: dict) -> dict:
    profile_id = matrix["simulation"]["profile_id"]
    base = PROFILE_SPECS[profile_id]
    spec = {**base, "profile_id": profile_id, "seed": int(matrix["simulation"].get("seed", 7))}
    budget = matrix["simulation"].get("budget")
    if budget:
        spec["budget"] = budget
    return spec


def failure_histogram(episodes: list[dict]) -> dict:
    histogram: dict[str, int] = {}
    for episode in episodes:
        code = episode.get("error_code") or "OK"
        histogram[code] = histogram.get(code, 0) + 1
    return dict(sorted(histogram.items()))


def category_histogram(episodes: list[dict]) -> dict:
    categories: dict[str, int] = {}
    for episode in episodes:
        code = episode.get("error_code")
        if not code:
            continue
        category = FAILURE_CATEGORIES.get(code, "unclassified")
        categories[category] = categories.get(category, 0) + 1
    return dict(sorted(categories.items()))


def verify_run(out_dir: Path) -> dict:
    """Check a finished run: every JSONL is parseable and matches report.json."""
    report_path = out_dir / "report.json"
    if not report_path.is_file():
        return {"ok": False, "problems": [f"missing {report_path}"]}
    report = json.loads(report_path.read_text(encoding="utf-8"))
    problems: list[str] = []
    totals = {"lines": 0, "events": 0, "frames": 0, "states": 0}
    for episode in report["episodes"]:
        path = Path(episode["jsonl"])
        if not path.is_file():
            problems.append(f"missing {path}")
            continue
        records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        kinds: dict[str, int] = {}
        for record in records:
            kinds[record["kind"]] = kinds.get(record["kind"], 0) + 1
        event_seqs = [record["seq"] for record in records if record["kind"] == "event"]
        frame_steps = [record["sim_step"] for record in records if record["kind"] == "frame"]
        if kinds.get("episode_start") != 1 or kinds.get("episode_end") != 1:
            problems.append(f"{path.name}: expected one episode_start and one episode_end, got {kinds}")
        if event_seqs != sorted(event_seqs) or len(event_seqs) != len(set(event_seqs)):
            problems.append(f"{path.name}: event seq is not strictly increasing")
        if len(frame_steps) > 1 and not all(b >= a for a, b in zip(frame_steps, frame_steps[1:])):
            problems.append(f"{path.name}: frame sim_step decreased: {frame_steps}")
        if episode.get("events") != len(event_seqs):
            problems.append(f"{path.name}: report events {episode.get('events')} != jsonl {len(event_seqs)}")
        if episode.get("frames") != len(frame_steps):
            problems.append(f"{path.name}: report frames {episode.get('frames')} != jsonl {len(frame_steps)}")
        totals["lines"] += len(records)
        totals["events"] += len(event_seqs)
        totals["frames"] += len(frame_steps)
        totals["states"] += kinds.get("state", 0)
    summary = report.get("summary") or {}
    if summary.get("events") != totals["events"]:
        problems.append(f"summary events {summary.get('events')} != jsonl {totals['events']}")
    if summary.get("frames") != totals["frames"]:
        problems.append(f"summary frames {summary.get('frames')} != jsonl {totals['frames']}")
    if summary.get("passed") != sum(1 for item in report["episodes"] if item.get("passed")):
        problems.append("summary passed does not match the episode list")
    return {
        "ok": not problems,
        "run_id": report.get("run_id"),
        "episodes": len(report["episodes"]),
        "jsonl_lines": totals["lines"],
        "events": totals["events"],
        "frames": totals["frames"],
        "states": totals["states"],
        "problems": problems,
    }


def write_report(out_dir: Path, report: dict) -> None:
    (out_dir / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = report["summary"]
    lines = [
        f"# Agent eval — {report['matrix']['name']}",
        "",
        f"- run_id: `{report['run_id']}`",
        f"- simulation: `{report['simulation'].get('simulation_id')}` ({report['simulation'].get('profile_id')}, seed {report['simulation'].get('seed')})",
        f"- episodes: {summary['episodes']} / passed {summary['passed']} (pass_rate {summary['pass_rate']:.2f})",
        f"- wall: total {summary['wall_s']['total']} s, mean {summary['wall_s']['mean']} s, median {summary['wall_s']['median']} s",
        f"- frames {summary['frames']} / events {summary['events']}",
        f"- failure codes: {json.dumps(summary['failure_codes'], ensure_ascii=False)}",
        f"- failure categories: {json.dumps(summary['failure_categories'], ensure_ascii=False)}",
        "",
        "| # | task | status | code | passed | expected_met | wall s | frames | events |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for episode in report["episodes"]:
        lines.append(
            f"| {episode['index']} | {episode['task_id']} | {episode.get('status')} | {episode.get('error_code') or 'OK'} | "
            f"{episode.get('passed')} | {episode.get('expected_met')} | {episode.get('wall_s')} | "
            f"{episode.get('frames')} | {episode.get('events')} |"
        )
    lines += ["", f"- verdict: {json.dumps(report['verdict'], ensure_ascii=False)}", ""]
    (out_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default="http://127.0.0.1:8714")
    parser.add_argument("--matrix", type=Path, default=Path("configs/agent_eval_tasks.json"))
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--episodes", type=int, default=0, help="limit the number of episodes (0 = all)")
    parser.add_argument("--simulation-id", default=None, help="reuse a live simulation instead of starting one")
    parser.add_argument("--keep-open", action="store_true", help="do not close the simulation at the end")
    parser.add_argument("--state-period", type=float, default=0.5)
    parser.add_argument("--frame-period", type=float, default=1.0)
    parser.add_argument("--job-timeout", type=float, default=180.0)
    parser.add_argument("--reference", type=Path, default=None, help="previous report.json to compare against")
    parser.add_argument("--verify", type=Path, default=None, help="verify a finished run directory and exit")
    args = parser.parse_args()

    if args.verify is not None:
        result = verify_run(args.verify)
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0 if result["ok"] else 1

    matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
    run_id = args.out.name if args.out else f"run_{time.strftime('%Y%m%d_%H%M%S')}"
    out_dir = args.out or Path("artifacts/agent_eval") / run_id
    out_dir.mkdir(parents=True, exist_ok=True)
    gw = Gateway(args.base)

    report: dict = {
        "run_id": run_id,
        "matrix": {"name": matrix["name"], "path": str(args.matrix), "description": matrix.get("description")},
        "started_at": now_iso(),
        "simulation": {"profile_id": matrix["simulation"]["profile_id"], "seed": matrix["simulation"].get("seed")},
        "episodes": [],
    }

    startup_wall = 0.0
    simulation_id = args.simulation_id
    if simulation_id is None:
        started = time.time()
        start = gw.json("POST", "/api/v1/simulations", {"spec": build_spec(matrix), "idempotency_key": f"agent-eval-{uuid.uuid4().hex}"})
        job = gw.wait_job(start["job_id"], timeout_s=300.0)
        startup_wall = time.time() - started
        report["start"] = {"job_id": start["job_id"], "status": job.get("status"), "error": job.get("error"), "wall_s": round(startup_wall, 2)}
        if job.get("status") != "succeeded":
            report["verdict"] = {"ok": False, "reason": "simulation start failed"}
            write_report(out_dir, report)
            print(json.dumps(report["verdict"], indent=2))
            return 1
        simulation_id = job["simulation_id"]
    report["simulation"]["simulation_id"] = simulation_id
    report["simulation"]["startup_wall_s"] = round(startup_wall, 2)

    episodes = matrix["episodes"][: args.episodes] if args.episodes else matrix["episodes"]
    for index, episode in enumerate(episodes, start=1):
        result = run_episode(
            gw,
            simulation_id,
            episode,
            index,
            out_dir,
            state_period=args.state_period,
            frame_period=args.frame_period,
            job_timeout=args.job_timeout,
        )
        report["episodes"].append(result)
        print(
            json.dumps(
                {
                    "episode": index,
                    "task": result.get("task_id"),
                    "status": result.get("status"),
                    "code": result.get("error_code"),
                    "passed": result.get("passed"),
                    "expected_met": result.get("expected_met"),
                    "wall_s": result.get("wall_s"),
                    "sim_step": result.get("sim_step_end"),
                    "block": (result.get("details") or {}).get("block"),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        # Keep a partial report on disk in case the run is interrupted.
        (out_dir / "report.partial.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    passed = sum(1 for item in report["episodes"] if item.get("passed"))
    expected = sum(1 for item in report["episodes"] if item.get("expected_met"))
    walls = [float(item.get("wall_s") or 0.0) for item in report["episodes"]]
    report["summary"] = {
        "episodes": len(report["episodes"]),
        "passed": passed,
        "pass_rate": round(passed / max(len(report["episodes"]), 1), 4),
        "expected_met": expected,
        "failure_codes": failure_histogram(report["episodes"]),
        "failure_categories": category_histogram(report["episodes"]),
        "wall_s": {
            "total": round(sum(walls), 2),
            "mean": round(statistics.mean(walls), 2) if walls else 0.0,
            "median": round(statistics.median(walls), 2) if walls else 0.0,
        },
        "frames": sum(int(item.get("frames") or 0) for item in report["episodes"]),
        "events": sum(int(item.get("events") or 0) for item in report["episodes"]),
    }

    verdict: dict = {"ok": True}
    if matrix["name"] == "baseline_pick_10" and len(report["episodes"]) == 10:
        verdict["matches_t09"] = passed == 10
        if passed != 10:
            verdict["deviation"] = f"{passed}/10 vs T09 10/10; see report.md per-episode codes"
        verdict["note"] = "T09 used task/plan/execute; this runner calls /skills pick with the same physical scene."
    else:
        verdict["expected_met_all"] = expected == len(report["episodes"])
    report["verdict"] = verdict
    report["finished_at"] = now_iso()

    if args.reference and args.reference.is_file():
        reference = json.loads(args.reference.read_text(encoding="utf-8"))
        report["reference"] = {
            "run_id": reference.get("run_id"),
            "episodes": (reference.get("summary") or {}).get("episodes"),
            "passed": (reference.get("summary") or {}).get("passed"),
        }

    write_report(out_dir, report)

    if simulation_id and not args.keep_open:
        try:
            gw.json("DELETE", f"/api/v1/simulations/{simulation_id}")
            report["closed"] = True
            write_report(out_dir, report)
        except Exception as error:  # noqa: BLE001
            report["closed"] = f"{type(error).__name__}: {error}"
            write_report(out_dir, report)

    print(json.dumps({"summary": report["summary"], "verdict": report["verdict"], "out": str(out_dir)}, indent=2, ensure_ascii=False))
    return 0 if verdict.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
