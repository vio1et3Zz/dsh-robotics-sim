"""T02 smoke entry: launch Kit, build the Franka scene, run one calibration.

This is an executable smoke test, not a production server.  It exists to prove
the real SDK path (AppLauncher -> SimulationContext -> InteractiveScene ->
Articulation) and to collect physical-response evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import time
import traceback
import uuid
from datetime import datetime, timezone
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]
# Isaac Lab checkout used only for runtime provenance (git revision); override
# with DSH_ISAACLAB_ROOT when your checkout lives elsewhere.
ISAAC_LAB = Path(os.environ.get("DSH_ISAACLAB_ROOT", str(Path.home() / "isaac" / "IsaacLab")))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="T02 Franka PhysX worker smoke test")
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--joint", default="panda_joint1")
    parser.add_argument("--delta_rad", type=float, default=0.05)
    parser.add_argument("--hold_s", type=float, default=0.25)
    parser.add_argument("--settle_s", type=float, default=0.20)
    parser.add_argument("--dt", type=float, default=0.005)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"output directory already exists: {args.output_dir}")
    return args


def git(*arguments: str) -> str:
    result = subprocess.run(
        ["git", "-c", f"safe.directory={ISAAC_LAB.as_posix()}", "-C", str(ISAAC_LAB), *arguments],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def package_versions() -> dict[str, str]:
    import importlib.metadata as metadata

    versions = {}
    for name in ("isaaclab", "isaacsim", "isaaclab-physx", "torch", "numpy"):
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = "missing"
    return versions


def gpu_info() -> dict[str, object]:
    import torch

    name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu"
    memory_mib = int(torch.cuda.get_device_properties(0).total_memory / (1024 * 1024)) if torch.cuda.is_available() else 0
    driver = "unknown"
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            check=True,
        )
        driver = result.stdout.strip().splitlines()[0] or "unknown"
    except Exception:
        pass
    return {"name": name, "memory_mib": max(memory_mib, 1), "driver": driver}


def build_manifest(args: argparse.Namespace) -> object:
    from robotics_protocol import (
        GpuInfo,
        PythonRuntime,
        RuntimeManifest,
        SourceRevision,
        WorkerRuntime,
    )

    diff = git("diff")
    status = git("status", "--porcelain=v1")
    config = {
        "joint": args.joint,
        "delta_rad": args.delta_rad,
        "hold_s": args.hold_s,
        "settle_s": args.settle_s,
        "dt": args.dt,
        "device": args.device,
    }
    encoded = json.dumps(config, sort_keys=True, separators=(",", ":")).encode("utf-8")
    runtime_id = f"runtime_t02_{uuid.uuid4().hex[:12]}"
    from robotics_backends.isaaclab_physx.scene import FRANKA_PANDA_USD

    assets = {"franka.usd": hashlib.sha256(FRANKA_PANDA_USD.read_bytes()).hexdigest()}
    return RuntimeManifest(
        runtime_id=runtime_id,
        created_at=datetime.now(timezone.utc),
        adapter_version="0.2.0",
        python=PythonRuntime(
            version=platform.python_version(),
            executable=sys.executable,
            platform=platform.platform(),
        ),
        packages=package_versions(),
        source=SourceRevision(
            repo=str(ISAAC_LAB),
            commit=git("rev-parse", "HEAD"),
            describe=git("describe", "--tags", "--always", "--dirty"),
            dirty=bool(status),
            diff_sha256=hashlib.sha256(diff.encode("utf-8")).hexdigest() if status else None,
        ),
        config_sha256=hashlib.sha256(encoded).hexdigest(),
        assets=assets,
        gpu=GpuInfo(**gpu_info()),
        worker=WorkerRuntime(
            cwd=str(ROOT),
            log_dir=str(args.output_dir),
            num_envs=1,
            device=args.device,
        ),
    )


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, default=str) + "\n", encoding="utf-8")


class _Tee:
    """Mirror Python stdout/stderr into worker.log without breaking Kit."""

    def __init__(self, *streams: object) -> None:
        self._streams = streams

    def write(self, data: str) -> int:
        for stream in self._streams:
            try:
                stream.write(data)
                stream.flush()
            except Exception:
                pass
        return len(data)

    def flush(self) -> None:
        for stream in self._streams:
            try:
                stream.flush()
            except Exception:
                pass

    def __getattr__(self, name: str) -> object:
        return getattr(self._streams[0], name)


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True)
    started = time.time()
    result: dict[str, object] = {"status": "failed", "stage": "startup"}
    simulation_app = None
    original_stdout, original_stderr = sys.stdout, sys.stderr
    log_file = (args.output_dir / "worker.log").open("w", encoding="utf-8", buffering=1)
    sys.stdout = _Tee(original_stdout, log_file)
    sys.stderr = _Tee(original_stderr, log_file)
    try:
        print("[T02] launching Kit", flush=True)
        app_launcher = AppLauncher(args)
        simulation_app = app_launcher.app
        args.device = getattr(app_launcher, "device", None) or args.device or "cuda:0"
        print("[T02] app started; importing worker", flush=True)
        from robotics_backends.isaaclab_physx.scene import build_franka_scene
        from robotics_backends.isaaclab_physx.worker import IsaacLabWorker

        manifest = build_manifest(args)
        write_json(args.output_dir / "runtime_manifest.json", manifest.model_dump(mode="json"))
        print(f"[T02] runtime manifest: {manifest.runtime_id}", flush=True)

        t0 = time.time()
        sim, scene, robot = build_franka_scene(device=args.device, dt=args.dt)
        build_s = time.time() - t0
        worker = IsaacLabWorker(sim, scene, robot, device=args.device, dt=args.dt)
        print(f"[T02] scene ready in {build_s:.2f}s", flush=True)

        before = worker.observe()
        robot_spec = worker.robot_spec()
        print(
            f"[T02] robot: {len(robot_spec.joints)} joints, state_version={before.state_version}",
            flush=True,
        )

        # Invalid requests must fail before touching the simulator buffers.
        illegal_before = worker.observe()
        illegal_rejected = False
        try:
            worker.calibrate_joint(joint_name=args.joint, delta_rad=10.0)
        except ValueError:
            illegal_rejected = True
        unknown_joint_rejected = False
        try:
            worker.calibrate_joint(joint_name="not_a_joint")
        except ValueError:
            unknown_joint_rejected = True
        illegal_after = worker.observe()

        calibration = worker.calibrate_joint(
            joint_name=args.joint,
            delta_rad=args.delta_rad,
            hold_s=args.hold_s,
            settle_s=args.settle_s,
        )
        print(f"[T02] calibration: {json.dumps(calibration, ensure_ascii=False)}", flush=True)
        after = worker.observe()

        reset_state = worker.reset()
        worker.close()
        worker.close()  # idempotent

        checks = {
            "scene_ready": True,
            "joint_count_matches": before.robots[0].dof == len(before.robots[0].joint_positions),
            "state_finite": all(math.isfinite(value) for value in before.robots[0].joint_positions),
            "illegal_action_rejected": illegal_rejected,
            "unknown_joint_rejected": unknown_joint_rejected,
            "illegal_action_not_written": (
                illegal_after.sim_step == illegal_before.sim_step
                and illegal_after.state_version == illegal_before.state_version
            ),
            "calibration_passed": bool(calibration["passed"]),
            "state_version_advanced": after.state_version > before.state_version,
            "episode_changed_on_reset": reset_state.episode_id != before.episode_id,
            "reset_state_valid": reset_state.state_version > after.state_version,
            "close_idempotent": True,
        }
        status = "passed" if all(checks.values()) else "failed"
        result = {
            "status": status,
            "stage": "complete",
            "checks": checks,
            "timings_s": {"build_scene": build_s, "total": time.time() - started},
            "before": before.model_dump(mode="json"),
            "robot_spec": robot_spec.model_dump(mode="json"),
            "calibration": calibration,
            "after": after.model_dump(mode="json"),
            "reset": reset_state.model_dump(mode="json"),
            "manifest_path": str(args.output_dir / "runtime_manifest.json"),
        }
        print(f"[T02] status={status} checks={json.dumps(checks)}", flush=True)
    except Exception as error:
        result = {
            "status": "failed",
            "stage": "exception",
            "error": f"{type(error).__name__}: {error}",
            "traceback": traceback.format_exc(),
            "timings_s": {"total": time.time() - started},
        }
        print(result["traceback"], file=sys.stderr, flush=True)
    finally:
        try:
            write_json(args.output_dir / "result.json", result)
        finally:
            sys.stdout, sys.stderr = original_stdout, original_stderr
            log_file.close()
            if simulation_app is not None:
                simulation_app.close()
    return 0 if result.get("status") == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
