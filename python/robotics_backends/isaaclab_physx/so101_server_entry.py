"""Run the real SO-ARM101 Worker as a long-lived server for the Gateway (T07)."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import tempfile
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DSH Robotics SO-ARM101 Worker server")
    parser.add_argument("--gateway-host", default="127.0.0.1")
    parser.add_argument("--gateway-port", type=int, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=0.005)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"output directory already exists: {args.output_dir}")
    return args


def connect(host: str, port: int, timeout_s: float = 60.0) -> socket.socket:
    deadline = time.time() + timeout_s
    last_error: Exception | None = None
    while time.time() < deadline:
        try:
            sock = socket.create_connection((host, port), timeout=10.0)
            sock.settimeout(None)
            return sock
        except OSError as error:
            last_error = error
            time.sleep(0.2)
    raise RuntimeError(f"cannot connect to gateway at {host}:{port}: {last_error}")


def main() -> int:
    args = parse_args()
    args.enable_cameras = True
    # Warp compiles the camera kernels once into an ASCII cache (Chinese/space
    # workspace paths break its CUDA build).  Set DSH_ROBOTICS_WARP_CACHE to
    # override; the default location is the precompiled development cache.
    default_cache = os.path.join(tempfile.gettempdir(), "dsh_robotics_warp")
    warp_cache = os.environ.get("DSH_ROBOTICS_WARP_CACHE") or default_cache
    if os.path.isdir(warp_cache):
        os.environ.setdefault("WARP_CACHE_PATH", warp_cache)
    args.output_dir.mkdir(parents=True)
    log_path = args.output_dir / "worker_server.log"
    simulation_app = None
    sock: socket.socket | None = None
    status = "failed"
    try:
        app_launcher = AppLauncher(args)
        simulation_app = app_launcher.app
        device = getattr(app_launcher, "device", None) or args.device or "cuda:0"

        from robotics_backends.isaaclab_physx.server import WorkerServer
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene
        from robotics_backends.isaaclab_physx.so101_worker import SO101Worker

        sim, scene, robot, block = build_so101_object_scene(device=device, dt=args.dt, with_camera=True)
        worker = SO101Worker(
            sim,
            scene,
            robot,
            block,
            dt=args.dt,
            camera=scene["front_camera"],
            output_dir=args.output_dir,
        )
        sock = connect(args.gateway_host, args.gateway_port)
        status = "running"
        server = WorkerServer(worker, sock)
        server.run()
        status = "closed"
    except Exception as error:
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(f"{datetime.now(timezone.utc).isoformat()} {type(error).__name__}: {error}\n")
            handle.write(traceback.format_exc() + "\n")
        print(traceback.format_exc(), file=sys.stderr, flush=True)
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
        (args.output_dir / "worker_server_status.json").write_text(
            json.dumps({"status": status, "at": datetime.now(timezone.utc).isoformat()}, indent=2),
            encoding="utf-8",
        )
        if simulation_app is not None:
            closer = threading.Thread(target=simulation_app.close, name="sim-close", daemon=True)
            closer.start()
            closer.join(10.0)
        os._exit(0 if status in {"closed", "running"} else 1)


if __name__ == "__main__":
    raise SystemExit(main())
