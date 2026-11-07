"""Run the Gateway: ``python -m robotics_gateway --port 8710``."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import uvicorn

from .app import create_app
from .endpoint import LoopbackEndpoint, SubprocessEndpoint
from .service import GatewayService
from .store import Store

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    parser = argparse.ArgumentParser(description="DSH Robotics Gateway")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8710)
    parser.add_argument("--db", type=Path, default=Path("artifacts/gateway.db"))
    parser.add_argument("--worker", choices=["loopback", "isaac"], default="loopback")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument(
        "--python",
        default=os.environ.get("DSH_ISAAC_PYTHON", sys.executable),
        help="Python interpreter used to launch Isaac workers (default: $DSH_ISAAC_PYTHON or this interpreter)",
    )
    args = parser.parse_args()

    args.db.parent.mkdir(parents=True, exist_ok=True)
    store = Store(args.db)
    if args.worker == "isaac":
        endpoint = SubprocessEndpoint(
            python=args.python,
            output_dir=args.db.parent / f"worker_{int(time.time())}",
            device=args.device,
        )
    else:
        endpoint = LoopbackEndpoint()
    service = GatewayService(store, endpoint)
    app = create_app(service)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
