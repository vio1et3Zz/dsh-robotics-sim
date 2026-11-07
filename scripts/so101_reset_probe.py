"""Verify SO101Worker.reset restores the block and joints."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app = AppLauncher(args).app
    report: dict = {}
    try:
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene
        from robotics_backends.isaaclab_physx.so101_worker import SO101Worker

        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=0.005)
        worker = SO101Worker(sim, scene, robot, block, dt=0.005)
        report["initial"] = {
            "block": worker._block_pos(),
            "joints": worker._joints(),
        }
        move = worker.run_skill("move_object", {"object_id": "red_block", "position": [0.25, 0.0, 0.15]})
        report["after_move"] = {"result": move, "block": worker._block_pos()}
        state = worker.reset()
        report["after_reset"] = {
            "block": worker._block_pos(),
            "joints": worker._joints(),
            "episode_id": state.episode_id,
            "state_version": state.state_version,
        }
        # second cycle to ensure repeated reset works
        worker.run_skill("move_object", {"object_id": "red_block", "position": [0.23, 0.03, 0.12]})
        worker.reset()
        report["after_reset2"] = {"block": worker._block_pos(), "joints": worker._joints()}
        initial = report["initial"]["block"]
        reset1 = report["after_reset"]["block"]
        reset2 = report["after_reset2"]["block"]
        report["checks"] = {
            "reset_block_within_1mm": abs(reset1[0] - initial[0]) < 0.001
            and abs(reset1[1] - initial[1]) < 0.001
            and abs(reset1[2] - initial[2]) < 0.001,
            "reset2_block_within_1mm": abs(reset2[0] - initial[0]) < 0.001
            and abs(reset2[1] - initial[1]) < 0.001
            and abs(reset2[2] - initial[2]) < 0.001,
            "joints_near_default": max(abs(v) for v in report["after_reset"]["joints"]) < 0.08,
        }
        report["status"] = "passed" if all(report["checks"].values()) else "failed"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({"status": report["status"], "checks": report["checks"], "after_reset": report["after_reset"]}))
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
