"""Minimal object-scene bootstrap probe with progress markers."""

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
    markers: list[str] = []

    def note(message: str) -> None:
        markers.append(message)
        print(f"[bootstrap] {message}", flush=True)
        args.output.write_text(json.dumps({"markers": markers}), encoding="utf-8")

    try:
        note("app-ready")
        import torch
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        note("import-done")
        device = getattr(app, "device", "cuda:0")
        sim, scene, robot, block = build_so101_object_scene(device=device, dt=0.005)
        note("scene-built")
        note(f"bodies={list(robot.body_names)}")
        note(f"block={[float(v) for v in block.data.root_pos_w.torch[0].detach().to('cpu', torch.float64)]}")
        for _ in range(10):
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(0.005)
        note("stepped")
        with args.output.open("a", encoding="utf-8") as handle:
            handle.write("\n")
    finally:
        note("closing")
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
