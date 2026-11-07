"""Probe the T08 front camera in the real SO-101 object scene."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
import time
from pathlib import Path

import numpy as np
from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.enable_cameras = True
    # Warp's CUDA build needs an ASCII, writable cache path.  The temp dir is
    # ASCII; the fixed subdirectory lets later runs reuse the compiled cache.
    # Override with DSH_ROBOTICS_WARP_CACHE.
    cache_root = Path(
        os.environ.get("DSH_ROBOTICS_WARP_CACHE") or (Path(tempfile.gettempdir()) / "dsh_robotics_warp")
    )
    cache_root.mkdir(parents=True, exist_ok=True)
    os.environ["WARP_CACHE_PATH"] = str(cache_root)
    print(f"[camera] warp-cache={cache_root}", flush=True)
    print("[camera] app-launcher-start", flush=True)
    app = AppLauncher(args).app
    print("[camera] app-launcher-done", flush=True)
    report: dict = {"phase": "startup"}
    try:
        import torch
        from PIL import Image
        from robotics_backends.isaaclab_physx.so101_objects_scene import build_so101_object_scene

        def save() -> None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")

        def step(count: int) -> None:
            for _ in range(count):
                scene.write_data_to_sim()
                sim.step(render=False)
                scene.update(dt)

        report["phase"] = "building"
        save()
        dt = 0.005
        sim, scene, robot, block = build_so101_object_scene(device=getattr(app, "device", "cuda:0"), dt=dt, with_camera=True)
        camera = scene["front_camera"]
        report["camera_cfg"] = {
            "width": camera.cfg.width,
            "height": camera.cfg.height,
            "data_types": list(camera.cfg.data_types),
            "prim_path": camera.cfg.prim_path,
        }
        step(10)
        report["phase"] = "capturing"
        save()
        frames = []
        for index in range(3):
            start = time.perf_counter()
            camera.update(dt)
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - start
            rgb = camera.data.output["rgb"].torch[0]
            depth = camera.data.output["distance_to_image_plane"].torch[0]
            intrinsic = camera.data.intrinsic_matrices.torch[0]
            entry = {
                "index": index,
                "capture_s": elapsed,
                "rgb_shape": list(rgb.shape),
                "rgb_dtype": str(rgb.dtype),
                "rgb_min": float(rgb.float().min()),
                "rgb_max": float(rgb.float().max()),
                "depth_shape": list(depth.shape),
                "depth_dtype": str(depth.dtype),
                "depth_min": float(depth.min()),
                "depth_max": float(depth.max()),
                "intrinsic": [[float(v) for v in row] for row in intrinsic],
                "pos_w": [float(v) for v in camera.data.pos_w.torch[0]],
                "quat_w_world": [float(v) for v in camera.data.quat_w_world.torch[0]],
                "gpu_allocated_mib": float(torch.cuda.memory_allocated() / 2**20),
                "gpu_reserved_mib": float(torch.cuda.memory_reserved() / 2**20),
            }
            frames.append(entry)
            if index == 0:
                if rgb.dtype.is_floating_point:
                    rgb_u8 = (rgb.detach().to("cpu").clamp(0.0, 1.0) * 255.0).to(torch.uint8).numpy()
                else:
                    rgb_u8 = rgb.detach().to("cpu").to(torch.uint8).numpy()
                Image.fromarray(rgb_u8).save(args.output.parent / "camera_frame0.png")
                np.save(args.output.parent / "camera_depth0.npy", depth.detach().to("cpu").float().numpy())
        report["frames"] = frames
        report["phase"] = "done"
        save()
        print(json.dumps({"frames": frames[0], "capture_s": [f["capture_s"] for f in frames]}, indent=2))
    except Exception as exc:  # noqa: BLE001
        import traceback

        report["exception"] = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        raise
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
