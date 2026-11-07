"""T05 object-scene validation: table, block and target box load and settle."""

from __future__ import annotations

import argparse
import json
import math
import time
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="T05 SO-ARM101 object scene check")
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=0.005)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"output directory already exists: {args.output_dir}")
    return args


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True)
    started = time.time()
    result: dict = {"status": "failed", "stage": "startup"}
    app = None
    try:
        app = AppLauncher(args).app
        device = getattr(app, "device", None) or args.device or "cuda:0"

        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.utils import configclass
        from robotics_backends.isaaclab_physx.so101_scene import SO101_CFG

        @configclass
        class ObjectSceneCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(
                prim_path="/World/ground",
                init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
                spawn=sim_utils.CuboidCfg(
                    size=(2.0, 2.0, 0.02),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15)),
                ),
            )
            table = AssetBaseCfg(
                prim_path="/World/Table",
                init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.01)),
                spawn=sim_utils.CuboidCfg(
                    size=(0.6, 0.5, 0.02),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.35, 0.2)),
                ),
            )
            robot = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
            red_block = RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/RedBlock",
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.18, 0.0, 0.03),
                    lin_vel=(0.0, 0.0, 0.0),
                    ang_vel=(0.0, 0.0, 0.0),
                ),
                spawn=sim_utils.CuboidCfg(
                    size=(0.03, 0.03, 0.03),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                    mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
                    collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1)),
                ),
            )
            target_box = AssetBaseCfg(
                prim_path="{ENV_REGEX_NS}/TargetBox",
                init_state=AssetBaseCfg.InitialStateCfg(pos=(0.20, 0.10, 0.02)),
                spawn=sim_utils.CuboidCfg(
                    size=(0.10, 0.10, 0.04),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.3, 0.8)),
                ),
            )

        sim = SimulationContext(SimulationCfg(device=device, dt=args.dt))
        scene = InteractiveScene(ObjectSceneCfg(num_envs=1, env_spacing=1.5))
        sim.reset()
        block = scene["red_block"]
        box = scene["target_box"]
        for _ in range(300):
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(args.dt)
        block_pos = [float(v) for v in block.data.root_pos_w.torch[0].detach().to("cpu", torch.float64)]
        checks = {
            "block_above_table": block_pos[2] > 0.0,
            "block_finite": all(math.isfinite(v) for v in block_pos),
            "block_near_spawn": abs(block_pos[0] - 0.18) < 0.05 and abs(block_pos[1]) < 0.05,
            "box_present": box is not None,
        }
        result = {
            "status": "passed" if all(checks.values()) else "failed",
            "stage": "complete",
            "checks": checks,
            "block_pos_world": block_pos,
            "timings_s": {"total": time.time() - started},
        }
        (args.output_dir / "so101_objects.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps({"status": result["status"], "checks": checks}))
    except Exception as error:
        result = {"status": "failed", "error": f"{type(error).__name__}: {error}", "traceback": traceback.format_exc()}
        (args.output_dir / "so101_objects.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(traceback.format_exc())
    finally:
        if app is not None:
            app.close()
    return 0 if result.get("status") == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
