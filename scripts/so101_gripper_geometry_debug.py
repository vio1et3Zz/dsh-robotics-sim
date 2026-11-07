"""Debug: locate SO-101 jaw/palm collision geometry step by step."""

from __future__ import annotations

import argparse
import json
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app = AppLauncher(args).app
    out = args.output
    out.parent.mkdir(parents=True, exist_ok=True)

    def note(message: str, **extra) -> None:
        print(f"[probe] {message}", flush=True)
        payload = {"message": message, **extra}
        with out.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + "\n")

    try:
        note("app-ready")
        import omni.usd
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaaclab.utils import configclass
        from robotics_backends.isaaclab_physx.so101_scene import SO101_CFG

        @configclass
        class MinimalCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(
                prim_path="/World/ground",
                init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
                spawn=sim_utils.CuboidCfg(
                    size=(2.0, 2.0, 0.02),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                ),
            )
            robot = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        note("building-scene")
        device = getattr(app, "device", "cuda:0")
        sim = SimulationContext(SimulationCfg(device=device, dt=0.005))
        scene = InteractiveScene(MinimalCfg(num_envs=1, env_spacing=1.5))
        note("scene-created")
        sim.reset()
        note("sim-reset")
        robot = scene["robot"]
        note("robot-ready", bodies=list(robot.body_names), joints=list(robot.joint_names))

        stage = omni.usd.get_context().get_stage()
        note("stage-ready", default_prims=sum(1 for _ in stage.Traverse()))

        from pxr import Gf, UsdGeom, UsdPhysics

        rigid_names = [
            prim.GetName()
            for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and "/Robot/" in str(prim.GetPath())
        ]
        note("rigid-bodies", names=rigid_names)

        xform_cache = UsdGeom.XformCache()

        def body_prim(name: str):
            for prim in stage.Traverse():
                if prim.GetName() == name and "/Robot/" in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    return prim
            raise RuntimeError(name)

        for body_name in ("gripper", "moving_jaw_so101_v1"):
            prim = body_prim(body_name)
            note("body-prim", body=body_name, path=str(prim.GetPath()))
            body_xf = xform_cache.GetLocalToWorldTransform(prim)
            note("body-xf", body=body_name, translation=[float(v) for v in body_xf.ExtractTranslation()])
            inv_body = body_xf.GetInverse()
            count = 0
            primitive_types: list[str] = []
            for candidate in stage.Traverse():
                if not str(candidate.GetPath()).startswith(str(prim.GetPath()) + "/"):
                    continue
                if not candidate.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                primitive_types.append(candidate.GetTypeName())
                xf = UsdGeom.Xformable(candidate)
                local = inv_body * xf.GetLocalToWorldTransform(Usd.TimeCode.Default())
                if candidate.IsA(UsdGeom.Mesh):
                    points = UsdGeom.Mesh(candidate).GetPointsAttr().Get() or []
                    count += len(points)
                    if points:
                        sample = local.Transform(Gf.Vec3d(points[0]))
                        note("mesh-point", body=body_name, path=str(candidate.GetPath()), sample=[float(v) for v in sample])
                elif candidate.IsA(UsdGeom.Cube):
                    count += 1
                    note("cube", body=body_name, path=str(candidate.GetPath()))
                else:
                    note("other-collider", body=body_name, path=str(candidate.GetPath()), type=candidate.GetTypeName())
            note("body-collisions", body=body_name, colliders=len(primitive_types), points=count, types=sorted(set(primitive_types)))

        note("done")
    except Exception:
        note("exception", traceback=traceback.format_exc())
        raise
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
