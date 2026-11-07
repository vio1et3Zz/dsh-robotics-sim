"""Compare SO-101 visual vs collision prototype geometry in body-local frames."""

from __future__ import annotations

import itertools
import json
from pathlib import Path

from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
USD = ROOT / "assets" / "isaac6" / "RobotStudio" / "so101_new_calib" / "so101_new_calib.usd"


def main() -> int:
    stage = Usd.Stage.Open(str(USD))
    xforms = UsdGeom.XformCache(Usd.TimeCode.Default())
    report = {}
    for body_name in ("gripper", "moving_jaw_so101_v1"):
        body = stage.GetPrimAtPath(f"/so101_new_calib/{body_name}")
        inv_body = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
        for kind in ("visuals", "collisions"):
            prim = stage.GetPrimAtPath(f"/so101_new_calib/{body_name}/{kind}")
            prototype = prim.GetPrototype()
            if prototype is None:
                report[f"{body_name}/{kind}"] = {"error": "no prototype"}
                continue
            meshes = []
            points: list[Gf.Vec3d] = []
            for candidate in Usd.PrimRange(prototype):
                if not candidate.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(candidate)
                values = mesh.GetPointsAttr().Get() or []
                mesh_world = xforms.GetLocalToWorldTransform(candidate)
                instance_world = xforms.GetLocalToWorldTransform(prim)
                to_body = inv_body * instance_world * mesh_world
                transformed = [to_body.Transform(Gf.Vec3d(v)) for v in values]
                points.extend(transformed)
                if transformed:
                    xs = [p[0] for p in transformed]
                    ys = [p[1] for p in transformed]
                    zs = [p[2] for p in transformed]
                    meshes.append(
                        {
                            "path": str(candidate.GetPath()),
                            "points": len(transformed),
                            "min": [min(xs), min(ys), min(zs)],
                            "max": [max(xs), max(ys), max(zs)],
                        }
                    )
            entry = {"mesh_count": len(meshes), "meshes": meshes}
            if points:
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                zs = [p[2] for p in points]
                entry["bounds"] = {"min": [min(xs), min(ys), min(zs)], "max": [max(xs), max(ys), max(zs)]}
            report[f"{body_name}/{kind}"] = entry
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
