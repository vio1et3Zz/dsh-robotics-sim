"""Dump SO-101 palm/jaw visual point clouds in each body's local frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pxr import Gf, Usd, UsdGeom


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(args.usd))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    xforms = UsdGeom.XformCache(Usd.TimeCode.Default())

    report: dict = {}
    for body_name in ("gripper", "moving_jaw_so101_v1"):
        body = stage.GetPrimAtPath(f"/so101_new_calib/{body_name}")
        body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        inv_body = body_world.GetInverse()
        visuals = stage.GetPrimAtPath(f"/so101_new_calib/{body_name}/visuals")
        visuals_world = xforms.GetLocalToWorldTransform(visuals)
        prototype = visuals.GetPrototype()
        transformed: list[Gf.Vec3d] = []
        meshes: list[dict] = []
        if prototype:
            for mesh_prim in Usd.PrimRange(prototype):
                if not mesh_prim.IsA(UsdGeom.Mesh):
                    continue
                mesh = UsdGeom.Mesh(mesh_prim)
                mesh_points = mesh.GetPointsAttr().Get() or []
                mesh_world = xforms.GetLocalToWorldTransform(mesh_prim)
                to_body = inv_body * visuals_world * mesh_world
                meshes.append(
                    {
                        "path": str(mesh_prim.GetPath()),
                        "points": len(mesh_points),
                        "to_body_translation": [float(v) for v in to_body.ExtractTranslation()],
                    }
                )
                transformed.extend(to_body.Transform(Gf.Vec3d(v)) for v in mesh_points)
        if transformed:
            xs = [p[0] for p in transformed]
            ys = [p[1] for p in transformed]
            zs = [p[2] for p in transformed]
            bounds = {
                "min": [min(xs), min(ys), min(zs)],
                "max": [max(xs), max(ys), max(zs)],
            }
        else:
            bounds = None
        report[body_name] = {
            "prototype": str(prototype.GetPath()) if prototype else None,
            "meshes": meshes,
            "point_count": len(transformed),
            "points_local_body": [[round(float(v), 5) for v in p] for p in transformed],
            "bounds_local_body": bounds,
        }
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({name: {"points": entry["point_count"], "bounds": entry["bounds_local_body"], "meshes": entry["meshes"]} for name, entry in report.items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
