"""Inspect the raw SO-101 USD gripper link tree (pxr only, no Kit)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(args.usd))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    report = {"bodies": {}}
    for body_name in ("gripper", "moving_jaw_so101_v1"):
        prim = stage.GetPrimAtPath(f"/so101_new_calib/{body_name}")
        entry: dict = {"path": str(prim.GetPath()), "children": [], "collision_prims": []}
        world = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        entry["world_aabb"] = {
            "min": [float(v) for v in world.GetMin()],
            "max": [float(v) for v in world.GetMax()],
        }
        for child in Usd.PrimRange(prim):
            if child == prim:
                continue
            row = {
                "path": str(child.GetPath()),
                "name": child.GetName(),
                "type": child.GetTypeName(),
                "applied_schemas": list(child.GetAppliedSchemas()),
                "has_collision_api": bool(child.HasAPI(UsdPhysics.CollisionAPI)),
                "has_rigid_api": bool(child.HasAPI(UsdPhysics.RigidBodyAPI)),
            }
            if child.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(child)
                points = mesh.GetPointsAttr().Get() or []
                row["points"] = len(points)
                bound = cache.ComputeWorldBound(child).ComputeAlignedRange()
                row["mesh_world_aabb"] = {
                    "min": [float(v) for v in bound.GetMin()],
                    "max": [float(v) for v in bound.GetMax()],
                }
            entry["children"].append(row)
            if child.HasAPI(UsdPhysics.CollisionAPI):
                entry["collision_prims"].append(str(child.GetPath()))
        report["bodies"][body_name] = entry
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({name: len(entry["children"]) for name, entry in report["bodies"].items()}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
