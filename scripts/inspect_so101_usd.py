"""Inspect the mirrored SO-ARM101 USD: joints, limits, drives and bodies."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdPhysics


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.usd))
    joints = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        entry = {
            "path": str(prim.GetPath()),
            "name": prim.GetName(),
            "type": prim.GetTypeName(),
            "body0": [str(p) for p in UsdPhysics.Joint(prim).GetBody0Rel().GetTargets()],
            "body1": [str(p) for p in UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()],
        }
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(prim)
            entry["axis"] = joint.GetAxisAttr().Get()
            entry["lower"] = joint.GetLowerLimitAttr().Get()
            entry["upper"] = joint.GetUpperLimitAttr().Get()
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            joint = UsdPhysics.PrismaticJoint(prim)
            entry["axis"] = joint.GetAxisAttr().Get()
            entry["lower"] = joint.GetLowerLimitAttr().Get()
            entry["upper"] = joint.GetUpperLimitAttr().Get()
            drive = UsdPhysics.DriveAPI.Get(prim, "linear")
        else:
            drive = None
        if drive:
            entry["drive"] = {
                "stiffness": drive.GetStiffnessAttr().Get(),
                "damping": drive.GetDampingAttr().Get(),
                "target_position": drive.GetTargetPositionAttr().Get(),
                "target_velocity": drive.GetTargetVelocityAttr().Get(),
                "max_force": drive.GetMaxForceAttr().Get(),
            }
        joints.append(entry)

    bodies = [
        {"path": str(prim.GetPath()), "name": prim.GetName(), "type": prim.GetTypeName()}
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    articulation_roots = [
        str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    layer = Sdf.Layer.FindOrOpen(str(args.usd))
    report = {
        "usd": str(args.usd),
        "sha256": sha256(args.usd),
        "sub_layers": list(layer.subLayerPaths) if layer else [],
        "composition_assets": list(layer.GetCompositionAssetDependencies()) if layer else [],
        "joints": joints,
        "bodies": bodies,
        "articulation_roots": articulation_roots,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
    }
    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"joints": len(joints), "bodies": len(bodies), "roots": articulation_roots}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
