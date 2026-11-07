"""Inspect SO-101 gripper/jaw local geometry to build a grasp frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics


def bbox_dict(box) -> dict:
    return {
        "min": [float(v) for v in box.GetMin()],
        "max": [float(v) for v in box.GetMax()],
        "size": [float(v) for v in box.GetSize()],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.usd))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    report = {"children": [], "boxes": {}}

    for path in ("/so101_new_calib/gripper", "/so101_new_calib/moving_jaw_so101_v1"):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        entry = {"path": path, "type": prim.GetTypeName(), "children": []}
        for child in prim.GetChildren():
            xform = UsdGeom.Xformable(child)
            matrix = xform.GetLocalTransformation() if xform else None
            entry["children"].append(
                {
                    "name": child.GetName(),
                    "type": child.GetTypeName(),
                    "local_translation": [float(v) for v in matrix.ExtractTranslation()] if matrix else None,
                }
            )
        report["children"].append(entry)
        world = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        report["boxes"][path] = {"world_rest": bbox_dict(world)}

    for joint_path in (
        "/so101_new_calib/joints/gripper",
    ):
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim:
            joint = UsdPhysics.Joint(joint_prim)
            rot0 = joint.GetLocalRot0Attr().Get()
            rot1 = joint.GetLocalRot1Attr().Get()
            report.setdefault("joints", []).append(
                {
                    "path": joint_path,
                    "local_pos0": [float(v) for v in joint.GetLocalPos0Attr().Get()],
                    "local_pos1": [float(v) for v in joint.GetLocalPos1Attr().Get()],
                    "local_rot0": [float(rot0.GetReal()), *[float(v) for v in rot0.GetImaginary()]],
                    "local_rot1": [float(rot1.GetReal()), *[float(v) for v in rot1.GetImaginary()]],
                }
            )

    args.out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
