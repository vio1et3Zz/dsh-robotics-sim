"""Build the SO-ARM101 robot spec, joint mapping, asset manifest and scene."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from robotics_protocol import (  # noqa: E402
    ControlMode,
    EndEffectorSpec,
    GripperSpec,
    JointSpec,
    JointType,
    RobotAsset,
    RobotSpec,
)

SOURCE_PREFIX = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/6.0/Isaac/Robots/RobotStudio/so101_new_calib/"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    names = calibration["joint_names"]
    limits = calibration["joint_limits_rad"]
    defaults = calibration["default_joint_pos"]
    asset_dir = args.asset_dir

    main_usd = asset_dir / "so101_new_calib.usd"
    joints = []
    mapping = []
    for index, (name, pair, home) in enumerate(zip(names, limits, defaults)):
        joints.append(
            JointSpec(
                name=name,
                logical_name=name,
                type=JointType.REVOLUTE,
                unit="rad",
                lower=float(pair[0]),
                upper=float(pair[1]),
                home=float(home),
                velocity_limit=10.0,
            )
        )
        mapping.append(
            {
                "logical_name": name,
                "asset_name": name,
                "index": index,
                "type": "revolute",
                "axis": "Z",
                "unit": "rad",
                "lower": float(pair[0]),
                "upper": float(pair[1]),
                "home": float(home),
                "velocity_limit": 10.0,
                "direction": 1,
                "direction_evidence": "positive target produced positive measured delta",
            }
        )

    spec = RobotSpec(
        robot_id="so101",
        model="SO-ARM101",
        dof=len(joints),
        joints=joints,
        end_effector=EndEffectorSpec(name="moving_jaw_so101_v1", parent_joint="gripper"),
        gripper=GripperSpec(
            kind="single",
            joint_names=["gripper"],
            open_value=1.0,
            close_value=0.0,
            max_width_m=0.04,
        ),
        control_modes=[ControlMode.POSITION],
        skills=["observe_scene", "home", "move_joint", "gripper", "hold_pose"],
        asset=RobotAsset(
            usd_path=str(main_usd),
            sha256=sha256(main_usd),
            license="Apache-2.0",
            source=SOURCE_PREFIX,
        ),
    )

    files = []
    for path in sorted(asset_dir.rglob("*")):
        if not path.is_file() or ".thumbs" in path.parts:
            continue
        files.append(
            {
                "path": str(path.relative_to(asset_dir)).replace("\\", "/"),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )

    now = datetime.now(timezone.utc).isoformat()
    asset_manifest = {
        "robot_id": "so101",
        "source_prefix": SOURCE_PREFIX,
        "local_root": str(asset_dir.relative_to(ROOT)).replace("\\", "/"),
        "main_usd": str(main_usd.relative_to(asset_dir)).replace("\\", "/"),
        "license": "Apache-2.0",
        "license_file": "LICENSE",
        "downloaded_at": now,
        "files": files,
        "note": "Vendored because this host's omni.client cannot stat remote Nucleus assets.",
    }
    joint_mapping = {
        "robot_id": "so101",
        "asset_usd": str(main_usd),
        "ee_body": "moving_jaw_so101_v1",
        "joints": mapping,
        "gripper": {
            "joint": "gripper",
            "close_command_rad": 0.0,
            "open_command_rad": 1.0,
            "measured_closed_rad": calibration["gripper"]["closed_command_0"],
            "measured_open_rad": calibration["gripper"]["open_command_1"],
            "limits_rad": calibration["gripper"]["limits_rad"],
            "max_width_m_estimate": 0.04,
            "width_status": "estimated_pending_T06_measurement",
        },
        "notes": [
            "USD stores revolute limits in degrees; Isaac Lab reports radians.",
            "USD drive gains were replaced by project-owned ImplicitActuatorCfg gains.",
            "All arm joints move in the commanded positive direction; gravity sag is visible in shoulder_lift.",
        ],
    }
    scene = {
        "scene_id": "table_pick_place",
        "robot_id": "so101",
        "table": {"top_z_m": 0.0, "size_m": [0.6, 0.5, 0.02]},
        "workspace": {"radius_m": 0.30, "z_min_m": 0.0, "z_max_m": 0.35},
        "objects": [
            {
                "object_id": "red_block",
                "kind": "cube",
                "size_m": [0.03, 0.03, 0.03],
                "spawn_radius_m": [0.15, 0.25],
                "spawn_z_m": 0.015,
            },
            {
                "object_id": "target_box",
                "kind": "box",
                "size_m": [0.10, 0.10, 0.04],
                "position_m": [0.20, 0.10, 0.02],
            },
        ],
        "success": {
            "object_in_region": {"min": [0.15, 0.05, 0.0], "max": [0.25, 0.15, 0.06]},
            "hold_s": 1.0,
        },
        "note": "Dimensions are SO-101 scale, not Franka values.",
    }

    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    (out / "robot_spec.json").write_text(json.dumps(spec.model_dump(mode="json"), indent=2), encoding="utf-8")
    (out / "joint_mapping.json").write_text(json.dumps(joint_mapping, indent=2), encoding="utf-8")
    (out / "asset_manifest.json").write_text(json.dumps(asset_manifest, indent=2), encoding="utf-8")
    (out / "scene.json").write_text(json.dumps(scene, indent=2), encoding="utf-8")
    print(json.dumps({"joints": len(joints), "files": len(files), "out": str(out)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
