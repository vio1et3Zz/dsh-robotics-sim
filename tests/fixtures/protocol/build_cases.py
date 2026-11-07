"""Build the shared valid/invalid protocol fixtures used by Python and Node.

Run from anywhere:
    python tests/fixtures/protocol/build_cases.py

The builder validates every payload with Pydantic before writing, so a fixture
cannot silently claim to be invalid when the model accepts it (or vice versa).
"""

from __future__ import annotations

import copy
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "python"))

from pydantic import ValidationError  # noqa: E402

from robotics_protocol import MODELS, compute_plan_hash  # noqa: E402
from robotics_protocol.tasks import PlanStep  # noqa: E402

HERE = Path(__file__).resolve().parent
NOW = "2026-11-07T00:30:00+08:00"
WALL = datetime.now(timezone.utc).isoformat()

COMMIT = "f" * 40
HASH_A = "a" * 64
HASH_B = "b" * 64
HASH_C = "c" * 64


def manifest() -> dict:
    return {
        "runtime_id": "runtime_t01_probe",
        "created_at": NOW,
        "adapter_version": "0.1.0",
        "python": {
            "version": "3.12.10",
            "executable": "<isaac-root>\\env_isaaclab\\Scripts\\python.exe",
            "platform": "Windows-11-10.0.26200",
        },
        "packages": {"isaaclab": "6.1.14", "torch": "2.11.0+cu128"},
        "source": {
            "repo": "<isaac-root>\\IsaacLab",
            "commit": COMMIT,
            "describe": "v3.0.0-beta2.patch1-dirty",
            "dirty": True,
            "diff_sha256": HASH_A,
        },
        "config_sha256": HASH_B,
        "assets": {"so101_new_calib.usd": HASH_C},
        "gpu": {"name": "RTX 4060 Laptop", "memory_mib": 8188, "driver": "616.92"},
        "worker": {
            "cwd": "<repo>",
            "log_dir": "artifacts\\logs",
            "num_envs": 1,
            "device": "cuda:0",
        },
        "notes": ["T01 fixture"],
    }


def capabilities() -> dict:
    return {
        "backend_id": "backend_mock",
        "backend_kind": "mock",
        "generated_at": NOW,
        "updated_at": NOW,
        "physics": {"engine": "none", "version": "0", "deterministic": True},
        "renderer": {"enabled": False},
        "visualizer": {"enabled": False},
        "installed": True,
        "verified": False,
        "evidence": ["fake backend; protocol tests only"],
        "combinations": [
            {
                "combination_id": "combo_mock_franka_pick_place",
                "robot_id": "franka",
                "task_id": "pick_place",
                "scene_id": "table_top",
                "backend_kind": "mock",
                "status": "installed",
                "evidence": ["fake backend"],
                "checked_at": NOW,
            }
        ],
    }


def robot() -> dict:
    joints = [
        ("shoulder_pan", "shoulder_pan", -2.0, 2.0, 0.0, 2.0),
        ("shoulder_lift", "shoulder_lift", -1.5, 1.5, 0.0, 2.0),
        ("elbow_flex", "elbow_flex", -1.8, 1.8, 0.0, 2.0),
        ("wrist_flex", "wrist_flex", -1.7, 1.7, 0.0, 2.0),
        ("wrist_roll", "wrist_roll", -2.5, 2.5, 0.0, 3.0),
    ]
    payload_joints = [
        {
            "name": name,
            "logical_name": logical,
            "type": "revolute",
            "unit": "rad",
            "lower": lower,
            "upper": upper,
            "home": home,
            "velocity_limit": velocity,
        }
        for name, logical, lower, upper, home, velocity in joints
    ]
    payload_joints.append(
        {
            "name": "gripper",
            "logical_name": "gripper",
            "type": "prismatic",
            "unit": "m",
            "lower": 0.0,
            "upper": 0.04,
            "home": 0.0,
            "velocity_limit": 0.1,
        }
    )
    return {
        "robot_id": "so101",
        "model": "SO-ARM101",
        "dof": 6,
        "joints": payload_joints,
        "end_effector": {"name": "tool0", "parent_joint": "wrist_roll", "offset_m": [0.0, 0.0, 0.1]},
        "gripper": {
            "kind": "parallel",
            "joint_names": ["gripper"],
            "open_value": 0.04,
            "close_value": 0.0,
            "max_width_m": 0.04,
        },
        "control_modes": ["position"],
        "skills": ["observe_scene", "home", "pick", "place"],
        "asset": {
            "usd_path": "https://example.invalid/isaac/so101_new_calib.usd",
            "license": "NVIDIA Isaac Sim asset license",
            "source": "Isaac Sim 6.0 asset root",
        },
    }


def simulation() -> dict:
    return {
        "profile_id": "profile_dev_franka_physx",
        "backend_kind": "isaaclab_physx",
        "robot_id": "franka",
        "scene_id": "table_top",
        "physics": {"engine": "physx", "dt": 0.005, "decimation": 4, "solver_iterations": 8},
        "renderer": {
            "enabled": True,
            "headless": True,
            "backend": "isaac_rtx",
            "cameras": [
                {
                    "name": "front",
                    "width": 320,
                    "height": 240,
                    "update_period_s": 0.2,
                    "data_types": ["rgb", "depth"],
                }
            ],
        },
        "seed": 7,
        "num_envs": 1,
        "budget": {"max_sim_time_s": 120.0, "max_wall_time_s": 300.0},
    }


def world() -> dict:
    return {
        "simulation_id": "sim_0123456789abcdef",
        "episode_id": "ep_0123456789abcdef",
        "env_id": 0,
        "state_version": 3,
        "event_seq": 5,
        "sim_step": 120,
        "sim_time_s": 0.6,
        "wall_time": WALL,
        "source": "sim_ground_truth",
        "frame": "world",
        "robots": [
            {
                "robot_id": "franka",
                "dof": 7,
                "joint_positions": [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.8],
                "joint_velocities": [0.0] * 7,
                "ee_pose": {
                    "frame": "world",
                    "position": [0.4, 0.0, 0.25],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "gripper": {"width_m": 0.04, "closed": False},
            }
        ],
        "objects": [
            {
                "object_id": "red_block",
                "pose": {
                    "frame": "world",
                    "position": [0.5, 0.0, 0.02],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "size_m": [0.04, 0.04, 0.04],
            },
            {
                "object_id": "target_box",
                "pose": {
                    "frame": "world",
                    "position": [0.6, 0.3, 0.04],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "size_m": [0.12, 0.12, 0.08],
            },
        ],
        "skills": [],
    }


def skill_spec() -> dict:
    return {
        "name": "pick",
        "robot_id": "franka",
        "description": "Close the gripper around the target and lift it by 5 cm.",
        "parameters_schema": {"type": "object", "properties": {}, "additionalProperties": False},
        "timeout_s": 30.0,
        "preconditions": ["target visible", "gripper open"],
    }


def skill_invocation() -> dict:
    return {
        "skill": "pick",
        "parameters": {"approach_height_m": 0.1},
        "target_object_id": "red_block",
        "frame": "world",
        "timeout_s": 30.0,
        "expected_duration_s": 10.0,
    }


def task_spec() -> dict:
    return {
        "task_id": "task_0123456789abcdef",
        "goal": "Place the red block inside the target box.",
        "robot_id": "franka",
        "scene_id": "table_top",
        "success_predicates": [
            {
                "predicate_id": "pred_in_box",
                "kind": "object_in_region",
                "object_id": "red_block",
                "region": {"min": [0.54, 0.24, 0.0], "max": [0.66, 0.36, 0.12]},
                "tolerance_m": 0.005,
            }
        ],
        "budgets": {
            "sim_time_s": 120.0,
            "wall_time_s": 300.0,
            "max_skill_calls": 12,
            "max_retries": 2,
        },
        "constraints": ["keep the target box upright"],
    }


def plan() -> dict:
    steps = [
        {"step_id": "step_observe", "skill": "observe_scene", "parameters": {}, "depends_on": [], "expected_duration_s": 2.0},
        {"step_id": "step_pick", "skill": "pick", "parameters": {"target": "red_block"}, "depends_on": ["step_observe"], "expected_duration_s": 10.0},
        {"step_id": "step_place", "skill": "place", "parameters": {"target": "target_box"}, "depends_on": ["step_pick"], "expected_duration_s": 10.0},
    ]
    plan_hash = compute_plan_hash(
        task_id="task_0123456789abcdef",
        episode_id="ep_0123456789abcdef",
        base_state_version=3,
        steps=[PlanStep.model_validate(step) for step in steps],
    )
    return {
        "plan_id": "plan_0123456789abcdef",
        "task_id": "task_0123456789abcdef",
        "episode_id": "ep_0123456789abcdef",
        "base_state_version": 3,
        "created_at": NOW,
        "steps": steps,
        "plan_hash": plan_hash,
        "budget": {
            "sim_time_s": 120.0,
            "wall_time_s": 300.0,
            "max_skill_calls": 12,
            "max_retries": 2,
        },
    }


def job_event() -> dict:
    return {
        "job_id": "job_0123456789abcdef",
        "event_seq": 4,
        "at": NOW,
        "type": "succeeded",
        "status": "succeeded",
        "skill": "place",
        "message": "placement verified",
        "progress": 1.0,
        "data": {"success_predicates": 1},
        "evidence": ["artifacts/run-001/placement.json"],
    }


def error_info() -> dict:
    return {
        "code": "TIMEOUT",
        "message": "skill exceeded its wall-clock budget",
        "retryable": True,
        "details": {"timeout_s": 30.0},
        "evidence": ["artifacts/run-001/worker.log"],
        "occurred_at": NOW,
    }


def worker_profile() -> dict:
    return {
        "profile_id": "profile_dev_franka_physx",
        "backend_id": "backend_isaac_physx",
        "backend_kind": "isaaclab_physx",
        "robot_id": "franka",
        "scene_id": "table_top",
        "python_executable": "<isaac-root>\\env_isaaclab\\Scripts\\python.exe",
        "entry_module": "robotics_backends.isaaclab_physx.entry",
        "allowed_imports": ["isaaclab", "isaacsim"],
        "max_envs": 1,
        "description": "Franka development profile for the T02-T04 software chain.",
    }


VALID = {
    "RuntimeManifest": manifest(),
    "BackendCapabilities": capabilities(),
    "CapabilityCombination": capabilities()["combinations"][0],
    "RobotSpec": robot(),
    "JointSpec": robot()["joints"][0],
    "SimulationSpec": simulation(),
    "WorkerProfile": worker_profile(),
    "WorldState": world(),
    "RobotState": world()["robots"][0],
    "ObjectState": world()["objects"][0],
    "SkillSpec": skill_spec(),
    "SkillInvocation": skill_invocation(),
    "TaskSpec": task_spec(),
    "TaskPlan": plan(),
    "JobEvent": job_event(),
    "ErrorInfo": error_info(),
}


def mutated(name: str, mutate) -> dict:
    payload = copy.deepcopy(VALID[name])
    mutate(payload)
    return payload


INVALID = [
    (
        "world_state_joint_length",
        "WorldState",
        mutated("WorldState", lambda data: data["robots"][0]["joint_positions"].pop()),
    ),
    (
        "robot_dof_mismatch",
        "RobotSpec",
        mutated("RobotSpec", lambda data: data.update({"dof": 7})),
    ),
    (
        "robot_joint_unit_mismatch",
        "RobotSpec",
        mutated("RobotSpec", lambda data: data["joints"][0].update({"unit": "m"})),
    ),
    (
        "simulation_zero_envs",
        "SimulationSpec",
        mutated("SimulationSpec", lambda data: data.update({"num_envs": 0})),
    ),
    (
        "world_state_negative_time",
        "WorldState",
        mutated("WorldState", lambda data: data.update({"sim_time_s": -1.0})),
    ),
    (
        "world_state_quaternion_not_normalized",
        "WorldState",
        mutated(
            "WorldState",
            lambda data: data["robots"][0]["ee_pose"].update({"orientation_xyzw": [0.0, 0.0, 0.0, 2.0]}),
        ),
    ),
    (
        "task_plan_cycle",
        "TaskPlan",
        mutated(
            "TaskPlan",
            lambda data: (
                data["steps"][0].update({"depends_on": ["step_place"]}),
                data["steps"][2].update({"depends_on": ["step_pick"]}),
            ),
        ),
    ),
    (
        "task_plan_duplicate_step",
        "TaskPlan",
        mutated("TaskPlan", lambda data: data["steps"][1].update({"step_id": "step_observe"})),
    ),
    (
        "task_spec_unknown_field",
        "TaskSpec",
        mutated("TaskSpec", lambda data: data.update({"surprise": True})),
    ),
    (
        "task_predicate_missing_region",
        "TaskSpec",
        mutated("TaskSpec", lambda data: data["success_predicates"][0].pop("region")),
    ),
    (
        "capabilities_verified_without_evidence",
        "BackendCapabilities",
        mutated("BackendCapabilities", lambda data: data.update({"verified": True, "evidence": []})),
    ),
    (
        "capabilities_mock_cannot_be_verified",
        "BackendCapabilities",
        mutated(
            "BackendCapabilities",
            lambda data: data.update({"verified": True, "evidence": ["wishful thinking"]}),
        ),
    ),
    (
        "error_retryable_mismatch",
        "ErrorInfo",
        mutated("ErrorInfo", lambda data: data.update({"code": "GRASP_FAILED", "retryable": True})),
    ),
    (
        "skill_invocation_nonfinite_timeout",
        "SkillInvocation",
        mutated("SkillInvocation", lambda data: data.update({"timeout_s": float("inf")})),
    ),
]


def build() -> dict:
    cases = []
    for model_name, payload in VALID.items():
        cases.append({"name": f"valid_{model_name.lower()}", "model": model_name, "valid": True, "payload": payload})
    for name, model_name, payload in INVALID:
        cases.append({"name": name, "model": model_name, "valid": False, "payload": payload})

    for case in cases:
        model = MODELS[case["model"]]
        accepted = True
        try:
            model.model_validate(case["payload"])
        except ValidationError:
            accepted = False
        if accepted != case["valid"]:
            raise SystemExit(f"fixture {case['name']} expected valid={case['valid']} but got {accepted}")

    return {
        "schemaVersion": "0.1",
        "generatedAt": datetime.now(timezone.utc).isoformat(),
        "cases": cases,
    }


def main() -> int:
    document = build()
    text = json.dumps(document, indent=2, ensure_ascii=False)
    # JSON has no Infinity literal; 1e999 parses to Infinity in both Python and JS.
    text = text.replace("-Infinity", "-1e999").replace("Infinity", "1e999")
    (HERE / "cases.json").write_text(text + "\n", encoding="utf-8")
    print(json.dumps({"cases": len(document["cases"]), "path": str(HERE / "cases.json")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
