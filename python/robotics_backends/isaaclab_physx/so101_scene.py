"""SO-ARM101 scene and articulation config (T05).

The USD is the mirrored RobotStudio asset; the gripper is a revolute joint
(degrees in USD, radians in Isaac Lab).  The actuator gains are project-owned
because the USD drive values are too soft for position control.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils import configclass

PROJECT_ROOT = Path(__file__).resolve().parents[3]
SO101_USD = PROJECT_ROOT / "assets" / "isaac6" / "RobotStudio" / "so101_new_calib" / "so101_new_calib.usd"
if not SO101_USD.is_file():
    raise FileNotFoundError(f"ASSET_NOT_READY: missing local SO-ARM101 asset {SO101_USD}")

SO101_ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SO101_GRIPPER_JOINT = "gripper"
SO101_JOINTS = [*SO101_ARM_JOINTS, SO101_GRIPPER_JOINT]

SO101_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(SO101_USD),
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=2.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={name: 0.0 for name in SO101_JOINTS},
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=SO101_ARM_JOINTS,
            stiffness=20.0,
            damping=2.0,
            effort_limit_sim=10.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[SO101_GRIPPER_JOINT],
            stiffness=10.0,
            damping=1.0,
            effort_limit_sim=10.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)


@configclass
class SO101SceneCfg(InteractiveSceneCfg):
    """Ground, light and one SO-ARM101."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.02)),
        spawn=sim_utils.CuboidCfg(
            size=(2.0, 2.0, 0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15)),
        ),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=1500.0),
    )
    robot: ArticulationCfg = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def build_so101_scene(
    *,
    device: str = "cuda:0",
    dt: float = 0.005,
    num_envs: int = 1,
    env_spacing: float = 1.5,
) -> tuple[SimulationContext, InteractiveScene, Any]:
    if num_envs != 1:
        raise ValueError("T05 only supports one environment")
    sim = SimulationContext(SimulationCfg(device=device, dt=dt))
    scene = InteractiveScene(SO101SceneCfg(num_envs=num_envs, env_spacing=env_spacing))
    sim.reset()
    robot = scene["robot"]
    return sim, scene, robot


__all__ = [
    "SO101_USD",
    "SO101_JOINTS",
    "SO101_ARM_JOINTS",
    "SO101_GRIPPER_JOINT",
    "SO101_CFG",
    "SO101SceneCfg",
    "build_so101_scene",
]
