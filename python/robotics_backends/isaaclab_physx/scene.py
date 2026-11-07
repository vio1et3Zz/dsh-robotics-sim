"""Build the minimal Franka PhysX scene used by the T02 software chain.

Only public Isaac Lab APIs are used.  The scene is intentionally small: ground,
dome light and one Franka Panda.  Objects and task logic arrive in T05/T06.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils import configclass
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG

# The Isaac 6.0 asset root no longer hosts ``panda_instanceable.usd`` at the
# path baked into FRANKA_PANDA_CFG, and this machine's omni.client cannot stat
# the remote nucleus URL.  The FrankaPanda tree is mirrored under assets/isaac6
# (sha256 recorded by the worker) and loaded from the filesystem instead.
PROJECT_ROOT = Path(__file__).resolve().parents[3]
FRANKA_PANDA_USD = PROJECT_ROOT / "assets" / "isaac6" / "FrankaPanda" / "franka.usd"
if not FRANKA_PANDA_USD.is_file():
    raise FileNotFoundError(f"ASSET_NOT_READY: missing local Franka asset {FRANKA_PANDA_USD}")

FRANKA_PANDA_USD_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/6.0/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)
FRANKA_CFG = FRANKA_PANDA_CFG.copy()
FRANKA_CFG.spawn.usd_path = str(FRANKA_PANDA_USD)


@configclass
class FrankaSceneCfg(InteractiveSceneCfg):
    """One ground plane, one light and one Franka."""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.02)),
        spawn=sim_utils.CuboidCfg(
            size=(20.0, 20.0, 0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15)),
        ),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2000.0),
    )
    robot: ArticulationCfg = FRANKA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


def build_franka_scene(
    *,
    device: str = "cuda:0",
    dt: float = 0.005,
    num_envs: int = 1,
    env_spacing: float = 2.5,
) -> tuple[SimulationContext, InteractiveScene, Any]:
    """Create and reset the scene; returns (sim, scene, robot articulation)."""

    if num_envs != 1:
        raise ValueError("T02 only supports one environment")
    sim = SimulationContext(SimulationCfg(device=device, dt=dt))
    scene = InteractiveScene(FrankaSceneCfg(num_envs=num_envs, env_spacing=env_spacing))
    sim.reset()
    robot = scene["robot"]
    return sim, scene, robot


__all__ = ["FrankaSceneCfg", "build_franka_scene"]
