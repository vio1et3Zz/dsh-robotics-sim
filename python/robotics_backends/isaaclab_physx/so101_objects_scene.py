"""SO-ARM101 object scene shared by T06 probes and skills."""

from __future__ import annotations

from typing import Any

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils import configclass

from isaaclab_physx.renderers import IsaacRtxRendererCfg

from .so101_scene import SO101_CFG

# Front camera: world pose looking at the platform from the robot's front-right.
FRONT_CAMERA_POS = (0.38, -0.42, 0.30)
FRONT_CAMERA_QUAT_XYZW = (-0.19947241, 0.12594694, 0.82169188, 0.51881652)
FRONT_CAMERA_WIDTH = 640
FRONT_CAMERA_HEIGHT = 480


@configclass
class SO101ObjectSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.05)),
        spawn=sim_utils.CuboidCfg(
            size=(2.0, 2.0, 0.02),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.15, 0.15, 0.15)),
        ),
    )
    table = AssetBaseCfg(
        prim_path="/World/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.01)),
        spawn=sim_utils.CuboidCfg(
            size=(0.6, 0.5, 0.02),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.35, 0.2)),
        ),
    )
    robot = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    block_platform = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/BlockPlatform",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.18, 0.0, 0.03)),
        spawn=sim_utils.CuboidCfg(
            size=(0.08, 0.08, 0.06),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
        ),
    )
    red_block = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/RedBlock",
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.18, 0.0, 0.075),
            lin_vel=(0.0, 0.0, 0.0),
            ang_vel=(0.0, 0.0, 0.0),
        ),
        spawn=sim_utils.CuboidCfg(
            size=(0.03, 0.03, 0.03),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1)),
        ),
    )
    target_box = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TargetBox",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.22, 0.05, 0.02)),
        spawn=sim_utils.CuboidCfg(
            size=(0.10, 0.10, 0.04),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.3, 0.8)),
        ),
    )


@configclass
class SO101CameraSceneCfg(SO101ObjectSceneCfg):
    """Object scene plus the T08 front RGB/depth camera."""

    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=1500.0),
    )
    front_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/FrontCamera",
        offset=CameraCfg.OffsetCfg(
            pos=FRONT_CAMERA_POS,
            rot=FRONT_CAMERA_QUAT_XYZW,
            convention="world",
        ),
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
        width=FRONT_CAMERA_WIDTH,
        height=FRONT_CAMERA_HEIGHT,
        renderer_cfg=IsaacRtxRendererCfg(),
    )


def build_so101_object_scene(
    *, device: str = "cuda:0", dt: float = 0.005, with_camera: bool = False
) -> tuple[SimulationContext, InteractiveScene, Any, Any]:
    sim = SimulationContext(SimulationCfg(device=device, dt=dt))
    if with_camera:
        cfg = SO101CameraSceneCfg(num_envs=1, env_spacing=1.5)
    else:
        cfg = SO101ObjectSceneCfg(num_envs=1, env_spacing=1.5)
    scene = InteractiveScene(cfg)
    sim.reset()
    return sim, scene, scene["robot"], scene["red_block"]


__all__ = ["SO101ObjectSceneCfg", "SO101CameraSceneCfg", "build_so101_object_scene", "FRONT_CAMERA_POS"]
