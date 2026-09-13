"""Spawn prepared asset instances using Isaac Lab's native USD spawner."""

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg

from loom_env.assets.catalog import prepared_directory


def instance_config(obj, prim_path, world_pose, asset_root):
    cls = AssetBaseCfg if obj["static"] else RigidObjectCfg
    spawn = sim_utils.UsdFileCfg(
        usd_path=str(
            (prepared_directory(asset_root, obj["asset"]) / "asset.usda").resolve()
        ),
    )
    return cls(
        prim_path=prim_path,
        spawn=spawn,
        init_state=cls.InitialStateCfg(
            pos=tuple(world_pose[:3]), rot=tuple(world_pose[3:])
        ),
    )


def simulation_config(deployment):
    from isaaclab_physx.physics import PhysxCfg

    return sim_utils.SimulationCfg(
        dt=deployment.physics_dt,
        render_interval=deployment.decimation,
        device="cuda:0",
        # PhysX warns that applying external forces only once per TGS step
        # produces noisy velocities. Apply gravity at every solver iteration.
        physics=PhysxCfg(enable_external_forces_every_iteration=True),
        render=render_config(),
    )


def render_config():
    """Explicit camera rendering shared by task environments and smoke checks."""
    return sim_utils.RenderCfg(
        rendering_mode="quality",
        antialiasing_mode="DLAA",
        enable_translucency=True,
        enable_reflections=True,
        enable_global_illumination=True,
        dlss_mode=2,
        enable_dlssg=False,
        ambient_light_intensity=0.3,
    )
