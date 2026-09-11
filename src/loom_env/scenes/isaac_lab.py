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
        render=sim_utils.RenderCfg(ambient_light_intensity=0.3),
    )
