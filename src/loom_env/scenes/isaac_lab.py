"""Spawn prepared asset instances using Isaac Lab's native USD spawner."""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg

from loom_env.assets.catalog import (
    asset_definition,
    is_articulated,
    load_prepared,
    prepared_directory,
)


def instance_config(obj, prim_path, world_pose, asset_root):
    definition = asset_definition(obj["asset"])
    if is_articulated(definition):
        from loom_env.embodiments.isaac_lab import spawn_robot
        from loom_env.scenes.workspace import transform

        physics = load_prepared(asset_root, obj["asset"])["physics_properties"]
        limits = physics["joint"]["limits_rad"]
        q = obj["joint_positions"][definition.joint]
        if not limits[0] <= q <= limits[1]:
            raise ValueError("Initial object joint position exceeds USD limits")
        base_pose = transform(
            world_pose, physics["bodies"][definition.root_body]["pose_asset"]
        )
        return ArticulationCfg(
            prim_path=prim_path,
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(
                    (
                        prepared_directory(asset_root, obj["asset"]) / "asset.usda"
                    ).resolve()
                ),
                func=spawn_robot,
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=tuple(base_pose[:3]),
                rot=tuple(base_pose[3:]),
                joint_pos=dict(obj["joint_positions"]),
                joint_vel={".*": 0.0},
            ),
            actuators={},
            soft_joint_pos_limit_factor=1.0,
        )
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
