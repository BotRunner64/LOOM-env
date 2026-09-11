"""Official fixed robot supports, shared by simulation and collision planning."""

from dataclasses import dataclass
from pathlib import Path
from functools import partial

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.embodiments.assets import MODELS, converted_usd, source_urdf, verify_asset
from loom_env.embodiments.frames import resolve_fixed_frame
from loom_env.scenes.workspace import transform


@dataclass(frozen=True)
class FixedSupport:
    model: str
    pose: tuple[float, ...]
    usd: Path
    collision: Path
    manifest: dict

    def riser(self):
        """Fixed floor-to-pedestal block, derived from the source footprint."""
        height = self.pose[2]
        if height <= 1e-6:
            return None
        rotation = Rotation.from_quat(self.pose[3:])
        if not np.allclose(rotation.apply([0, 0, 1]), [0, 0, 1], atol=1e-6):
            raise ValueError("Raised pedestal requires an upright installation")
        with np.load(self.collision, allow_pickle=False) as mesh:
            vertices = mesh["vertices"]
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        size = np.r_[high[:2] - low[:2], height]
        center = np.r_[(low[:2] + high[:2]) / 2, -height / 2]
        return tuple(size), tuple(transform(self.pose, np.r_[center, [0, 0, 0, 1]]))


def fixed_support(deployment, asset_root):
    """Recover one common pedestal pose from the authored shoulder transforms.

    Deployment arm base poses remain world poses. Their relationship is checked
    against the pinned full URDF, so the two arms cannot silently detach from it.
    """
    mounted = [
        arm
        for arm in deployment.arms.values()
        if arm.asset.startswith("enactic:openarm-")
    ]
    if not mounted:
        return None
    name = "openarm_support"
    manifest = verify_asset(asset_root, name)
    body = MODELS[name]["base"]
    poses = []
    for arm in mounted:
        side = arm.asset.rsplit("-", 1)[-1]
        _, local = resolve_fixed_frame(
            source_urdf(asset_root, name), f"openarm_{side}_link0", {body}
        )
        inverse = Rotation.from_quat(local[3:]).inv()
        poses.append(
            transform(
                arm.base_pose, np.r_[inverse.apply(-local[:3]), inverse.as_quat()]
            )
        )
    for candidate in poses[1:]:
        if (
            np.linalg.norm(candidate[:3] - poses[0][:3]) > 1e-5
            or (
                Rotation.from_quat(candidate[3:]).inv()
                * Rotation.from_quat(poses[0][3:])
            ).magnitude()
            > 1e-5
        ):
            raise ValueError(
                "OpenArm shoulder poses disagree with the official shared pedestal"
            )
    return FixedSupport(
        name,
        tuple(poses[0]),
        converted_usd(asset_root, name).resolve(),
        Path(asset_root) / name / "meshes/collision.npz",
        manifest,
    )


def _spawn_static_support(
    prim_path, cfg, translation=None, orientation=None, *, riser=None, **kwargs
):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Usd, UsdPhysics

    result = sim_utils.spawn_from_usd(
        prim_path, cfg, translation=translation, orientation=orientation, **kwargs
    )
    stage = get_current_stage()
    for path in sim_utils.find_matching_prim_paths(prim_path):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
            for api in (UsdPhysics.ArticulationRootAPI, UsdPhysics.RigidBodyAPI):
                if prim.HasAPI(api):
                    prim.RemoveAPI(api)
    if riser is not None:
        size, pose = riser
        block = sim_utils.CuboidCfg(
            size=size,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.22, 0.24, 0.27)
            ),
        )
        block.func(
            prim_path + "_riser", block, translation=pose[:3], orientation=pose[3:]
        )
    return result


def support_config(deployment, asset_root, prim_path):
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg

    support = fixed_support(deployment, asset_root)
    if support is None:
        return None
    return AssetBaseCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(support.usd),
            func=partial(_spawn_static_support, riser=support.riser()),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=support.pose[:3], rot=support.pose[3:]
        ),
    )
