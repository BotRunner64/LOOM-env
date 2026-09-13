"""Pinned model sources and cache paths; this module does not import a simulator."""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

PANDA_ASSET = "isaaclab_assets.robots.franka:FRANKA_PANDA_CFG"
PANDA_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/IsaacLab/Robots/FrankaEmika/panda_instanceable.usd"
)
PANDA_VERSION = "Isaac-assets-5.1"
SOURCES = {
    "i2rt": {
        "repository": "https://github.com/i2rt-robotics/i2rt",
        "revision": "5b72c47239bd056d0fa6c1a39edeb0537c89443c",
        "archive": "i2rt.zip",
        "sha256": "162635cbdeaa1792313b3645a91850d4b850d13821474cb0e8b605a5a729c529",
        "license": "MIT",
    },
    "robotwin": {
        "repository": "https://huggingface.co/datasets/TianxingChen/RoboTwin2.0",
        "revision": "785feb15aa4a4f532395ad2b1d2be5f28cb561ad",
        "archive": "embodiments.zip",
        "sha256": "6b87d7d55e106d8ff25917e0538eb1e177fc549280e8a742a8cec3cb9f953fc6",
    },
    "xarm6": {
        "repository": "https://github.com/haosulab/ManiSkill-XArm6",
        "revision": "438e7e4ecbdc0b21e2c2e24d7b19a731d98599bf",
        "archive": "xarm6.zip",
        "sha256": "4da3a80c2fa13adf0af0383ec7be83c48e2afb452c61b16d806a7f5506fc9312",
    },
}
# Only model-specific choices live here. Joint names, limits and deployment poses
# belong to the deployment preset; physical effort/velocity limits come from URDF.
MODELS = {
    "yam": dict(
        asset="i2rt:yam-v1",
        source="i2rt",
        urdf="i2rt/robot_models/arm/yam/v1/yam.urdf",
        base="base",
        arm_gains=(400.0, 40.0),
        gripper_gains=(1000.0, 50.0),
        # The finger CAD parts interfere internally even with convex decomposition.
        # Match I2RT's parallel-gripper MJCF exclusion; keep external contacts.
        collision_groups=(("tip_left", "tip_right"),),
    ),
    "piper": dict(
        asset="robotwin:piper",
        source="robotwin",
        urdf="embodiments/piper/piper.urdf",
        base="base_link",
        arm_gains=(400.0, 40.0),
        gripper_gains=(1000.0, 50.0),
    ),
    "x5": dict(
        asset="robotwin:x5",
        source="robotwin",
        urdf="embodiments/ARX-X5/X5A.urdf",
        base="base_link",
        arm_gains=(1000.0, 80.0),
        gripper_gains=(1000.0, 50.0),
    ),
    "ur5_wsg": dict(
        asset="robotwin:ur5-wsg",
        source="robotwin",
        urdf="embodiments/ur5-wsg/ur5_wsg_gripper.urdf",
        base="base_link",
        arm_gains=(1000.0, 80.0),
        gripper_gains=(1000.0, 50.0),
    ),
    "xarm6_robotiq": dict(
        asset="maniskill:xarm6-robotiq",
        source="xarm6",
        urdf="xarm6_robotiq.urdf",
        base="link_base",
        arm_gains=(1000.0, 80.0),
        gripper_gains=(1000.0, 50.0),
        gripper_effort_limit=1.0,
        gripper_armature=0.001,
        collision_groups=(
            (
                "link5",
                "link6",
                "left_outer_knuckle",
                "right_outer_knuckle",
                "left_inner_knuckle",
                "right_inner_knuckle",
                "left_inner_finger",
                "right_inner_finger",
            ),
        ),
    ),
}
CONVERSION_ARGS = (
    "--fix-base",
    "--merge-joints",
    "--joint-stiffness",
    "400",
    "--joint-damping",
    "40",
    "--headless",
)
PREPARATION_VERSION = 3


def preparation_version(name):
    """Allow a model-specific conversion change without invalidating unrelated assets."""
    return MODELS[name].get("preparation_version", PREPARATION_VERSION)


def model_name(asset: str) -> str:
    for name, model in MODELS.items():
        if model["asset"] == asset:
            return name
    raise ValueError(f"Unsupported URDF asset: {asset}")


def source_dir(asset_root: str | Path, source: str) -> Path:
    info = SOURCES[source]
    return Path(asset_root) / f"{source}-{info['revision'][:8]}"


def source_tree(asset_root: str | Path, source: str) -> Path:
    root = source_dir(asset_root, source)
    info = SOURCES[source]
    if source != "robotwin":
        root /= f"{info['repository'].rsplit('/', 1)[1]}-{info['revision']}"
    return root


def source_urdf(asset_root: str | Path, name: str) -> Path:
    model = MODELS[name]
    return source_tree(asset_root, model["source"]) / model["urdf"]


def prepared_urdf(asset_root: str | Path, name: str) -> Path:
    return Path(asset_root) / name / f"{name}.urdf"


def converted_usd(asset_root: str | Path, name: str) -> Path:
    return Path(asset_root) / name / "usd" / name / f"{name}.usda"


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def joint_definitions(urdf: Path) -> dict:
    return {
        joint.get("name"): joint
        for joint in ET.parse(urdf).getroot().findall("joint")
        if joint.get("type") != "fixed"
    }


def verify_asset(asset_root: str | Path, name: str) -> dict:
    """Reject stale, incomplete or modified conversion output before physics."""
    root = Path(asset_root).resolve()
    path = root / name / "asset.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}; run scripts/prepare_assets.py {name}")
    manifest = json.loads(path.read_text())
    if (
        manifest.get("source") != SOURCES[MODELS[name]["source"]]
        or manifest.get("conversion") != list(CONVERSION_ARGS)
        or manifest.get("preparation_version") != preparation_version(name)
    ):
        raise ValueError(f"Stale {name} cache; run scripts/prepare_assets.py {name}")
    for key, file in (
        ("source_urdf_sha256", source_urdf(root, name)),
        ("urdf_sha256", prepared_urdf(root, name)),
    ):
        if sha256(file) != manifest[key]:
            raise ValueError(f"{name} URDF changed after conversion: {file}")
    for mesh in ET.parse(prepared_urdf(root, name)).findall(
        "link/collision/geometry/mesh"
    ):
        if any(float(value) < 0 for value in mesh.get("scale", "1 1 1").split()):
            raise ValueError(
                f"Unsafe mirrored collision cache; run scripts/prepare_assets.py {name}"
            )
    files = manifest["usd_files"]
    if str(converted_usd(root, name).relative_to(root)) not in files:
        raise ValueError(f"{name} manifest is missing the root USD")
    for relative, expected in files.items():
        asset = (root / relative).resolve()
        if not asset.is_relative_to(root / name / "usd") or sha256(asset) != expected:
            raise ValueError(f"Converted asset changed: {relative}")
    for relative, expected in manifest["prepared_mesh_files"].items():
        file = (root / relative).resolve()
        if not file.is_relative_to(root / name / "meshes") or sha256(file) != expected:
            raise ValueError(f"Prepared mesh changed: {relative}")
    return manifest


def validate_visuals(root):
    """Require actual renderable meshes on every rigid body, not only colliders."""
    from pxr import Usd, UsdGeom, UsdPhysics

    counts = {
        str(p.GetPath()): 0
        for p in Usd.PrimRange(root)
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        if (
            mesh.ComputePurpose() not in {"default", "render"}
            or mesh.ComputeVisibility() == "invisible"
        ):
            continue
        if not mesh.GetPointsAttr().Get():
            continue
        parent = prim
        while parent and str(parent.GetPath()) not in counts:
            parent = parent.GetParent()
        if parent:
            counts[str(parent.GetPath())] += 1
    missing = [name for name, count in counts.items() if not count]
    if not counts or missing:
        raise ValueError(
            f"Rigid bodies without visual meshes: {missing or str(root.GetPath())}"
        )
    return {Path(name).name: count for name, count in counts.items()}
