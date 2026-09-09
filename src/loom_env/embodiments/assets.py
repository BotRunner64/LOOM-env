"""Pinned asset provenance. Large source and converted files stay in the cache."""

import hashlib
import json
from pathlib import Path

PANDA_ASSET = "isaaclab_assets.robots.franka:FRANKA_PANDA_CFG"
PANDA_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/IsaacLab/Robots/FrankaEmika/panda_instanceable.usd"
)
PANDA_VERSION = "Isaac-assets-5.1"

PIPER_ASSET = "robotwin:piper"
PIPER_SOURCE = {
    "repository": "https://huggingface.co/datasets/TianxingChen/RoboTwin2.0",
    "revision": "785feb15aa4a4f532395ad2b1d2be5f28cb561ad",
    "archive": "embodiments.zip",
    "sha256": "6b87d7d55e106d8ff25917e0538eb1e177fc549280e8a742a8cec3cb9f953fc6",
}
PIPER_CONVERSION_ARGS = (
    "--fix-base",
    "--merge-joints",
    "--joint-stiffness",
    "400",
    "--joint-damping",
    "40",
)
PIPER_VERSION = f"RoboTwin-{PIPER_SOURCE['revision']}:isaac-import-v1"


def piper_source_dir(asset_root: str | Path) -> Path:
    return (
        Path(asset_root)
        / f"robotwin-{PIPER_SOURCE['revision'][:8]}"
        / "embodiments/piper"
    )


def piper_usd(asset_root: str | Path) -> Path:
    return Path(asset_root) / "piper/usd/piper/piper.usda"


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def verify_piper_asset(asset_root: str | Path) -> dict:
    """Reject incomplete or modified conversion output before launching physics."""
    root = Path(asset_root).resolve()
    path = root / "piper/asset.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}; run scripts/prepare_piper.py first")
    manifest = json.loads(path.read_text())
    if manifest["source"] != PIPER_SOURCE or manifest["conversion"] != list(
        PIPER_CONVERSION_ARGS
    ):
        raise ValueError(
            "Piper source or conversion configuration does not match this version"
        )
    if sha256(piper_source_dir(root) / "piper.urdf") != manifest["urdf_sha256"]:
        raise ValueError("Piper URDF changed after conversion")
    files = manifest["usd_files"]
    if str(piper_usd(root).relative_to(root)) not in files:
        raise ValueError("Piper manifest is missing the root USD")
    for name, expected in files.items():
        asset = (root / name).resolve()
        if not asset.is_relative_to(root / "piper/usd") or sha256(asset) != expected:
            raise ValueError(f"Piper converted asset changed: {name}")
    return manifest
