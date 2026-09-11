"""Small, explicit inventory of reviewed source models and local annotations."""

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class AssetDefinition:
    name: str
    source: str
    revision: str
    path: str
    sha256: str
    category: str
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]]
    dynamic: bool = False
    # Object-local Panda grasp centre; TCP offset belongs to the embodiment.
    grasp: tuple[float, float, float] | None = None
    # Conservative, manually checked free interior: centre and full extents.
    interior: tuple[tuple[float, float, float], tuple[float, float, float]] | None = (
        None
    )
    source_scale: float = 1.0
    source_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def fingerprint(self):
        return hashlib.sha256(
            json.dumps(asdict(self), sort_keys=True).encode()
        ).hexdigest()


TABLE_SCALE = 0.75 / 0.5255104303359985
ASSETS = {
    "maniskill:table": AssetDefinition(
        "table",
        "https://github.com/haosulab/ManiSkill",
        "62ff3a5896b4d5b4cf0ac4c8d79afe600c9404a3",
        "ManiSkill/mani_skill/utils/scene_builder/table/assets/table.glb",
        "cb0ebd8ad6438c1160f095d902bf55415f9d643f8c6990a186a052251fe7951a",
        "workspace",
        (
            (-0.34545048 * TABLE_SCALE, -0.69082248 * TABLE_SCALE, -0.75),
            (0.34545048 * TABLE_SCALE, 0.69082248 * TABLE_SCALE, 0.0),
        ),
        source_scale=TABLE_SCALE,
        source_translation=(
            -0.008959189 * TABLE_SCALE,
            -0.003384531 * TABLE_SCALE,
            -0.75,
        ),
    ),
    "robodojo:brick": AssetDefinition(
        "brick",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Rigid/brick/00000/object.usdz",
        "539abd853016ba9c85d1ba2d943bd6edc208e8f0388892203bef470e36bc597a",
        "graspable_object",
        ((-0.03022, -0.01450, -0.01220), (0.03016, 0.01456, 0.01220)),
        dynamic=True,
        grasp=(0.0, 0.0, 0.002),
    ),
    "robodojo:basket": AssetDefinition(
        "basket",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Geometry/basket/00002/object.usdz",
        "437f8357aba77943b451b57e0848d91a8d77cd6a00af367111cde21beac6548e",
        "receptacle",
        ((-0.12601, -0.08401, -0.03857), (0.12601, 0.08401, 0.03844)),
        dynamic=True,
        interior=((0.0, 0.0, 0.00435), (0.20, 0.12, 0.0675)),
    ),
}
PREPARATION_VERSION = 5


def asset_definition(asset_id):
    try:
        return ASSETS[asset_id]
    except KeyError as error:
        raise ValueError(f"Unknown scene asset: {asset_id}") from error


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def prepared_directory(root, asset_id):
    return Path(root) / "scenes" / asset_definition(asset_id).name


def load_prepared(root, asset_id):
    """Reject missing, stale or modified prepared assets before launching physics."""
    definition = asset_definition(asset_id)
    directory = prepared_directory(root, asset_id)
    path = directory / "asset.json"
    if not path.is_file():
        raise ValueError(
            f"Unprepared asset {asset_id}; run scripts/prepare_assets.py scene"
        )
    manifest = json.loads(path.read_text())
    if (
        manifest.get("definition") != definition.fingerprint
        or manifest.get("preparation_version") != PREPARATION_VERSION
    ):
        raise ValueError(f"Stale preparation for {asset_id}; prepare it again")
    if not {"asset.usda", "collision.npz"} <= manifest.get("files", {}).keys():
        raise ValueError(f"Incomplete prepared asset: {asset_id}")
    for relative, expected in manifest["files"].items():
        member = (directory / relative).resolve()
        if not member.is_relative_to(directory.resolve()) or not member.is_file():
            raise ValueError(f"Missing or invalid prepared asset member: {relative}")
        if sha256(member) != expected:
            raise ValueError(f"Prepared asset checksum mismatch: {member}")
    return manifest


def collision_mesh(root, asset_id):
    with np.load(
        prepared_directory(root, asset_id) / "collision.npz", allow_pickle=False
    ) as data:
        return data["vertices"].copy(), data["faces"].copy()
