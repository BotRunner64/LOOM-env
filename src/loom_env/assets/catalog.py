"""Small, explicit inventory of reviewed source models and local annotations."""

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

PREPARATION_VERSION = 3


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
    # Object-local grasp centre; TCP offset belongs to the embodiment.
    grasp: tuple[float, float, float] | None = None
    # Conservative, manually checked free interior: centre and full extents.
    interior: tuple[tuple[float, float, float], tuple[float, float, float]] | None = (
        None
    )
    tabletop_bounds: (
        tuple[tuple[float, float, float], tuple[float, float, float]] | None
    ) = None
    # Reviewed closed-Panda finger centre height above the supporting plane.
    push_height: float | None = None
    source_scale: float = 1.0
    source_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)


TABLE_SCALE = 0.75 / 0.5255104303359985
ASSETS = {
    "robodojo:juice_carton": AssetDefinition(
        "juice_carton",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Rigid/juice_carton/00000/object.usda",
        "7696cb3300ce0caac3cd127609b8544952e5f862799d34837569eaff5d888a11",
        "graspable_object",
        (
            (-0.03375473991036415, -0.025292236357927322, -0.09011700749397278),
            (0.03359805792570114, 0.025788072496652603, 0.0898829847574234),
        ),
        dynamic=True,
    ),
    "robodojo:tea_carton_pack": AssetDefinition(
        "tea_carton_pack",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Clutter/tissue/00001/object.usda",
        "ea6363be9f8f1469c1a9e2a9ead5a857677f6ad726f30a7c728f6edb8c2380a3",
        "graspable_object",
        (
            (-0.1602, -0.0268, -0.0411),
            (0.1598, 0.0271, 0.0403),
        ),
        dynamic=True,
        grasp=(0.12, 0.0, 0.0),
    ),
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
        # Measured top component bounds after source normalization; legs retain
        # their original triangles. The slab replaces only this component.
        tabletop_bounds=(
            (-0.49302134, -0.98593069, -0.035712568),
            (0.49302134, 0.98593069, 0.0),
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
        "RoboDojo/Assets/Object/RoboDojo/Rigid/brick/00000/object.usda",
        "1e45a258dc8219d1a3ee34ea389b7aef48dd963c0f2ef579f69b4c190e0999f8",
        "graspable_object",
        ((-0.03022, -0.01450, -0.01220), (0.03016, 0.01456, 0.01220)),
        dynamic=True,
        grasp=(0.0, 0.0, 0.002),
        push_height=0.016,
    ),
    "robodojo:basket": AssetDefinition(
        "basket",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Geometry/basket/00002/object.usda",
        "cb916c49fa7eb221674cfc20ce169b94544d50c86a940027b5e7e13e1014d5f4",
        "receptacle",
        ((-0.12601, -0.08401, -0.03857), (0.12601, 0.08401, 0.03844)),
        dynamic=True,
        interior=((0.0, 0.0, 0.00435), (0.20, 0.12, 0.0675)),
    ),
    "robodojo:plate": AssetDefinition(
        "plate",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Rigid/plate/00002/object.usda",
        "e0366c5c58361b7abf03031179cebbb637dad92834ccb126ec342d4159cc70ee",
        "graspable_object",
        (
            (-0.06484473496675491, -0.06470753252506256, -0.017830800265073776),
            (0.0647168755531311, 0.06483729928731918, 0.017830800265073776),
        ),
        dynamic=True,
        push_height=0.016,
    ),
    "robodojo:box": AssetDefinition(
        "box",
        "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "a14409d7fae673c00499e01fd88b4457df6351b1",
        "RoboDojo/Assets/Object/RoboDojo/Rigid/box/00000/object.usda",
        "bb17c66674dd2bf0fc4a210ce1d3c59b4e4df2eb9cb5bbe3387af355e4c3b020",
        "graspable_object",
        (
            (-0.08399545401334763, -0.15063320100307465, -3.3527612686157227e-08),
            (0.08600454777479172, 0.1406424343585968, 0.18181779980659485),
        ),
        dynamic=True,
        push_height=0.08,
    ),
    "loom:storage_zone": AssetDefinition(
        "storage_zone",
        "loom-env",
        "1",
        "configs/assets/regions/storage_zone.usda",
        "37f3041edcd2798fe938ebb3618ba51ea0424bff4d04f47219bb525badc62839",
        "target_region",
        ((-0.14, -0.19, 0.0002), (0.14, 0.19, 0.0004)),
    ),
    "loom:placemat": AssetDefinition(
        "placemat",
        "loom-env",
        "1",
        "configs/assets/regions/placemat.usda",
        "4e15f0b9800f3610299b0ab2647c89b0ad5bd1aec1f9ad86dcc447f1378d3eb4",
        "target_region",
        ((-0.11, -0.11, 0), (0.11, 0.11, 0.001)),
    ),
}


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
    """Load existing prepared assets, checking file presence and integrity."""
    directory = prepared_directory(root, asset_id)
    path = directory / "asset.json"
    if not path.is_file():
        raise ValueError(
            f"Unprepared asset {asset_id}; run scripts/prepare_assets.py scene"
        )
    manifest = json.loads(path.read_text())
    if manifest.get("preparation_version") != PREPARATION_VERSION:
        raise ValueError(
            f"Obsolete prepared asset {asset_id}; run scripts/prepare_assets.py scene"
        )
    if manifest.get("source") != json.loads(
        json.dumps(asdict(asset_definition(asset_id)))
    ):
        raise ValueError(
            f"Prepared definition changed for {asset_id}; run scripts/prepare_assets.py scene"
        )
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
