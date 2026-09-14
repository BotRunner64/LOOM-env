import json

import pytest

from loom_env.embodiments.assets import (
    CONVERSION_ARGS,
    preparation_version,
    SOURCES,
    MODELS,
    source_urdf,
    prepared_urdf,
    converted_usd,
    sha256,
    verify_asset,
)


@pytest.mark.parametrize("name", MODELS)
def test_converted_asset_verification_rejects_tampering(tmp_path, name):
    root = tmp_path / "assets"
    original = source_urdf(root, name)
    urdf = prepared_urdf(root, name)
    usd = converted_usd(root, name)
    for path in (original, urdf, usd):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            '<robot name="fixture"/>' if path.suffix == ".urdf" else "fixture"
        )
    manifest = {
        "source": SOURCES[MODELS[name]["source"]],
        "conversion": list(CONVERSION_ARGS),
        "preparation_version": preparation_version(name),
        "prepared_mesh_files": {},
        "source_urdf_sha256": sha256(original),
        "urdf_sha256": sha256(urdf),
        "usd_files": {str(usd.relative_to(root)): sha256(usd)},
    }
    path = root / name / "asset.json"
    path.write_text(json.dumps(manifest))
    assert verify_asset(root, name) == manifest
    usd.write_text("changed")
    with pytest.raises(ValueError, match="Converted asset changed"):
        verify_asset(root, name)
    path.unlink()
    with pytest.raises(FileNotFoundError, match="prepare_assets"):
        verify_asset(root, name)


def test_verified_hash_does_not_allow_mirrored_collision(tmp_path):
    name = "yam"
    root = tmp_path / "assets"
    original, urdf, usd = (
        source_urdf(root, name),
        prepared_urdf(root, name),
        converted_usd(root, name),
    )
    for file in (original, urdf, usd):
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text('<robot name="fixture"/>')
    urdf.write_text(
        '<robot name="fixture"><link name="finger"><collision><geometry><mesh filename="finger.stl" scale=".001 -.001 .001"/></geometry></collision></link></robot>'
    )
    manifest = {
        "source": SOURCES[MODELS[name]["source"]],
        "conversion": list(CONVERSION_ARGS),
        "preparation_version": preparation_version(name),
        "prepared_mesh_files": {},
        "source_urdf_sha256": sha256(original),
        "urdf_sha256": sha256(urdf),
        "usd_files": {str(usd.relative_to(root)): sha256(usd)},
    }
    (root / name / "asset.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="mirrored collision"):
        verify_asset(root, name)


def test_ur5_preparation_restores_official_ranges_and_preserves_source(tmp_path):
    import math
    from pathlib import Path
    import runpy
    import xml.etree.ElementTree as ET

    from loom_env.specs.config import load_deployment

    repo = Path(__file__).parents[1]
    deployment = load_deployment(repo / "configs/deployments/dual_ur5_wsg.yaml")
    arm = deployment.arms["right"]
    root = ET.Element("robot", name="source")
    for name in arm.joint_names:
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(
            joint, "limit", lower="-1.75", upper="1.75", effort="100", velocity="3.2"
        )
    finger = ET.SubElement(
        root, "joint", name="base_joint_gripper_left", type="prismatic"
    )
    ET.SubElement(
        finger, "limit", lower="-0.055", upper="-0.0027", effort="1", velocity="1"
    )
    original = source_urdf(tmp_path, "ur5_wsg")
    original.parent.mkdir(parents=True)
    ET.ElementTree(root).write(original)
    before = original.read_bytes()
    normalize = runpy.run_path(str(repo / "scripts/prepare_assets.py"))[
        "normalize_urdf"
    ]
    changes = normalize(tmp_path, "ur5_wsg")
    prepared = ET.parse(prepared_urdf(tmp_path, "ur5_wsg"))
    assert original.read_bytes() == before
    for name, bounds in zip(arm.joint_names, arm.joint_limits):
        limit = prepared.find(f"joint[@name='{name}']/limit")
        assert tuple(float(limit.get(key)) for key in ("lower", "upper")) == bounds
        assert bounds == (-math.tau, math.tau)
        assert (limit.get("effort"), limit.get("velocity")) == ("100", "3.2")
    assert (
        prepared.find("joint[@name='base_joint_gripper_left']/limit").get("lower")
        == "-0.055"
    )
    assert (
        sum("Restore official UR5 position range" in change for change in changes) == 6
    )
    assert preparation_version("ur5_wsg") == 4
