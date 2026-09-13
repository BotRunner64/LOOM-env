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
