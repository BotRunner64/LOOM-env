import json

import pytest

from loom_env.embodiments.assets import (
    CONVERSION_ARGS,
    PREPARATION_VERSION,
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
        path.write_text("fixture")
    manifest = {
        "source": SOURCES[MODELS[name]["source"]],
        "conversion": list(CONVERSION_ARGS),
        "preparation_version": PREPARATION_VERSION,
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
