import json

import pytest

from loom_env.embodiments.assets import (
    PIPER_CONVERSION_ARGS,
    PIPER_SOURCE,
    piper_source_dir,
    piper_usd,
    sha256,
    verify_piper_asset,
)


def test_converted_asset_verification_rejects_tampering(tmp_path):
    root = tmp_path / "assets"
    urdf = piper_source_dir(root) / "piper.urdf"
    usd = piper_usd(root)
    urdf.parent.mkdir(parents=True)
    usd.parent.mkdir(parents=True)
    urdf.write_text('<robot name="fixture"/>')
    usd.write_text("#usda 1.0\n")
    manifest = {
        "source": PIPER_SOURCE,
        "conversion": list(PIPER_CONVERSION_ARGS),
        "urdf_sha256": sha256(urdf),
        "usd_files": {str(usd.relative_to(root)): sha256(usd)},
    }
    path = root / "piper/asset.json"
    path.write_text(json.dumps(manifest))
    assert verify_piper_asset(root) == manifest
    usd.write_text("changed")
    with pytest.raises(ValueError, match="converted asset changed"):
        verify_piper_asset(root)
    path.unlink()
    with pytest.raises(FileNotFoundError, match="prepare_piper"):
        verify_piper_asset(root)
