import json

import pytest

from loom_env.assets.catalog import (
    PREPARATION_VERSION,
    asset_definition,
    load_prepared,
    prepared_directory,
    sha256,
)


def prepared_fixture(root):
    key = "robodojo:brick"
    directory = prepared_directory(root, key)
    directory.mkdir(parents=True)
    for name in ("asset.usda", "collision.npz", "source.usdz"):
        (directory / name).write_bytes(b"integrity-test-content")
    manifest = {
        "asset_id": key,
        "definition": asset_definition(key).fingerprint,
        "preparation_version": PREPARATION_VERSION,
        "files": {
            name: sha256(directory / name)
            for name in ("asset.usda", "collision.npz", "source.usdz")
        },
    }
    (directory / "asset.json").write_text(json.dumps(manifest))
    return key, directory, manifest


@pytest.mark.parametrize("name", ["asset.usda", "collision.npz", "source.usdz"])
def test_modified_asset_or_collision_cache_is_rejected(tmp_path, name):
    key, directory, _ = prepared_fixture(tmp_path)
    load_prepared(tmp_path, key)
    (directory / name).write_bytes(b"modified")
    with pytest.raises(ValueError, match="checksum"):
        load_prepared(tmp_path, key)


@pytest.mark.parametrize("change", ["missing", "stale", "escape", "incomplete"])
def test_unusable_prepared_assets_are_rejected(tmp_path, change):
    key, directory, manifest = prepared_fixture(tmp_path)
    if change == "missing":
        (directory / "asset.usda").unlink()
    elif change == "stale":
        manifest["definition"] = "different-annotations"
    elif change == "escape":
        manifest["files"]["../outside.usd"] = "unexpected"
    else:
        manifest["files"].pop("collision.npz")
    (directory / "asset.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        load_prepared(tmp_path, key)


def test_usdz_reference_preserves_authored_physics(tmp_path, monkeypatch):
    pytest.importorskip("pxr.Usd")
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils
    from loom_env.assets import prepare
    from loom_env.assets.catalog import AssetDefinition, ASSETS

    source = tmp_path / "fixture.usda"
    stage = Usd.Stage.CreateNew(str(source))
    root = UsdGeom.Xform.Define(stage, "/Source").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.123)
    mesh = UsdGeom.Mesh.Define(stage, "/Source/collision/model")
    mesh.CreatePointsAttr([(0, 0, 0), (0.02, 0, 0), (0, 0.02, 0), (0, 0, 0.02)])
    mesh.CreateFaceVertexCountsAttr([3, 3, 3, 3])
    mesh.CreateFaceVertexIndicesAttr([0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        "convexDecomposition"
    )
    stage.GetRootLayer().Save()
    archive = tmp_path / "fixture.usdz"
    assert UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(source)), str(archive))
    asset = AssetDefinition(
        "fixture",
        "test-source",
        "test-revision",
        archive.name,
        sha256(archive),
        "graspable_object",
        ((0, 0, 0), (0.02, 0.02, 0.02)),
        dynamic=True,
    )
    monkeypatch.setitem(ASSETS, "test:fixture", asset)
    monkeypatch.setattr(prepare, "ASSETS", {"test:fixture": asset})
    prepare.prepare_scene_assets(tmp_path / "cache", tmp_path)
    directory = prepared_directory(tmp_path / "cache", "test:fixture")
    result = Usd.Stage.Open(str(directory / "asset.usda"))
    original = Usd.Stage.Open(str(archive))
    assert prepare.source_physics(result) == prepare.source_physics(original)
    assert (directory / "source.usdz").read_bytes() == archive.read_bytes()
    layer = result.GetRootLayer().ExportToString()
    assert "physics:" not in layer
    assert "PhysicsRigidBodyAPI" not in layer  # Inherited, never rewritten.
    assert (
        UsdPhysics.RigidBodyAPI(result.GetDefaultPrim()).GetRigidBodyEnabledAttr().Get()
    )
    assert (
        UsdPhysics.MeshCollisionAPI(result.GetPrimAtPath("/Asset/collision/model"))
        .GetApproximationAttr()
        .Get()
        == "convexDecomposition"
    )


def test_tabletop_replacement_preserves_lower_geometry():
    import numpy as np
    import trimesh
    from loom_env.assets.prepare import table_collision_parts

    top = trimesh.creation.box(extents=(1, 2, 0.04))
    top.apply_translation((0, 0, -0.02))
    leg = trimesh.creation.box(extents=(0.1, 0.1, 0.7))
    leg.apply_translation((0.3, 0.6, -0.4))
    source = trimesh.util.concatenate([top, leg])
    lower, slab = table_collision_parts(source.vertices, source.faces, top.bounds)
    np.testing.assert_allclose(lower.bounds, leg.bounds)
    assert lower.volume == pytest.approx(leg.volume)
    np.testing.assert_allclose(slab.bounds, top.bounds)
    assert slab.is_watertight
    with pytest.raises(ValueError, match="one tabletop"):
        table_collision_parts(source.vertices, source.faces, top.bounds + 0.1)
