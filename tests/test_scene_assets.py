import json
import shutil
from dataclasses import asdict

import pytest

from loom_env.assets.catalog import (
    PREPARATION_VERSION,
    asset_definition,
    load_prepared,
    prepared_directory,
    sha256,
)


def test_converted_texture_resolves_after_moving_directory(tmp_path):
    pytest.importorskip("pxr")
    from pxr import Sdf, Usd

    from loom_env.assets.convert import relativize_local_assets

    original = tmp_path / "original"
    original.mkdir()
    texture = original / "textures" / "color.png"
    texture.parent.mkdir()
    texture.write_bytes(b"texture-content")
    usd = original / "mesh.usda"
    stage = Usd.Stage.CreateNew(str(usd))
    prim = stage.DefinePrim("/Material")
    prim.CreateAttribute("texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(texture))
    )
    prim.CreateAttribute("module", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("gltf/pbr.mdl")
    )
    stage.GetRootLayer().Save()
    relativize_local_assets(usd)
    moved = tmp_path / "moved"
    shutil.move(original, moved)
    relocated = Usd.Stage.Open(str(moved / "mesh.usda"))
    material = relocated.GetPrimAtPath("/Material")
    value = material.GetAttribute("texture").Get()
    assert value.path == "textures/color.png"
    assert value.resolvedPath == str(moved / "textures/color.png")
    assert material.GetAttribute("module").Get().path == "gltf/pbr.mdl"


def prepared_fixture(root):
    key = "robodojo:brick"
    directory = prepared_directory(root, key)
    directory.mkdir(parents=True)
    for name in ("asset.usda", "collision.npz", "source.usdz"):
        (directory / name).write_bytes(b"integrity-test-content")
    manifest = {
        "preparation_version": PREPARATION_VERSION,
        "source": json.loads(json.dumps(asdict(asset_definition(key)))),
        "asset_id": key,
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


@pytest.mark.parametrize("change", ["missing", "escape", "incomplete"])
def test_unusable_prepared_assets_are_rejected(tmp_path, change):
    key, directory, manifest = prepared_fixture(tmp_path)
    if change == "missing":
        (directory / "asset.usda").unlink()
    elif change == "escape":
        manifest["files"]["../outside.usd"] = "unexpected"
    else:
        manifest["files"].pop("collision.npz")
    (directory / "asset.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        load_prepared(tmp_path, key)


@pytest.mark.parametrize("layered", [False, True])
def test_prepared_usd_preserves_source_physics(tmp_path, monkeypatch, layered):
    from dataclasses import replace

    pytest.importorskip("pxr.Usd")
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

    from loom_env.assets import prepare
    from loom_env.assets.catalog import ASSETS, AssetDefinition

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
    material = UsdShade.Material.Define(stage, "/Source/Material")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(0.5)
    api.CreateDynamicFrictionAttr(0.4)
    api.CreateRestitutionAttr(0)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        material, materialPurpose="physics"
    )
    stage.GetRootLayer().Save()
    archive = tmp_path / "fixture.usdz"
    assert UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(source)), str(archive))
    entry = archive
    if layered:
        entry = tmp_path / "object.usda"
        wrapper = Usd.Stage.CreateNew(str(entry))
        wrapper_root = UsdGeom.Xform.Define(wrapper, "/Asset").GetPrim()
        wrapper.SetDefaultPrim(wrapper_root)
        UsdGeom.SetStageUpAxis(wrapper, "Z")
        UsdGeom.SetStageMetersPerUnit(wrapper, 1.0)
        wrapper_root.GetReferences().AddReference(archive.name)
        wrapper.GetRootLayer().Save()
    asset = AssetDefinition(
        "fixture",
        "test-source",
        "test-revision",
        entry.name,
        sha256(entry),
        "graspable_object",
        ((0, 0, 0), (0.02, 0.02, 0.02)),
        dynamic=True,
    )
    monkeypatch.setitem(ASSETS, "test:fixture", asset)
    monkeypatch.setattr(
        prepare,
        "ASSETS",
        {
            "test:fixture": asset,
            "test:unselected": replace(asset, name="unselected", path="missing.usdz"),
        },
    )
    with pytest.raises(ValueError, match="Unknown scene assets"):
        prepare.prepare_scene_assets(tmp_path / "cache", tmp_path, ["test:unknown"])
    assert not (tmp_path / "cache").exists()
    prepare.prepare_scene_assets(tmp_path / "cache", tmp_path, ["test:fixture"])
    assert not (tmp_path / "cache/scenes/unselected").exists()
    directory = prepared_directory(tmp_path / "cache", "test:fixture")
    result = Usd.Stage.Open(str(directory / "asset.usda"))
    assert UsdPhysics.MassAPI(
        result.GetDefaultPrim()
    ).GetMassAttr().Get() == pytest.approx(0.123)
    assert prepare.physics_properties(result)["materials"]["collision/model"][
        "staticFriction"
    ] == pytest.approx(0.5)
    assert (
        directory / (archive.name if layered else "source.usdz")
    ).read_bytes() == archive.read_bytes()
    layer = result.GetRootLayer().ExportToString()
    assert "physics:mass" not in layer
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


@pytest.mark.parametrize("key", ["loom:placemat", "loom:storage_zone"])
def test_region_preparation_matches_physics_and_planner_geometry(
    tmp_path, monkeypatch, key
):
    pytest.importorskip("pxr.Usd")
    import numpy as np
    from pxr import Usd, UsdPhysics

    from loom_env.assets import prepare
    from loom_env.assets.catalog import collision_mesh

    asset = asset_definition(key)
    monkeypatch.setattr(prepare, "ASSETS", {key: asset})
    prepare.prepare_scene_assets(tmp_path, tmp_path)
    stage = Usd.Stage.Open(str(prepared_directory(tmp_path, key) / "asset.usda"))
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    vertices, faces = collision_mesh(tmp_path, key)
    if key == "loom:placemat":
        assert len(colliders) == 1
        assert len(faces) == 12
        np.testing.assert_allclose(
            [vertices.min(0), vertices.max(0)], asset.bounds, atol=1e-8
        )
    else:
        assert not colliders
        assert vertices.shape == faces.shape == (0, 3)
    assert load_prepared(tmp_path, key)["preparation_version"] == PREPARATION_VERSION


def test_incomplete_usd_physics_is_rejected():
    from pxr import Usd, UsdGeom, UsdPhysics

    from loom_env.assets.prepare import physics_properties

    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, "/Asset/Body").GetPrim())
    with pytest.raises(ValueError, match="explicit positive mass"):
        physics_properties(stage)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.1)
    with pytest.raises(ValueError, match="Missing bound physics material"):
        physics_properties(stage)


def test_old_physics_cache_requires_repreparation(tmp_path):
    key, directory, manifest = prepared_fixture(tmp_path)
    manifest.pop("preparation_version")
    (directory / "asset.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="Obsolete"):
        load_prepared(tmp_path, key)


@pytest.mark.parametrize("missing", [None, "mass", "material", "second_body"])
def test_library_checks_child_body_physics(tmp_path, missing):
    import runpy
    from pathlib import Path

    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    inspect_asset = runpy.run_path(
        str(Path(__file__).resolve().parents[1] / "scripts/check_asset_library.py")
    )["inspect_asset"]
    path = tmp_path / "child-body.usda"
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, "/Object").GetPrim()
    stage.SetDefaultPrim(root)
    body = UsdGeom.Cube.Define(stage, "/Object/Body").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    UsdPhysics.CollisionAPI.Apply(body)
    if missing != "mass":
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(0.02)
    if missing != "material":
        material = UsdShade.Material.Define(stage, "/Object/Material")
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(0.5)
        api.CreateDynamicFrictionAttr(0.4)
        api.CreateRestitutionAttr(0)
        UsdShade.MaterialBindingAPI.Apply(body).Bind(
            material, materialPurpose="physics"
        )
    if missing == "second_body":
        UsdPhysics.RigidBodyAPI.Apply(
            UsdGeom.Cube.Define(stage, "/Object/Other").GetPrim()
        )
    stage.GetRootLayer().Save()
    result = inspect_asset(path, dynamic=True)
    assert result["status"] == ("ready" if missing is None else "pending")
    if missing is None:
        assert result["physics"]["mass_kg"] == pytest.approx(0.02)
