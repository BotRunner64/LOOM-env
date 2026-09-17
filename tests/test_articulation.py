import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.experts.articulation import hinge_step
from loom_env.specs.config import load_collection
from loom_env.tasks import create_task


def test_hinge_step_preserves_radius_and_rotates_gripper_frame():
    body = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    hinge = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    result = hinge_step(body, hinge, [0, 0, 1], np.pi / 2)
    np.testing.assert_allclose(result[:3], [1, 1, 0], atol=1e-12)
    np.testing.assert_allclose(
        Rotation.from_quat(result[3:]).apply([1, 0, 0]), [0, 1, 0], atol=1e-12
    )


def test_opening_requires_contact_history_and_release():
    task = create_task(load_collection("configs/collection/open_laptop.yaml"))
    world = {task.q_key: np.array(-0.4), task.force_key: np.zeros((2, 2, 3))}
    task.reset(world)
    world[task.q_key] = np.array(task.parameters["target_angle"])
    for _ in range(10):
        assert task.update(world, 0.05).outcome is None
    world[task.force_key][task.side] = [[0, 2, 0], [0, -2, 0]]
    for _ in range(10):
        assert task.update(world, 0.05).outcome is None
    world[task.force_key][:] = 0
    for _ in range(5):
        assert task.update(world, 0.05).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"
    with pytest.raises(ValueError, match="outside"):
        task.reset(world)


def test_usd_hinge_preparation_exports_link_local_collisions(tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    from loom_env.assets.articulation import describe
    from loom_env.assets.catalog import asset_definition

    asset = asset_definition("robodojo:laptop_fixed")
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/Asset")
    root.AddTranslateOp().Set((10, 0, 0))
    stage.SetDefaultPrim(root.GetPrim())
    material = UsdShade.Material.Define(stage, "/Asset/Material")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(0.5)
    api.CreateDynamicFrictionAttr(0.4)
    api.CreateRestitutionAttr(0)
    for name in (asset.root_body, asset.moving_body):
        body = UsdGeom.Cube.Define(stage, "/Asset/" + name)
        body.CreateSizeAttr(0.1)
        body.AddTranslateOp().Set((1, 0, 0))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.CollisionAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(0.1)
        UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(
            material, materialPurpose="physics"
        )
    hinge = UsdPhysics.RevoluteJoint.Define(stage, "/Asset/" + asset.joint)
    hinge.CreateBody0Rel().SetTargets(["/Asset/" + asset.root_body])
    hinge.CreateBody1Rel().SetTargets(["/Asset/" + asset.moving_body])
    hinge.CreateLowerLimitAttr(-110)
    hinge.CreateUpperLimitAttr(0)
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Asset/FixedBase")
    fixed.CreateBody1Rel().SetTargets(["/Asset/" + asset.root_body])
    report = describe(stage, asset, tmp_path)
    np.testing.assert_allclose(report["joint"]["limits_rad"], np.deg2rad([-110, 0]))
    np.testing.assert_allclose(
        report["bodies"][asset.root_body]["pose_asset"][:3], [1, 0, 0]
    )
    with np.load(tmp_path / f"collision-{asset.moving_body}.npz") as mesh:
        np.testing.assert_allclose(mesh["vertices"].min(0), [-0.05] * 3)
        np.testing.assert_allclose(mesh["vertices"].max(0), [0.05] * 3)
    fixed.CreateBody1Rel().SetTargets(["/Asset/" + asset.moving_body])
    with pytest.raises(ValueError, match="world-fixed"):
        describe(stage, asset)
