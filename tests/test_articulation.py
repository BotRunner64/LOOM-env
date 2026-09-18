import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.experts.articulation import joint_step
from loom_env.specs.config import load_collection
from loom_env.tasks import create_task


def test_hinge_step_preserves_radius_and_rotates_gripper_frame():
    body = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    hinge = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    result = joint_step(body, hinge, [0, 0, 1], np.pi / 2, "revolute")
    np.testing.assert_allclose(result[:3], [1, 1, 0], atol=1e-12)
    np.testing.assert_allclose(
        Rotation.from_quat(result[3:]).apply([1, 0, 0]), [0, 1, 0], atol=1e-12
    )


@pytest.mark.parametrize(
    ("collection", "initial"),
    [("open_laptop_small", -np.deg2rad(35)), ("close_laptop_partway", -np.deg2rad(95))],
)
def test_hinge_target_requires_contact_history_and_release(collection, initial):
    task = create_task(load_collection(f"configs/collection/{collection}.yaml"))
    world = {task.q_key: np.array(initial), task.force_key: np.zeros((2, 2, 3))}
    task.reset(world)
    world[task.q_key] = np.array(task.parameters["target_position"])
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


@pytest.mark.parametrize("joint_type", ["revolute", "prismatic"])
def test_usd_hinge_preparation_exports_link_local_collisions(tmp_path, joint_type):
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
    schema = (
        UsdPhysics.RevoluteJoint
        if joint_type == "revolute"
        else UsdPhysics.PrismaticJoint
    )
    hinge = schema.Define(stage, "/Asset/" + asset.joint)
    hinge.CreateBody0Rel().SetTargets(["/Asset/" + asset.root_body])
    hinge.CreateBody1Rel().SetTargets(["/Asset/" + asset.moving_body])
    hinge.CreateLowerLimitAttr(-110)
    hinge.CreateUpperLimitAttr(0)
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Asset/FixedBase")
    fixed.CreateBody1Rel().SetTargets(["/Asset/" + asset.root_body])
    report = describe(stage, asset, tmp_path)
    np.testing.assert_allclose(
        report["joint"]["limits"],
        np.deg2rad([-110, 0]) if joint_type == "revolute" else [-110, 0],
    )
    assert report["joint"]["unit"] == ("rad" if joint_type == "revolute" else "m")
    np.testing.assert_allclose(
        report["bodies"][asset.root_body]["pose_asset"][:3], [1, 0, 0]
    )
    with np.load(tmp_path / f"collision-{asset.moving_body}.npz") as mesh:
        np.testing.assert_allclose(mesh["vertices"].min(0), [-0.05] * 3)
        np.testing.assert_allclose(mesh["vertices"].max(0), [0.05] * 3)
    fixed.CreateBody1Rel().SetTargets(["/Asset/" + asset.moving_body])
    with pytest.raises(ValueError, match="world-fixed"):
        describe(stage, asset)


@pytest.mark.parametrize(
    ("collection", "wrong_initial"),
    [("open_laptop_small", -np.deg2rad(95)), ("close_laptop_partway", -np.deg2rad(25))],
)
def test_hinge_task_rejects_reversed_instruction(collection, wrong_initial):
    task = create_task(load_collection(f"configs/collection/{collection}.yaml"))
    with pytest.raises(ValueError, match="direction"):
        task.reset({task.q_key: np.array(wrong_initial)})


def test_prismatic_motion_uses_world_joint_axis_and_preserves_orientation():
    frame = np.r_[[0.4, 0.3, 0.2], Rotation.from_euler("y", 90, degrees=True).as_quat()]
    tcp = np.r_[[0.1, 0.2, 0.3], Rotation.from_euler("z", 0.5).as_quat()]
    moved = joint_step(tcp, frame, [0, 0, 1], -0.025, "prismatic")
    np.testing.assert_allclose(moved[:3], tcp[:3] + [-0.025, 0, 0])
    np.testing.assert_allclose(moved[3:], tcp[3:])


def test_contact_frame_is_link_local_and_independent_of_task_name(monkeypatch):
    from dataclasses import replace
    from types import SimpleNamespace

    from loom_env.assets.catalog import ASSETS
    from loom_env.experts.articulation import ArticulationExpert
    from loom_env.scenes.workspace import transform

    collection = load_collection("configs/collection/open_laptop.yaml")
    contact = (0.03, 0.04, 0.05, 0, 0, 0, 1)
    monkeypatch.setitem(
        ASSETS,
        "test:hinged_panel",
        replace(ASSETS["robodojo:laptop_fixed"], contact_poses=(contact,)),
    )
    objects = {
        **collection.scene.objects,
        "laptop": {**collection.scene.objects["laptop"], "asset": "test:hinged_panel"},
    }
    collection = replace(
        collection,
        scene=replace(collection.scene, objects=objects),
        task=replace(
            collection.task,
            id="move_panel",
            parameters={**collection.task.parameters, "contact_index": 0},
        ),
    )
    monkeypatch.setattr(
        "loom_env.experts.articulation.load_prepared",
        lambda *_: {
            "physics_properties": {"joint": {"type": "revolute", "limits": [-3, 3]}}
        },
    )
    planner = SimpleNamespace(
        asset_root=None, profile=SimpleNamespace(tcp_to_grasp=[0, 0, 0.1])
    )
    expert = ArticulationExpert(collection, planner, dict)
    body = np.r_[[0.2, 0.3, 0.4], Rotation.from_euler("y", 90, degrees=True).as_quat()]
    expected = transform(body, contact)
    expected[:3] -= Rotation.from_quat(expected[3:]).apply([0, 0, 0.165])
    np.testing.assert_allclose(expert._tcp(body, 0.065), expected)


def test_joint_action_starts_from_measured_link_grasp():
    from types import SimpleNamespace

    from loom_env.experts.articulation import MoveJoint
    from loom_env.runtime.runner import SourceFailure

    world = {
        "panel/joints/hinge/position": np.array(0.4),
        "panel/links/lid/finger_contact_forces_world": np.zeros((2, 2, 3)),
        "panel/links/base/pose_world": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
    }
    goals = []
    arm = SimpleNamespace(
        obj="panel",
        side="right",
        asset=SimpleNamespace(joint="hinge", moving_body="lid", root_body="base"),
        world=world,
        tcp=np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        dt=0.05,
        tick=0,
        closed=0.0,
        grip_slice=slice(7, 8),
        arm_slice=slice(0, 7),
        command=np.zeros(8),
        observation=None,
        event=lambda *args: None,
        planner=SimpleNamespace(
            cartesian_step=lambda obs, state, goal: (
                goals.append(goal.copy()) or np.zeros(7)
            )
        ),
    )
    joint = {
        "type": "revolute",
        "axis": [0, 0, 1],
        "pose_parent": [0, 0, 0, 0, 0, 0, 1],
    }
    with pytest.raises(SourceFailure, match="moving link"):
        MoveJoint(joint, 0.8, 0.01, 0.2).step(arm)
    world["panel/links/lid/finger_contact_forces_world"][1] = [[0, 2, 0], [0, -2, 0]]
    move = MoveJoint(joint, 0.8, 0.01, 0.2)
    assert not move.step(arm)
    np.testing.assert_allclose(goals[0][:3], [-np.sin(0.01), np.cos(0.01), 0])
    # Completion uses measured q, not the commanded reference reaching target.
    world["panel/joints/hinge/position"] = np.array(0.8)
    assert move.step(arm)
