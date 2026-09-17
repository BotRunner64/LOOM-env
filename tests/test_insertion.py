from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.scenes.workspace import sample_objects
from loom_env.specs.config import load_collection, plain
from loom_env.tasks import create_task

ROOT = Path(__file__).parents[1]


@pytest.fixture
def insertion():
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    task = create_task(collection)
    world = {
        f"{n}/pose_world": p for n, p in sample_objects(collection.scene, 0).items()
    }
    world["coin/grasped_by"] = np.zeros(2, dtype=bool)
    world["coin/finger_contact_forces_world"] = np.zeros((2, 2, 3))
    task.reset(world)
    return task, world


def at_depth(task, world, depth):
    fixture = task.geometry.frame(world, task.geometry.target)
    frame = Rotation.from_quat(fixture[3:])
    orientation = frame * Rotation.from_euler("x", 90, degrees=True)
    center = frame.apply([0, 0, task.geometry.body.radius - depth]) + fixture[:3]
    world["coin/pose_world"] = np.r_[
        center - orientation.apply(task.geometry.body.center), orientation.as_quat()
    ]


def approach_and_insert(task, world):
    world["coin/grasped_by"][1] = True
    world["coin/pose_world"][2] += 0.08
    task.update(world, 0.05)
    at_depth(task, world, -0.005)
    task.update(world, 0.05)
    at_depth(task, world, 0.012)
    task.update(world, 0.05)


def test_success_requires_extraction_approach_held_insertion_and_release(insertion):
    task, world = insertion
    at_depth(task, world, 0.012)
    assert task.update(world, 1).outcome is None
    approach_and_insert(task, world)
    assert task.update(world, 1).outcome is None  # Still grasped.
    world["coin/grasped_by"][:] = False
    world["coin/finger_contact_forces_world"][1, 0, 0] = 1
    assert task.update(world, 1).outcome is None  # Finger still touching.
    world["coin/finger_contact_forces_world"][:] = 0
    for _ in range(4):
        assert task.update(world, 0.05).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


def test_drop_into_slot_without_held_entry_is_not_success(insertion):
    task, world = insertion
    world["coin/grasped_by"][1] = True
    world["coin/pose_world"][2] += 0.08
    task.update(world, 0.05)
    at_depth(task, world, -0.005)
    task.update(world, 0.05)
    world["coin/grasped_by"][:] = False
    at_depth(task, world, 0.012)
    assert task.update(world, 1).outcome is None


@pytest.mark.parametrize("kind", ["across", "along", "tilted", "shallow", "source"])
def test_incorrect_placement_rejected(insertion, kind):
    task, world = insertion
    approach_and_insert(task, world)
    world["coin/grasped_by"][:] = False
    frame = Rotation.from_quat(world[f"{task.geometry.target}/pose_world"][3:])
    if kind == "across":
        world["coin/pose_world"][:3] += frame.apply([0, 0.02, 0])
    elif kind == "along":
        world["coin/pose_world"][:3] += frame.apply([0.04, 0, 0])
    elif kind == "tilted":
        world["coin/pose_world"][3:] = [0, 0, 0, 1]
    elif kind == "shallow":
        at_depth(task, world, 0.003)
    else:
        world["coin/pose_world"][:3] += (
            world[f"{task.geometry.source}/pose_world"][:3]
            - world[f"{task.geometry.target}/pose_world"][:3]
        )
    result = task.update(world, 1).outcome
    assert result is None or result.code != "success"


def test_coin_spin_does_not_change_plane_alignment(insertion):
    task, world = insertion
    approach_and_insert(task, world)
    pose = world["coin/pose_world"]
    old = task.metrics(world)
    pose[3:] = (
        Rotation.from_quat(pose[3:]) * Rotation.from_euler("z", 73, degrees=True)
    ).as_quat()
    new = task.metrics(world)
    assert new["angle"] == pytest.approx(old["angle"])
    assert new["depth"] == pytest.approx(old["depth"])


def test_released_coin_can_be_off_center_and_tilted(insertion):
    task, world = insertion
    approach_and_insert(task, world)
    frame = Rotation.from_quat(world[f"{task.geometry.target}/pose_world"][3:])
    pose = world["coin/pose_world"]
    center = Rotation.from_quat(pose[3:]).apply(task.geometry.body.center) + pose[:3]
    rotation = frame * Rotation.from_euler("x", 95, degrees=True)
    pose[3:] = rotation.as_quat()
    pose[:3] = (
        center
        + frame.apply([0.002, 0.001, 0])
        - rotation.apply(task.geometry.body.center)
    )
    world["coin/grasped_by"][:] = False
    assert task.metrics(world)["angle"] > np.deg2rad(2)
    assert task.update(world, 0.25).outcome.code == "success"


def test_coin_must_remain_in_target_after_release(insertion):
    task, world = insertion
    approach_and_insert(task, world)
    world["coin/grasped_by"][:] = False
    assert task.update(world, 0.15).outcome is None
    frame = Rotation.from_quat(world[f"{task.geometry.target}/pose_world"][3:])
    world["coin/pose_world"][:3] += frame.apply([0, 0.02, 0])
    assert task.update(world, 0.15).outcome is None
    world["coin/pose_world"][:3] -= frame.apply([0, 0.02, 0])
    assert task.update(world, 0.15).outcome is None
    assert task.update(world, 0.1).outcome.code == "success"


def test_excessive_depth_is_failure(insertion):
    task, world = insertion
    at_depth(task, world, 0.025)
    assert task.update(world, 0.05).outcome.reason == "object_below_socket_floor"


def test_explicit_initial_fixture_does_not_disable_other_separation():
    scene = load_collection(ROOT / "configs/collection/insert_coin.yaml").scene
    sample_objects(scene, 0)
    objects = plain(scene.objects)
    objects["coin"].pop("initial_fixture")
    with pytest.raises(ValueError, match="separated"):
        sample_objects(replace(scene, objects=objects), 0)
    objects = plain(scene.objects)
    objects["coin"]["initial_fixture"] = "target_slot"
    with pytest.raises(ValueError, match="outside initial fixture"):
        sample_objects(replace(scene, objects=objects), 0)


def test_thin_coin_grasp_still_requires_real_opposing_contacts():
    from loom_env.embodiments.contacts import opposing_contacts

    force = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]])
    assert opposing_contacts(force)
    assert not opposing_contacts(np.zeros((2, 3)))
    assert not opposing_contacts(abs(force))
    assert not opposing_contacts(force * 0.1)


def test_preinsertion_goal_compensates_measured_coin_slip(insertion):
    from loom_env.assets.insertion import InsertionGeometry
    from loom_env.experts.insertion import Insert
    from loom_env.specs.episode import Observation

    _, world = insertion
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    expert = Insert(InsertionGeometry(collection), collection.task.parameters["depth"])
    tcp = np.r_[world["coin/pose_world"][:3] + [0.0, 0.0, 0.12], [1.0, 0.0, 0.0, 0.0]]
    observation = Observation(0.0, {"robot/right/tcp_pose_world": tcp})
    first = expert.goal(observation, world, -0.004)
    slip = np.array([0.001, -0.002, 0.003])
    world["coin/pose_world"][:3] += slip
    second = expert.goal(observation, world, -0.004)
    # The physical coin is untouched; the commanded TCP compensates its measured slip.
    np.testing.assert_allclose(
        second[:3] - first[:3],
        -Rotation.from_quat(first[3:]).apply(
            Rotation.from_quat(tcp[3:]).inv().apply(slip)
        ),
        atol=1e-8,
    )
    np.testing.assert_allclose(second[3:], first[3:], atol=1e-8)


def test_insertion_does_not_chase_coin_tilt(insertion):
    from loom_env.assets.insertion import InsertionGeometry
    from loom_env.experts.insertion import Insert
    from loom_env.specs.episode import Observation

    _, world = insertion
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    expert = Insert(InsertionGeometry(collection), collection.task.parameters["depth"])
    tcp = np.r_[world["coin/pose_world"][:3] + [0, 0, 0.12], [1, 0, 0, 0]]
    observation = Observation(0.0, {"robot/right/tcp_pose_world": tcp})
    expert.goal(observation, world, -0.004)
    first = expert.goal(observation, world, 0.012, hold_orientation=True)
    np.testing.assert_allclose(first[3:], tcp[3:], atol=1e-8)
    world["coin/pose_world"][3:] = (
        Rotation.from_euler("y", 5, degrees=True)
        * Rotation.from_quat(world["coin/pose_world"][3:])
    ).as_quat()
    second = expert.goal(observation, world, 0.012, hold_orientation=True)
    # Coin rotation alone must not induce TCP rotation or a pivot translation.
    np.testing.assert_allclose(second, first, atol=1e-8)
    moved_tcp = tcp.copy()
    moved_tcp[3:] = (
        Rotation.from_euler("y", 1, degrees=True) * Rotation.from_quat(tcp[3:])
    ).as_quat()
    third = expert.goal(
        Observation(0.05, {"robot/right/tcp_pose_world": moved_tcp}),
        world,
        0.012,
        hold_orientation=True,
    )
    np.testing.assert_allclose(third[3:], first[3:], atol=1e-8)
    fresh = Insert(InsertionGeometry(collection), collection.task.parameters["depth"])
    assert fresh.insert_rotation is None


def test_insertion_accepts_readonly_world_poses(insertion):
    from loom_env.assets.insertion import InsertionGeometry
    from loom_env.experts.insertion import Insert
    from loom_env.specs.episode import Observation

    task, world = insertion
    expected = task.metrics(world)
    for value in world.values():
        value.flags.writeable = False
    assert task.metrics(world) == expected
    assert task.update(world, 0.05).outcome is None
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    expert = Insert(InsertionGeometry(collection), collection.task.parameters["depth"])
    tcp = np.r_[world["coin/pose_world"][:3] + [0, 0, 0.12], [1, 0, 0, 0]]
    observation = Observation(0.0, {"robot/right/tcp_pose_world": tcp})
    assert np.isfinite(expert.goal(observation, world, -0.004)).all()


def test_insertion_features_support_new_asset_ids_and_local_frames(
    insertion, monkeypatch
):
    from loom_env.assets.catalog import ASSETS
    from loom_env.assets.insertion import InsertionGeometry
    from loom_env.experts.insertion import Insert
    from loom_env.scenes.workspace import transform
    from loom_env.specs.episode import Observation

    original, world = insertion
    expected = original.metrics(world)
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    expert = Insert(InsertionGeometry(collection), collection.task.parameters["depth"])
    tcp = np.array([0.4, -0.2, 0.9, 1, 0, 0, 0])
    observation = Observation(0, {"robot/right/tcp_pose_world": tcp})
    expected_goal = expert.goal(observation, world, -0.004)
    objects = plain(collection.scene.objects)
    # Relabel assets and rotate their local coordinate systems without changing
    # physical mating geometry. Neither task nor controller may know the old IDs.
    turn = Rotation.from_euler("xyz", [20, 35, 55], degrees=True)
    coin = ASSETS["robodojo:coin"]
    body = replace(
        coin.insertion_body,
        center=tuple(turn.inv().apply(coin.insertion_body.center)),
        axis=tuple(turn.inv().apply(coin.insertion_body.axis)),
    )
    monkeypatch.setitem(ASSETS, "test:disc", replace(coin, insertion_body=body))
    socket = ASSETS["robodojo:coin_slot"]
    inverse = np.r_[[0, 0, 0], turn.inv().as_quat()]
    feature = replace(
        socket.insertion_socket,
        pose=tuple(transform(inverse, socket.insertion_socket.pose)),
    )
    monkeypatch.setitem(ASSETS, "test:slot", replace(socket, insertion_socket=feature))
    objects["coin"]["asset"] = "test:disc"
    for name in ("source_slot", "target_slot"):
        objects[name]["asset"] = "test:slot"
    for name in ("coin", "source_slot", "target_slot"):
        pose = world[f"{name}/pose_world"].copy()
        pose[3:] = (Rotation.from_quat(pose[3:]) * turn).as_quat()
        world[f"{name}/pose_world"] = pose
    changed = replace(collection, scene=replace(collection.scene, objects=objects))
    task = create_task(changed)
    expert = Insert(InsertionGeometry(changed), changed.task.parameters["depth"])
    actual_goal = expert.goal(observation, world, -0.004)
    np.testing.assert_allclose(actual_goal[:3], expected_goal[:3], atol=1e-10)
    delta = (
        Rotation.from_quat(actual_goal[3:])
        * Rotation.from_quat(expected_goal[3:]).inv()
    )
    assert delta.magnitude() < 1e-10
    actual = task.metrics(world)
    for key in expected:
        assert actual[key] == pytest.approx(expected[key], abs=1e-10)


def test_shared_lift_extracts_along_fixture_frame(insertion):
    from types import SimpleNamespace

    from loom_env.experts.pick_place import LiftExpert, lift_from_support
    from loom_env.scenes.workspace import workspace
    from loom_env.specs.episode import Observation

    _, world = insertion
    collection = load_collection(ROOT / "configs/collection/lift_coin.yaml")
    world["coin/grasped_by"][1] = True
    world["source_slot/pose_world"][3:] = Rotation.from_euler(
        "y", 90, degrees=True
    ).as_quat()
    goals = []

    def step(observation, truth, goal, **kwargs):
        goals.append((goal.copy(), kwargs))
        return np.zeros(7)

    expert = LiftExpert(
        collection,
        SimpleNamespace(detach=lambda: None, cartesian_step=step),
        lambda: world,
    )
    expert.reset(None)
    tcp = np.array([0.4, 0.2, 0.9, 1, 0, 0, 0])
    arm = expert.arm
    arm.update(
        Observation(
            0,
            {
                "robot/right/tcp_pose_world": tcp,
                "robot/left/joint_position": np.array(
                    collection.deployment.arms["left"].initial_positions
                ),
            },
        ),
        world,
    )
    action = lift_from_support(arm, workspace(collection.scene)[0])
    action.step(arm)
    np.testing.assert_allclose(goals[0][0][:3] - tcp[:3], [0.001, 0, 0], atol=1e-12)
    assert goals[0][1]["contact_objects"] == ("source_slot",)


def test_existing_held_part_skips_grasp_and_extraction(insertion):
    from types import SimpleNamespace

    from loom_env.experts.actions import Grasp, MoveHeld
    from loom_env.experts.insertion import InsertionExpert
    from loom_env.specs.episode import Observation

    _, world = insertion
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    world["coin/grasped_by"][1] = True
    world["coin/pose_world"][2] += 0.08
    attached = []
    planner = SimpleNamespace(
        attached=False,
        detach=lambda: None,
        attach=lambda *args, **kwargs: attached.append(True),
    )
    expert = InsertionExpert(collection, planner, lambda: world)
    expert.reset(None)
    arm = expert.arm
    observation = Observation(
        0,
        {
            "robot/left/joint_position": np.array(
                collection.deployment.arms["left"].initial_positions
            ),
            "robot/right/joint_position": np.array(
                collection.deployment.arms["right"].initial_positions
            ),
            "robot/right/tcp_pose_world": np.r_[
                world["coin/pose_world"][:3] + [0, 0, 0.12], [1, 0, 0, 0]
            ],
        },
    )
    arm.update(observation, world)
    routine = expert.routine()
    grasp = next(routine)
    assert isinstance(grasp, Grasp)
    assert grasp.step(arm)  # No planner.plan exists; no approach is attempted.
    transfer = next(routine)
    assert isinstance(transfer, MoveHeld)  # No extraction action was requested.
    planner.plan = lambda observation, *_args, **_kwargs: (
        np.array([observation.values["robot/right/joint_position"]]),
        {},
    )
    transfer.step(arm)
    assert attached == [True]


def test_insert_action_starts_with_held_part_at_entry(insertion):
    from types import SimpleNamespace

    from loom_env.experts.actions import Manipulator
    from loom_env.experts.insertion import Insert
    from loom_env.specs.episode import Observation

    task, world = insertion
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    world["coin/grasped_by"][1] = True
    at_depth(task, world, -0.004)
    calls = []

    def cartesian(observation, truth, goal, **kwargs):
        calls.append(goal.copy())
        return np.array(collection.deployment.arms["right"].initial_positions)

    arm = Manipulator(
        collection.deployment,
        collection.scene,
        "right",
        "coin",
        SimpleNamespace(
            attached=False,
            attach=lambda *args, **kwargs: None,
            cartesian_step=cartesian,
        ),
    )
    observation = Observation(
        0,
        {
            "robot/left/joint_position": np.array(
                collection.deployment.arms["left"].initial_positions
            ),
            "robot/right/joint_position": np.array(
                collection.deployment.arms["right"].initial_positions
            ),
            "robot/right/tcp_pose_world": np.r_[
                world["coin/pose_world"][:3] + [0, 0, 0.12], [1, 0, 0, 0]
            ],
        },
    )
    action = Insert(task.geometry, task.parameters["depth"])
    for _ in range(4):
        arm.update(observation, world)
        action.step(arm)
    assert not action.align
    assert calls  # Insertion motion began without grasp/extraction/transfer.
