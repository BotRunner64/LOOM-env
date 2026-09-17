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
    fixture = world[f"{task.target}/pose_world"]
    frame = Rotation.from_quat(fixture[3:])
    orientation = frame * Rotation.from_euler("x", 90, degrees=True)
    center = (
        frame.apply([*task.slot_center, task.slot_top + task.radius - depth])
        + fixture[:3]
    )
    world["coin/pose_world"] = np.r_[
        center - orientation.apply(task.center), orientation.as_quat()
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
    frame = Rotation.from_quat(world[f"{task.target}/pose_world"][3:])
    if kind == "across":
        world["coin/pose_world"][:3] += frame.apply([0, 0.002, 0])
    elif kind == "along":
        world["coin/pose_world"][:3] += frame.apply([0.004, 0, 0])
    elif kind == "tilted":
        world["coin/pose_world"][3:] = [0, 0, 0, 1]
    elif kind == "shallow":
        at_depth(task, world, 0.003)
    else:
        world["coin/pose_world"][:3] += (
            world[f"{task.source}/pose_world"][:3]
            - world[f"{task.target}/pose_world"][:3]
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


def test_excessive_depth_is_failure(insertion):
    task, world = insertion
    at_depth(task, world, 0.025)
    assert task.update(world, 0.05).outcome.reason == "coin_below_slot_floor"


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


def test_insertion_goal_compensates_measured_coin_slip(insertion):
    from types import SimpleNamespace
    from loom_env.experts.insertion import CoinInsertionExpert
    from loom_env.specs.episode import Observation

    task, world = insertion
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    expert = CoinInsertionExpert(
        collection, SimpleNamespace(detach=lambda: None), lambda: world
    )
    expert.reset(None)
    tcp = np.r_[world["coin/pose_world"][:3] + [0.0, 0.0, 0.12], [1.0, 0.0, 0.0, 0.0]]
    observation = Observation(0.0, {"robot/right/tcp_pose_world": tcp})
    first = expert._insertion_goal(observation, world, -0.004)
    slip = np.array([0.001, -0.002, 0.003])
    world["coin/pose_world"][:3] += slip
    second = expert._insertion_goal(observation, world, -0.004)
    # The physical coin is untouched; the commanded TCP compensates its measured slip.
    np.testing.assert_allclose(
        second[:3] - first[:3],
        -Rotation.from_quat(first[3:]).apply(
            Rotation.from_quat(tcp[3:]).inv().apply(slip)
        ),
        atol=1e-8,
    )
    np.testing.assert_allclose(second[3:], first[3:], atol=1e-8)


def test_insertion_accepts_readonly_world_poses(insertion):
    from types import SimpleNamespace
    from loom_env.experts.insertion import CoinInsertionExpert
    from loom_env.specs.episode import Observation

    task, world = insertion
    expected = task.metrics(world)
    for value in world.values():
        value.flags.writeable = False
    assert task.metrics(world) == expected
    assert task.update(world, 0.05).outcome is None
    collection = load_collection(ROOT / "configs/collection/insert_coin.yaml")
    expert = CoinInsertionExpert(
        collection, SimpleNamespace(detach=lambda: None), lambda: world
    )
    expert.reset(None)
    tcp = np.r_[world["coin/pose_world"][:3] + [0, 0, 0.12], [1, 0, 0, 0]]
    observation = Observation(0.0, {"robot/right/tcp_pose_world": tcp})
    assert np.isfinite(expert._insertion_goal(observation, world, -0.004)).all()
