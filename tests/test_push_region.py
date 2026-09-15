from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.scenes.workspace import sample_objects
from loom_env.specs.config import load_collection, plain, collection_from_dict
from loom_env.tasks import create_task


@pytest.fixture
def region_push():
    collection = load_collection(
        Path(__file__).parents[1] / "configs/collection/push_box.yaml"
    )
    task = create_task(collection)
    world = {
        f"{name}/pose_world": value
        for name, value in sample_objects(collection.scene, 0).items()
    }
    world["box/pose_world"][2] = task.support[2] - task.asset.bounds[0][2]
    world["box/grasped_by"] = np.zeros(2, dtype=bool)
    world["box/finger_contact_forces_world"] = np.zeros((2, 2, 3))
    task.reset(world)
    return collection, task, world


def test_region_requires_entire_footprint_and_released_contact(region_push):
    _, task, world = region_push
    world["box/finger_contact_forces_world"][1, 0, 0] = 1
    task.update(world, 0.05)
    world["box/finger_contact_forces_world"][:] = 0
    world["box/pose_world"][:2] = task.target_position_world[:2] + [0.10, 0]
    assert task.metrics(world)["region_margin"] < 0
    assert task.update(world, 1).outcome is None
    world["box/pose_world"][:2] = task.target_position_world[:2]
    assert task.update(world, 0.20).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


def test_region_follows_scene_translation_and_rotation(region_push):
    collection, task, world = region_push
    value = plain(collection)
    quat = Rotation.from_euler("z", 90, degrees=True).as_quat()
    value["scene"]["objects"]["storage"]["pose"] = [0.2, 0.2, 0, *quat.tolist()]
    moved = create_task(collection_from_dict(value))
    assert np.allclose(moved.target_position_world[:2], [0.65, 0.2])
    world["box/pose_world"][:2] = moved.target_position_world[:2]
    world["box/pose_world"][3:] = quat
    assert moved.at_goal(moved.metrics(world))
    assert not task.at_goal(task.metrics(world))


def test_region_tipping_is_a_sticky_failure(region_push):
    _, task, world = region_push
    original = world["box/pose_world"].copy()
    rotation = Rotation.from_euler("x", 20, degrees=True)
    world["box/pose_world"][3:] = rotation.as_quat()
    world["box/pose_world"][2] = (
        task.support[2] - rotation.apply(task._corners)[:, 2].min()
    )
    assert task.update(world, 0.05).outcome.reason == "push_object_tipped"
    world["box/pose_world"] = original
    assert task.update(world, 0.05).outcome.reason == "push_object_tipped"


def test_visible_region_can_overlap_objects_but_cannot_start_solved(region_push):
    collection, task, world = region_push
    value = plain(collection)
    value["scene"]["objects"]["box"]["pose"][:2] = value["scene"]["objects"]["storage"][
        "pose"
    ][:2]
    poses = sample_objects(collection_from_dict(value).scene, 0)
    world["box/pose_world"][:2] = poses["box"][:2]
    with pytest.raises(ValueError, match="outside"):
        task.reset(world)


def test_region_must_be_horizontal(region_push):
    collection, _, _ = region_push
    value = plain(collection)
    value["scene"]["objects"]["storage"]["pose"][3:] = (
        Rotation.from_euler("x", 30, degrees=True).as_quat().tolist()
    )
    with pytest.raises(ValueError, match="horizontal"):
        create_task(collection_from_dict(value))
