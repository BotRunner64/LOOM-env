from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.scenes.workspace import sample_objects
from loom_env.specs.config import load_collection
from loom_env.tasks import create_task


@pytest.fixture
def push():
    collection = load_collection(
        Path(__file__).parents[1] / "configs/collection/push.yaml"
    )
    task = create_task(collection)
    world = {
        f"{k}/pose_world": v for k, v in sample_objects(collection.scene, 0).items()
    }
    world["object/pose_world"][2] = task.support[2] - task.asset.bounds[0][2]
    world["object/grasped_by"] = np.zeros(2, dtype=bool)
    world["object/finger_contact_forces_world"] = np.zeros((2, 2, 3))
    task.reset(world)
    return collection, task, world


@pytest.mark.parametrize("force", [0.17, 1.0])
def test_push_requires_contact_then_continuous_released_goal(push, force):
    _, task, world = push
    world["object/pose_world"] = task.target_pose.copy()
    assert task.update(world, 1).outcome is None
    world["object/finger_contact_forces_world"][1, 0, 0] = force
    assert task.update(world, 1).outcome is None
    world["object/finger_contact_forces_world"][:] = 0
    assert task.update(world, 0.20).outcome is None
    world["object/pose_world"][0] += 0.02
    assert task.update(world, 0.05).outcome is None
    world["object/pose_world"] = task.target_pose.copy()
    assert task.update(world, 0.20).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


@pytest.mark.parametrize("fault", ["lift", "grasp", "penetrate"])
def test_push_forbidden_history_cannot_be_erased_by_returning_to_goal(push, fault):
    _, task, world = push
    if fault == "grasp":
        world["object/grasped_by"][0] = True
    else:
        world["object/pose_world"][2] += 0.02 if fault == "lift" else -0.02
    assert task.update(world, 0.05).outcome.code == "task_failure"
    world["object/grasped_by"][:] = False
    world["object/pose_world"] = task.target_pose.copy()
    assert task.update(world, 1).outcome.code == "task_failure"


def test_push_checks_orientation_and_not_velocity(push):
    _, task, world = push
    world["object/finger_contact_forces_world"][1, 0, 0] = 1
    task.update(world, 0.05)
    world["object/finger_contact_forces_world"][:] = 0
    world["object/pose_world"] = task.target_pose.copy()
    world["object/pose_world"][3:] = Rotation.from_euler(
        "z", 15, degrees=True
    ).as_quat()
    assert task.update(world, 1).outcome is None
    world["object/pose_world"] = task.target_pose.copy()
    world["object/velocity_world"] = np.ones(6)
    assert task.update(world, 0.25).outcome.code == "success"


def test_push_target_uses_workspace_frame_and_checks_bounds(push):
    collection, task, _ = push
    assert np.allclose(task.target_pose[:2], [0.50, -0.10])
    rotated = replace(
        collection.scene,
        parameters={
            **collection.scene.parameters,
            "workspace_pose": [
                0.45,
                0,
                0.75,
                *Rotation.from_euler("z", 90, degrees=True).as_quat().tolist(),
            ],
        },
    )
    assert np.allclose(
        create_task(replace(collection, scene=rotated)).target_pose[:2], [0.55, 0.05]
    )
    with pytest.raises(ValueError, match="outside"):
        create_task(
            replace(
                collection,
                task=replace(
                    collection.task,
                    parameters={
                        **collection.task.parameters,
                        "target_position": [10, 0],
                    },
                ),
            )
        )


def test_push_reset_rejects_already_at_goal_and_clears_contact(push):
    _, task, world = push
    world["object/finger_contact_forces_world"][1, 0, 0] = 1
    task.update(world, 0.05)
    world["object/finger_contact_forces_world"][:] = 0
    task.reset(world)
    assert not task.contact_seen
    world["object/pose_world"] = task.target_pose.copy()
    with pytest.raises(ValueError, match="outside"):
        task.reset(world)
