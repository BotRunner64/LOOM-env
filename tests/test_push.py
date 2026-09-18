from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

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
    world["object/pose_world"][:2] = task.target_position_world[:2]
    assert task.update(world, 1).outcome is None
    world["object/finger_contact_forces_world"][1, 0, 0] = force
    assert task.update(world, 1).outcome is None
    world["object/finger_contact_forces_world"][:] = 0
    assert task.update(world, 0.20).outcome is None
    world["object/pose_world"][0] += 0.02
    assert task.update(world, 0.05).outcome is None
    world["object/pose_world"][:2] = task.target_position_world[:2]
    assert task.update(world, 0.20).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


@pytest.mark.parametrize("fault", ["lift", "grasp", "penetrate"])
def test_push_forbidden_history_cannot_be_erased_by_returning_to_goal(push, fault):
    _, task, world = push
    supported_pose = world["object/pose_world"].copy()
    if fault == "grasp":
        world["object/grasped_by"][0] = True
    else:
        world["object/pose_world"][2] += 0.02 if fault == "lift" else -0.02
    assert task.update(world, 0.05).outcome.code == "task_failure"
    world["object/grasped_by"][:] = False
    world["object/pose_world"] = supported_pose
    world["object/pose_world"][:2] = task.target_position_world[:2]
    assert task.update(world, 1).outcome.code == "task_failure"


@pytest.mark.parametrize("yaw", [-135, 15, 90])
def test_push_goal_does_not_constrain_yaw_or_velocity(push, yaw):
    _, task, world = push
    world["object/finger_contact_forces_world"][1, 0, 0] = 1
    task.update(world, 0.05)
    world["object/finger_contact_forces_world"][:] = 0
    world["object/pose_world"][:2] = task.target_position_world[:2]
    world["object/pose_world"][3:] = Rotation.from_euler(
        "z", yaw, degrees=True
    ).as_quat()
    world["object/velocity_world"] = np.ones(6)
    assert task.update(world, 0.25).outcome.code == "success"


def test_push_target_uses_workspace_frame_and_checks_region_bounds(push):
    collection, task, _ = push
    assert np.allclose(task.target_position_world, [0.50, -0.10, 0.75])
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
        create_task(replace(collection, scene=rotated)).target_position_world[:2],
        [0.55, 0.05],
    )
    # Centre alone is inside, but the one-centimetre goal circle crosses the edge.
    with pytest.raises(ValueError, match="outside"):
        create_task(
            replace(
                collection,
                task=replace(
                    collection.task,
                    parameters={
                        **collection.task.parameters,
                        "target_position": [0.475, 0],
                    },
                ),
            )
        )


def test_push_reset_rejects_already_at_goal_and_clears_contact(push):
    _, task, world = push
    world["object/finger_contact_forces_world"][1, 0, 0] = 1
    task.update(world, 0.05)
    task.reset(world)
    assert not task.contact_seen
    world["object/pose_world"][:2] = task.target_position_world[:2]
    with pytest.raises(ValueError, match="outside"):
        task.reset(world)


@pytest.mark.parametrize("heading", [-90, 0, 90, 180])
def test_push_expert_faces_travel_direction_and_releases_at_any_yaw(push, heading):
    from loom_env.embodiments.manipulation import manipulation_profile
    from loom_env.experts.push import PushContact, PushExpert

    collection, task, world = push
    direction = Rotation.from_euler("z", heading, degrees=True).apply([1, 0, 0])
    world["object/pose_world"][:2] = (
        task.target_position_world[:2] - 0.10 * direction[:2]
    )
    world["object/pose_world"][3:] = Rotation.from_euler(
        "z", 45, degrees=True
    ).as_quat()
    planner = SimpleNamespace(
        detach=lambda: None,
        profile=manipulation_profile(collection.deployment.arms["right"]),
    )
    expert = PushExpert(collection, planner, lambda: world)
    expert.reset(None)
    rotation = Rotation.from_quat(expert.tcp_goal[3:])
    assert np.allclose(rotation.apply([1, 0, 0]), direction)
    assert np.allclose(rotation.apply([0, 0, 1]), [0, 0, -1])
    assert np.allclose(
        expert.tcp_goal[:3] + rotation.apply(planner.profile.tcp_to_grasp),
        expert.push_point,
    )
    world["object/pose_world"][:2] = task.target_position_world[:2]
    observation = SimpleNamespace(
        values={"robot/right/tcp_pose_world": expert.tcp_goal}
    )
    expert.arm.update(observation, world)
    push_action = next(
        action for action in expert.routine() if isinstance(action, PushContact)
    )
    assert push_action.step(expert.arm)
