from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from loom_env.scenes.workspace import sample_objects
from loom_env.specs.config import load_collection
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.embodiments.commands import gripper_command
from loom_env.experts.pick_place import PickPlaceExpert
from loom_env.runtime.runner import SourceFailure
from loom_env.tasks import create_task


def test_same_scene_supports_two_tasks_with_identical_initial_layout():
    root = Path(__file__).parents[1] / "configs/collection"
    place, lift = [
        load_collection(root / f"{name}.yaml") for name in ("pick_place", "lift")
    ]
    assert place.scene == lift.scene
    assert type(create_task(place)) is not type(create_task(lift))
    for seed in range(20):
        first, second = (
            sample_objects(place.scene, seed),
            sample_objects(lift.scene, seed),
        )
        for name in first:
            np.testing.assert_array_equal(first[name], second[name])
        assert np.linalg.norm(first["object"][:2] - first["basket"][:2]) > 0.18


@pytest.mark.parametrize(
    "change",
    [
        "overlap",
        "outside",
        "unsupported",
        "unknown_asset",
        "extra_field",
        "static_override",
    ],
)
def test_invalid_scene_fails_before_simulation(collection, change):
    objects = {key: dict(value) for key, value in collection.scene.objects.items()}
    if change == "overlap":
        objects["cube"]["position_min"] = objects["cube"]["position_max"] = [0.13, 0.12]
    elif change == "outside":
        objects["cube"]["position_min"] = objects["cube"]["position_max"] = [2, 0]
    elif change == "unsupported":
        objects["cube"]["pose"] = [-0.05, 0, 0.2, 0, 0, 0, 1]
    elif change == "unknown_asset":
        objects["cube"]["asset"] = "unreviewed:mesh"
    elif change == "static_override":
        objects["container"]["static"] = True
    else:
        objects["cube"]["color"] = [1, 0, 0]
    with pytest.raises(ValueError):
        sample_objects(replace(collection.scene, objects=objects), 0)


def test_workspace_transform_moves_all_instances_consistently(collection):
    original = sample_objects(collection.scene, 3)
    params = dict(collection.scene.parameters)
    params["workspace_pose"] = [1.45, 0, 0.75, 0, 0, 0, 1]
    moved = sample_objects(replace(collection.scene, parameters=params), 3)
    for name in original:
        np.testing.assert_allclose(moved[name][:3] - original[name][:3], [1, 0, 0])


@pytest.mark.parametrize(
    "condition", ["no_force", "one_finger", "parallel", "closed", "fully_open"]
)
def test_grasp_requires_physical_opposing_contacts(condition, collection):
    gripper = collection.deployment.arms["right"].gripper
    forces = np.array([[0.0, 2.0, 0.0], [0.0, -2.0, 0.0]])
    fingers = [0.012, 0.012]
    assert opposing_contacts(
        forces, gripper_command(gripper, np.array(fingers)), gripper.command_limits[0]
    )
    if condition == "no_force":
        forces *= 0
    elif condition == "one_finger":
        forces[1] *= 0
    elif condition == "parallel":
        forces[1] *= -1
    elif condition == "closed":
        fingers = [0, 0]
    else:
        fingers = [0.04, 0.04]
    assert not opposing_contacts(
        forces, gripper_command(gripper, np.array(fingers)), gripper.command_limits[0]
    )


class UnusedPlanner:
    def detach(self):
        pass

    def plan(self, *args, **kwargs):
        raise AssertionError("Planning must not begin without physical feedback")

    def attach(self, *args):
        raise AssertionError("Attachment must not begin without a measured lift")


def test_close_command_alone_cannot_advance_expert(spec, frame_factory):
    frame = frame_factory(spec, grasped=(False, False))
    expert = PickPlaceExpert(
        spec.collection, UnusedPlanner(), lambda: frame.world_state
    )
    expert.reset(None)
    expert.stage_index = expert.STAGES.index("close")
    with pytest.raises(SourceFailure, match="close feedback timed out"):
        for _ in range(50):
            expert.act(frame.observation)
    assert expert.stage == "close"


def test_contacts_without_lift_cannot_start_transfer(spec, frame_factory):
    frame = frame_factory(spec, position=(0.4, -0.1, 0.77), grasped=(True, True))
    expert = PickPlaceExpert(
        spec.collection, UnusedPlanner(), lambda: frame.world_state
    )
    expert.reset(None)
    expert.stage_index = expert.STAGES.index("transfer")
    with pytest.raises(SourceFailure, match="not physically lifted"):
        expert.act(frame.observation)
