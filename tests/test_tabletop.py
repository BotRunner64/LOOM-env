from dataclasses import replace

import numpy as np
import pytest

from loom_env.environments.tabletop import (
    grasp_evidence,
    sample_cube,
    tabletop_geometry,
)
from loom_env.experts.pick_place import PickPlaceExpert
from loom_env.runtime.runner import SourceFailure


def test_sampling_is_reproducible_supported_and_separated(collection):
    np.testing.assert_array_equal(
        sample_cube(collection, 8), sample_cube(collection, 8)
    )
    box = tabletop_geometry(collection)
    base = box["container_base"]
    assert base["position"][2] - base["size"][2] / 2 == pytest.approx(
        collection.scene.parameters["table_height"]
    )
    container = np.asarray(collection.scene.parameters["container_position"])
    clearance = (
        np.asarray(collection.task.parameters["region_size"])[:2] / 2
        + 0.01
        + 0.02
        + 0.02
    )
    for seed in range(100):
        position = sample_cube(collection, seed)
        assert position[2] - 0.02 >= collection.scene.parameters["table_height"]
        assert np.any(np.abs(position[:2] - container[:2]) > clearance)


def test_impossible_sample_is_rejected(collection):
    p = dict(collection.scene.parameters)
    p["cube_position_min"] = p["cube_position_max"] = [0.58, 0.08, 0.78]
    impossible = replace(collection, scene=replace(collection.scene, parameters=p))
    with pytest.raises(ValueError, match="separated"):
        sample_cube(impossible, 0)


@pytest.mark.parametrize(
    "condition",
    ["no_force", "one_finger", "parallel", "missed", "above_pads", "closed_empty"],
)
def test_grasp_requires_physical_opposing_contacts(condition):
    force = np.array([[0.0, 2.0, 0.0], [0.0, -2.0, 0.0]])
    fingers, local = [0.02, 0.02], np.array([0.0, 0.0, 0.10])
    assert grasp_evidence(force, fingers, local, [0.04] * 3)
    if condition == "no_force":
        force *= 0
    elif condition == "one_finger":
        force[1] *= 0
    elif condition == "parallel":
        force[1] *= -1
    elif condition == "missed":
        local[1] = 0.06
    elif condition == "above_pads":
        local[2] = 0.03
    else:
        fingers = [0.0, 0.0]
    assert not grasp_evidence(force, fingers, local, [0.04] * 3)


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


@pytest.mark.parametrize("change", ["parameter", "object", "asset"])
def test_unsupported_scene_fields_are_not_silently_ignored(collection, change):
    parameters = dict(collection.scene.parameters)
    objects = {k: dict(v) for k, v in collection.scene.objects.items()}
    if change == "parameter":
        parameters["cube_mass"] = 100
    elif change == "object":
        objects["obstacle"] = {"asset": "primitive:cube", "category": "obstacle"}
    else:
        objects["cube"]["asset"] = "custom:mesh"
    collection = replace(
        collection,
        scene=replace(collection.scene, parameters=parameters, objects=objects),
    )
    with pytest.raises(ValueError):
        tabletop_geometry(collection)
