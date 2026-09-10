from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.tasks.place import PlaceTask


def test_success_requires_release_and_continuous_settling(spec, frame_factory):
    task = PlaceTask(spec.collection)
    free = frame_factory(spec).world_state
    held = frame_factory(spec, grasped=(True, False)).world_state
    task.reset(free)
    for _ in range(10):
        assert task.update(held, 0.05).outcome is None
    for _ in range(4):
        assert task.update(free, 0.05).outcome is None
    task.update(held, 0.05)
    for _ in range(4):
        assert task.update(free, 0.05).outcome is None
    assert task.update(free, 0.05).outcome.code == "success"


@pytest.mark.parametrize(
    "condition", ["protruding", "moving", "spinning", "other_arm_grasp"]
)
def test_false_positive_placements_are_rejected(spec, frame_factory, condition):
    task = PlaceTask(spec.collection)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    if condition == "protruding":
        world["cube/pose_world"][0] += 0.06  # Centre inside, corner outside.
    elif condition == "moving":
        world["cube/velocity_world"][0] = 0.1
    elif condition == "spinning":
        world["cube/velocity_world"][3] = 1.0
    else:
        world["cube/grasped_by"][1] = True
    task.reset(world)
    for _ in range(10):
        assert task.update(world, 0.05).outcome is None


def test_rotation_aware_containment(spec, frame_factory):
    objects = {name: dict(obj) for name, obj in spec.collection.scene.objects.items()}
    objects["cube"]["size"] = [0.10, 0.02, 0.04]
    objects["container"]["size"] = [0.04, 0.12, 0.1]
    task = PlaceTask(
        replace(spec.collection, scene=replace(spec.collection.scene, objects=objects))
    )
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    task.reset(world)
    assert task.update(world, 0.25).outcome is None
    world["cube/pose_world"][3:] = Rotation.from_euler("z", 90, degrees=True).as_quat()
    assert task.update(world, 0.25).outcome.code == "success"


def test_dropped_object_and_bad_truth(spec, frame_factory):
    task = PlaceTask(spec.collection)
    world = frame_factory(spec, position=(0.5, 0.0, 0.2)).world_state
    task.reset(world)
    assert task.update(world, 0.05).outcome.code == "task_failure"
    with pytest.raises(ValueError):
        task.update(world, float("nan"))
    bad = dict(world)
    bad["cube/grasped_by"] = np.zeros(2)
    with pytest.raises(ValueError, match="boolean"):
        task.reset(bad)


def test_contact_roundoff_tolerance_does_not_accept_protrusion(spec, frame_factory):
    task = PlaceTask(spec.collection)
    # Float32 physics can place a resting corner a fraction of a micrometre
    # past the ideal boundary. Larger geometric violations remain failures.
    numerical = frame_factory(spec, position=(0.550001, 0.0, 0.8)).world_state
    task.reset(numerical)
    assert task.update(numerical, 0.25).outcome.code == "success"
    protruding = frame_factory(spec, position=(0.55002, 0.0, 0.8)).world_state
    task.reset(protruding)
    assert task.update(protruding, 0.25).outcome is None


@pytest.mark.parametrize(
    "moved, destination, success",
    [
        ("blue_cube", "green_container", False),
        ("red_cube", "yellow_container", False),
        ("red_cube", "green_container", True),
    ],
)
def test_success_respects_both_object_and_container_binding(
    moved, destination, success
):
    from pathlib import Path
    from loom_env.specs.config import load_collection
    from loom_env.environments.tabletop import sample_objects

    collection = load_collection(
        Path(__file__).resolve().parents[1] / "configs/collection/red_to_green.yaml"
    )
    world = {}
    for name, position in sample_objects(collection, 0).items():
        world[f"{name}/pose_world"] = np.r_[position, [0, 0, 0, 1]]
        world[f"{name}/velocity_world"] = np.zeros(6)
        world[f"{name}/grasped_by"] = np.zeros(2, dtype=bool)
    for name in ("green_container", "yellow_container"):
        world[f"{name}/region_pose_world"] = np.r_[
            collection.scene.objects[name]["position"], [0, 0, 0, 1]
        ]
    task = PlaceTask(collection)
    task.reset(world)
    world[f"{moved}/pose_world"] = world[f"{destination}/region_pose_world"].copy()
    outcome = task.update(world, 0.25).outcome
    assert (outcome is not None and outcome.code == "success") is success
