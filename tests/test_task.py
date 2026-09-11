from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.tasks.place import PlaceTask


def test_success_requires_continuous_release(spec, frame_factory):
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


@pytest.mark.parametrize("condition", ["protruding", "other_arm_grasp"])
def test_false_positive_placements_are_rejected(spec, frame_factory, condition):
    task = PlaceTask(spec.collection)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    if condition == "protruding":
        world["cube/pose_world"][0] += 0.08  # Centre inside, corner outside.
    else:
        world["cube/grasped_by"][1] = True
    task.reset(world)
    for _ in range(10):
        assert task.update(world, 0.05).outcome is None


def test_rotation_aware_containment(spec, frame_factory, monkeypatch):
    from loom_env.assets.catalog import ASSETS

    container = ASSETS["robodojo:basket"]
    monkeypatch.setitem(
        ASSETS,
        "robodojo:basket",
        replace(container, interior=(container.interior[0], (0.07, 0.04, 0.055))),
    )
    task = PlaceTask(spec.collection)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    task.reset(world)
    assert task.update(world, 0.25).outcome.code == "success"
    # The object does not fit across this test region's narrow axis.
    world["cube/pose_world"][3:] = Rotation.from_euler("z", 90, degrees=True).as_quat()
    assert task.update(world, 0.25).outcome is None
    world["container/region_pose_world"][3:] = world["cube/pose_world"][3:]
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


def test_contact_tolerance_does_not_accept_protrusion(spec, frame_factory):
    task = PlaceTask(spec.collection)
    # Allow submillimetre contact penetration of the conservative envelope.
    # A corner protruding beyond the configured tolerance remains a failure.
    numerical = frame_factory(spec, position=(0.57024, 0.0, 0.8)).world_state
    task.reset(numerical)
    assert task.update(numerical, 0.25).outcome.code == "success"
    protruding = frame_factory(spec, position=(0.57044, 0.0, 0.8)).world_state
    task.reset(protruding)
    assert task.update(protruding, 0.25).outcome is None


def test_success_respects_object_and_container_binding(collection, frame_factory, spec):
    objects = dict(collection.scene.objects)
    objects["other"] = objects["cube"]
    objects["other_container"] = objects["container"]
    collection = replace(collection, scene=replace(collection.scene, objects=objects))
    task = PlaceTask(collection)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    world["cube/pose_world"][:3] = [0.38, -0.10, 0.75]
    for key in ("pose_world", "velocity_world", "grasped_by"):
        world[f"other/{key}"] = world[f"cube/{key}"].copy()
    world["other_container/region_pose_world"] = np.array([0.6, 0.2, 0.8, 0, 0, 0, 1])
    world["other/pose_world"][:3] = world["container/region_pose_world"][:3]
    task.reset(world)
    assert task.update(world, 0.25).outcome is None
    world["cube/pose_world"][:3] = world["other_container/region_pose_world"][:3]
    assert task.update(world, 0.25).outcome is None
    world["cube/pose_world"][:3] = world["container/region_pose_world"][:3]
    assert task.update(world, 0.25).outcome.code == "success"


def test_lift_requires_continuous_clearance_and_contacts(spec, frame_factory):
    from loom_env.tasks import create_task
    from loom_env.specs.config import load_collection
    from pathlib import Path

    lift = load_collection(Path(__file__).parents[1] / "configs/collection/lift.yaml")
    lift = replace(
        lift, scene=spec.collection.scene, role_bindings={"target_object": "cube"}
    )
    task = create_task(lift)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    task.reset(world)
    world["cube/velocity_world"][:] = [-0.03, 0.017, 0.026, -1.15, -0.33, 0.36]
    world["cube/pose_world"][2] = 0.90
    assert task.update(world, 0.5).outcome is None  # Height alone is insufficient.
    world["cube/grasped_by"][1] = True
    world["cube/pose_world"][2] = 0.80
    assert task.update(world, 0.5).outcome is None  # Contact alone is insufficient.
    world["cube/pose_world"][2] = 0.90
    assert task.update(world, 0.20).outcome is None
    world["cube/grasped_by"][1] = False
    assert task.update(world, 0.05).outcome is None
    world["cube/grasped_by"][1] = True
    assert task.update(world, 0.20).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


@pytest.mark.parametrize(
    "velocity_key",
    ["cube/velocity_world", "container/velocity_world"],
)
def test_place_success_does_not_depend_on_velocity(spec, frame_factory, velocity_key):
    task = PlaceTask(spec.collection)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    world[velocity_key][:] = [0.1, -0.2, 0.3, 1.0, -2.0, 3.0]
    task.reset(world)
    assert task.update(world, 0.20).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


def test_place_requires_continuous_containment_in_moving_container(spec, frame_factory):
    task = PlaceTask(spec.collection)
    world = {
        key: value.copy() for key, value in frame_factory(spec).world_state.items()
    }
    world["container/velocity_world"][0] = 0.1
    world["cube/velocity_world"][0] = 0.1
    task.reset(world)
    assert task.update(world, 0.20).outcome is None
    world["container/region_pose_world"][0] += 0.3
    assert task.update(world, 0.05).outcome is None  # Leaving resets the timer.
    world["cube/pose_world"][0] += 0.3
    for _ in range(4):
        world["container/region_pose_world"][0] += 0.005
        world["cube/pose_world"][0] += 0.005
        assert task.update(world, 0.05).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"
