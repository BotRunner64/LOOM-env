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


@pytest.mark.parametrize("side", [0, 1])
def test_held_object_is_not_placed(spec, frame_factory, side):
    task = PlaceTask(spec.collection)
    world = {k: v.copy() for k, v in frame_factory(spec).world_state.items()}
    world["cube/grasped_by"][side] = True
    task.reset(world)
    assert task.update(world, 0.5).outcome is None


def test_position_inside_accepts_corners_outside_and_object_rotation(
    spec, frame_factory
):
    task = PlaceTask(spec.collection)
    world = {
        k: v.copy()
        for k, v in frame_factory(spec, position=(0.62, 0.07, 0.8)).world_state.items()
    }
    task.reset(world)
    assert task.update(world, 0.25).outcome.code == "success"
    world["cube/pose_world"][3:] = Rotation.from_euler("z", 90, degrees=True).as_quat()
    assert task.update(world, 0.25).outcome.code == "success"


def test_containment_follows_container_rotation(spec, frame_factory):
    task = PlaceTask(spec.collection)
    world = {
        k: v.copy()
        for k, v in frame_factory(spec, position=(0.61, 0.0, 0.8)).world_state.items()
    }
    task.reset(world)
    assert task.update(world, 0.25).outcome.code == "success"
    # Rotating the basket puts the same point outside its narrower local y axis.
    world["container/pose_world"][3:] = Rotation.from_euler(
        "z", 90, degrees=True
    ).as_quat()
    assert task.update(world, 0.25).outcome is None
    world["cube/pose_world"][:2] = [0.5, 0.11]
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


@pytest.mark.parametrize(
    "axis, boundary, offset, succeeds",
    [
        (0, "upper", -0.001, True),
        (0, "upper", 0.001, False),
        (0, "lower", -0.001, False),
        (1, "upper", 0.001, False),
        (1, "lower", -0.001, False),
        (2, "upper", 0.009, True),
        (2, "upper", 0.011, False),
        (2, "lower", -0.001, False),
    ],
)
def test_container_position_boundaries(
    spec, frame_factory, axis, boundary, offset, succeeds
):
    task = PlaceTask(spec.collection)
    world = {k: v.copy() for k, v in frame_factory(spec).world_state.items()}
    lower, upper = task.container_bounds
    local = np.zeros(3)
    local[axis] = (upper if boundary == "upper" else lower)[axis] + offset
    world["cube/pose_world"][:3] = world["container/pose_world"][:3] + local
    task.reset(world)
    outcome = task.update(world, 0.25).outcome
    assert (outcome is not None and outcome.code == "success") == succeeds


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
    world["other_container/pose_world"] = np.array([0.6, 0.2, 0.8, 0, 0, 0, 1])
    world["other/pose_world"][:3] = world["container/region_pose_world"][:3]
    task.reset(world)
    assert task.update(world, 0.25).outcome is None
    world["cube/pose_world"][:3] = world["other_container/pose_world"][:3]
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
    world["container/pose_world"][0] += 0.3
    assert task.update(world, 0.05).outcome is None  # Leaving resets the timer.
    world["cube/pose_world"][0] += 0.3
    for _ in range(4):
        world["container/pose_world"][0] += 0.005
        world["cube/pose_world"][0] += 0.005
        assert task.update(world, 0.05).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"
