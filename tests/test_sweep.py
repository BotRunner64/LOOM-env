from pathlib import Path

import numpy as np
import pytest

from loom_env.scenes.workspace import sample_objects
from loom_env.specs.config import load_collection
from loom_env.tasks import create_task


@pytest.fixture
def sweep():
    collection = load_collection(
        Path(__file__).parents[1] / "configs/collection/sweep.yaml"
    )
    task = create_task(collection)
    world = {
        f"{n}/pose_world": p for n, p in sample_objects(collection.scene, 0).items()
    }
    for n in ("brick", "brush"):
        world[f"{n}/grasped_by"] = np.zeros(2, dtype=bool)
        world[f"{n}/finger_contact_forces_world"] = np.zeros((2, 2, 3))
    world["brush/object_contact_forces_world/brick"] = np.zeros(3)
    task.reset(world)
    return task, world


def held_contact(task, world):
    world["brush/grasped_by"][1] = True
    world["brush/pose_world"][2] += 0.08
    assert task.update(world, 0.05).outcome is None
    world["brush/pose_world"][2] -= 0.066
    world["brush/object_contact_forces_world/brick"][0] = 1.0
    assert task.update(world, 0.05).outcome is None


def finish_pose(task, world):
    world["brick/pose_world"][:2] = task.target_position_world[:2]
    world["brush/object_contact_forces_world/brick"][:] = 0
    world["brush/pose_world"][2] += 0.08


def test_sweep_requires_held_tool_contact_containment_and_lift(sweep):
    task, world = sweep
    held_contact(task, world)
    world["brick/pose_world"][:2] = task.target_position_world[:2] + [0.13, 0]
    world["brush/object_contact_forces_world/brick"][:] = 0
    assert task.update(world, 1).outcome is None  # Partly outside the region.
    world["brick/pose_world"][:2] = task.target_position_world[:2]
    assert task.update(world, 1).outcome is None  # Brush is still down.
    finish_pose(task, world)
    assert task.update(world, 0.20).outcome is None
    assert task.update(world, 0.05).outcome.code == "success"


def test_motion_without_tool_contact_cannot_succeed(sweep):
    task, world = sweep
    world["brush/grasped_by"][1] = True
    finish_pose(task, world)
    assert task.update(world, 2).outcome is None


def test_unheld_tool_contact_cannot_satisfy_sweep(sweep):
    task, world = sweep
    world["brush/object_contact_forces_world/brick"][0] = 2
    assert task.update(world, 0.05).outcome is None
    world["brush/grasped_by"][1] = True
    finish_pose(task, world)
    assert task.update(world, 2).outcome is None


@pytest.mark.parametrize("cheat", ["finger", "lift", "grasp", "drop_tool"])
def test_sweep_rejects_shortcuts_and_keeps_failure(sweep, cheat):
    task, world = sweep
    held_contact(task, world)
    if cheat == "finger":
        world["brick/finger_contact_forces_world"][1, 0, 0] = 1
    elif cheat == "lift":
        world["brick/pose_world"][2] += 0.03
    elif cheat == "grasp":
        world["brick/grasped_by"][1] = True
    else:
        world["brush/grasped_by"][1] = False
    assert task.update(world, 0.3).outcome.code == "task_failure"
    world["brick/finger_contact_forces_world"][:] = 0
    world["brick/grasped_by"][:] = False
    world["brush/grasped_by"][1] = True
    world["brick/pose_world"][2] = task.support[2] - task.asset.bounds[0][2]
    finish_pose(task, world)
    assert task.update(world, 1).outcome.code == "task_failure"


def test_sweep_requires_contact_measurement(sweep):
    task, world = sweep
    world["brush/object_contact_forces_world/brick"][:] = np.nan
    with pytest.raises(ValueError, match="finite"):
        task.update(world, 0.05)


@pytest.mark.parametrize(
    "name", ["sweep", "sweep_plate", "sweep_mouse", "sweep_waffle"]
)
def test_sweep_accepts_geometry_instead_of_target_asset_id(name):
    from types import SimpleNamespace

    from loom_env.experts.sweep import SweepExpert

    c = load_collection(Path(__file__).parents[1] / f"configs/collection/{name}.yaml")
    world = {f"{n}/pose_world": p for n, p in sample_objects(c.scene, 0).items()}
    expert = SweepExpert(c, SimpleNamespace(detach=lambda: None), lambda: world)
    expert.reset(None)
    np.testing.assert_allclose(expert.direction, [1, 0])


def test_sweep_rejects_a_target_below_the_bristle_path():
    from dataclasses import replace
    from types import SimpleNamespace

    from loom_env.experts.sweep import SweepExpert

    c = load_collection(Path(__file__).parents[1] / "configs/collection/sweep.yaml")
    world = {f"{n}/pose_world": p for n, p in sample_objects(c.scene, 0).items()}
    expert = SweepExpert(c, SimpleNamespace(detach=lambda: None), lambda: world)
    expert.target_asset = replace(
        expert.target_asset, bounds=((-0.03, -0.015, -0.005), (0.03, 0.015, 0.005))
    )
    world["brick/pose_world"][2] = expert.task.support[2] + 0.005
    with pytest.raises(ValueError, match="too low"):
        expert.reset(None)


def test_retreat_height_uses_measured_tool_clearance(sweep):
    from types import SimpleNamespace

    from loom_env.experts.sweep import SweepExpert

    _, world = sweep
    c = load_collection(Path(__file__).parents[1] / "configs/collection/sweep.yaml")
    expert = SweepExpert(c, SimpleNamespace(detach=lambda: None), lambda: world)
    expert.reset(None)
    observation = SimpleNamespace(
        values={
            "robot/right/tcp_pose_world": np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        }
    )
    expert.arm.update(observation, world)
    first = expert.retreat_goal()
    world["brush/pose_world"][2] -= 0.03
    second = expert.retreat_goal()
    assert second[2] == pytest.approx(first[2] + 0.03)
    assert observation.values["robot/right/tcp_pose_world"][2] == 1.0


def test_retreat_rechecks_low_tool_and_bounds_retries(sweep):
    from types import SimpleNamespace

    from loom_env.experts.sweep import SweepExpert
    from loom_env.runtime.runner import SourceFailure

    _, world = sweep
    c = load_collection(Path(__file__).parents[1] / "configs/collection/sweep.yaml")
    expert = SweepExpert(c, SimpleNamespace(detach=lambda: None), lambda: world)
    expert.reset(None)
    observation = SimpleNamespace(
        values={
            "robot/right/tcp_pose_world": np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        }
    )
    expert.arm.update(observation, world)
    retreat = expert.retreat()
    for attempt in range(3):
        move = next(retreat)
        assert move.name == "retreat"
        np.testing.assert_allclose(move.goal, expert.retreat_goal())
    with pytest.raises(SourceFailure, match="did not clear"):
        next(retreat)
    assert sum(e.name == "retreat_clearance_retry" for e in expert.arm.events) == 2
