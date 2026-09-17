"""Actions must work from measured state without any preceding expert phases."""

from types import SimpleNamespace

import numpy as np
import pytest

from loom_env.experts.actions import Grasp, Manipulator, MoveHeld, Release
from loom_env.runtime.runner import SourceFailure


def manipulator(spec, frame, **methods):
    planner = SimpleNamespace(attached=False, detach=lambda: None, **methods)
    planner.attach = lambda *args, **kwargs: setattr(planner, "attached", True)
    arm = Manipulator(
        spec.collection.deployment, spec.collection.scene, "right", "cube", planner
    )
    arm.update(frame.observation, frame.world_state)
    return arm


def test_held_motion_can_start_without_grasp_workflow(spec, frame_factory):
    frame = frame_factory(spec, grasped=(False, True))
    calls = []

    def plan(observation, world, goal, **kwargs):
        calls.append(goal.copy())
        return np.array([observation.values["robot/right/joint_position"]]), {}

    arm = manipulator(spec, frame, plan=plan)
    goal = np.array([0.5, 0.0, 0.9, 0, 0, 0, 1])
    move = MoveHeld(goal)
    for _ in range(4):
        arm.update(frame.observation, frame.world_state)
        move.step(arm)
    assert move.done
    assert arm.planner.attached
    np.testing.assert_array_equal(calls, [goal])
    assert arm.command[arm.grip_slice] == arm.closed
    assert all(event.name != "grasp" for event in arm.events)


def test_grasp_preserves_existing_grasp_without_approach_or_opening(
    spec, frame_factory
):
    arm = manipulator(spec, frame_factory(spec, grasped=(False, True)))
    # Planner has no plan method: an unnecessary approach would fail this test.
    assert Grasp().step(arm)
    np.testing.assert_allclose(arm.rotation.as_quat(), arm.tcp[3:])
    assert arm.command[arm.grip_slice] == arm.closed


def test_held_motion_rejects_missing_initial_grasp(spec, frame_factory):
    arm = manipulator(spec, frame_factory(spec))
    with pytest.raises(SourceFailure, match="existing grasp"):
        MoveHeld(np.array([0.5, 0, 1, 0, 0, 0, 1])).step(arm)


def test_held_motion_fails_after_losing_real_contact(spec, frame_factory):
    frame = frame_factory(spec, grasped=(False, True))
    arm = manipulator(
        spec,
        frame,
        plan=lambda observation, *_args, **_kwargs: (
            np.tile(observation.values["robot/right/joint_position"], (100, 1)),
            {},
        ),
    )
    move = MoveHeld(np.array([0.5, 0, 1, 0, 0, 0, 1]))
    move.step(arm)
    dropped = frame_factory(spec, grasped=(False, False))
    for _ in range(4):
        arm.update(dropped.observation, dropped.world_state)
        assert not move.step(arm)
    arm.update(dropped.observation, dropped.world_state)
    with pytest.raises(SourceFailure, match="lost opposing"):
        move.step(arm)


def test_release_requires_measurement_not_open_command(spec, frame_factory):
    frame = frame_factory(spec, grasped=(False, True))
    arm = manipulator(spec, frame)
    release = Release(stable_samples=2, min_time=0)
    for _ in range(4):
        assert not release.step(arm)
    assert arm.command[arm.grip_slice] == arm.opened
    frame = frame_factory(spec, grasped=(False, False))
    arm.update(frame.observation, frame.world_state)
    assert not release.step(arm)
    assert release.step(arm)


def test_action_completion_does_not_advance_another_action(spec, frame_factory):
    arm = manipulator(spec, frame_factory(spec, grasped=(False, True)))
    grasp = Grasp()
    assert grasp.step(arm)
    events = tuple(arm.events)
    assert grasp.step(arm)
    assert tuple(arm.events) == events
    assert arm.command[arm.grip_slice] == arm.closed
