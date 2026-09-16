from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from loom_env.specs.config import load_collection
from loom_env.tasks.handover import HandoverTask

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def handover():
    return load_collection(ROOT / "configs/collection/handover.yaml")


def world(grasped=(False, False), z=1.0, giver_touch=False):
    force = np.zeros((2, 2, 3))
    for side, held in enumerate(grasped):
        if held or (side == 0 and giver_touch):
            force[side, :, 0] = [1, -1]
    return {
        "tea_pack/pose_world": np.array([0.45, 0, z, 0.5, 0.5, 0.5, 0.5]),
        "tea_pack/grasped_by": np.array(grasped),
        "tea_pack/velocity_world": np.zeros(6),
        "tea_pack/finger_contact_forces_world": force,
    }


def tick(task, state, count=5):
    result = None
    for _ in range(count):
        result = task.update(state, 0.05)
    return result


def prepared_task(handover):
    task = HandoverTask(handover)
    task.reset(world(z=0.763))
    tick(task, world((True, False)))
    tick(task, world((True, True)))
    return task


def test_requires_ordered_handover_and_independent_motion(handover):
    task = HandoverTask(handover)
    task.reset(world(z=0.763))
    # Picking up with the receiver alone is not a handover.
    assert tick(task, world((False, True)), 20).outcome is None
    tick(task, world((True, False)))
    tick(task, world((True, True)))
    assert tick(task, world((False, True)), 20).outcome is None
    assert tick(task, world((False, True), z=1.08), 20).outcome.code == "success"


def test_rejects_release_before_receiver_and_return_to_table(handover):
    task = HandoverTask(handover)
    task.reset(world(z=0.763))
    tick(task, world((True, False)))
    result = task.update(world((False, True)), 0.05)
    assert result.outcome.code == "task_failure"
    task = prepared_task(handover)
    result = task.update(world((False, True), z=0.763), 0.05)
    assert result.outcome.code == "task_failure"


def test_giver_contact_and_interrupted_hold_block_success(handover):
    task = prepared_task(handover)
    assert tick(task, world((False, True), 1.08, giver_touch=True), 20).outcome is None
    assert tick(task, world((False, True), 1.08), 19).outcome is None
    task.update(world((True, True), 1.08), 0.05)
    assert tick(task, world((False, True), 1.08), 19).outcome is None
    assert task.update(world((False, True), 1.08), 0.05).outcome.code == "success"


def test_arm_role_validation_and_swap(handover):
    with pytest.raises(ValueError, match="different"):
        HandoverTask(replace(handover, arm_roles={"giver": "left", "receiver": "left"}))
    task = HandoverTask(
        replace(handover, arm_roles={"giver": "right", "receiver": "left"})
    )
    task.reset(world(z=0.763))
    tick(task, world((False, True)))
    tick(task, world((True, True)))
    assert tick(task, world((True, False), 1.08), 20).outcome.code == "success"


def test_sliding_and_rotating_contacts_are_not_stable_holds(handover):
    task = prepared_task(handover)
    # Contact evidence survives while the object slides through the fingers.
    falling = world((False, True), 1.08)
    falling["tea_pack/velocity_world"][2] = -0.65
    assert tick(task, falling, 40).outcome is None
    rotating = world((False, True), 1.08)
    rotating["tea_pack/velocity_world"][3] = 4.0
    assert tick(task, rotating, 40).outcome is None
    # Downward displacement, even after coming to rest, is not a lift proof.
    assert tick(task, world((False, True), .94), 40).outcome is None
    assert tick(task, world((False, True), 1.08), 19).outcome is None
    assert task.update(world((False, True), 1.08), .05).outcome.code == "success"


def test_expert_does_not_release_without_both_measured_grasps(handover):
    from types import SimpleNamespace

    from loom_env.experts.handover import HandoverExpert
    from loom_env.runtime.runner import SourceFailure
    from loom_env.specs.episode import Observation

    planners = {
        side: SimpleNamespace(detach=lambda: None, profile=SimpleNamespace(tcp_to_grasp=(0, 0, 0.1034))) for side in ("left", "right")
    }
    state = world((True, False))
    expert = HandoverExpert(handover, planners, lambda: state)
    expert.reset(None)
    expert.index = expert.STAGES.index("receiver_close")
    giver_slice = handover.deployment.action_slices["left/gripper"]
    expert.command[giver_slice] = 0.0
    observation = Observation(
        0,
        {
            **{
                f"robot/{side}/joint_position": np.array(arm.initial_positions)
                for side, arm in handover.deployment.arms.items()
            },
            "robot/left/tcp_pose_world": np.array([0, 0, 1, 0, 0, 0, 1]),
        },
    )
    with pytest.raises(SourceFailure, match="receiver_close"):
        for _ in range(100):
            expert.act(observation)
    assert expert.stage == "receiver_close"
    assert expert.index < expert.STAGES.index("giver_release")
    assert expert.command[giver_slice][0] == 0.0
