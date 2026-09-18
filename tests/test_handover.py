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
    assert tick(task, world((False, True), 0.94), 40).outcome is None
    assert tick(task, world((False, True), 1.08), 19).outcome is None
    assert task.update(world((False, True), 1.08), 0.05).outcome.code == "success"


@pytest.mark.parametrize(
    "contacts,message", [((True, False), "close"), ((False, True), "lost opposing")]
)
def test_expert_does_not_release_without_both_measured_grasps(
    handover, contacts, message
):
    from types import SimpleNamespace

    from loom_env.embodiments.manipulation import manipulation_profile
    from loom_env.experts.handover import HandoverExpert
    from loom_env.runtime.runner import SourceFailure
    from loom_env.specs.episode import Observation

    planners = {
        side: SimpleNamespace(
            detach=lambda: None,
            profile=manipulation_profile(handover.deployment.arms[side]),
        )
        for side in ("left", "right")
    }
    state = world(contacts)
    state["tea_pack/pose_world"][3:] = [0, 0, np.sqrt(0.5), np.sqrt(0.5)]
    state["tea_pack/center_of_mass_local"] = np.zeros(3)
    expert = HandoverExpert(handover, planners, lambda: state)
    expert.reset(None)
    giver_slice = handover.deployment.action_slices["left/gripper"]
    expert.arm.command[giver_slice] = 0.0
    observation = Observation(
        0,
        {
            **{
                f"robot/{side}/joint_position": np.array(arm.initial_positions)
                for side, arm in handover.deployment.arms.items()
            },
            **{
                f"robot/{side}/tcp_pose_world": np.array(
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                )
                for side in ("left", "right")
            },
        },
    )
    from loom_env.experts.actions import CloseGripper

    for arm in expert.arms.values():
        arm.update(observation, state)
    # Reach the receiver's closing action through the real routine, without
    # injecting legacy phase indices. Planning geometry is tested separately.
    closes = 0
    for action in expert.actions:
        if isinstance(action, CloseGripper):
            closes += 1
            if closes == 2:
                expert.current = action
                break
    with pytest.raises(SourceFailure, match=message):
        for _ in range(100):
            expert.act(observation)
    assert not expert.current.done
    assert expert.arm is expert.arms[expert.receiver]
    assert expert.arm.command[giver_slice][0] == 0.0
    assert expert.arms[expert.giver].command is expert.arms[expert.receiver].command
    expert.reset(None)
    assert expert.arm is expert.arms[expert.giver]
    assert expert.current is None and not expert.finished and not expert.holding
    assert all(arm.tick == -1 and arm.lost == 0 for arm in expert.arms.values())
    assert expert.arms[expert.giver].command is expert.arms[expert.receiver].command


@pytest.mark.parametrize("long_axis", [0, 1, 2])
def test_automatic_grasps_follow_box_axis_com_and_arm_roles(handover, long_axis):
    from types import SimpleNamespace

    from scipy.spatial.transform import Rotation

    from loom_env.embodiments.manipulation import manipulation_profile
    from loom_env.experts.handover import HandoverExpert

    planners = {
        s: SimpleNamespace(detach=lambda: None, profile=manipulation_profile(a))
        for s, a in handover.deployment.arms.items()
    }
    sizes = np.array([0.04, 0.04, 0.04])
    sizes[long_axis] = 0.30
    com = np.zeros(3)
    com[long_axis] = 0.025
    axis = np.eye(3)[long_axis]
    rotation, _ = Rotation.align_vectors([[0, 1, 0]], [axis])
    state = world()
    state["tea_pack/pose_world"][3:] = rotation.as_quat()
    state["tea_pack/center_of_mass_local"] = com
    for giver, receiver in [("left", "right"), ("right", "left")]:
        cfg = replace(handover, arm_roles={"giver": giver, "receiver": receiver})
        expert = HandoverExpert(cfg, planners, lambda: state)
        expert.asset = replace(
            expert.asset, bounds=tuple(map(tuple, [-sizes / 2, sizes / 2]))
        )
        expert.reset(None)
        a, b = expert.sites[giver], expert.sites[receiver]
        np.testing.assert_allclose((a + b) / 2, com, atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(a - b), 0.10)
        assert rotation.apply(a - b)[1] * (1 if giver == "left" else -1) > 0
        assert np.all(np.abs(a) < sizes / 2)
        assert np.all(np.abs(b) < sizes / 2)


@pytest.mark.parametrize(
    "size, message",
    [((0.10, 0.04, 0.04), "too short"), ((0.30, 0.10, 0.10), "jaw opening")],
)
def test_automatic_grasps_reject_boxes_that_do_not_fit(handover, size, message):
    from types import SimpleNamespace

    from loom_env.embodiments.manipulation import manipulation_profile
    from loom_env.experts.handover import HandoverExpert

    state = world()
    state["tea_pack/center_of_mass_local"] = np.zeros(3)
    planners = {
        s: SimpleNamespace(detach=lambda: None, profile=manipulation_profile(a))
        for s, a in handover.deployment.arms.items()
    }
    expert = HandoverExpert(handover, planners, lambda: state)
    size = np.asarray(size)
    expert.asset = replace(
        expert.asset, bounds=tuple(map(tuple, [-size / 2, size / 2]))
    )
    with pytest.raises(ValueError, match=message):
        expert.reset(None)


def test_automatic_grasps_keep_finger_margin_for_offset_com(handover):
    from types import SimpleNamespace

    from loom_env.embodiments.manipulation import manipulation_profile
    from loom_env.experts.handover import HandoverExpert

    state = world()
    state["tea_pack/pose_world"][3:] = [0, 0, np.sqrt(0.5), np.sqrt(0.5)]
    state["tea_pack/center_of_mass_local"] = np.array([0.15, 0, 0])
    planners = {
        s: SimpleNamespace(detach=lambda: None, profile=manipulation_profile(a))
        for s, a in handover.deployment.arms.items()
    }
    expert = HandoverExpert(handover, planners, lambda: state)
    expert.reset(None)
    assert expert.sites["left"][0] == pytest.approx(expert.asset.bounds[1][0] - 0.011)
    assert expert.sites["left"][0] - expert.sites["right"][0] == pytest.approx(0.1)
    state["tea_pack/pose_world"][3:] = [0, np.sqrt(0.5), 0, np.sqrt(0.5)]
    with pytest.raises(ValueError, match="horizontal"):
        expert.reset(None)
