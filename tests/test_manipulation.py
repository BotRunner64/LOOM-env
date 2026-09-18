from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.embodiments.contacts import opposing_contacts
from loom_env.experts.actions import CloseGripper, MoveHeld
from loom_env.experts.pick_place import LiftExpert, PickPlaceExpert
from loom_env.specs.config import load_deployment

ROOT = Path(__file__).parents[1]


class UnusedPlanner:
    attached = False

    def attach(self, *args, **kwargs):
        self.attached = True

    def detach(self):
        pass


@pytest.mark.parametrize(
    "name", ["panda", "piper", "x5", "ur5_wsg", "yam", "xarm6_robotiq"]
)
@pytest.mark.parametrize("base_yaw", [0.0, 0.7])
def test_grasp_goal_places_contact_center_on_object(collection, name, base_yaw):
    deployment = load_deployment(ROOT / f"configs/deployments/dual_{name}.yaml")
    arm = deployment.arms["right"]
    arm = replace(
        arm,
        base_pose=(
            *arm.base_pose[:3],
            *(
                Rotation.from_euler("z", base_yaw)
                * Rotation.from_quat(arm.base_pose[3:])
            ).as_quat(),
        ),
    )
    deployment = replace(deployment, arms={**deployment.arms, "right": arm})
    collection = replace(collection, deployment=deployment)
    expert = LiftExpert(collection, UnusedPlanner(), None)
    object_pose = np.array([0.32, -0.24, 0.7622, 0, 0, 0, 1])
    expert.arm.world = {"cube/pose_world": object_pose}
    goal = expert.arm.grasp_goal()
    contact_center = goal[:3] + Rotation.from_quat(goal[3:]).apply(
        expert.arm.profile.tcp_to_grasp
    )
    np.testing.assert_allclose(contact_center, object_pose[:3] + expert.arm.asset.grasp)
    opening_axis = Rotation.from_quat(goal[3:]).apply(
        [1, 0, 0] if name == "ur5_wsg" else [0, 1, 0]
    )
    assert abs(opening_axis[2]) < 1e-12
    if name == "piper":
        # A top grasp must preserve Piper's normal finger order, not roll it over.
        base_y = Rotation.from_quat(arm.base_pose[3:]).apply([0, 1, 0])
        assert np.dot(opening_axis, base_y) > 0
    if name == "ur5_wsg":
        # UR's initial local +X points along -base-Y; reversing it flips the wrist.
        base_rotation = Rotation.from_quat(arm.base_pose[3:])
        np.testing.assert_allclose(
            opening_axis, base_rotation.apply([0, -1, 0]), atol=1e-12
        )
        np.testing.assert_allclose(
            Rotation.from_quat(goal[3:]).apply([0, 1, 0]), [0, 0, -1], atol=1e-12
        )


def test_expert_uses_deployment_gripper_command_limits(spec, frame_factory):
    arm = spec.collection.deployment.arms["right"]
    arm = replace(arm, gripper=replace(arm.gripper, command_limits=((0.01, 0.06),)))
    deployment = replace(
        spec.collection.deployment,
        arms={**spec.collection.deployment.arms, "right": arm},
    )
    spec = replace(spec, collection=replace(spec.collection, deployment=deployment))
    frame = frame_factory(spec)
    expert = LiftExpert(spec.collection, UnusedPlanner(), lambda: frame.world_state)
    expert.reset(None)
    part = deployment.action_slices["right/gripper"]
    assert expert.arm.command[part][0] == pytest.approx(0.06)
    expert.arm.update(frame.observation, frame.world_state)
    CloseGripper().step(expert.arm)
    assert expert.arm.command[part][0] == pytest.approx(0.01)


@pytest.mark.parametrize("position_error, advances", [(0.0, True), (0.02, False)])
def test_stage_completion_uses_position_not_reported_velocity(
    spec, frame_factory, position_error, advances
):
    frame = frame_factory(spec, grasped=(False, True))
    expert = LiftExpert(spec.collection, UnusedPlanner(), lambda: frame.world_state)
    expert.reset(None)
    arm = expert.arm
    expert.arm.planner.plan = lambda *args, **kwargs: (
        np.array([arm.command[arm.arm_slice]]),
        {},
    )
    motion = MoveHeld(np.array([0.5, 0, 1, 0, 0, 0, 1]))
    values = {key: value.copy() for key, value in frame.observation.values.items()}
    values["robot/right/joint_position"][0] += position_error
    values["robot/right/joint_velocity"][:] = 0.2
    observation = replace(frame.observation, values=values)
    for _ in range(4):
        arm.update(observation, frame.world_state)
        motion.step(arm)
    assert motion.done is advances


def test_retreat_returns_to_measured_pose_before_lowering(spec, frame_factory):
    frame = frame_factory(spec, position=(0.5, 0, 0.95), grasped=(False, True))
    expert = PickPlaceExpert(
        spec.collection, UnusedPlanner(), lambda: frame.world_state
    )
    expert.reset(None)
    goals = []

    def plan(observation, world, goal, **kwargs):
        goals.append(goal.copy())
        return np.array([expert.arm.command[expert.arm.arm_slice]]), {}

    expert.arm.planner.plan = plan
    expert.arm.planner.attach = lambda *args, **kwargs: setattr(
        expert.arm.planner, "attached", True
    )
    values = dict(frame.observation.values)
    reached = np.array([0.5, 0, 1.05, 1, 0, 0, 0])
    values["robot/right/tcp_pose_world"] = reached
    world = dict(frame.world_state)
    expert.world_state = lambda: world
    for _ in range(100):
        expert.act(replace(frame.observation, values=values))
        if expert.current is not None and expert.current.name == "lower":
            values["robot/right/tcp_pose_world"] = reached - [0, 0, 0.08, 0, 0, 0, 0]
        if expert.current is not None and expert.current.name == "release":
            world["cube/grasped_by"] = np.array([False, False])
        if expert.finished:
            break
    assert expert.finished
    np.testing.assert_array_equal(goals[-1], reached)


def test_contact_evidence_has_no_opening_or_command_parameters():
    import inspect

    assert tuple(inspect.signature(opposing_contacts).parameters) == ("forces",)
    forces = np.array([[0, 2, 0], [0, -2, 0]])
    assert opposing_contacts(forces)
    assert not opposing_contacts(-np.abs(forces))


@pytest.mark.parametrize("name", ["yam", "xarm6_robotiq"])
def test_float32_opening_boundary_survives_validation_and_revalidation(name):
    from loom_env.embodiments.commands import initial_command

    deployment = load_deployment(ROOT / f"configs/deployments/dual_{name}.yaml")
    command = initial_command(deployment).astype(np.float32)
    accepted = deployment.validate_action(command)
    np.testing.assert_array_equal(deployment.validate_action(accepted), command)
    command[deployment.action_slices["right/gripper"]] += 1e-5
    with pytest.raises(ValueError, match="exceeds right/gripper"):
        deployment.validate_action(command)


def test_yam_opening_matches_tip_geometry_direction():
    deployment = load_deployment(ROOT / "configs/deployments/dual_yam.yaml")
    gripper = deployment.arms["right"].gripper
    mapping = np.asarray(gripper.joint_map)
    # At q=0 the CAD tips meet; negative joint travel separates them.
    np.testing.assert_allclose(mapping @ [0.0] + gripper.joint_offset, [0.0, 0.0])
    np.testing.assert_allclose(
        mapping @ [0.0939] + gripper.joint_offset, [-0.04695, -0.04695]
    )
