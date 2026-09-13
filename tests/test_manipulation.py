from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.embodiments.commands import gripper_command
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.experts.pick_place import LiftExpert, PickPlaceExpert
from loom_env.specs.config import load_deployment

ROOT = Path(__file__).parents[1]


class UnusedPlanner:
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
    expert.stage_index = expert.STAGES.index("descend")
    object_pose = np.array([0.32, -0.24, 0.7622, 0, 0, 0, 1])
    goal = expert._goal({"cube/pose_world": object_pose}, None)
    contact_center = goal[:3] + Rotation.from_quat(goal[3:]).apply(
        expert.profile.tcp_to_grasp
    )
    np.testing.assert_allclose(contact_center, object_pose[:3] + expert.asset.grasp)
    opening_axis = Rotation.from_quat(goal[3:]).apply(
        [1, 0, 0] if name == "ur5_wsg" else [0, 1, 0]
    )
    assert abs(opening_axis[2]) < 1e-12
    if name == "piper":
        # A top grasp must preserve Piper's normal finger order, not roll it over.
        base_y = Rotation.from_quat(arm.base_pose[3:]).apply([0, 1, 0])
        assert np.dot(opening_axis, base_y) > 0


def test_grasp_evidence_uses_signed_joint_mapping(collection):
    gripper = replace(
        collection.deployment.arms["right"].gripper,
        joint_map=((-0.5,), (0.5,)),
    )
    forces = np.array([[0, 2, 0], [0, -2, 0]])
    opening = gripper_command(gripper, np.array([-0.015, 0.015]))
    assert opposing_contacts(forces, opening, gripper.command_limits[0])
    assert not opposing_contacts(forces, 0.0, gripper.command_limits[0])


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
    assert expert.command[part][0] == pytest.approx(0.06)
    expert.stage_index = expert.STAGES.index("close")
    assert expert.act(frame.observation).values[part][0] == pytest.approx(0.01)


@pytest.mark.parametrize("position_error, advances", [(0.0, True), (0.02, False)])
def test_stage_completion_uses_position_not_reported_velocity(
    spec, frame_factory, position_error, advances
):
    frame = frame_factory(spec, grasped=(False, True))
    expert = LiftExpert(spec.collection, UnusedPlanner(), lambda: frame.world_state)
    expert.reset(None)
    expert.stage_index = expert.STAGES.index("lift")
    expert.started = True
    expert.path = np.array([expert.command[expert.arm_slice]])
    expert.path_index = len(expert.path)
    values = {key: value.copy() for key, value in frame.observation.values.items()}
    values["robot/right/joint_position"][0] += position_error
    values["robot/right/joint_velocity"][:] = 0.2
    observation = replace(frame.observation, values=values)
    for _ in range(3):
        expert.act(observation)
    assert expert.stage == ("wait" if advances else "lift")


def test_retreat_returns_to_measured_pose_before_lowering(spec, frame_factory):
    frame = frame_factory(spec, position=(0.5, 0, 0.95), grasped=(False, True))
    expert = PickPlaceExpert(
        spec.collection, UnusedPlanner(), lambda: frame.world_state
    )
    expert.reset(None)
    expert.stage_index = expert.STAGES.index("lower")
    values = dict(frame.observation.values)
    reached = np.array([0.5, 0, 1.05, 1, 0, 0, 0])
    values["robot/right/tcp_pose_world"] = reached
    expert._goal(frame.world_state, replace(frame.observation, values=values))
    expert.stage_index = expert.STAGES.index("retreat")
    values["robot/right/tcp_pose_world"] = reached - [0, 0, 0.08, 0, 0, 0, 0]
    goal = expert._goal(frame.world_state, replace(frame.observation, values=values))
    np.testing.assert_array_equal(goal, reached)


@pytest.mark.parametrize("limits", [(0.0, 0.08), (0.0, 0.81), (0.0054, 0.11)])
def test_contact_evidence_uses_fraction_for_linear_and_angular_commands(limits):
    low, high = limits
    forces = np.array([[0, 2, 0], [0, -2, 0]])
    assert opposing_contacts(forces, low + 0.4 * (high - low), limits)
    for fraction in (0.0, 0.02, 0.995, 1.0):
        assert not opposing_contacts(forces, low + fraction * (high - low), limits)
    assert not opposing_contacts(-np.abs(forces), low + 0.4 * (high - low), limits)


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
