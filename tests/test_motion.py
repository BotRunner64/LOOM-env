from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from loom_env.runtime.motion import (
    MotionCheck,
    MotionSource,
    gripper_command,
    gripper_mapping_error,
)
from loom_env.specs.config import load_deployment
from loom_env.specs.episode import Observation

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "name,dimension",
    [
        ("panda", 16),
        ("piper", 14),
        ("x5", 14),
        ("ur5_wsg", 14),
        ("xarm6_robotiq", 14),
        ("openarm", 16),
    ],
)
def test_native_motion_layout_limits_and_stationary_arm(name, dimension):
    dep = load_deployment(ROOT / f"configs/deployments/dual_{name}.yaml")
    source = MotionSource(dep)
    source.reset(None)
    assert dep.action_dim == dimension
    for step in range(240):
        time = step * dep.control_dt
        command = source.act(Observation(time, {"test": np.zeros(1)})).values
        dep.validate_action(command)
        if time < 8:
            held = "left" if time < 4 else "right"
            part = dep.action_slices[f"{held}/arm"]
            np.testing.assert_array_equal(command[part], source.home[part])
    for side in ("left", "right"):
        assert np.all(np.abs(source.offsets[side]) >= 0.15)


def test_gripper_measurement_uses_signed_mapping_not_joint_sum():
    dep = load_deployment(ROOT / "configs/deployments/dual_piper.yaml")
    grip = replace(dep.arms["left"].gripper, joint_map=((0.5,), (-0.5,)))
    assert gripper_command(grip, np.array([0.03, -0.03])) == pytest.approx(0.06)
    with pytest.raises(ValueError, match="observable"):
        gripper_command(replace(grip, joint_map=((0.0,), (0.0,))), np.zeros(2))


def test_motion_check_rejects_a_stuck_joint_despite_other_joints_moving():
    dep = load_deployment(ROOT / "configs/deployments/dual_piper.yaml")
    task = MotionCheck(dep)
    task.reset({})
    state = {
        "diagnostic/time": np.array(9.0),
        "diagnostic/tracking_error": np.zeros(2),
        "diagnostic/gripper_command": np.zeros(2),
        "diagnostic/left/joint_excursion": np.array([0.55, 0.15, 0, 0.15, 0.15, 0.15]),
        "diagnostic/right/joint_excursion": np.full(6, 0.15),
    }
    assert task.update(state, 0.05).outcome is None
    state.update(
        {
            "diagnostic/time": np.array(12.0),
            "diagnostic/gripper_command": np.full(2, 0.08),
        }
    )
    assert task.update(state, 0.05).outcome.code == "task_failure"


def test_gripper_motion_uses_configured_range():
    dep = load_deployment(ROOT / "configs/deployments/dual_piper.yaml")
    arms = {
        side: replace(arm, gripper=replace(arm.gripper, command_limits=((0.02, 0.06),)))
        for side, arm in dep.arms.items()
    }
    dep = replace(dep, arms=arms)
    source = MotionSource(dep)
    source.reset(None)
    closed = source.act(Observation(9.0, {"test": np.zeros(1)})).values
    for side in ("left", "right"):
        part = dep.action_slices[f"{side}/gripper"]
        assert source.home[part][0] == pytest.approx(0.06)
        assert closed[part][0] == pytest.approx(0.02)


def test_native_rotary_gripper_mapping_preserves_all_six_links():
    dep = load_deployment(ROOT / "configs/deployments/dual_xarm6_robotiq.yaml")
    grip = dep.arms["left"].gripper
    assert grip.unit == "rad"
    assert len(grip.joint_names) == 6
    for opening in (0.0, 0.4, 0.81):
        targets = np.asarray(grip.joint_map) @ [opening] + grip.joint_offset
        np.testing.assert_allclose(
            targets, np.array([1, 1, -1, 1, 1, -1]) * (0.81 - opening)
        )
        assert gripper_command(grip, targets) == pytest.approx(opening)


def test_linkage_check_detects_one_stuck_robotiq_joint():
    dep = load_deployment(ROOT / "configs/deployments/dual_xarm6_robotiq.yaml")
    grip = dep.arms["left"].gripper
    positions = np.array([[0, 0, 0, 0, 0, 0], [0.81, 0.81, -0.81, 0.81, 0.81, -0.81]])
    assert gripper_mapping_error(grip, positions) < 1e-12
    positions[1, 1] = 0
    assert gripper_mapping_error(grip, positions) > 0.6
