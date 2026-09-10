from dataclasses import replace
import json
import subprocess
import sys

import numpy as np
import pytest

from loom_env.specs.config import (
    collection_from_dict,
    episode_from_dict,
    load_collection,
    plain,
)
from loom_env.specs.episode import Frame, Observation


def test_config_roundtrip_and_fixed_left_right_order(collection, spec, action):
    assert episode_from_dict(json.loads(json.dumps(plain(spec)))) == spec
    dep = replace(
        collection.deployment,
        arms=dict(reversed(list(collection.deployment.arms.items()))),
    )
    assert list(dep.action_slices) == [
        "left/arm",
        "left/gripper",
        "right/arm",
        "right/gripper",
    ]
    assert dep.action_dim == 16 and dep.decimation == 6
    np.testing.assert_array_equal(dep.validate_action(action), action)
    np.testing.assert_allclose(
        np.array(dep.arms["left"].gripper.joint_map) @ [0.08], [0.04, 0.04]
    )


def test_six_dof_dimension_is_independent_of_gripper(collection):
    arm = collection.deployment.arms["left"]
    arm = replace(
        arm,
        joint_names=tuple(f"joint{i}" for i in range(6)),
        joint_limits=((-2, 2),) * 6,
        initial_positions=(0,) * 6,
    )
    dep = replace(collection.deployment, arms={"left": arm, "right": arm})
    assert dep.action_dim == 14
    assert len(arm.gripper.joint_names) == 2
    assert len(arm.gripper.command_names) == 1


@pytest.mark.parametrize(
    "mutation",
    [
        lambda v: v["deployment"]["arms"].pop("left"),
        lambda v: v["deployment"].update(control_type="tcp_delta"),
        lambda v: v["deployment"].update(physics_dt=0),
        lambda v: v["deployment"].update(control_dt=0.051),
        lambda v: v["deployment"]["arms"]["left"].update(base_pose=[0] * 7),
        lambda v: v["deployment"]["arms"]["left"].update(joint_names=["same"] * 7),
        lambda v: v["deployment"]["arms"]["left"]["gripper"].update(joint_map=[[1]]),
        lambda v: v["role_bindings"].update(target_object="missing"),
        lambda v: v["role_bindings"].update(target_object="container"),
        lambda v: v["deployment"].update(capabilities=["grasp"]),
        lambda v: v.update(max_steps=True),
        lambda v: v.update(unknown_field=1),
    ],
)
def test_invalid_config_is_rejected(collection, mutation):
    value = plain(collection)
    mutation(value)
    with pytest.raises((ValueError, TypeError)):
        collection_from_dict(value)


def test_duplicate_yaml_and_unknown_keys_fail(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("task: a\ntask: b\n")
    with pytest.raises(ValueError, match="unique"):
        load_collection(path)


@pytest.mark.parametrize("value", [np.zeros(14), np.full(16, np.nan), np.full(16, 100)])
def test_invalid_action_fails_without_silent_clipping(collection, value):
    with pytest.raises(ValueError):
        collection.deployment.validate_action(value)


def test_spec_deep_immutability_and_version_validation(spec):
    with pytest.raises(TypeError):
        spec.collection.task.parameters["settle_time"] = 1
    with pytest.raises(TypeError):
        spec.initial_state["test_state"][0] = 1
    with pytest.raises(ValueError, match="asset"):
        replace(spec, asset_versions={})
    with pytest.raises(ValueError, match="schema_version"):
        replace(spec, schema_version=2)
    with pytest.raises(ValueError, match="JSON"):
        replace(spec, provenance={"bad": float("nan")})
    with pytest.raises(ValueError, match="identifier"):
        replace(spec, id="../escape")


def test_observation_owns_buffers_and_separates_truth(spec, frame_factory):
    frame = frame_factory(spec)
    raw = np.ones(7)
    obs = Observation(0.0, {"test": raw})
    raw[:] = 0
    assert obs.values["test"].sum() == 7
    with pytest.raises(ValueError):
        obs.values["test"][0] = 3
    assert not any("cube" in key for key in frame.observation.values)
    with pytest.raises(ValueError, match="conflicts"):
        Frame(frame.observation, {"cube": [0], "cube/pose": [1]})


def test_offline_imports_block_all_simulation_libraries():
    script = """
import importlib.abc
import sys
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'isaaclab', 'isaacsim', 'omni', 'torch', 'curobo'}:
            raise RuntimeError('Simulation dependency imported: ' + fullname)
sys.meta_path.insert(0, Block())
import loom_env
import loom_env.specs.config
import loom_env.data.episodes
import loom_env.runtime.runner
import loom_env.runtime.replay
import loom_env.tasks.place
import loom_env.embodiments.assets
import loom_env.runtime.motion
"""
    subprocess.run([sys.executable, "-c", script], check=True)


@pytest.mark.parametrize(
    "parent", [None, 0, "", "left", "wrist/link6", "left/../link6", "right/a/b"]
)
def test_camera_rejects_ambiguous_mount_frames(collection, parent):
    with pytest.raises(ValueError, match="parent_frame|identifier"):
        replace(collection.deployment.cameras[0], parent_frame=parent)


@pytest.mark.parametrize(
    "change",
    [
        {"focal_length": 0},
        {"horizontal_aperture": float("nan")},
        {"clipping_range": [0.1, 0.01]},
        {"clipping_range": [0, 20]},
    ],
)
def test_camera_rejects_invalid_optics(collection, change):
    with pytest.raises(ValueError, match="Camera"):
        replace(collection.deployment.cameras[0], **change)


def test_instruction_follows_role_bindings_without_changing_scene():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1] / "configs/collection"
    red = load_collection(root / "red_to_green.yaml")
    blue = load_collection(root / "blue_to_green.yaml")
    yellow = load_collection(root / "red_to_yellow.yaml")
    assert red.scene == blue.scene == yellow.scene
    assert red.instruction == "将红色方块放入绿色容器，然后松开夹爪。"
    assert blue.instruction == "将蓝色方块放入绿色容器，然后松开夹爪。"
    assert yellow.instruction == "将红色方块放入黄色容器，然后松开夹爪。"
    for instruction in (
        "{missing}",
        "{target_object.foo}",
        "{target_object!r}",
        "{target_object:10}",
    ):
        with pytest.raises(ValueError, match="placeholder"):
            replace(red, task=replace(red.task, instruction=instruction))
