"""Small deterministic test doubles; no alternate simulation backend."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from loom_env.specs.config import EpisodeSpec, load_collection
from loom_env.specs.episode import Frame, Observation, observation_shapes

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def collection():
    value = load_collection(ROOT / "configs/collection/pick_place.yaml")
    # Stable synthetic instance names keep storage tests independent of presets.
    objects = dict(value.scene.objects)
    objects["cube"] = objects.pop("object")
    objects["container"] = objects.pop("basket")
    return replace(
        value,
        scene=replace(value.scene, objects=objects),
        role_bindings={"target_object": "cube", "container": "container"},
        max_steps=3,
    )


@pytest.fixture
def spec(collection):
    assets = {arm.asset for arm in collection.deployment.arms.values()} | {
        obj["asset"] for obj in collection.scene.objects.values()
    }
    return EpisodeSpec(
        id="test-episode",
        collection=collection,
        seed=7,
        initial_state={"test_state": [0.0]},
        sampled_parameters={"test": True},
        asset_versions=dict.fromkeys(assets, "test-only-v1"),
        runtime_versions={"test-double": "1"},
        provenance={"source": "test"},
    )


@pytest.fixture
def frame_factory():
    def make(spec, step=0, *, position=(0.5, 0.0, 0.8), grasped=(False, False)):
        dep = spec.collection.deployment
        values = {
            key: np.zeros(shape) for key, shape in observation_shapes(dep).items()
        }
        for side, arm in dep.arms.items():
            values[f"robot/{side}/joint_position"] = np.array(arm.initial_positions)
            values[f"robot/{side}/tcp_pose_world"][-1] = 1.0
        for camera in dep.cameras:
            prefix = f"cameras/{camera.name}"
            values[f"{prefix}/rgb"] = np.zeros(
                (camera.height, camera.width, 3), dtype=np.uint8
            )
            values[f"{prefix}/valid"] = np.array(step % camera.period_steps == 0)
            values[f"{prefix}/timestamp"] = np.array(
                (step // camera.period_steps) * camera.period_steps * dep.control_dt
            )
        return Frame(
            Observation(step * dep.control_dt, values),
            {
                "cube/pose_world": np.array([*position, 0.0, 0.0, 0.0, 1.0]),
                "cube/velocity_world": np.zeros(6),
                "cube/grasped_by": np.array(grasped),
                "container/pose_world": np.array(
                    [0.5, 0.0, 0.79565, 0.0, 0.0, 0.0, 1.0]
                ),
                "container/velocity_world": np.zeros(6),
                "container/grasped_by": np.zeros(2, dtype=bool),
                "container/region_pose_world": np.array(
                    [0.5, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]
                ),
            },
        )

    return make


@pytest.fixture
def action(spec):
    dep = spec.collection.deployment
    result = []
    for arm in dep.arms.values():
        result.extend(arm.initial_positions)
        result.extend(high for low, high in arm.gripper.command_limits)
    return np.array(result)
