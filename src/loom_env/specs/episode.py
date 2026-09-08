"""Control-time samples and results shared by execution and offline data."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Any, Literal

import numpy as np

from .config import ARMS, DeploymentSpec, frozen_json

OutcomeCode = Literal[
    "success", "task_failure", "timeout", "invalid_setup", "runtime_error"
]


def arrays(values: Mapping[str, Any]) -> Mapping[str, np.ndarray]:
    """Own read-only copies, including views into a simulator's reusable buffers."""
    result = {}
    for key, value in values.items():
        if not isinstance(key, str) or any(
            not part or part in {".", ".."} for part in key.split("/")
        ):
            raise ValueError(f"Invalid array path: {key!r}")
        array = np.array(value, copy=True)
        if array.dtype.kind not in "biuf" or not np.isfinite(array).all():
            raise ValueError(f"{key} must contain finite numeric or boolean values")
        array.setflags(write=False)
        result[key] = array
    for key in result:
        parts = key.split("/")
        if any("/".join(parts[:i]) in result for i in range(1, len(parts))):
            raise ValueError(f"Array path conflicts with a parent dataset: {key}")
    return MappingProxyType(result)


def observation_shapes(deployment: DeploymentSpec) -> dict[str, tuple[int, ...]]:
    shapes = {}
    for side in ARMS:
        arm = deployment.arms[side]
        for key, size in (
            ("joint_position", len(arm.joint_names)),
            ("joint_velocity", len(arm.joint_names)),
            ("gripper_position", len(arm.gripper.joint_names)),
            ("gripper_velocity", len(arm.gripper.joint_names)),
            ("tcp_pose_world", 7),
        ):
            shapes[f"robot/{side}/{key}"] = (size,)
    for camera in deployment.cameras:
        shapes[f"cameras/{camera.name}/rgb"] = (camera.height, camera.width, 3)
        shapes[f"cameras/{camera.name}/timestamp"] = ()
        shapes[f"cameras/{camera.name}/valid"] = ()
    return shapes


@dataclass(frozen=True)
class Observation:
    timestamp: float
    values: Mapping[str, np.ndarray]

    def __post_init__(self):
        if not math.isfinite(self.timestamp) or self.timestamp < 0:
            raise ValueError("Observation timestamp must be finite and nonnegative")
        object.__setattr__(self, "values", arrays(self.values))

    def validate(self, deployment: DeploymentSpec) -> None:
        expected = observation_shapes(deployment)
        if set(self.values) != set(expected):
            raise ValueError("Observation fields do not match deployment descriptor")
        for key, shape in expected.items():
            if self.values[key].shape != shape:
                raise ValueError(
                    f"{key}: expected shape {shape}, got {self.values[key].shape}"
                )
            if key.endswith("/tcp_pose_world") and not np.isclose(
                np.linalg.norm(self.values[key][3:]), 1.0, atol=1e-6, rtol=0
            ):
                raise ValueError(f"{key}: TCP quaternion must have unit length")
        for camera in deployment.cameras:
            prefix = f"cameras/{camera.name}"
            if self.values[f"{prefix}/rgb"].dtype != np.uint8:
                raise ValueError(f"{prefix}/rgb must be uint8 RGB")
            if self.values[f"{prefix}/valid"].dtype != np.bool_:
                raise ValueError(f"{prefix}/valid must be boolean")
            capture_time = float(self.values[f"{prefix}/timestamp"])
            if not 0 <= capture_time <= self.timestamp + 1e-9:
                raise ValueError(
                    f"{prefix}: camera capture time is outside episode history"
                )


@dataclass(frozen=True)
class Frame:
    observation: Observation
    # Scene truth is stored separately and is never passed to ActionSource.act.
    world_state: Mapping[str, np.ndarray]

    def __post_init__(self):
        object.__setattr__(self, "world_state", arrays(self.world_state))


@dataclass(frozen=True)
class Transition:
    frame: Frame
    # Processed targets in the same command layout and units as the input action.
    applied_control: np.ndarray


@dataclass(frozen=True)
class Action:
    values: np.ndarray
    events: tuple[Event, ...] = ()

    def __post_init__(self):
        object.__setattr__(self, "values", arrays({"action": self.values})["action"])
        object.__setattr__(self, "events", tuple(self.events))


@dataclass(frozen=True)
class Event:
    kind: Literal["planning", "skill", "task", "runtime"]
    name: str
    # An event at step k refers to observation_k (0 <= k <= T).
    step: int
    details: Mapping[str, Any]

    def __post_init__(self):
        if self.kind not in {"planning", "skill", "task", "runtime"} or not self.name:
            raise ValueError("Event kind and name are required")
        if type(self.step) is not int or self.step < 0:
            raise ValueError("Event step must be a nonnegative integer")
        object.__setattr__(self, "details", frozen_json(self.details))


@dataclass(frozen=True)
class Outcome:
    code: OutcomeCode
    reason: str

    def __post_init__(self):
        if self.code not in {
            "success",
            "task_failure",
            "timeout",
            "invalid_setup",
            "runtime_error",
        }:
            raise ValueError(f"Unknown outcome: {self.code}")
        if not isinstance(self.reason, str) or not self.reason.strip():
            raise ValueError("An outcome must have an explicit reason")


@dataclass(frozen=True)
class TaskStatus:
    outcome: Outcome | None = None

    def __post_init__(self):
        if self.outcome is not None and self.outcome.code not in {
            "success",
            "task_failure",
        }:
            raise ValueError(
                "Tasks can report success or task_failure, not runtime outcomes"
            )


@dataclass(frozen=True)
class EpisodeInput:
    instruction: str
    action_descriptor: Mapping[str, Any]
    observation_descriptor: Mapping[str, Any]
    context: Mapping[str, Any]

    def __post_init__(self):
        for name in ("action_descriptor", "observation_descriptor", "context"):
            object.__setattr__(self, name, frozen_json(getattr(self, name)))
