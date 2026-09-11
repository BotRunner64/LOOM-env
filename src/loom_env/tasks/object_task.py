"""Measured object geometry and continuous task-condition timing."""

from collections.abc import Mapping
import math

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.scenes.workspace import corners, workspace
from loom_env.specs.config import pose
from loom_env.specs.episode import Outcome, TaskStatus


class ObjectTask:
    def __init__(self, collection, extra_parameters=()):
        self.object_name = collection.role_bindings["target_object"]
        obj = collection.scene.objects[self.object_name]
        if obj["static"]:
            raise ValueError("Task target_object must be dynamic")
        self.asset = asset_definition(obj["asset"])
        self._corners = corners(self.asset)
        self.support, _ = workspace(collection.scene)
        parameters = dict(collection.task.parameters)
        required = {
            "hold_time",
            "failure_drop",
            *extra_parameters,
        }
        if set(parameters) != required:
            raise ValueError(f"Task parameters must be exactly {sorted(required)}")
        if any(
            not isinstance(v, (int, float))
            or isinstance(v, bool)
            or not math.isfinite(v)
            or v <= 0
            for v in parameters.values()
        ):
            raise ValueError("Task thresholds must be finite and positive")
        self.parameters = parameters
        self._condition_time = 0.0

    def _state(self, world: Mapping[str, np.ndarray]):
        obj = np.asarray(pose(world[f"{self.object_name}/pose_world"]))
        grasped = np.asarray(world[f"{self.object_name}/grasped_by"])
        if grasped.shape != (2,) or grasped.dtype != np.bool_:
            raise ValueError(
                "grasped_by must contain boolean evidence for left and right"
            )
        points = Rotation.from_quat(obj[3:]).apply(self._corners) + obj[:3]
        return points, grasped

    def reset(self, initial_state):
        self._state(initial_state)
        self._condition_time = 0.0

    def finish(self, points, condition, dt, reason):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be finite and positive")
        if points[:, 2].min() < self.support[2] - self.parameters["failure_drop"]:
            return TaskStatus(Outcome("task_failure", "object_below_workspace"))
        self._condition_time = self._condition_time + dt if condition else 0.0
        if self._condition_time + 1e-9 >= self.parameters["hold_time"]:
            return TaskStatus(Outcome("success", reason))
        return TaskStatus()
