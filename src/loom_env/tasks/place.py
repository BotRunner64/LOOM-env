"""An oriented box must be inside a target region, released and settled.

The environment supplies measured poses, velocities and two-arm grasp evidence.
Planner success never enters this predicate. Stable time resets when any
condition stops holding.
"""

from collections.abc import Mapping
import math
from itertools import product

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.specs.config import CollectionSpec, pose, vector
from loom_env.specs.episode import Outcome, TaskStatus


class PlaceTask:
    def __init__(self, collection: CollectionSpec):
        parameters = dict(collection.task.parameters)
        self.object_name = collection.role_bindings["target_object"]
        self.container_name = collection.role_bindings["container"]
        self.object_size = np.array(
            vector(parameters.pop("object_size"), 3, "object_size")
        )
        self.region_size = np.array(
            vector(parameters.pop("region_size"), 3, "region_size")
        )
        self.settle_time = parameters.pop("settle_time")
        self.linear_speed = parameters.pop("max_linear_speed")
        self.angular_speed = parameters.pop("max_angular_speed")
        self.failure_height = parameters.pop("failure_height")
        if parameters:
            raise ValueError(f"Unknown place task parameters: {sorted(parameters)}")
        if np.any(self.object_size <= 0) or np.any(self.region_size <= 0):
            raise ValueError("Object and target region dimensions must be positive")
        if not all(
            math.isfinite(value) and value > 0
            for value in (
                self.settle_time,
                self.linear_speed,
                self.angular_speed,
            )
        ) or not math.isfinite(self.failure_height):
            raise ValueError("Invalid task thresholds")
        self._corners = (
            np.array(list(product((-0.5, 0.5), repeat=3))) * self.object_size
        )
        self._settled = 0.0

    def _state(self, world: Mapping[str, np.ndarray]):
        obj = np.asarray(pose(world[f"{self.object_name}/pose_world"]))
        region = np.asarray(pose(world[f"{self.container_name}/region_pose_world"]))
        velocity = np.asarray(
            vector(world[f"{self.object_name}/velocity_world"], 6, "velocity")
        )
        grasped = np.asarray(world[f"{self.object_name}/grasped_by"])
        if grasped.shape != (2,) or grasped.dtype != np.bool_:
            raise ValueError(
                "grasped_by must contain boolean evidence for left and right"
            )
        return obj, region, velocity, grasped

    def reset(self, initial_state: Mapping[str, np.ndarray]) -> None:
        self._state(initial_state)
        self._settled = 0.0

    def update(self, world_state: Mapping[str, np.ndarray], dt: float) -> TaskStatus:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be finite and positive")
        obj, region, velocity, grasped = self._state(world_state)
        if obj[2] < self.failure_height:
            return TaskStatus(Outcome("task_failure", "object_below_workspace"))
        world_corners = Rotation.from_quat(obj[3:]).apply(self._corners) + obj[:3]
        local_corners = (
            Rotation.from_quat(region[3:]).inv().apply(world_corners - region[:3])
        )
        # Float32 poses and resting contacts need a small geometric tolerance.
        # Ten micrometres accommodates numerical penetration at the box floor.
        inside = np.all(np.abs(local_corners) <= self.region_size / 2 + 1e-5)
        settled = (
            inside
            and not grasped.any()
            and np.linalg.norm(velocity[:3]) <= self.linear_speed
            and np.linalg.norm(velocity[3:]) <= self.angular_speed
        )
        self._settled = self._settled + dt if settled else 0.0
        if self._settled + 1e-9 >= self.settle_time:
            return TaskStatus(Outcome("success", "object_inside_released_and_settled"))
        return TaskStatus()
