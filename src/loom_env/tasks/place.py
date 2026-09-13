"""Detect a released object position within the moving container bounds."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.specs.config import pose
from .object_task import ObjectTask


class PlaceTask(ObjectTask):
    def __init__(self, collection):
        super().__init__(collection, extra_parameters=("rim_tolerance",))
        if set(collection.role_bindings) != {"target_object", "container"}:
            raise ValueError("Place requires target_object and container roles")
        self.container_name = collection.role_bindings["container"]
        container = asset_definition(
            collection.scene.objects[self.container_name]["asset"]
        )
        if container.interior is None:
            raise ValueError("Placement requires a reviewed container interior")
        self.container_bounds = np.asarray(container.bounds)

    def reset(self, initial_state):
        super().reset(initial_state)
        pose(initial_state[f"{self.container_name}/pose_world"])

    def update(self, world_state, dt):
        points, grasped = self._state(world_state)
        container_pose = np.asarray(
            pose(world_state[f"{self.container_name}/pose_world"])
        )
        object_position = np.asarray(
            pose(world_state[f"{self.object_name}/pose_world"])
        )[:3]
        local = (
            Rotation.from_quat(container_pose[3:])
            .inv()
            .apply(object_position - container_pose[:3])
        )
        lower, upper = self.container_bounds
        # Match the position-based basket check used by RoboDojo. The upper
        # height allowance is explicit; it does not expand the horizontal bounds.
        inside = (
            np.all(local[:2] > lower[:2])
            and np.all(local[:2] < upper[:2])
            and lower[2] < local[2] < upper[2] + self.parameters["rim_tolerance"]
        )
        return self.finish(
            points,
            inside and not grasped.any(),
            dt,
            "object_inside_and_released",
        )
