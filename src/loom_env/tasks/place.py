"""Keep a released object's conservative envelope inside a reviewed interior."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.specs.config import pose
from .object_task import ObjectTask


class PlaceTask(ObjectTask):
    def __init__(self, collection):
        super().__init__(collection, extra_parameters=("containment_tolerance",))
        if set(collection.role_bindings) != {"target_object", "container"}:
            raise ValueError("Place requires target_object and container roles")
        self.container_name = collection.role_bindings["container"]
        container = asset_definition(
            collection.scene.objects[self.container_name]["asset"]
        )
        if container.interior is None:
            raise ValueError("Placement requires a reviewed container interior")
        self.region_size = np.array(container.interior[1])

    def reset(self, initial_state):
        super().reset(initial_state)
        pose(initial_state[f"{self.container_name}/region_pose_world"])

    def update(self, world_state, dt):
        points, grasped = self._state(world_state)
        region = np.asarray(
            pose(world_state[f"{self.container_name}/region_pose_world"])
        )
        local = Rotation.from_quat(region[3:]).inv().apply(points - region[:3])
        inside = np.all(
            np.abs(local)
            <= self.region_size / 2 + self.parameters["containment_tolerance"]
        )
        return self.finish(
            points,
            inside and not grasped.any(),
            dt,
            "object_inside_and_released",
        )
