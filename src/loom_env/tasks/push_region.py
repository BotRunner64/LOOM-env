"""Push a whole object footprint into a visible, scene-bound rectangular region."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.scenes.workspace import transform
from .push import PushTask


class PushIntoRegionTask(PushTask):
    thresholds = (*PushTask.thresholds, "max_tilt")
    goal_parameters = ()

    def configure_goal(self, collection):
        if set(collection.role_bindings) != {"target_object", "target_region"}:
            raise ValueError("Region push requires target_object and target_region")
        self.region_name = collection.role_bindings["target_region"]
        obj = collection.scene.objects[self.region_name]
        region = asset_definition(obj["asset"])
        if not obj["static"] or region.category != "target_region":
            raise ValueError("Push target region must be a fixed scene region")
        self.region_pose = transform(self.support, obj["pose"])
        self.region_rotation = Rotation.from_quat(self.region_pose[3:])
        if not np.allclose(self.region_rotation.apply([0, 0, 1]), [0, 0, 1]):
            raise ValueError("Push target region must be horizontal")
        self.region_bounds = np.asarray(region.bounds)[:, :2]
        centre = self.region_bounds.mean(axis=0)
        self.target_position_world = (
            self.region_rotation.apply([*centre, 0]) + self.region_pose[:3]
        )
        if np.any(np.diff(self.region_bounds, axis=0)[0] <= 0):
            raise ValueError("Push target region must have positive area")

    def metrics(self, world):
        result = super().metrics(world)
        points, _ = self._state(world)
        local = self.region_rotation.inv().apply(points - self.region_pose[:3])
        margin = np.minimum(
            local[:, :2] - self.region_bounds[0],
            self.region_bounds[1] - local[:, :2],
        ).min()
        up = Rotation.from_quat(world[f"{self.object_name}/pose_world"][3:]).apply(
            [0, 0, 1]
        )
        result.update(
            region_margin=float(margin),
            tilt=float(np.arccos(np.clip(up[2], -1, 1))),
        )
        return result

    def at_goal(self, metrics):
        return metrics["region_margin"] >= 0

    def failure_reason(self, metrics):
        reason = super().failure_reason(metrics)
        if reason is None and metrics["tilt"] > self.parameters["max_tilt"]:
            return "push_object_tipped"
        return reason
