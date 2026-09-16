"""Tool-mediated sweeping, judged from object poses and measured contacts."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.scenes.workspace import corners
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Outcome, TaskStatus

from .push_region import PushIntoRegionTask


class SweepTask(PushIntoRegionTask):
    roles = frozenset({"target_object", "target_region", "tool"})
    thresholds = (*PushIntoRegionTask.thresholds, "tool_clearance")

    def configure_goal(self, collection):
        super().configure_goal(collection)
        if set(collection.arm_roles) != {"manipulator"}:
            raise ValueError("Sweep requires one manipulator arm role")
        self.side = ARMS.index(collection.arm_roles["manipulator"])
        self.tool = collection.role_bindings["tool"]
        obj = collection.scene.objects[self.tool]
        if self.tool == self.object_name or obj["static"]:
            raise ValueError("Sweep tool must be a distinct dynamic object")
        self.tool_corners = corners(asset_definition(obj["asset"]))

    def metrics(self, world):
        result = super().metrics(world)
        tool_pose = np.asarray(world[f"{self.tool}/pose_world"])
        points = (
            Rotation.from_quat(tool_pose[3:]).apply(self.tool_corners) + tool_pose[:3]
        )
        force = np.asarray(
            world[f"{self.tool}/object_contact_forces_world/{self.object_name}"]
        )
        if force.shape != (3,) or not np.isfinite(force).all():
            raise ValueError("Sweep requires finite tool-object contact force")
        result.update(
            tool_contact_force=float(np.linalg.norm(force)),
            tool_grasped=bool(world[f"{self.tool}/grasped_by"][self.side]),
            tool_clearance=float(points[:, 2].min() - self.support[2]),
        )
        return result

    def reset(self, initial_state):
        super().reset(initial_state)
        self.tool_lifted = False
        self.tool_held = False
        self.lost_time = 0.0

    def update(self, world_state, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be finite and positive")
        m = self.metrics(world_state)
        self.tool_held |= m["tool_grasped"]
        self.lost_time = 0.0 if m["tool_grasped"] else self.lost_time + dt
        self.tool_lifted |= (
            m["tool_grasped"]
            and m["tool_clearance"] >= self.parameters["tool_clearance"]
        )
        reason = self.failure_reason(m)
        if m["contact_force"] > self.parameters["contact_force"]:
            reason = "sweep_target_touched_by_fingers"
        if self.tool_held and self.lost_time >= self.parameters["hold_time"]:
            reason = "sweep_tool_lost"
        self.failure = reason or self.failure
        if self.failure:
            return TaskStatus(Outcome("task_failure", self.failure))
        contact = m["tool_contact_force"] > self.parameters["contact_force"]
        self.contact_seen |= contact and m["tool_grasped"] and self.tool_lifted
        condition = (
            self.contact_seen
            and not contact
            and m["tool_grasped"]
            and m["tool_clearance"] >= self.parameters["tool_clearance"]
            and self.at_goal(m)
        )
        points, _ = self._state(world_state)
        return self.finish(
            points, condition, dt, "object_swept_into_region_and_tool_lifted"
        )
