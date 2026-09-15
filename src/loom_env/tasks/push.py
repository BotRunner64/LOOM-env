"""Push a supported object into a workspace-relative goal region without grasping."""

import math

import numpy as np

from loom_env.scenes.workspace import transform, workspace
from loom_env.specs.config import pose, vector
from loom_env.specs.episode import Outcome, TaskStatus
from .object_task import ObjectTask


class PushTask(ObjectTask):
    thresholds = ("position_tolerance", "max_clearance", "contact_force")
    goal_parameters = ("target_position",)

    def __init__(self, collection):
        super().__init__(
            collection,
            self.thresholds,
            self.goal_parameters,
        )
        self.configure_goal(collection)
        self.contact_seen = False
        self.failure = None

    def configure_goal(self, collection):
        if set(collection.role_bindings) != {"target_object"}:
            raise ValueError("Push requires only target_object")
        position = vector(self.parameters["target_position"], 2, "target_position")
        frame, size = workspace(collection.scene)
        if np.any(np.abs(position) + self.parameters["position_tolerance"] > size / 2):
            raise ValueError("Push goal region extends outside the workspace")
        # The goal is a circle on the support plane, independent of object yaw.
        self.target_position_world = transform(frame, [*position, 0, 0, 0, 0, 1])[:3]

    def metrics(self, world):
        points, grasped = self._state(world)
        measured = np.asarray(pose(world[f"{self.object_name}/pose_world"]))
        forces = np.asarray(world[f"{self.object_name}/finger_contact_forces_world"])
        if forces.shape != (2, 2, 3) or not np.isfinite(forces).all():
            raise ValueError(
                "Push requires finite contact forces for both arms and fingers"
            )
        return {
            "position_error": float(
                np.linalg.norm(measured[:2] - self.target_position_world[:2])
            ),
            "clearance": float(points[:, 2].min() - self.support[2]),
            "contact_force": float(np.linalg.norm(forces, axis=-1).max()),
            "grasped": bool(grasped.any()),
        }

    def reset(self, initial_state):
        super().reset(initial_state)
        self.contact_seen = False
        self.failure = None
        metrics = self.metrics(initial_state)
        if metrics["position_error"] <= 2 * self.parameters[
            "position_tolerance"
        ] or self.at_goal(metrics):
            raise ValueError(
                "Push initial object must start outside twice the goal tolerance"
            )
        if (
            metrics["grasped"]
            or abs(metrics["clearance"]) > self.parameters["max_clearance"]
        ):
            raise ValueError("Push initial object must be ungrasped and supported")

    def update(self, world_state, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be finite and positive")
        m = self.metrics(world_state)
        self.failure = self.failure_reason(m) or self.failure
        if self.failure is not None:
            return TaskStatus(Outcome("task_failure", self.failure))
        contact = m["contact_force"] > self.parameters["contact_force"]
        self.contact_seen |= contact
        condition = self.contact_seen and not contact and self.at_goal(m)
        points, _ = self._state(world_state)
        return self.finish(
            points, condition, dt, "object_pushed_to_region_and_released"
        )

    def at_goal(self, metrics):
        return metrics["position_error"] <= self.parameters["position_tolerance"]

    def failure_reason(self, metrics):
        if metrics["grasped"]:
            return "push_object_grasped"
        if metrics["clearance"] > self.parameters["max_clearance"]:
            return "push_object_lifted"
        if metrics["clearance"] < -self.parameters["max_clearance"]:
            return "push_object_below_support"
        return None
