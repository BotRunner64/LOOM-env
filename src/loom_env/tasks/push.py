"""Push a supported object to a workspace-relative pose without grasping it."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.scenes.workspace import corners, transform, workspace
from loom_env.specs.config import pose, vector
from loom_env.specs.episode import Outcome, TaskStatus
from .object_task import ObjectTask


class PushTask(ObjectTask):
    def __init__(self, collection):
        super().__init__(
            collection,
            ("position_tolerance", "angle_tolerance", "max_clearance", "contact_force"),
            ("target_position", "target_yaw"),
        )
        if set(collection.role_bindings) != {"target_object"}:
            raise ValueError("Push requires only target_object")
        position = vector(self.parameters["target_position"], 2, "target_position")
        yaw = self.parameters["target_yaw"]
        if (
            isinstance(yaw, bool)
            or not isinstance(yaw, (float, int))
            or not math.isfinite(yaw)
        ):
            raise ValueError("target_yaw must be a finite angle in radians")
        frame, size = workspace(collection.scene)
        local_rotation = Rotation.from_euler("z", yaw)
        rotated = local_rotation.apply(corners(self.asset))
        if np.any(rotated[:, :2] + position < -size / 2) or np.any(
            rotated[:, :2] + position > size / 2
        ):
            raise ValueError("Push target extends outside the workspace")
        self.target_pose = transform(
            frame, [*position, -rotated[:, 2].min(), *local_rotation.as_quat()]
        )
        self.contact_seen = False
        self.failure = None

    def metrics(self, world):
        points, grasped = self._state(world)
        measured = np.asarray(pose(world[f"{self.object_name}/pose_world"]))
        forces = np.asarray(world[f"{self.object_name}/finger_contact_forces_world"])
        if forces.shape != (2, 2, 3) or not np.isfinite(forces).all():
            raise ValueError(
                "Push requires finite contact forces for both arms and fingers"
            )
        rotation_error = Rotation.from_quat(
            self.target_pose[3:]
        ).inv() * Rotation.from_quat(measured[3:])
        return {
            "position_error": float(
                np.linalg.norm(measured[:2] - self.target_pose[:2])
            ),
            "angle_error": float(rotation_error.magnitude()),
            "clearance": float(points[:, 2].min() - self.support[2]),
            "contact_force": float(np.linalg.norm(forces, axis=-1).max()),
            "grasped": bool(grasped.any()),
        }

    def reset(self, initial_state):
        super().reset(initial_state)
        self.contact_seen = False
        self.failure = None
        metrics = self.metrics(initial_state)
        if metrics["position_error"] <= 2 * self.parameters["position_tolerance"]:
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
        if m["grasped"]:
            self.failure = "push_object_grasped"
        elif m["clearance"] > self.parameters["max_clearance"]:
            self.failure = "push_object_lifted"
        elif m["clearance"] < -self.parameters["max_clearance"]:
            self.failure = "push_object_below_support"
        if self.failure is not None:
            return TaskStatus(Outcome("task_failure", self.failure))
        contact = m["contact_force"] > self.parameters["contact_force"]
        self.contact_seen |= contact
        condition = (
            self.contact_seen
            and not contact
            and m["position_error"] <= self.parameters["position_tolerance"]
            and m["angle_error"] <= self.parameters["angle_tolerance"]
        )
        points, _ = self._state(world_state)
        return self.finish(points, condition, dt, "object_pushed_to_pose_and_released")
