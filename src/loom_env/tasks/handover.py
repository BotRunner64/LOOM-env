"""Verify an airborne handover from measured contact history, not expert events."""

import numpy as np

from loom_env.specs.config import ARMS
from loom_env.specs.episode import Outcome, TaskStatus

from .object_task import ObjectTask


class HandoverTask(ObjectTask):
    def __init__(self, collection):
        super().__init__(collection, ("clearance", "transfer_distance", "stable_time", "linear_speed", "angular_speed"))
        if set(collection.role_bindings) != {"target_object"}:
            raise ValueError("Handover requires only target_object")
        if (
            set(collection.arm_roles) != {"giver", "receiver"}
            or len(set(collection.arm_roles.values())) != 2
        ):
            raise ValueError("Handover requires different giver and receiver arms")
        self.giver = ARMS.index(collection.arm_roles["giver"])
        self.receiver = ARMS.index(collection.arm_roles["receiver"])

    def reset(self, initial_state):
        super().reset(initial_state)
        if self._state(initial_state)[1].any():
            raise ValueError("Handover must start with an ungrasped object")
        self.phase = "giver"
        self.elapsed = 0.0
        self.exchange_position = None

    def update(self, world_state, dt):
        points, grasped = self._state(world_state)
        # Also validates dt and checks a physical fall before phase-specific logic.
        status = self.finish(points, False, dt, "unused")
        if status.outcome is not None:
            return status
        lifted = points[:, 2].min() >= self.support[2] + self.parameters["clearance"]
        giver, receiver = grasped[self.giver], grasped[self.receiver]
        if self.phase != "giver" and (not lifted or not grasped.any()):
            return TaskStatus(Outcome("task_failure", "handover_lost_airborne_support"))
        position = np.asarray(world_state[f"{self.object_name}/pose_world"])[:3]
        if self.phase == "giver":
            ready = lifted and giver and not receiver
        elif self.phase == "overlap":
            if not giver:
                return TaskStatus(
                    Outcome("task_failure", "giver_released_before_receiver_verified")
                )
            ready = lifted and giver and receiver
        else:
            velocity = np.asarray(world_state[f"{self.object_name}/velocity_world"])
            if velocity.shape != (6,) or not np.isfinite(velocity).all():
                raise ValueError("Handover requires a finite six-dimensional object velocity")
            forces = np.asarray(
                world_state[f"{self.object_name}/finger_contact_forces_world"]
            )
            giver_clear = np.linalg.norm(forces[self.giver], axis=1).max() < 0.02
            ready = (
                lifted
                and receiver
                and not giver
                and giver_clear
                and position[2] - self.exchange_position[2]
                >= self.parameters["transfer_distance"]
                and np.linalg.norm(velocity[:3]) <= self.parameters["linear_speed"]
                and np.linalg.norm(velocity[3:]) <= self.parameters["angular_speed"]
            )
        self.elapsed = self.elapsed + dt if ready else 0.0
        duration = self.parameters["stable_time" if self.phase == "receiver" else "hold_time"]
        if self.elapsed + 1e-9 >= duration:
            if self.phase == "receiver":
                return TaskStatus(
                    Outcome("success", "object_handed_over_and_independently_held")
                )
            if self.phase == "overlap":
                self.exchange_position = position.copy()
            self.phase = "overlap" if self.phase == "giver" else "receiver"
            self.elapsed = 0.0
        return TaskStatus()
