"""Measured hinge opening, prior contact, and released gripper; no expert phases."""

import numpy as np

from loom_env.assets.catalog import asset_definition, is_articulated
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.specs.episode import Outcome, TaskStatus


class OpenLaptopTask:
    def __init__(self, collection):
        if set(collection.role_bindings) != {"target_object"} or set(
            collection.arm_roles
        ) != {"manipulator"}:
            raise ValueError("Opening needs one object and one manipulator")
        self.name = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.name]["asset"])
        if not is_articulated(self.asset):
            raise ValueError("Opening requires an articulated object")
        self.side = ("left", "right").index(collection.arm_roles["manipulator"])
        self.parameters = dict(collection.task.parameters)
        if set(self.parameters) != {
            "target_angle",
            "angle_tolerance",
            "hold_time",
        } or not all(np.isfinite(v) for v in self.parameters.values()):
            raise ValueError(
                "Opening needs finite angle target, tolerance and hold time"
            )
        if self.parameters["angle_tolerance"] <= 0 or self.parameters["hold_time"] <= 0:
            raise ValueError("Opening tolerances must be positive")
        self.q_key = f"{self.name}/joints/{self.asset.joint}/position"
        self.force_key = (
            f"{self.name}/links/{self.asset.moving_body}/finger_contact_forces_world"
        )

    def reset(self, initial_state):
        self.initial_q = float(initial_state[self.q_key])
        if (
            abs(self.initial_q - self.parameters["target_angle"])
            <= self.parameters["angle_tolerance"]
        ):
            raise ValueError("Opening must begin outside the target interval")
        self.contact_seen = False
        self.hold = 0.0

    def update(self, world_state, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be positive")
        forces = np.asarray(world_state[self.force_key])[self.side]
        self.contact_seen |= opposing_contacts(forces)
        opened = (
            abs(float(world_state[self.q_key]) - self.parameters["target_angle"])
            <= self.parameters["angle_tolerance"]
        )
        released = np.max(np.linalg.norm(forces, axis=-1)) < 0.1
        self.hold = self.hold + dt if opened and released and self.contact_seen else 0.0
        if self.hold + 1e-9 >= self.parameters["hold_time"]:
            return TaskStatus(Outcome("success", "lid_opened_and_released"))
        return TaskStatus()
