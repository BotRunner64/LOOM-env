"""Measured joint target, prior contact, and released gripper; no expert phases."""

import numpy as np

from loom_env.assets.catalog import asset_definition, is_articulated
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.specs.episode import Outcome, TaskStatus


class ArticulationTask:
    def __init__(self, collection):
        if set(collection.role_bindings) != {"target_object"} or set(
            collection.arm_roles
        ) != {"manipulator"}:
            raise ValueError("Articulation task needs one object and one manipulator")
        self.name = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.name]["asset"])
        if not is_articulated(self.asset):
            raise ValueError("Articulation task requires an articulated object")
        self.side = ("left", "right").index(collection.arm_roles["manipulator"])
        self.parameters = dict(collection.task.parameters)
        if set(self.parameters) != {
            "target_position",
            "position_tolerance",
            "hold_time",
            "direction",
            "contact_index",
        } or not all(np.isfinite(v) for v in self.parameters.values()):
            raise ValueError(
                "Articulation task needs finite joint target, tolerance and hold time"
            )
        if (
            self.parameters["position_tolerance"] <= 0
            or self.parameters["hold_time"] <= 0
        ):
            raise ValueError("Articulation task tolerances must be positive")
        self.direction = self.parameters["direction"]
        if self.direction not in (-1, 1):
            raise ValueError("Joint direction must be -1 or 1")
        contact_index = self.parameters["contact_index"]
        if int(contact_index) != contact_index or not 0 <= contact_index < len(
            self.asset.contact_poses
        ):
            raise ValueError("Invalid articulated contact index")
        self.q_key = f"{self.name}/joints/{self.asset.joint}/position"
        self.force_key = (
            f"{self.name}/links/{self.asset.moving_body}/finger_contact_forces_world"
        )

    def reset(self, initial_state):
        self.initial_q = float(initial_state[self.q_key])
        if (
            abs(self.initial_q - self.parameters["target_position"])
            <= self.parameters["position_tolerance"]
        ):
            raise ValueError("Articulation task must begin outside the target interval")
        if self.direction * (self.parameters["target_position"] - self.initial_q) <= 0:
            raise ValueError("Initial position and target contradict the task direction")
        self.contact_seen = False
        self.hold = 0.0

    def update(self, world_state, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be positive")
        forces = np.asarray(world_state[self.force_key])[self.side]
        self.contact_seen |= opposing_contacts(forces)
        at_target = (
            abs(float(world_state[self.q_key]) - self.parameters["target_position"])
            <= self.parameters["position_tolerance"]
        )
        released = np.max(np.linalg.norm(forces, axis=-1)) < 0.1
        self.hold = (
            self.hold + dt if at_target and released and self.contact_seen else 0.0
        )
        if self.hold + 1e-9 >= self.parameters["hold_time"]:
            return TaskStatus(Outcome("success", "joint_at_target_and_released"))
        return TaskStatus()
