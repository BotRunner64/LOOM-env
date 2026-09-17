"""Insertion judged in the fixed slot frame, independently of the expert."""

import math

from loom_env.assets.insertion import InsertionGeometry
from loom_env.specs.episode import Outcome, TaskStatus

from .object_task import ObjectTask


class InsertionTask(ObjectTask):
    def __init__(self, collection):
        super().__init__(
            collection,
            (
                "depth",
                "max_depth",
                "lift_clearance",
                "contact_force",
            ),
        )
        if set(collection.role_bindings) != {
            "target_object",
            "source_fixture",
            "target_fixture",
        }:
            raise ValueError("Insertion requires object, source and target fixtures")
        if set(collection.arm_roles) != {"manipulator"}:
            raise ValueError("Insertion requires one manipulator")
        self.geometry = InsertionGeometry(collection)
        if self.parameters["depth"] >= self.parameters["max_depth"]:
            raise ValueError("Insertion depth must be below maximum depth")

    def metrics(self, world):
        self._state(world)
        return self.geometry.metrics(world)

    def in_target(self, metrics):
        return self.geometry.in_target(metrics)

    def reset(self, initial_state):
        super().reset(initial_state)
        self.lifted = self.approached = self.inserted_held = False

    def update(self, world_state, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Task dt must be finite and positive")
        m = self.metrics(world_state)
        self.lifted |= (
            m["grasped"] and m["source_clearance"] >= self.parameters["lift_clearance"]
        )
        self.approached |= (
            self.lifted and m["grasped"] and self.in_target(m) and m["depth"] < -0.002
        )
        self.inserted_held |= (
            self.approached
            and m["grasped"]
            and self.in_target(m)
            and m["depth"] >= self.parameters["depth"]
        )
        points, _ = self._state(world_state)
        if self.in_target(m) and m["depth"] > self.parameters["max_depth"]:
            return TaskStatus(Outcome("task_failure", "object_below_socket_floor"))
        condition = (
            self.inserted_held
            and self.in_target(m)
            and self.parameters["depth"] <= m["depth"] <= self.parameters["max_depth"]
            and not m["any_grasped"]
            and m["finger_force"] <= self.parameters["contact_force"]
        )
        return self.finish(points, condition, dt, "object_inserted_and_released")
