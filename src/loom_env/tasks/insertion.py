"""Coin insertion judged in the fixed slot frame, independently of the expert."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Outcome, TaskStatus

from .object_task import ObjectTask


class CoinInsertionTask(ObjectTask):
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
            raise ValueError(
                "Coin insertion requires object, source and target fixtures"
            )
        if set(collection.arm_roles) != {"manipulator"}:
            raise ValueError("Coin insertion requires one manipulator")
        self.side = ARMS.index(collection.arm_roles["manipulator"])
        self.source = collection.role_bindings["source_fixture"]
        self.target = collection.role_bindings["target_fixture"]
        if self.source == self.target:
            raise ValueError("Insertion requires distinct source and target slots")
        if collection.scene.objects[self.object_name]["asset"] != "robodojo:coin":
            raise ValueError("Insertion currently supports the reviewed coin")
        for name in (self.source, self.target):
            obj = collection.scene.objects[name]
            if obj["asset"] != "robodojo:coin_slot" or not obj["static"]:
                raise ValueError("Insertion requires the reviewed fixed coin slots")
        self.fixture = asset_definition("robodojo:coin_slot")
        bounds = np.asarray(self.fixture.bounds)
        self.slot_center = bounds.mean(0)[:2]
        self.slot_half_size = (bounds[1, :2] - bounds[0, :2]) / 2
        self.slot_top = bounds[1, 2]
        coin_bounds = np.asarray(self.asset.bounds)
        self.center = coin_bounds.mean(0)
        self.radius = (coin_bounds[1, 0] - coin_bounds[0, 0]) / 2
        self.half_thickness = (coin_bounds[1, 2] - coin_bounds[0, 2]) / 2
        if self.parameters["depth"] >= self.parameters["max_depth"]:
            raise ValueError("Insertion depth must be below maximum depth")

    def metrics(self, world):
        self._state(world)  # Validate the same pose/grasp evidence as other tasks.
        pose = world[f"{self.object_name}/pose_world"]
        rotation = Rotation.from_quat(pose[3:].copy())
        center = rotation.apply(self.center) + pose[:3]
        normal = rotation.apply([0, 0, 1])
        fixture = np.asarray(world[f"{self.target}/pose_world"])
        frame = Rotation.from_quat(fixture[3:].copy())
        local = frame.inv().apply(center - fixture[:3])
        n = frame.inv().apply(normal)
        # Exact support function for the reviewed circular cylinder envelope.
        vertical_radius = self.radius * math.sqrt(
            max(0.0, 1 - n[2] ** 2)
        ) + self.half_thickness * abs(n[2])
        source = np.asarray(world[f"{self.source}/pose_world"])
        source_r = Rotation.from_quat(source[3:].copy())
        source_center = source_r.inv().apply(center - source[:3])
        source_n = source_r.inv().apply(normal)
        source_radius = self.radius * math.sqrt(
            max(0.0, 1 - source_n[2] ** 2)
        ) + self.half_thickness * abs(source_n[2])
        forces = np.asarray(world[f"{self.object_name}/finger_contact_forces_world"])
        if forces.shape != (2, 2, 3) or not np.isfinite(forces).all():
            raise ValueError("Insertion requires finite per-finger contacts")
        grasped = world[f"{self.object_name}/grasped_by"]
        return {
            "depth": float(self.slot_top - (local[2] - vertical_radius)),
            "along_error": float(abs(local[0] - self.slot_center[0])),
            "across_error": float(abs(local[1] - self.slot_center[1])),
            "angle": float(np.arccos(np.clip(abs(n[1]), 0, 1))),
            "source_clearance": float(source_center[2] - source_radius - self.slot_top),
            "grasped": bool(grasped[self.side]),
            "any_grasped": bool(grasped.any()),
            "finger_force": float(np.linalg.norm(forces, axis=-1).max()),
        }

    def in_target(self, m):
        """Locate the coin over this fixture, without requiring precise centering.

        Combined with depth and release, the physical slot contains the coin;
        this footprint check excludes placements beside or in the other slot.
        """
        return bool(
            m["along_error"] <= self.slot_half_size[0]
            and m["across_error"] <= self.slot_half_size[1]
        )

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
            return TaskStatus(Outcome("task_failure", "coin_below_slot_floor"))
        condition = (
            self.inserted_held
            and self.in_target(m)
            and self.parameters["depth"] <= m["depth"] <= self.parameters["max_depth"]
            and not m["any_grasped"]
            and m["finger_force"] <= self.parameters["contact_force"]
        )
        return self.finish(points, condition, dt, "coin_inserted_and_released")
