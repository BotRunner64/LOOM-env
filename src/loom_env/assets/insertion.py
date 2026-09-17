"""Measured mating geometry shared by insertion control and task evaluation.

Currently supports a circular cylinder and a slot. Physics and containment are
still provided by the USD collisions; the footprint only identifies the fixture.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.scenes.workspace import transform
from loom_env.specs.config import ARMS


class InsertionGeometry:
    def __init__(self, collection):
        self.obj = collection.role_bindings["target_object"]
        self.source = collection.role_bindings["source_fixture"]
        self.target = collection.role_bindings["target_fixture"]
        self.side = ARMS.index(collection.arm_roles["manipulator"])
        self.body = asset_definition(
            collection.scene.objects[self.obj]["asset"]
        ).insertion_body
        if self.body is None:
            raise ValueError("Insertion requires a reviewed circular-cylinder feature")
        self.sockets = {}
        for name in (self.source, self.target):
            obj = collection.scene.objects[name]
            socket = asset_definition(obj["asset"]).insertion_socket
            if socket is None or not obj["static"]:
                raise ValueError("Insertion requires reviewed fixed socket features")
            self.sockets[name] = socket
        if self.source == self.target:
            raise ValueError("Insertion requires distinct source and target slots")
        if collection.scene.objects[self.obj].get("initial_fixture") != self.source:
            raise ValueError("Insertion source must match the object's initial fixture")

    def frame(self, world, name):
        return transform(world[f"{name}/pose_world"], self.sockets[name].pose)

    def metrics(self, world):
        pose = world[f"{self.obj}/pose_world"]
        rotation = Rotation.from_quat(pose[3:].copy())
        center = rotation.apply(self.body.center) + pose[:3]
        normal = rotation.apply(self.body.axis)
        values = {}
        for name in (self.source, self.target):
            frame = self.frame(world, name)
            inverse = Rotation.from_quat(frame[3:].copy()).inv()
            local = inverse.apply(center - frame[:3])
            n = inverse.apply(normal)
            support = self.body.radius * math.sqrt(
                max(0.0, 1 - n[2] ** 2)
            ) + self.body.half_thickness * abs(n[2])
            values[name] = (local, n, support)
        local, normal, support = values[self.target]
        source, _, source_support = values[self.source]
        forces = np.asarray(world[f"{self.obj}/finger_contact_forces_world"])
        if forces.shape != (2, 2, 3) or not np.isfinite(forces).all():
            raise ValueError("Insertion requires finite per-finger contacts")
        grasped = np.asarray(world[f"{self.obj}/grasped_by"])
        return {
            "depth": float(support - local[2]),
            "along_error": float(abs(local[0])),
            "across_error": float(abs(local[1])),
            "angle": float(np.arccos(np.clip(abs(normal[1]), 0, 1))),
            "source_clearance": float(source[2] - source_support),
            "grasped": bool(grasped[self.side]),
            "any_grasped": bool(grasped.any()),
            "finger_force": float(np.linalg.norm(forces, axis=-1).max()),
        }

    def in_target(self, metrics):
        footprint = self.sockets[self.target].footprint
        return bool(
            metrics["along_error"] <= footprint[0]
            and metrics["across_error"] <= footprint[1]
        )
