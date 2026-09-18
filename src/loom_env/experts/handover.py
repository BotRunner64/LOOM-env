"""Sequential dual-arm demonstration composed from shared feedback actions."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.scenes.workspace import transform, workspace

from .actions import ActionExpert, CloseGripper, Manipulator, Move, MoveHeld, Release


class HandoverExpert(ActionExpert):
    def __init__(self, collection, planners, world_state):
        self.giver, self.receiver = (
            collection.arm_roles[k] for k in ("giver", "receiver")
        )
        super().__init__(collection, planners[self.giver], world_state, side=self.giver)
        self.obj, self.asset = self.arm.obj, self.arm.asset
        if any(arm.asset != PANDA_ASSET for arm in collection.deployment.arms.values()):
            raise ValueError("Handover currently requires dual Panda")
        self.arms[self.receiver] = Manipulator(
            collection.deployment,
            collection.scene,
            self.receiver,
            self.obj,
            planners[self.receiver],
        )
        self.frame, _ = workspace(collection.scene)
        self.dt = collection.deployment.control_dt
        self.hold_time = max(0.6, collection.task.parameters["hold_time"] + self.dt)

    def reset(self, episode_input):
        self.arm = self.arms[self.giver]
        super().reset(episode_input)
        self._configure_grasps(self.world_state())

    def _configure_grasps(self, world):
        """One pair, centred around the USD-derived COM; no object annotations."""
        bounds = np.asarray(self.asset.bounds, dtype=float)
        size = bounds[1] - bounds[0]
        center = bounds.mean(axis=0)
        com = np.asarray(world[f"{self.obj}/center_of_mass_local"], dtype=float)
        if com.shape != (3,) or not np.isfinite(com).all():
            raise ValueError("Handover requires a finite USD-derived centre of mass")
        axis_index = int(np.argmax(size))
        self.axis_local = np.eye(3)[axis_index]
        rotation = Rotation.from_quat(world[f"{self.obj}/pose_world"][3:])
        axis = rotation.apply(self.axis_local)
        if abs(axis[2]) > 0.5:
            raise ValueError("Handover requires a roughly horizontal longest box axis")
        down = np.array([0.0, 0.0, -1.0])
        approach = down - np.dot(down, axis) * axis
        approach /= np.linalg.norm(approach)
        closing = np.cross(approach, axis)
        width = float(np.abs(rotation.inv().apply(closing)) @ size)
        profiles = [self.arms[side].profile for side in (self.giver, self.receiver)]
        # Clearance is shared by all objects; dimensions belong to the robot.
        separation = sum(p.handover_half_span for p in profiles) + 0.01
        end_margin = max(p.finger_half_span for p in profiles)
        for side in (self.giver, self.receiver):
            opening = self.collection.deployment.arms[side].gripper.command_limits[0][1]
            if width + 0.004 > opening:
                raise ValueError(
                    f"Handover object width {width:.3f} m exceeds {side} jaw opening"
                )
        lower = bounds[0, axis_index] + end_margin + separation / 2
        upper = bounds[1, axis_index] - end_margin - separation / 2
        if lower > upper:
            raise ValueError("Handover object is too short for two separated grippers")
        center[axis_index] = np.clip(com[axis_index], lower, upper)
        bases = self.collection.deployment.arms
        toward_giver = np.dot(
            axis,
            np.asarray(bases[self.giver].base_pose[:3])
            - np.asarray(bases[self.receiver].base_pose[:3]),
        )
        sign = 1 if toward_giver >= 0 else -1
        self.sites = {
            self.giver: center + sign * separation / 2 * self.axis_local,
            self.receiver: center - sign * separation / 2 * self.axis_local,
        }
        radius = np.linalg.norm(size / 2)
        receiver_offset = np.linalg.norm(
            self.sites[self.receiver] - bounds.mean(axis=0)
        )
        self.exchange_height = (
            radius
            + receiver_offset
            + np.linalg.norm(self.arms[self.receiver].profile.tcp_to_grasp)
            + self.collection.task.parameters["clearance"]
            + self.collection.task.parameters["transfer_distance"]
        )
        self.grasp_geometry = {
            "center_of_mass_local": com.tolist(),
            "long_axis_local": self.axis_local.tolist(),
            "sites_local": {side: point.tolist() for side, point in self.sites.items()},
            "separation_m": separation,
            "object_width_m": width,
        }

    def _tcp(self, world, side, clearance=0.0):
        obj = world[f"{self.obj}/pose_world"]
        point = transform(obj, [*self.sites[side], 0, 0, 0, 1])[:3]
        # Align fingers across the box; both tools approach from above.
        axis = Rotation.from_quat(obj[3:]).apply(self.axis_local)
        # The object's long axis is a line, not a directed vector. Flipping the
        # object for the opposite giver must not rotate the gripper by 180°.
        if axis[1] < 0:
            axis = -axis
        axis = axis / np.linalg.norm(axis)
        down = np.array([0.0, 0.0, -1.0])
        approach = down - np.dot(down, axis) * axis
        approach /= np.linalg.norm(approach)
        rotation = Rotation.from_matrix(
            np.column_stack((axis, np.cross(approach, axis), approach))
        )
        point[2] += clearance
        return np.r_[
            point - rotation.apply(self.arms[side].profile.tcp_to_grasp),
            rotation.as_quat(),
        ]

    def routine(self):
        giver, receiver = self.arms[self.giver], self.arms[self.receiver]
        giver.event("planning", "handover_grasp_geometry", self.grasp_geometry)
        home = giver.tcp.copy()
        samples = int(np.ceil(self.hold_time / self.dt))
        # Loaded Panda joints retain measured deflection under the tea pack.
        motion = {"tolerance": 0.02, "stable_samples": samples}
        for name, clearance in (("giver_approach", 0.12), ("giver_descend", 0)):
            yield Move(
                self._tcp(giver.world, giver.side, clearance),
                name=name,
                allow_object_contact=clearance == 0,
                **motion,
            )
        yield CloseGripper(stable_samples=samples, min_time=0, timeout=3)
        self.holding = (giver,)
        goal = giver.tcp.copy()
        goal[2] += 0.20
        yield MoveHeld(
            goal, name="giver_lift", attach=False, allow_object_contact=True, **motion
        )
        goal = giver.tcp.copy()
        target = transform(self.frame, [0, 0, self.exchange_height, 0, 0, 0, 1])[:3]
        goal[:3] += target - giver.world[f"{self.obj}/pose_world"][:3]
        yield MoveHeld(goal, name="giver_present", **motion)
        giver.planner.detach()
        self.arm = receiver
        for name, clearance in (("receiver_approach", 0.12), ("receiver_descend", 0)):
            yield Move(
                self._tcp(receiver.world, receiver.side, clearance),
                name=name,
                allow_object_contact=True,
                **motion,
            )
        yield CloseGripper(
            stable_samples=samples,
            min_time=0,
            timeout=3,
            contact=lambda arm: giver.held and arm.held,
        )
        exchange = receiver.world[f"{self.obj}/pose_world"][:3].copy()
        self.holding = (receiver,)
        self.arm = giver
        yield Release(
            stable_samples=samples,
            min_time=0,
            timeout=3,
            released=lambda arm: not arm.held and receiver.held,
        )
        goal = giver.tcp.copy()
        goal[2] += 0.15
        yield Move(goal, name="giver_retreat", allow_object_contact=True, **motion)
        yield Move(home, name="giver_clear", **motion)
        self.arm = receiver
        goal = receiver.tcp.copy()
        height = (
            exchange[2] + self.collection.task.parameters["transfer_distance"] + 0.04
        )
        goal[2] += max(0, height - receiver.world[f"{self.obj}/pose_world"][2])
        yield MoveHeld(goal, name="receiver_lift", **motion)
