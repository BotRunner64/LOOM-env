"""Insertion workflow and its mating-specific feedback motion."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.insertion import InsertionGeometry
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import workspace
from loom_env.specs.config import ARMS

from .actions import ActionExpert, FeedbackAction, Grasp, Move, MoveHeld, Release
from .pick_place import initial_contacts, lift_from_support


class Insert(FeedbackAction):
    """Align and insert an already held part; does not grasp, carry, or release."""

    def __init__(self, geometry, depth):
        super().__init__("insertion", timeout=36)
        self.geometry, self.depth = geometry, depth
        self.desired_rotation = self.insert_rotation = None
        self.best_depth, self.stalled = -np.inf, 0
        self.align, self.stable, self.phase_steps = True, 0, 0

    def start(self, arm):
        if not arm.held:
            raise SourceFailure("Insertion requires an existing grasp", kind="skill")
        if not arm.planner.attached:
            arm.planner.attach(arm.observation, arm.world, max_cell_size=0.01)

    def goal(self, observation, world, depth, *, hold_orientation=False):
        part = world[f"{self.geometry.obj}/pose_world"]
        part_r = Rotation.from_quat(part[3:].copy())
        tcp = observation.values[f"robot/{ARMS[self.geometry.side]}/tcp_pose_world"]
        tcp_r = Rotation.from_quat(tcp[3:].copy())
        fixture = self.geometry.frame(world, self.geometry.target)
        fixture_r = Rotation.from_quat(fixture[3:].copy())
        if self.desired_rotation is None:
            normal = part_r.apply(self.geometry.body.axis)
            desired_normal = fixture_r.apply([0, 1, 0])
            if normal @ desired_normal < 0:
                desired_normal *= -1
            correction, _ = Rotation.align_vectors([desired_normal], [normal])
            self.desired_rotation = correction * part_r
        center = (
            fixture_r.apply([0, 0, self.geometry.body.radius - depth]) + fixture[:3]
        )
        desired_part_position = center - self.desired_rotation.apply(
            self.geometry.body.center
        )
        # Correct position from the measured grasp, but hold the TCP orientation
        # during insertion: rotating against the slot amplified part tilt.
        part_in_tcp = tcp_r.inv().apply(part[:3] - tcp[:3])
        if hold_orientation:
            if self.insert_rotation is None:
                self.insert_rotation = tcp_r
            goal_r = self.insert_rotation
        else:
            relative_r = tcp_r.inv() * part_r
            goal_r = self.desired_rotation * relative_r.inv()
        return np.r_[
            desired_part_position - goal_r.apply(part_in_tcp), goal_r.as_quat()
        ]

    def advance(self, arm):
        arm.command[arm.grip_slice] = arm.closed
        arm.monitor_grasp()
        m = self.geometry.metrics(arm.world)
        depth = -0.004 if self.align else self.depth + 0.002
        if self.align:
            ready = (
                m["along_error"] <= 0.0015
                and m["across_error"] <= 0.0004
                and m["angle"] <= np.deg2rad(2)
                and abs(m["depth"] - depth) < 0.0006
            )
        else:
            ready = self.geometry.in_target(m) and m["depth"] >= self.depth
        self.stable = self.stable + 1 if ready else 0
        if self.stable >= 3:
            arm.event(
                "planning", "insertion_alignment" if self.align else "inserted", m
            )
            if not self.align:
                return True
            self.align, self.stable, self.phase_steps = False, 0, 0
            return False
        goal = self.goal(
            arm.observation, arm.world, depth, hold_orientation=not self.align
        )
        tcp = arm.tcp
        delta = goal[:3] - tcp[:3]
        limit = (0.06 if self.align else 0.02) * arm.dt
        goal[:3] = tcp[:3] + delta * min(1, limit / max(np.linalg.norm(delta), 1e-12))
        delta_r = (
            Rotation.from_quat(goal[3:]) * Rotation.from_quat(tcp[3:].copy()).inv()
        ).as_rotvec()
        delta_r *= min(1, 0.025 / max(np.linalg.norm(delta_r), 1e-12))
        goal[3:] = (
            Rotation.from_rotvec(delta_r) * Rotation.from_quat(tcp[3:].copy())
        ).as_quat()
        arm.command[arm.arm_slice] = arm.planner.cartesian_step(
            arm.observation, arm.world, goal, contact_objects=(self.geometry.target,)
        )
        if not self.align:
            if m["depth"] > self.best_depth + 0.0002:
                self.best_depth, self.stalled = m["depth"], 0
            else:
                self.stalled += 1
            if self.stalled * arm.dt > 2:
                raise SourceFailure(
                    f"Object insertion stalled at {m['depth']:.6f} m", kind="skill"
                )
        if self.phase_steps * arm.dt > 18:
            raise SourceFailure("Insertion motion phase timed out", kind="skill")
        self.phase_steps += 1
        return False


class InsertionExpert(ActionExpert):
    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        self.geometry = InsertionGeometry(collection)

    def routine(self):
        arm = self.arm
        parameters = self.collection.task.parameters
        support, _ = workspace(self.collection.scene)
        yield Grasp(
            contact_objects=initial_contacts(arm),
            stable_samples=int(np.ceil(0.6 / arm.dt)),
            min_time=0,
            timeout=3,
        )
        if (
            self.geometry.metrics(arm.world)["source_clearance"]
            < parameters["lift_clearance"]
        ):
            yield lift_from_support(arm, support)
        m = self.geometry.metrics(arm.world)
        if not m["grasped"] or m["source_clearance"] < parameters["lift_clearance"]:
            raise SourceFailure("Object was not physically extracted", kind="skill")
        insertion = Insert(self.geometry, parameters["depth"])
        yield MoveHeld(
            insertion.goal(arm.observation, arm.world, -0.035),
            name="transfer",
            time_scale=2,
            max_cell_size=0.01,
        )
        yield insertion
        yield Release(
            stable_samples=int(np.ceil(0.1 / arm.dt)),
            min_time=0,
            timeout=3,
            all_arms=True,
        )
        retreat = arm.tcp.copy()
        retreat[2] += 0.08
        yield Move(
            retreat,
            name="retreat",
            allow_object_contact=True,
            contact_objects=(self.geometry.target,),
        )
