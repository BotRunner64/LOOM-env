"""Grasp, extract, align mating features, insert, and release."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.insertion import InsertionGeometry
from loom_env.runtime.runner import SourceFailure
from loom_env.specs.episode import Action, Event

from .pick_place import GraspTransport


class InsertionExpert(GraspTransport):
    STAGES = (
        "approach",
        "descend",
        "close",
        "lift",
        "transfer",
        "align",
        "insert",
        "release",
        "retreat",
        "wait",
    )

    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        self.geometry = InsertionGeometry(collection)
        self.parameters = collection.task.parameters

    def reset(self, episode_input):
        super().reset(episode_input)
        self.desired_rotation = None
        self.insert_rotation = None
        self.best_depth = -np.inf
        self.stalled = 0

    def _insertion_goal(self, observation, world, depth):
        part = world[f"{self.obj}/pose_world"]
        part_r = Rotation.from_quat(part[3:].copy())
        tcp = observation.values[f"robot/{self.side}/tcp_pose_world"]
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
        if self.stage == "insert":
            if self.insert_rotation is None:
                self.insert_rotation = tcp_r
            goal_r = self.insert_rotation
        else:
            relative_r = tcp_r.inv() * part_r
            goal_r = self.desired_rotation * relative_r.inv()
        return np.r_[
            desired_part_position - goal_r.apply(part_in_tcp), goal_r.as_quat()
        ]

    def _motion_goal(self, observation, world):
        if self.stage in {"approach", "descend", "lift"}:
            return super()._goal(world, observation)
        if self.stage == "transfer":
            return self._insertion_goal(observation, world, -0.035)
        goal = observation.values[f"robot/{self.side}/tcp_pose_world"].copy()
        goal[2] += 0.08
        return goal

    def act(self, observation):
        if self.stage == "lift":
            return self._extract_step(observation)
        world, events = self.world_state(), []
        m = self.geometry.metrics(world)
        other = "left" if self.side == "right" else "right"
        other_slice = self.collection.deployment.action_slices[f"{other}/arm"]
        if (
            np.max(
                np.abs(
                    observation.values[f"robot/{other}/joint_position"]
                    - self.command[other_slice]
                )
            )
            > 0.03
        ):
            raise SourceFailure("Holding arm moved outside tolerance", kind="skill")
        if self.stage in {"lift", "transfer", "align", "insert"}:
            self.lost_contact = 0 if m["grasped"] else self.lost_contact + 1
            if self.lost_contact >= 5:
                raise SourceFailure(
                    "Object lost opposing finger contacts", kind="skill"
                )
        if not self.started:
            self.started = True
            print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
            events.append(
                Event(
                    "skill", self.stage, self.step, {"phase": "begin", "arm": self.side}
                )
            )
            if self.stage == "transfer":
                if (
                    not m["grasped"]
                    or m["source_clearance"] < self.parameters["lift_clearance"]
                ):
                    raise SourceFailure(
                        "Object was not physically extracted", kind="skill"
                    )
                self.planner.attach(observation, world, max_cell_size=0.01)
            if self.stage == "release":
                self.planner.detach()
            if self.stage in {"approach", "descend", "lift", "transfer", "retreat"}:
                contact = (
                    (self.geometry.source,)
                    if self.stage in {"descend", "lift"}
                    else (self.geometry.target,)
                    if self.stage == "retreat"
                    else ()
                )
                self.path, detail = self.planner.plan(
                    observation,
                    world,
                    self._motion_goal(observation, world),
                    allow_object_contact=self.stage in {"descend", "lift", "retreat"},
                    contact_objects=contact,
                )
                if self.stage == "transfer":
                    # Execute the collision-checked path with twofold time
                    # scaling to avoid accelerating the thin part out of the jaws.
                    knots = np.vstack(
                        [
                            observation.values[f"robot/{self.side}/joint_position"],
                            self.path,
                        ]
                    )
                    time = np.arange(1, (len(knots) - 1) * 2 + 1) / 2
                    self.path = np.column_stack(
                        [
                            np.interp(time, np.arange(len(knots)), knots[:, j])
                            for j in range(knots.shape[1])
                        ]
                    )
                    detail["execution_time_scale"] = 2
                events.append(Event("planning", self.stage, self.step, detail))
        if self.stage in {"align", "insert"}:
            align = self.stage == "align"
            depth = -0.004 if align else self.parameters["depth"] + 0.002
            if align:
                # Entry precision guides the motion, not task acceptance.
                ready = (
                    m["along_error"] <= 0.0015
                    and m["across_error"] <= 0.0004
                    and m["angle"] <= np.deg2rad(2)
                    and abs(m["depth"] - depth) < 0.0006
                )
            else:
                ready = (
                    self.geometry.in_target(m)
                    and m["depth"] >= self.parameters["depth"]
                )
            self.stable = self.stable + 1 if ready else 0
            if self.stable >= 3:
                events.append(Event("planning", "insertion_alignment", self.step, m))
                self._advance(events)
            else:
                goal = self._insertion_goal(observation, world, depth)
                tcp = observation.values[f"robot/{self.side}/tcp_pose_world"]
                delta = goal[:3] - tcp[:3]
                # 20 mm/s insertion, 60 mm/s pre-insertion alignment.
                limit = (0.06 if align else 0.02) * self.dt
                goal[:3] = tcp[:3] + delta * min(
                    1, limit / max(np.linalg.norm(delta), 1e-12)
                )
                delta_r = (
                    Rotation.from_quat(goal[3:])
                    * Rotation.from_quat(tcp[3:].copy()).inv()
                ).as_rotvec()
                delta_r *= min(1, 0.025 / max(np.linalg.norm(delta_r), 1e-12))
                goal[3:] = (
                    Rotation.from_rotvec(delta_r) * Rotation.from_quat(tcp[3:].copy())
                ).as_quat()
                self.command[self.arm_slice] = self.planner.cartesian_step(
                    observation,
                    world,
                    goal,
                    contact_objects=(self.geometry.target,),
                )
                if not align:
                    if m["depth"] > self.best_depth + 0.0002:
                        self.best_depth, self.stalled = m["depth"], 0
                    else:
                        self.stalled += 1
                    if self.stalled * self.dt > 2:
                        raise SourceFailure(
                            f"Object insertion stalled at {m['depth']:.6f} m",
                            kind="skill",
                        )
        elif self.stage in {"close", "release"}:
            closing = self.stage == "close"
            self.command[self.grip_slice] = self.closed if closing else self.opened
            ready = m["grasped"] if closing else not m["any_grasped"]
            self.stable = self.stable + 1 if ready else 0
            if self.stable * self.dt >= (0.6 if closing else 0.1):
                self._advance(events)
            elif self.stage_steps * self.dt > 3:
                raise SourceFailure(
                    f"Object {self.stage} feedback timed out", kind="skill"
                )
        elif self.stage != "wait":
            if self.path_index < len(self.path):
                self.command[self.arm_slice] = self.path[self.path_index]
                self.path_index += 1
            else:
                error = np.max(
                    np.abs(
                        observation.values[f"robot/{self.side}/joint_position"]
                        - self.command[self.arm_slice]
                    )
                )
                self.stable = self.stable + 1 if error < 0.01 else 0
                if self.stable >= 3:
                    self._advance(events)
        if self.stage != "wait" and self.stage_steps * self.dt > 18:
            raise SourceFailure(f"Object stage {self.stage} timed out", kind="skill")
        self.stage_steps += 1
        self.step += 1
        return Action(self.command.copy(), tuple(events))
