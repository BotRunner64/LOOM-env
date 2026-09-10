"""A small feedback-driven expert; truth is explicitly injected at construction."""

import numpy as np

from loom_env.environments.tabletop import initial_command
from loom_env.runtime.runner import SourceFailure
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Action, Event


class PickPlaceExpert:
    STAGES = (
        "approach",
        "descend",
        "close",
        "lift",
        "transfer",
        "lower",
        "release",
        "retreat",
        "wait",
    )
    # Official Panda hand frame -> grasp center along the local +Z axis.
    HAND_TO_GRASP = 0.1034

    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.side = collection.arm_roles["manipulator"]
        self.other = "left" if self.side == "right" else "right"
        self.arm_slice = collection.deployment.action_slices[f"{self.side}/arm"]
        self.grip_slice = collection.deployment.action_slices[f"{self.side}/gripper"]
        self.obj = collection.role_bindings["target_object"]
        self.receptacle = collection.role_bindings["container"]
        self.dt = collection.deployment.control_dt

    def reset(self, episode_input):
        self.command = initial_command(self.collection.deployment)
        self.stage_index = self.step = self.stage_steps = self.path_index = (
            self.stable
        ) = self.lost_contact = 0
        self.path = None
        self.started = False
        self.planner.detach()

    @property
    def stage(self):
        return self.STAGES[self.stage_index]

    def _goal(self, truth, observation):
        cube = truth[f"{self.obj}/pose_world"][:3]
        region = truth[f"{self.receptacle}/region_pose_world"][:3]
        hand = cube + [0, 0, self.HAND_TO_GRASP]
        if self.stage == "approach":
            hand[2] += 0.12
        elif self.stage == "lift":
            hand[2] = (
                self.collection.scene.parameters["table_height"]
                + 0.22
                + self.HAND_TO_GRASP
            )
        elif self.stage in {"transfer", "lower"}:
            hand = region.copy()
            hand[2] += (
                self.collection.scene.objects[self.receptacle]["size"][2] / 2
                + self.collection.scene.objects[self.obj]["size"][2] / 2
                + self.HAND_TO_GRASP
            )
            hand[2] += 0.10 if self.stage == "transfer" else 0.025
        elif self.stage == "retreat":
            hand = observation.values[f"robot/{self.side}/tcp_pose_world"][:3].copy()
            hand[2] += 0.12
        return np.r_[hand, [1.0, 0.0, 0.0, 0.0]]  # XYZW, palm points down

    def _advance(self, events):
        events.append(
            Event(
                "skill",
                self.stage,
                self.step,
                {"success": True, "phase": "end", "arm": self.side},
            )
        )
        self.stage_index += 1
        self.stage_steps = self.path_index = self.stable = 0
        self.path = None
        self.started = False

    def act(self, observation):
        truth = self.world_state()
        events = []
        grip = bool(truth[f"{self.obj}/grasped_by"][ARMS.index(self.side)])
        if (
            np.max(
                np.abs(
                    observation.values[f"robot/{self.other}/joint_position"]
                    - self.command[
                        self.collection.deployment.action_slices[f"{self.other}/arm"]
                    ]
                )
            )
            > 0.03
        ):
            raise SourceFailure("Holding arm moved outside tolerance", kind="skill")
        while True:
            if not self.started:
                events.append(
                    Event(
                        "skill",
                        self.stage,
                        self.step,
                        {"phase": "begin", "arm": self.side},
                    )
                )
                print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
                self.started = True
                if self.stage == "transfer":
                    if (
                        truth[f"{self.obj}/pose_world"][2]
                        < self.collection.scene.parameters["table_height"] + 0.12
                        or not grip
                    ):
                        raise SourceFailure(
                            "Cube was not physically lifted", kind="skill"
                        )
                    self.planner.attach(observation, truth)
                if self.stage == "release":
                    self.planner.detach()
                if self.stage not in {"close", "release", "wait"}:
                    self.path, detail = self.planner.plan(
                        observation,
                        truth,
                        self._goal(truth, observation),
                        allow_object_contact=self.stage in {"descend", "lift"},
                    )
                    events.append(Event("planning", self.stage, self.step, detail))
            if self.stage == "wait":
                break
            if self.stage in {"close", "release"}:
                self.command[self.grip_slice] = 0.0 if self.stage == "close" else 0.08
                ready = grip if self.stage == "close" else not grip
                self.stable = self.stable + 1 if ready else 0
                if self.stable >= 5 and self.stage_steps * self.dt >= 0.6:
                    self._advance(events)
                    continue
                if self.stage_steps * self.dt > 2.0:
                    raise SourceFailure(
                        f"Measured {self.stage} feedback timed out", kind="skill"
                    )
            elif self.path_index < len(self.path):
                self.command[self.arm_slice] = self.path[self.path_index]
                self.path_index += 1
            else:
                q = observation.values[f"robot/{self.side}/joint_position"]
                dq = observation.values[f"robot/{self.side}/joint_velocity"]
                self.stable = (
                    self.stable + 1
                    if np.max(np.abs(q - self.command[self.arm_slice])) < 0.01
                    and np.max(np.abs(dq)) < 0.05
                    else 0
                )
                if self.stable >= 3:
                    self._advance(events)
                    continue
                if (self.stage_steps - len(self.path)) * self.dt > 3.0:
                    raise SourceFailure(
                        f"{self.stage} tracking did not settle", kind="skill"
                    )
            break
        if self.stage in {"lift", "transfer", "lower"}:
            self.lost_contact = 0 if grip else self.lost_contact + 1
            if self.lost_contact >= 5:
                raise SourceFailure("Cube lost opposing finger contacts", kind="skill")
        self.stage_steps += 1
        self.step += 1
        return Action(self.command.copy(), tuple(events))
