"""A small feedback-driven expert; truth is explicitly injected at construction."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.embodiments.commands import initial_command
from loom_env.embodiments.manipulation import manipulation_profile
from loom_env.assets.catalog import asset_definition
from loom_env.scenes.workspace import transform, workspace
from loom_env.runtime.runner import SourceFailure
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Action, Event


class LiftExpert:
    STAGES = ("approach", "descend", "close", "lift", "wait")

    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.side = collection.arm_roles["manipulator"]
        arm = collection.deployment.arms[self.side]
        self.profile = manipulation_profile(arm)
        self.tool_rotation = Rotation.from_quat(arm.base_pose[3:]) * Rotation.from_quat(
            self.profile.grasp_rotation
        )
        self.closed, self.opened = arm.gripper.command_limits[0]
        self.other = "left" if self.side == "right" else "right"
        self.arm_slice = collection.deployment.action_slices[f"{self.side}/arm"]
        self.grip_slice = collection.deployment.action_slices[f"{self.side}/gripper"]
        self.obj = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.obj]["asset"])
        if self.asset.grasp is None:
            raise ValueError("Expert requires a reviewed grasp annotation")
        self.support, _ = workspace(collection.scene)
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

    def _tcp_goal(self, grasp_point):
        position = np.asarray(grasp_point) - self.tool_rotation.apply(
            self.profile.tcp_to_grasp
        )
        return np.r_[position, self.tool_rotation.as_quat()]

    def _goal(self, truth, observation):
        grasp = transform(
            truth[f"{self.obj}/pose_world"], [*self.asset.grasp, 0, 0, 0, 1]
        )[:3]
        if self.stage == "approach":
            grasp[2] += 0.12
        elif self.stage == "lift":
            grasp[2] = self.support[2] + 0.18 + self.asset.grasp[2]
        return self._tcp_goal(grasp)

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
                        truth[f"{self.obj}/pose_world"][2] < self.support[2] + 0.10
                        or not grip
                    ):
                        raise SourceFailure(
                            "Object was not physically lifted", kind="skill"
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
                self.command[self.grip_slice] = (
                    self.closed if self.stage == "close" else self.opened
                )
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
                self.stable = (
                    self.stable + 1
                    if np.max(np.abs(q - self.command[self.arm_slice])) < 0.01
                    else 0
                )
                if self.stable >= 3:
                    self._advance(events)
                    continue
                if (self.stage_steps - len(self.path)) * self.dt > 3.0:
                    raise SourceFailure(
                        f"{self.stage} did not reach its joint target", kind="skill"
                    )
            break
        if self.stage in {"lift", "transfer", "lower"} or (
            self.stage == "wait" and "release" not in self.STAGES
        ):
            self.lost_contact = 0 if grip else self.lost_contact + 1
            if self.lost_contact >= 5:
                raise SourceFailure(
                    "Object lost opposing finger contacts", kind="skill"
                )
        self.stage_steps += 1
        self.step += 1
        return Action(self.command.copy(), tuple(events))


class PickPlaceExpert(LiftExpert):
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

    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        self.receptacle = collection.role_bindings["container"]
        self.container = asset_definition(
            collection.scene.objects[self.receptacle]["asset"]
        )
        if self.container.interior is None:
            raise ValueError("Expert requires a reviewed container interior")

    def _goal(self, truth, observation):
        if self.stage in {"transfer", "lower"}:
            if self.stage == "lower":
                self.retreat_pose = observation.values[
                    f"robot/{self.side}/tcp_pose_world"
                ].copy()
            region = truth[f"{self.receptacle}/region_pose_world"]
            # Place above the opening; release after physical transfer and tracking.
            local = [
                0,
                0,
                self.container.interior[1][2] / 2 - self.asset.bounds[0][2] + 0.015,
                0,
                0,
                0,
                1,
            ]
            grasp = transform(region, local)[:3]
            grasp[2] += self.asset.grasp[2]
            if self.stage == "transfer":
                held_grasp = transform(
                    truth[f"{self.obj}/pose_world"], [*self.asset.grasp, 0, 0, 0, 1]
                )[:3]
                # Carry at the measured lifted height, or above the rim if higher.
                grasp[2] = max(grasp[2], held_grasp[2])
            return self._tcp_goal(grasp)
        if self.stage == "retreat":
            # Return to the measured pose before insertion, already reached in transfer.
            return self.retreat_pose.copy()
        return super()._goal(truth, observation)
