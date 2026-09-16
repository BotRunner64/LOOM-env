"""Closed-finger pushing, with measured object and robot feedback."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.commands import initial_command
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import corners
from loom_env.specs.episode import Action, Event
from loom_env.tasks import create_task


class PushExpert:
    """First reviewed pusher is Panda; other grippers need contact geometry review."""

    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.side = collection.arm_roles["manipulator"]
        self.other = "left" if self.side == "right" else "right"
        self.arm = collection.deployment.arms[self.side]
        if self.arm.asset != PANDA_ASSET:
            raise ValueError("Pushing currently requires the reviewed Panda fingers")
        self.arm_slice = collection.deployment.action_slices[f"{self.side}/arm"]
        self.grip_slice = collection.deployment.action_slices[f"{self.side}/gripper"]
        self.obj = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.obj]["asset"])
        if self.asset.push_height is None:
            raise ValueError("Push object needs a reviewed Panda contact height")
        self.task = create_task(collection)
        self.dt = collection.deployment.control_dt

    def reset(self, episode_input):
        self.command = initial_command(self.collection.deployment)
        self.command[self.grip_slice] = self.arm.gripper.command_limits[0][0]
        self.stage, self.step, self.stage_steps = "close", 0, 0
        self.path = None
        self.path_index = self.stable = 0
        self.planner.detach()
        world = self.world_state()
        obj = np.asarray(world[f"{self.obj}/pose_world"])
        self.direction = self.task.target_position_world[:2] - obj[:2]
        self.distance = np.linalg.norm(self.direction)
        self.direction /= self.distance
        # A single straight push: face the direction of travel, irrespective of
        # the object's initial or final yaw.
        yaw = np.arctan2(self.direction[1], self.direction[0])
        self.rotation = Rotation.from_euler("z", yaw) * Rotation.from_quat([1, 0, 0, 0])
        points = Rotation.from_quat(obj[3:]).apply(corners(self.asset))
        extent = -(points[:, :2] @ self.direction).min()
        # Closed Panda finger leading surface is roughly 1 cm ahead of the
        # fingertip centre. Start a further 2 cm clear of the object.
        self.push_point = obj[:3].copy()
        self.push_point[:2] -= self.direction * (extent + 0.01 + 0.02)
        self.push_point[2] = self.task.support[2] + self.asset.push_height
        self.tcp_goal = self._tcp(self.push_point)
        self.travel = 0.0
        self.began = False

    def _tcp(self, point):
        return np.r_[
            point - self.rotation.apply(self.planner.profile.tcp_to_grasp),
            self.rotation.as_quat(),
        ]

    def _change(self, stage, events):
        events.append(
            Event("skill", self.stage, self.step, {"phase": "end", "arm": self.side})
        )
        self.stage, self.stage_steps, self.stable = stage, 0, 0
        self.path, self.path_index, self.began = None, 0, False

    def act(self, observation):
        truth = self.world_state()
        events = []
        other_q = observation.values[f"robot/{self.other}/joint_position"]
        if (
            np.max(
                np.abs(
                    other_q
                    - self.collection.deployment.arms[self.other].initial_positions
                )
            )
            > 0.03
        ):
            raise SourceFailure("Holding arm moved outside tolerance", kind="skill")
        if not self.began:
            print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
            events.append(
                Event(
                    "skill", self.stage, self.step, {"phase": "begin", "arm": self.side}
                )
            )
            self.began = True
        if self.stage == "close":
            opening = observation.values[f"robot/{self.side}/gripper_position"]
            if np.max(np.abs(opening)) < 0.001 and self.stage_steps * self.dt >= 0.3:
                self._change("approach", events)
        elif self.stage in {"approach", "descend"}:
            if self.path is None:
                goal = self.tcp_goal.copy()
                if self.stage == "approach":
                    goal[2] += 0.10
                self.path, detail = self.planner.plan(observation, truth, goal)
                events.append(Event("planning", self.stage, self.step, detail))
            if self.path_index < len(self.path):
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
                    next_stage = "descend" if self.stage == "approach" else "push"
                    if next_stage == "push":
                        # Track a Cartesian reference continuously from the
                        # measured reached pose, never from an assumed IK pose.
                        self.tcp_goal = observation.values[
                            f"robot/{self.side}/tcp_pose_world"
                        ].copy()
                    self._change(next_stage, events)
        elif self.stage == "push":
            error = (
                self.task.target_position_world[:2]
                - truth[f"{self.obj}/pose_world"][:2]
            )
            remaining = float(error @ self.direction)
            if remaining < 0.003:
                if np.linalg.norm(error) > self.task.parameters["position_tolerance"]:
                    raise SourceFailure(
                        "Push missed the target laterally", kind="skill"
                    )
                self.retreat_start = self.tcp_goal.copy()
                self._change("retreat", events)
            else:
                distance = min(0.08 * self.dt, remaining)
                # Correct contact-induced sideways drift while keeping the flat
                # pusher orientation fixed. Limit lateral speed to 5 mm/s.
                lateral = error - remaining * self.direction
                correction = 0.5 * lateral
                correction *= min(1.0, 0.005 / max(np.linalg.norm(correction), 1e-9))
                self.tcp_goal[:2] += distance * self.direction + self.dt * correction
                self.travel += distance
                if self.travel > self.distance + 0.07:
                    raise SourceFailure(
                        "Pusher travelled past the object without reaching the goal",
                        kind="skill",
                    )
                self.command[self.arm_slice] = self.planner.cartesian_step(
                    observation, truth, self.tcp_goal
                )
        elif self.stage == "retreat":
            retreat = min(0.025, (self.stage_steps + 1) * 0.025 * self.dt)
            self.tcp_goal = self.retreat_start.copy()
            self.tcp_goal[:2] -= self.direction * retreat
            self.command[self.arm_slice] = self.planner.cartesian_step(
                observation, truth, self.tcp_goal
            )
            if self.stage_steps * self.dt > 1.3:
                self._change("wait", events)
        if self.stage_steps * self.dt > 18 and self.stage != "wait":
            raise SourceFailure(f"Push stage {self.stage} timed out", kind="skill")
        self.step += 1
        self.stage_steps += 1
        return Action(self.command.copy(), tuple(events))
