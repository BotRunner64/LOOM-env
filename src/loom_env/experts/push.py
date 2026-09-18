"""Closed-finger push workflow and reusable measured contact translation."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import corners
from loom_env.tasks import create_task

from .actions import ActionExpert, FeedbackAction, Move


class CloseEmpty(FeedbackAction):
    """Close a pusher, requiring measured finger position rather than a grasp."""

    def __init__(self):
        super().__init__("close")

    def advance(self, arm):
        arm.command[arm.grip_slice] = arm.closed
        opening = arm.observation.values[f"robot/{arm.side}/gripper_position"]
        return np.max(np.abs(opening)) < 0.001 and self.steps * arm.dt >= 0.3


class PushContact(FeedbackAction):
    """Translate contact until the measured object reaches its destination."""

    def __init__(
        self,
        obj,
        target,
        direction,
        distance,
        reached,
        *,
        speed=0.08,
        margin=0.07,
        lateral_speed=0.005,
        contact_objects=(),
        held=False,
    ):
        super().__init__("push_contact")
        self.obj, self.target = obj, np.asarray(target)
        self.direction = np.asarray(direction)
        self.max_travel, self.reached = distance + margin, reached
        self.speed, self.lateral_speed = speed, lateral_speed
        self.contacts, self.held = tuple(contact_objects), held
        self.travel = 0.0

    def start(self, arm):
        if self.held and not arm.held:
            raise SourceFailure("Contact motion requires the held tool", kind="skill")
        self.goal = arm.tcp.copy()

    def advance(self, arm):
        if self.held:
            arm.command[arm.grip_slice] = arm.closed
            arm.monitor_grasp()
        error = self.target[:2] - arm.world[f"{self.obj}/pose_world"][:2]
        remaining = float(error @ self.direction)
        if remaining <= 0.003:
            if not self.reached(arm.world):
                raise SourceFailure(
                    "Contact motion missed the target laterally", kind="skill"
                )
            return True
        distance = min(self.speed * arm.dt, remaining)
        correction = 0.5 * (error - remaining * self.direction)
        correction *= min(
            1.0, self.lateral_speed / max(np.linalg.norm(correction), 1e-9)
        )
        self.goal[:2] += distance * self.direction + arm.dt * correction
        self.travel += distance
        if self.travel > self.max_travel:
            raise SourceFailure(
                "Pusher travelled past the object without reaching the goal",
                kind="skill",
            )
        arm.command[arm.arm_slice] = arm.planner.cartesian_step(
            arm.observation, arm.world, self.goal, contact_objects=self.contacts
        )
        return False


class Withdraw(FeedbackAction):
    """Back away from contact with the original timed Cartesian motion."""

    def __init__(self, origin, direction):
        super().__init__("retreat")
        self.origin, self.direction = origin.copy(), direction

    def advance(self, arm):
        goal = self.origin.copy()
        goal[:2] -= self.direction * min(0.025, (self.steps + 1) * 0.025 * arm.dt)
        arm.command[arm.arm_slice] = arm.planner.cartesian_step(
            arm.observation, arm.world, goal
        )
        return self.steps * arm.dt > 1.3


class PushExpert(ActionExpert):
    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        if collection.deployment.arms[self.arm.side].asset != PANDA_ASSET:
            raise ValueError("Pushing currently requires the reviewed Panda fingers")
        if self.arm.asset.push_height is None:
            raise ValueError("Push object needs a reviewed Panda contact height")
        self.task = create_task(collection)

    def reset(self, episode_input):
        super().reset(episode_input)
        arm = self.arm
        obj = np.asarray(self.world_state()[f"{arm.obj}/pose_world"])
        self.direction = self.task.target_position_world[:2] - obj[:2]
        self.distance = np.linalg.norm(self.direction)
        self.direction /= self.distance
        yaw = np.arctan2(self.direction[1], self.direction[0])
        arm.rotation = Rotation.from_euler("z", yaw) * Rotation.from_quat([1, 0, 0, 0])
        points = Rotation.from_quat(obj[3:]).apply(corners(arm.asset))
        extent = -(points[:, :2] @ self.direction).min()
        self.push_point = obj[:3].copy()
        self.push_point[:2] -= self.direction * (extent + 0.01 + 0.02)
        self.push_point[2] = self.task.support[2] + arm.asset.push_height
        self.tcp_goal = arm.tcp_at(self.push_point)

    def routine(self):
        yield CloseEmpty()
        approach = self.tcp_goal.copy()
        approach[2] += 0.10
        yield Move(approach, name="approach")
        yield Move(self.tcp_goal, name="descend")
        push = PushContact(
            self.arm.obj,
            self.task.target_position_world,
            self.direction,
            self.distance,
            lambda world: (
                np.linalg.norm(
                    self.task.target_position_world[:2]
                    - world[f"{self.arm.obj}/pose_world"][:2]
                )
                <= self.task.parameters["position_tolerance"]
            ),
        )
        yield push
        yield Withdraw(push.goal, self.direction)
