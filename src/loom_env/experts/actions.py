"""Feedback actions independent of task IDs and complete expert workflows.

Call action.step(arm) once per control tick after arm.update(observation, world).
True means this action finished, not that the task succeeded. Actions modify the
shared command and emit events; only the environment applies physical actions.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.embodiments.commands import initial_command
from loom_env.embodiments.manipulation import manipulation_profile
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import transform
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Action, Event


class Manipulator:
    """Measured arm/object state and command buffer; contains no action sequence."""

    def __init__(self, deployment, scene, side, obj, planner):
        self.deployment, self.scene = deployment, scene
        self.side, self.obj, self.planner = side, obj, planner
        self.asset = asset_definition(scene.objects[obj]["asset"])
        arm = deployment.arms[side]
        self.profile = manipulation_profile(arm)
        self.default_rotation = Rotation.from_quat(
            arm.base_pose[3:]
        ) * Rotation.from_quat(self.profile.grasp_rotation)
        self.closed, self.opened = arm.gripper.command_limits[0]
        self.arm_slice = deployment.action_slices[f"{side}/arm"]
        self.grip_slice = deployment.action_slices[f"{side}/gripper"]
        self.dt = deployment.control_dt
        self.reset()

    def reset(self):
        self.command = initial_command(self.deployment)
        self.tick, self.lost, self.checked_tick = -1, 0, -2
        self.events = []
        self.rotation = self.default_rotation

    def update(self, observation, world):
        self.tick += 1
        self.observation, self.world = observation, world
        self.events = []
        other = "left" if self.side == "right" else "right"
        part = self.deployment.action_slices[f"{other}/arm"]
        if (
            np.max(
                np.abs(
                    observation.values[f"robot/{other}/joint_position"]
                    - self.command[part]
                )
            )
            > 0.03
        ):
            raise SourceFailure("Holding arm moved outside tolerance", kind="skill")

    @property
    def held(self):
        return bool(self.world[f"{self.obj}/grasped_by"][ARMS.index(self.side)])

    @property
    def tcp(self):
        return self.observation.values[f"robot/{self.side}/tcp_pose_world"]

    def monitor_grasp(self):
        # Several actions may finish in one tick; count physical samples once.
        if self.checked_tick != self.tick:
            self.lost = 0 if self.held else self.lost + 1
            self.checked_tick = self.tick
        if self.lost >= 5:
            raise SourceFailure("Object lost opposing finger contacts", kind="skill")

    def tcp_at(self, point):
        return np.r_[
            np.asarray(point) - self.rotation.apply(self.profile.tcp_to_grasp),
            self.rotation.as_quat(),
        ]

    def grasp_goal(self, clearance=0.0):
        if self.asset.grasp is None:
            raise ValueError("Grasp requires a reviewed grasp annotation")
        point = transform(
            self.world[f"{self.obj}/pose_world"], [*self.asset.grasp, 0, 0, 0, 1]
        )[:3]
        point[2] += clearance
        return self.tcp_at(point)

    def event(self, kind, name, detail):
        self.events.append(Event(kind, name, self.tick, detail))


class FeedbackAction:
    def __init__(self, name, timeout=18.0):
        self.name, self.timeout = name, timeout
        self.steps, self.started, self.done = 0, False, False

    def start(self, arm):
        pass

    def step(self, arm):
        if self.done:
            return True
        if not self.started:
            self.started = True
            arm.event("skill", self.name, {"phase": "begin", "arm": arm.side})
            print(f"EXPERT step={arm.tick} action={self.name}", flush=True)
            self.start(arm)
        self.done = bool(self.advance(arm))
        if self.done:
            arm.event(
                "skill", self.name, {"phase": "end", "success": True, "arm": arm.side}
            )
        elif self.steps * arm.dt > self.timeout:
            raise SourceFailure(f"Action {self.name} timed out", kind="skill")
        self.steps += 1
        return self.done


class Move(FeedbackAction):
    """Execute a collision-checked path and confirm measured arrival."""

    def __init__(
        self,
        goal,
        *,
        name="move",
        contact_objects=(),
        allow_object_contact=False,
        time_scale=1,
    ):
        super().__init__(name)
        self.goal = np.asarray(goal).copy()
        self.contact_objects = tuple(contact_objects)
        self.allow_object_contact = allow_object_contact
        self.time_scale = time_scale
        self.index, self.stable = 0, 0

    def start(self, arm):
        self.path, detail = arm.planner.plan(
            arm.observation,
            arm.world,
            self.goal,
            allow_object_contact=self.allow_object_contact,
            contact_objects=self.contact_objects,
        )
        if self.time_scale != 1:
            knots = np.vstack(
                [arm.observation.values[f"robot/{arm.side}/joint_position"], self.path]
            )
            time = (
                np.arange(1, (len(knots) - 1) * self.time_scale + 1) / self.time_scale
            )
            self.path = np.column_stack(
                [
                    np.interp(time, np.arange(len(knots)), knots[:, j])
                    for j in range(knots.shape[1])
                ]
            )
            detail["execution_time_scale"] = self.time_scale
        arm.event("planning", self.name, detail)

    def advance(self, arm):
        if self.index < len(self.path):
            arm.command[arm.arm_slice] = self.path[self.index]
            self.index += 1
        else:
            q = arm.observation.values[f"robot/{arm.side}/joint_position"]
            self.stable = (
                self.stable + 1
                if np.max(np.abs(q - arm.command[arm.arm_slice])) < 0.01
                else 0
            )
            if self.stable >= 3:
                return True
            if (self.steps - len(self.path)) * arm.dt > 3:
                raise SourceFailure(
                    f"{self.name} did not reach its joint target", kind="skill"
                )
        return False


class MoveHeld(Move):
    """A planned move that can start from an existing physical grasp."""

    def __init__(self, goal, *, attach=True, max_cell_size=None, **kwargs):
        super().__init__(goal, **kwargs)
        self.attach, self.max_cell_size = attach, max_cell_size

    def start(self, arm):
        if not arm.held:
            raise SourceFailure("Held motion requires an existing grasp", kind="skill")
        if self.attach and not arm.planner.attached:
            arm.planner.attach(
                arm.observation, arm.world, max_cell_size=self.max_cell_size
            )
        super().start(arm)

    def advance(self, arm):
        arm.command[arm.grip_slice] = arm.closed
        arm.monitor_grasp()
        return super().advance(arm)


class CloseGripper(FeedbackAction):
    def __init__(self, *, stable_samples=5, min_time=0.6, timeout=2.0):
        super().__init__("close", timeout)
        self.stable_samples, self.min_time, self.stable = stable_samples, min_time, 0

    def advance(self, arm):
        arm.command[arm.grip_slice] = arm.closed
        self.stable = self.stable + 1 if arm.held else 0
        return (
            self.stable >= self.stable_samples and self.steps * arm.dt >= self.min_time
        )


class Grasp(FeedbackAction):
    """Approach, descend, and establish a measured grasp; no lift or transport."""

    def __init__(
        self, *, contact_objects=(), stable_samples=5, min_time=0.6, timeout=2.0
    ):
        super().__init__("grasp", timeout=40)
        self.contacts = tuple(contact_objects)
        self.closing = CloseGripper(
            stable_samples=stable_samples, min_time=min_time, timeout=timeout
        )
        self.phase = 0

    def start(self, arm):
        self.already_held = arm.held
        if self.already_held:
            arm.rotation = Rotation.from_quat(arm.tcp[3:].copy())
            return
        if arm.asset.grasp_rotation is not None:
            pose = arm.world[f"{arm.obj}/pose_world"]
            arm.rotation = Rotation.from_quat(pose[3:].copy()) * Rotation.from_quat(
                arm.asset.grasp_rotation
            )
        self.motion = Move(arm.grasp_goal(0.12), name="approach")

    def advance(self, arm):
        if self.already_held:
            arm.command[arm.grip_slice] = arm.closed
            return True
        while self.phase < 2:
            if not self.motion.step(arm):
                return False
            self.phase += 1
            if self.phase == 1:
                self.motion = Move(
                    arm.grasp_goal(),
                    name="descend",
                    allow_object_contact=True,
                    contact_objects=self.contacts,
                )
        return self.closing.step(arm)


class Release(FeedbackAction):
    """Open and confirm loss of grasp; detaches planning geometry only."""

    def __init__(self, *, stable_samples=5, min_time=0.6, timeout=2.0, all_arms=False):
        super().__init__("release", timeout)
        self.stable_samples, self.min_time = stable_samples, min_time
        self.all_arms, self.stable = all_arms, 0

    def start(self, arm):
        arm.planner.detach()

    def advance(self, arm):
        arm.command[arm.grip_slice] = arm.opened
        held = (
            bool(np.any(arm.world[f"{arm.obj}/grasped_by"]))
            if self.all_arms
            else arm.held
        )
        self.stable = 0 if held else self.stable + 1
        return (
            self.stable >= self.stable_samples and self.steps * arm.dt >= self.min_time
        )


class Extract(FeedbackAction):
    """Held straight motion along a supplied exit axis, with measured progress."""

    def __init__(self, axis, *, distance=0.125, speed=0.02, contact_objects=()):
        super().__init__("extraction", timeout=20)
        axis = np.asarray(axis, dtype=float)
        if not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
            raise ValueError("Extraction needs a finite nonzero axis")
        self.axis = axis / np.linalg.norm(axis)
        self.distance, self.speed = distance, speed
        self.contacts = tuple(contact_objects)
        self.reference = 0.0

    def start(self, arm):
        if not arm.held:
            raise SourceFailure("Extraction requires an existing grasp", kind="skill")
        self.origin = arm.tcp.copy()

    def advance(self, arm):
        arm.command[arm.grip_slice] = arm.closed
        arm.monitor_grasp()
        progress = float((arm.tcp[:3] - self.origin[:3]) @ self.axis)
        if self.distance - progress < 0.001:
            return True
        self.reference = min(
            self.distance, self.reference + self.speed * arm.dt, progress + 0.003
        )
        goal = self.origin.copy()
        goal[:3] += self.axis * self.reference
        arm.command[arm.arm_slice] = arm.planner.cartesian_step(
            arm.observation, arm.world, goal, contact_objects=self.contacts
        )
        return False


class ActionExpert:
    """Drive a Python routine yielding feedback actions; owns no task stages."""

    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.arm = Manipulator(
            collection.deployment,
            collection.scene,
            collection.arm_roles["manipulator"],
            collection.role_bindings["target_object"],
            planner,
        )

    def reset(self, episode_input):
        self.arm.reset()
        self.planner.detach()
        self.actions = self.routine()
        self.current = None
        self.finished = False
        self.hold_at_end = False

    def close(self):
        self.planner.planner.destroy()

    def act(self, observation):
        arm = self.arm
        arm.update(observation, self.world_state())
        while not self.finished:
            if self.current is None:
                self.current = next(self.actions, None)
                if self.current is None:
                    self.finished = True
                    break
            if not self.current.step(arm):
                break
            self.current = None
        if self.finished and self.hold_at_end:
            arm.monitor_grasp()
        return Action(arm.command.copy(), tuple(arm.events))
