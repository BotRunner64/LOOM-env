"""Grasp a hand brush and compose measured tool transport and contact motion."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import corners
from loom_env.tasks import create_task

from .actions import ActionExpert, CloseGripper, Move, MoveHeld
from .push import PushContact


class SweepExpert(ActionExpert):
    BRISTLE_CLEARANCE = 0.014
    MIN_TARGET_OVERLAP = 0.005

    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state, object_role="tool")
        self.side, self.tool, self.asset = self.arm.side, self.arm.obj, self.arm.asset
        if (
            collection.deployment.arms[self.side].asset != PANDA_ASSET
            or collection.scene.objects[self.tool]["asset"] != "robodojo:hand_brush"
        ):
            raise ValueError(
                "Sweep currently requires the reviewed Panda and hand brush"
            )
        self.obj = collection.role_bindings["target_object"]
        self.target_asset = asset_definition(
            collection.scene.objects[self.obj]["asset"]
        )
        self.task = create_task(collection)

    def reset(self, episode_input):
        super().reset(episode_input)
        world = self.world_state()
        target = world[f"{self.obj}/pose_world"]
        target_top = (
            Rotation.from_quat(target[3:]).apply(corners(self.target_asset))[:, 2].max()
            + target[2]
            - self.task.support[2]
        )
        if target_top < self.BRISTLE_CLEARANCE + self.MIN_TARGET_OVERLAP:
            raise ValueError(
                "Sweep target is too low for the brush contact path (needs 5 mm overlap)"
            )
        tool_pose = world[f"{self.tool}/pose_world"]
        # Source X spans the narrow handle. Fingers close across it from above.
        closing = Rotation.from_quat(tool_pose[3:]).apply([1.0, 0, 0])
        closing[2] = 0
        closing /= np.linalg.norm(closing)
        down = np.array([0.0, 0, -1.0])
        self.arm.rotation = Rotation.from_matrix(
            np.column_stack((np.cross(closing, down), closing, down))
        )
        delta = (
            self.task.target_position_world[:2] - world[f"{self.obj}/pose_world"][:2]
        )
        self.distance = np.linalg.norm(delta)
        self.direction = delta / self.distance

    def _prepare_sweep(self, observation, world):
        tcp = observation.values[f"robot/{self.side}/tcp_pose_world"]
        tool_pose = world[f"{self.tool}/pose_world"]
        tcp_r = Rotation.from_quat(tcp[3:])
        # Freeze only the planning transform, never the simulated rigid body.
        self.tool_in_tcp = np.r_[
            tcp_r.inv().apply(tool_pose[:3] - tcp[:3]),
            (tcp_r.inv() * Rotation.from_quat(tool_pose[3:])).as_quat(),
        ]
        # Keep local Y up (bristles down), local X along the sweep direction.
        x = np.r_[self.direction, 0.0]
        up = np.array([0.0, 0, 1.0])
        tool_r = Rotation.from_matrix(np.column_stack((x, up, np.cross(x, up))))
        points = tool_r.apply(corners(self.asset))
        target = world[f"{self.obj}/pose_world"]
        target_points = Rotation.from_quat(target[3:]).apply(corners(self.target_asset))
        edge = float((target_points[:, :2] @ self.direction).min())
        front = float((points[:, :2] @ self.direction).max())
        # Centre the bristle section (local Z 0..12.5 cm) across the target.
        blade = tool_r.apply([0, 0, 0.06])
        position = target[:3].copy() - blade
        position[:2] += self.direction * (edge - front - 0.025)
        position[2] = self.task.support[2] - points[:, 2].min() + self.BRISTLE_CLEARANCE
        desired_tcp_r = tool_r * Rotation.from_quat(self.tool_in_tcp[3:]).inv()
        self.sweep_goal = np.r_[
            position - desired_tcp_r.apply(self.tool_in_tcp[:3]),
            desired_tcp_r.as_quat(),
        ]

    def retreat_goal(self):
        goal = self.arm.tcp.copy()
        clearance = self.task.metrics(self.arm.world)["tool_clearance"]
        goal[2] += max(0.04, self.task.parameters["tool_clearance"] + 0.04 - clearance)
        return goal

    def retreat(self):
        for attempt in range(3):
            yield MoveHeld(
                self.retreat_goal(),
                name="retreat",
                tolerance=0.015,
                contact_objects=(self.obj,),
            )
            clearance = self.task.metrics(self.arm.world)["tool_clearance"]
            if clearance >= self.task.parameters["tool_clearance"] + 0.02:
                return
            if attempt == 2:
                raise SourceFailure(
                    "Brush did not clear the table after measured retreat", kind="skill"
                )
            self.arm.event(
                "planning",
                "retreat_clearance_retry",
                {"clearance_m": clearance, "attempt": attempt + 1},
            )

    def routine(self):
        arm = self.arm
        yield Move(arm.grasp_goal(0.12), name="approach", tolerance=0.015)
        yield Move(
            arm.grasp_goal(), name="descend", allow_object_contact=True, tolerance=0.015
        )
        yield CloseGripper(
            stable_samples=int(np.ceil(0.6 / arm.dt)), min_time=0, timeout=3
        )
        self.holding = (arm,)
        goal = arm.tcp.copy()
        goal[2] += 0.12
        yield MoveHeld(
            goal, name="lift", attach=False, allow_object_contact=True, tolerance=0.015
        )
        self._prepare_sweep(arm.observation, arm.world)
        arm.event(
            "planning",
            "sweep_geometry",
            {
                "tool_in_tcp": self.tool_in_tcp.tolist(),
                "direction": self.direction.tolist(),
                "sweep_tcp_goal": self.sweep_goal.tolist(),
            },
        )
        goal = self.sweep_goal.copy()
        goal[2] += 0.10
        yield MoveHeld(goal, name="transfer", max_cell_size=0.025, tolerance=0.015)
        yield MoveHeld(self.sweep_goal, name="lower", tolerance=0.015)
        yield PushContact(
            self.obj,
            self.task.target_position_world,
            self.direction,
            self.distance,
            lambda world: self.task.at_goal(self.task.metrics(world)),
            speed=0.06,
            margin=0.10,
            lateral_speed=0,
            contact_objects=(self.obj,),
            held=True,
        )
        yield from self.retreat()
