"""Grasp a rigid hand brush and sweep with its measured carried geometry."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.commands import initial_command
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import corners, transform
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Action, Event
from loom_env.tasks import create_task


class SweepExpert:
    # Clears the conservative carried-tool envelope above the table.
    BRISTLE_CLEARANCE = 0.014
    MIN_TARGET_OVERLAP = 0.005
    STAGES = (
        "approach",
        "descend",
        "close",
        "lift",
        "transfer",
        "lower",
        "sweep",
        "retreat",
        "wait",
    )

    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.side = collection.arm_roles["manipulator"]
        self.arm = collection.deployment.arms[self.side]
        self.tool = collection.role_bindings["tool"]
        tool_spec = collection.scene.objects[self.tool]
        if self.arm.asset != PANDA_ASSET or tool_spec["asset"] != "robodojo:hand_brush":
            raise ValueError(
                "Sweep currently requires the reviewed Panda and hand brush"
            )
        self.asset = asset_definition(tool_spec["asset"])
        self.obj = collection.role_bindings["target_object"]
        self.target_asset = asset_definition(
            collection.scene.objects[self.obj]["asset"]
        )
        self.task = create_task(collection)
        self.dt = collection.deployment.control_dt
        self.arm_slice = collection.deployment.action_slices[f"{self.side}/arm"]
        self.grip_slice = collection.deployment.action_slices[f"{self.side}/gripper"]

    def close(self):
        self.planner.planner.destroy()

    @property
    def stage(self):
        return self.STAGES[self.index]

    def reset(self, episode_input):
        self.command = initial_command(self.collection.deployment)
        self.index = self.step = self.stage_steps = self.path_index = self.stable = (
            self.lost
        ) = 0
        self.path = None
        self.started = False
        self.retreat_replans = 0
        self.planner.detach()
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
        self.grasp_rotation = Rotation.from_matrix(
            np.column_stack((np.cross(closing, down), closing, down))
        )
        delta = (
            self.task.target_position_world[:2] - world[f"{self.obj}/pose_world"][:2]
        )
        self.distance = np.linalg.norm(delta)
        self.direction = delta / self.distance
        self.travel = 0.0

    def _advance(self, events):
        events.append(
            Event("skill", self.stage, self.step, {"phase": "end", "arm": self.side})
        )
        self.index += 1
        self.stage_steps = self.path_index = self.stable = 0
        self.path, self.started = None, False

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

    def _goal(self, observation, world):
        if self.stage in {"approach", "descend"}:
            grasp = transform(
                world[f"{self.tool}/pose_world"], [*self.asset.grasp, 0, 0, 0, 1]
            )[:3]
            if self.stage == "approach":
                grasp[2] += 0.12
            return np.r_[
                grasp - self.grasp_rotation.apply(self.planner.profile.tcp_to_grasp),
                self.grasp_rotation.as_quat(),
            ]
        if self.stage in {"lift", "retreat"}:
            goal = observation.values[f"robot/{self.side}/tcp_pose_world"].copy()
            if self.stage == "lift":
                goal[2] += 0.12
            else:
                clearance = self.task.metrics(world)["tool_clearance"]
                goal[2] += max(
                    0.04, self.task.parameters["tool_clearance"] + 0.04 - clearance
                )
            return goal
        goal = self.sweep_goal.copy()
        if self.stage == "transfer":
            goal[2] += 0.10
        return goal

    def act(self, observation):
        world, events = self.world_state(), []
        grip = bool(world[f"{self.tool}/grasped_by"][ARMS.index(self.side)])
        if self.index >= self.STAGES.index("lift"):
            self.lost = 0 if grip else self.lost + 1
            if self.lost >= 5:
                raise SourceFailure(
                    "Sweep tool lost opposing finger contacts", kind="skill"
                )
        other = "left" if self.side == "right" else "right"
        if (
            np.max(
                np.abs(
                    observation.values[f"robot/{other}/joint_position"]
                    - self.collection.deployment.arms[other].initial_positions
                )
            )
            > 0.03
        ):
            raise SourceFailure("Holding arm moved outside tolerance", kind="skill")
        if not self.started:
            self.started = True
            print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
            events.append(
                Event(
                    "skill", self.stage, self.step, {"phase": "begin", "arm": self.side}
                )
            )
            if self.stage == "transfer":
                self._prepare_sweep(observation, world)
                self.planner.attach(observation, world, max_cell_size=0.025)
                events.append(
                    Event(
                        "planning",
                        "sweep_geometry",
                        self.step,
                        {
                            "tool_in_tcp": self.tool_in_tcp.tolist(),
                            "direction": self.direction.tolist(),
                            "sweep_tcp_goal": self.sweep_goal.tolist(),
                        },
                    )
                )
            if self.stage == "sweep":
                self.tcp_goal = observation.values[
                    f"robot/{self.side}/tcp_pose_world"
                ].copy()
            elif self.stage not in {"close", "wait"}:
                self.path, detail = self.planner.plan(
                    observation,
                    world,
                    self._goal(observation, world),
                    allow_object_contact=self.stage in {"descend", "lift"},
                    contact_objects=(self.obj,) if self.stage == "retreat" else (),
                )
                events.append(Event("planning", self.stage, self.step, detail))
        if self.stage == "close":
            self.command[self.grip_slice] = self.arm.gripper.command_limits[0][0]
            self.stable = self.stable + 1 if grip else 0
            if self.stable * self.dt >= 0.6:
                self._advance(events)
            elif self.stage_steps * self.dt > 3:
                raise SourceFailure("Brush handle grasp timed out", kind="skill")
        elif self.stage == "sweep":
            error = (
                self.task.target_position_world[:2]
                - world[f"{self.obj}/pose_world"][:2]
            )
            remaining = float(error @ self.direction)
            if remaining <= 0.003:
                if not self.task.at_goal(self.task.metrics(world)):
                    raise SourceFailure(
                        "Sweep missed the region laterally", kind="skill"
                    )
                self._advance(events)
            else:
                distance = min(0.06 * self.dt, remaining)
                self.tcp_goal[:2] += self.direction * distance
                self.travel += distance
                if self.travel > self.distance + 0.10:
                    raise SourceFailure(
                        "Brush passed the target without sweeping it", kind="skill"
                    )
                self.command[self.arm_slice] = self.planner.cartesian_step(
                    observation, world, self.tcp_goal, contact_objects=(self.obj,)
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
                self.stable = self.stable + 1 if error < 0.015 else 0
                if self.stable >= 3:
                    clearance = (
                        self.task.metrics(world)["tool_clearance"]
                        if self.stage == "retreat"
                        else None
                    )
                    if (
                        clearance is not None
                        and clearance < self.task.parameters["tool_clearance"] + 0.02
                    ):
                        if self.retreat_replans >= 2:
                            raise SourceFailure(
                                "Brush did not clear the table after measured retreat",
                                kind="skill",
                            )
                        self.retreat_replans += 1
                        events.append(
                            Event(
                                "planning",
                                "retreat_clearance_retry",
                                self.step,
                                {
                                    "clearance_m": clearance,
                                    "attempt": self.retreat_replans,
                                },
                            )
                        )
                        self.path = None
                        self.started = False
                        self.path_index = self.stable = 0
                    else:
                        self._advance(events)
        if self.stage_steps * self.dt > 18 and self.stage != "wait":
            raise SourceFailure(f"Sweep stage {self.stage} timed out", kind="skill")
        self.step += 1
        self.stage_steps += 1
        return Action(self.command.copy(), tuple(events))
