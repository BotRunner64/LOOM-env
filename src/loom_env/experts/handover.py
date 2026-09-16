"""Sequential dual-arm handover; object motion remains entirely physical."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.commands import initial_command
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import transform, workspace
from loom_env.specs.config import ARMS
from loom_env.specs.episode import Action, Event


class HandoverExpert:
    # Loaded Panda joints retain controller deflection for the tea pack.
    # This gates stage progression only; task success continues
    # to use measured contacts and object motion.
    JOINT_TRACKING_TOLERANCE = 0.02
    STAGES = (
        "giver_approach",
        "giver_descend",
        "giver_close",
        "giver_lift",
        "giver_present",
        "receiver_approach",
        "receiver_descend",
        "receiver_close",
        "giver_release",
        "giver_retreat",
        "giver_clear",
        "receiver_lift",
        "wait",
    )

    def __init__(self, collection, planners, world_state):
        self.collection, self.planners, self.world_state = (
            collection,
            planners,
            world_state,
        )
        self.giver, self.receiver = (
            collection.arm_roles[k] for k in ("giver", "receiver")
        )
        self.obj = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.obj]["asset"])
        if any(arm.asset != PANDA_ASSET for arm in collection.deployment.arms.values()):
            raise ValueError("Handover currently requires dual Panda")
        self.frame, _ = workspace(collection.scene)
        self.dt = collection.deployment.control_dt
        self.hold_time = max(0.6, collection.task.parameters["hold_time"] + self.dt)

    def close(self):
        for planner in self.planners.values():
            planner.planner.destroy()

    def reset(self, episode_input):
        self._configure_grasps(self.world_state())
        self.command = initial_command(self.collection.deployment)
        self.index = self.step = self.stage_steps = self.path_index = self.stable = 0
        self.path = None
        self.started = False
        self.exchange_position = None
        self.lost = 0
        for planner in self.planners.values():
            planner.detach()

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
        profiles = [self.planners[side].profile for side in (self.giver, self.receiver)]
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
            + np.linalg.norm(self.planners[self.receiver].profile.tcp_to_grasp)
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

    @property
    def stage(self):
        return self.STAGES[self.index]

    def _side(self):
        return (
            self.receiver
            if self.stage.startswith("receiver") or self.stage == "wait"
            else self.giver
        )

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
            point - rotation.apply(self.planners[side].profile.tcp_to_grasp),
            rotation.as_quat(),
        ]

    def _goal(self, observation, world, side):
        if self.stage.endswith("approach"):
            return self._tcp(world, side, 0.12)
        if self.stage.endswith("descend"):
            return self._tcp(world, side)
        measured = observation.values[f"robot/{side}/tcp_pose_world"].copy()
        if self.stage == "giver_lift":
            measured[2] += 0.20
        elif self.stage == "giver_present":
            # Translate the measured grasp to the shared workspace without resetting object pose.
            target = transform(self.frame, [0, 0, self.exchange_height, 0, 0, 0, 1])[:3]
            measured[:3] += target - world[f"{self.obj}/pose_world"][:3]
        elif self.stage == "giver_retreat":
            measured[2] += 0.15
        elif self.stage == "giver_clear":
            return self.giver_home.copy()
        elif self.stage == "receiver_lift":
            target_height = (
                self.exchange_position[2]
                + self.collection.task.parameters["transfer_distance"]
                + 0.04
            )
            measured[2] += max(0, target_height - world[f"{self.obj}/pose_world"][2])
        return measured

    def _advance(self, events):
        events.append(
            Event(
                "skill",
                self.stage,
                self.step,
                {"phase": "end", "arm": self._side(), "success": True},
            )
        )
        self.index += 1
        self.path = None
        self.started = False
        self.stage_steps = self.path_index = self.stable = 0

    def act(self, observation):
        if self.step == 0:
            self.giver_home = observation.values[
                f"robot/{self.giver}/tcp_pose_world"
            ].copy()
        world = self.world_state()
        grasped = world[f"{self.obj}/grasped_by"]
        giver, receiver = (
            bool(grasped[ARMS.index(s)]) for s in (self.giver, self.receiver)
        )
        # Losing the responsible grasp is a measured failure, never a reason to teleport/attach in physics.
        required = (
            receiver if self.index >= self.STAGES.index("giver_release") else giver
        )
        if self.index >= self.STAGES.index("giver_lift"):
            self.lost = 0 if required else self.lost + 1
            if self.lost >= 5:
                raise SourceFailure(
                    "Handover lost the responsible arm's grasp", kind="skill"
                )
        events = []
        if self.step == 0:
            events.append(
                Event("planning", "handover_grasp_geometry", 0, self.grasp_geometry)
            )
        side = self._side()
        slices = self.collection.deployment.action_slices
        arm_slice, grip_slice = slices[f"{side}/arm"], slices[f"{side}/gripper"]
        other = self.giver if side == self.receiver else self.receiver
        if (
            np.max(
                np.abs(
                    observation.values[f"robot/{other}/joint_position"]
                    - self.command[slices[f"{other}/arm"]]
                )
            )
            > 0.03
        ):
            raise SourceFailure(
                "Stationary handover arm moved outside tolerance", kind="skill"
            )
        planner = self.planners[side]
        if not self.started:
            self.started = True
            events.append(
                Event("skill", self.stage, self.step, {"phase": "begin", "arm": side})
            )
            print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
            if self.stage == "giver_present":
                planner.attach(observation, world)
            if self.stage == "receiver_approach":
                # The giver holds still; the receiver planner sees the measured giver collision geometry.
                self.planners[self.giver].detach()
            if self.stage == "giver_release":
                self.exchange_position = world[f"{self.obj}/pose_world"][:3].copy()
                # Receiver attachment is delayed until the giver has retreated: its envelope
                # would otherwise overlap the intentionally touching giver fingers.
                self.planners[self.giver].detach()
            if self.stage == "receiver_lift":
                planner.attach(observation, world)
            if self.stage not in {
                "giver_close",
                "receiver_close",
                "giver_release",
                "wait",
            }:
                self.path, detail = planner.plan(
                    observation,
                    world,
                    self._goal(observation, world, side),
                    allow_object_contact=self.stage.endswith("descend")
                    or self.stage
                    in {"giver_lift", "giver_retreat", "receiver_approach"},
                )
                events.append(Event("planning", self.stage, self.step, detail))
        if self.stage in {"giver_close", "receiver_close", "giver_release"}:
            closed, opened = self.collection.deployment.arms[
                side
            ].gripper.command_limits[0]
            release = self.stage == "giver_release"
            self.command[grip_slice] = opened if release else closed
            ready = (
                not giver and receiver
                if release
                else (giver and receiver if side == self.receiver else giver)
            )
            self.stable = self.stable + 1 if ready else 0
            if self.stable * self.dt >= self.hold_time:
                self._advance(events)
            elif self.stage_steps * self.dt > 3:
                raise SourceFailure(
                    f"Measured {self.stage} feedback timed out", kind="skill"
                )
        elif self.stage != "wait":
            if self.path_index < len(self.path):
                self.command[arm_slice] = self.path[self.path_index]
                self.path_index += 1
            else:
                reached = (
                    np.max(
                        np.abs(
                            observation.values[f"robot/{side}/joint_position"]
                            - self.command[arm_slice]
                        )
                    )
                    < self.JOINT_TRACKING_TOLERANCE
                )
                self.stable = self.stable + 1 if reached else 0
                if self.stable * self.dt >= self.hold_time:
                    self._advance(events)
                elif (self.stage_steps - len(self.path)) * self.dt > 3:
                    raise SourceFailure(
                        f"{self.stage} tracking timed out", kind="skill"
                    )
        self.step += 1
        self.stage_steps += 1
        return Action(self.command.copy(), tuple(events))
