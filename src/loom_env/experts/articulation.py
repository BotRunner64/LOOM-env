"""Grasp a screen edge and open the passive hinge through robot contact."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition, load_prepared
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.commands import initial_command
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import transform
from loom_env.specs.episode import Action, Event


def hinge_step(body_pose, hinge_pose, axis, delta):
    """Rigid motion around the measured hinge frame; radians, XYZW poses."""
    axis_world = Rotation.from_quat(hinge_pose[3:].copy()).apply(axis)
    turn = Rotation.from_rotvec(np.asarray(axis_world) * delta)
    return np.r_[
        hinge_pose[:3] + turn.apply(body_pose[:3] - hinge_pose[:3]),
        (turn * Rotation.from_quat(body_pose[3:].copy())).as_quat(),
    ]


class OpenLaptopExpert:
    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.side = collection.arm_roles["manipulator"]
        if collection.deployment.arms[self.side].asset != PANDA_ASSET:
            raise ValueError("Laptop opening currently requires the reviewed Panda fingers")
        self.obj = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.obj]["asset"])
        self.arm_slice = collection.deployment.action_slices[f"{self.side}/arm"]
        self.grip_slice = collection.deployment.action_slices[f"{self.side}/gripper"]
        self.dt = collection.deployment.control_dt
        self.target = collection.task.parameters["target_angle"]
        physics = load_prepared(
            planner.asset_root, collection.scene.objects[self.obj]["asset"]
        )["physics_properties"]
        self.hinge = physics["joint"]
        if (
            not self.hinge["limits_rad"][0]
            <= self.target
            <= self.hinge["limits_rad"][1]
        ):
            raise ValueError("Opening target exceeds USD joint limits")

    def close(self):
        self.planner.planner.destroy()

    def reset(self, episode_input):
        self.command = initial_command(self.collection.deployment)
        self.stage, self.step, self.stage_steps = "approach", 0, 0
        self.path, self.index, self.stable = None, 0, 0
        self.planner.detach()

    def _change(self, stage, events):
        events.append(Event("skill", self.stage, self.step, {"phase": "end"}))
        self.stage, self.stage_steps, self.path, self.index, self.stable = (
            stage,
            -1,
            None,
            0,
            0,
        )

    def _tcp(self, body, offset=0.0):
        # Finger closing direction is screen thickness; approach along its edge.
        rotation = Rotation.from_quat(body[3:].copy()) * Rotation.from_euler(
            "xz", [np.pi, np.pi]
        )
        point = transform(body, [*self.asset.contact, 0, 0, 0, 1])[:3]
        point += Rotation.from_quat(body[3:].copy()).apply([0, 0, offset])
        return np.r_[
            point - rotation.apply(self.planner.profile.tcp_to_grasp),
            rotation.as_quat(),
        ]

    def act(self, observation):
        world, events = self.world_state(), []
        if self.stage_steps == 0:
            print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
            events.append(Event("skill", self.stage, self.step, {"phase": "begin"}))
        body = world[f"{self.obj}/links/{self.asset.moving_body}/pose_world"]
        q = float(world[f"{self.obj}/joints/{self.asset.joint}/position"])
        forces = world[
            f"{self.obj}/links/{self.asset.moving_body}/finger_contact_forces_world"
        ][("left", "right").index(self.side)]
        if self.stage in {"approach", "grasp_pose"}:
            if self.path is None:
                goal = self._tcp(body, 0.065 if self.stage == "approach" else 0.0)
                self.path, detail = self.planner.plan(
                    observation,
                    world,
                    goal,
                    allow_object_contact=self.stage == "grasp_pose",
                )
                events.append(Event("planning", self.stage, self.step, detail))
            if self.index < len(self.path):
                self.command[self.arm_slice] = self.path[self.index]
                self.index += 1
            else:
                error = np.max(
                    np.abs(
                        observation.values[f"robot/{self.side}/joint_position"]
                        - self.command[self.arm_slice]
                    )
                )
                self.stable = self.stable + 1 if error < 0.01 else 0
                if self.stable >= 3:
                    self._change(
                        "grasp_pose" if self.stage == "approach" else "close", events
                    )
        elif self.stage == "close":
            self.command[self.grip_slice] = 0.0
            self.stable = self.stable + 1 if opposing_contacts(forces) else 0
            if self.stable >= 3:
                self.grasp_q = q
                self.reference_q = q
                self.grasp_tcp = observation.values[
                    f"robot/{self.side}/tcp_pose_world"
                ].copy()
                self._change("open", events)
        elif self.stage == "open":
            remaining = self.target - q
            if abs(remaining) < np.deg2rad(3):
                self._change("release", events)
            else:
                parent = world[f"{self.obj}/links/{self.asset.root_body}/pose_world"]
                hinge = transform(parent, self.hinge["pose_parent"])
                self.reference_q += np.clip(
                    self.target - self.reference_q,
                    -np.deg2rad(15) * self.dt,
                    np.deg2rad(15) * self.dt,
                )
                desired = hinge_step(
                    self.grasp_tcp,
                    hinge,
                    self.hinge["axis"],
                    self.reference_q - self.grasp_q,
                )
                self.command[self.arm_slice] = self.planner.cartesian_step(
                    observation, world, desired
                )
                if self.stage_steps % 20 == 0:
                    print(
                        f"HINGE step={self.step} measured_deg={np.rad2deg(q):.2f} reference_deg={np.rad2deg(self.reference_q):.2f}",
                        flush=True,
                    )
        elif self.stage == "release":
            self.command[self.grip_slice] = 0.08
        if self.stage_steps * self.dt > 20:
            raise SourceFailure(f"Opening stage {self.stage} timed out", kind="skill")
        self.step += 1
        self.stage_steps += 1
        return Action(self.command.copy(), tuple(events))
