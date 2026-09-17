"""Move a passive joint through a measured link-local grasp."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition, load_prepared
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.commands import initial_command
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import transform
from loom_env.specs.episode import Action, Event


def joint_step(body_pose, joint_pose, axis, delta, joint_type):
    """Motion in the measured joint frame; radians/metres and XYZW poses."""
    axis_world = Rotation.from_quat(joint_pose[3:].copy()).apply(axis)
    if joint_type == "prismatic":
        return np.r_[body_pose[:3] + np.asarray(axis_world) * delta, body_pose[3:]]
    if joint_type != "revolute":
        raise ValueError(f"Unsupported joint type: {joint_type}")
    turn = Rotation.from_rotvec(np.asarray(axis_world) * delta)
    return np.r_[
        joint_pose[:3] + turn.apply(body_pose[:3] - joint_pose[:3]),
        (turn * Rotation.from_quat(body_pose[3:].copy())).as_quat(),
    ]


class ArticulationExpert:
    def __init__(self, collection, planner, world_state):
        self.collection, self.planner, self.world_state = (
            collection,
            planner,
            world_state,
        )
        self.side = collection.arm_roles["manipulator"]
        if collection.deployment.arms[self.side].asset != PANDA_ASSET:
            raise ValueError(
                "Articulation grasp manipulation currently requires the reviewed Panda fingers"
            )
        self.obj = collection.role_bindings["target_object"]
        self.asset = asset_definition(collection.scene.objects[self.obj]["asset"])
        self.arm_slice = collection.deployment.action_slices[f"{self.side}/arm"]
        self.grip_slice = collection.deployment.action_slices[f"{self.side}/gripper"]
        self.dt = collection.deployment.control_dt
        self.target = collection.task.parameters["target_position"]
        physics = load_prepared(
            planner.asset_root, collection.scene.objects[self.obj]["asset"]
        )["physics_properties"]
        self.joint = physics["joint"]
        self.contact_pose = self.asset.contact_poses[
            int(collection.task.parameters["contact_index"])
        ]
        self.speed, self.precision = (
            (np.deg2rad(15), np.deg2rad(3))
            if self.joint["type"] == "revolute"
            else (0.02, 0.001)
        )
        self.closed, self.opened = collection.deployment.arms[
            self.side
        ].gripper.command_limits[0]
        if not self.joint["limits"][0] <= self.target <= self.joint["limits"][1]:
            raise ValueError("Joint target exceeds USD joint limits")

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
        contact = transform(body, self.contact_pose)
        rotation = Rotation.from_quat(contact[3:].copy())
        # Grasp-frame +Z is the tool approach direction.
        point = contact[:3] - rotation.apply([0, 0, offset])
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
                        "grasp_pose" if self.stage == "approach" else "grasp", events
                    )
        elif self.stage == "grasp":
            self.command[self.grip_slice] = self.closed
            self.stable = self.stable + 1 if opposing_contacts(forces) else 0
            if self.stable >= 3:
                self.grasp_q = q
                self.reference_q = q
                self.grasp_tcp = observation.values[
                    f"robot/{self.side}/tcp_pose_world"
                ].copy()
                self._change("move_joint", events)
        elif self.stage == "move_joint":
            remaining = self.target - q
            if abs(remaining) < min(
                self.precision, self.collection.task.parameters["position_tolerance"]
            ):
                self._change("release", events)
            else:
                parent = world[f"{self.obj}/links/{self.asset.root_body}/pose_world"]
                joint_frame = transform(parent, self.joint["pose_parent"])
                self.reference_q += np.clip(
                    self.target - self.reference_q,
                    -self.speed * self.dt,
                    self.speed * self.dt,
                )
                desired = joint_step(
                    self.grasp_tcp,
                    joint_frame,
                    self.joint["axis"],
                    self.reference_q - self.grasp_q,
                    self.joint["type"],
                )
                self.command[self.arm_slice] = self.planner.cartesian_step(
                    observation, world, desired
                )
                if self.stage_steps % 20 == 0:
                    print(
                        f"JOINT step={self.step} measured={q:.5f} reference={self.reference_q:.5f} unit={self.joint['unit']}",
                        flush=True,
                    )
        elif self.stage == "release":
            self.command[self.grip_slice] = self.opened
        if self.stage_steps * self.dt > 20:
            raise SourceFailure(
                f"Articulation stage {self.stage} timed out", kind="skill"
            )
        self.step += 1
        self.stage_steps += 1
        return Action(self.command.copy(), tuple(events))
