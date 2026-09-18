"""Grasp a link, move along its measured joint, and release."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import load_prepared
from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import transform
from loom_env.specs.config import ARMS

from .actions import ActionExpert, CloseGripper, FeedbackAction, Move, Release


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


def link_held(arm):
    return opposing_contacts(
        arm.world[
            f"{arm.obj}/links/{arm.asset.moving_body}/finger_contact_forces_world"
        ][ARMS.index(arm.side)]
    )


class MoveJoint(FeedbackAction):
    """Move an already grasped link along its USD joint axis and frame."""

    def __init__(self, joint, target, tolerance, speed):
        super().__init__("move_joint", timeout=20)
        self.joint, self.target, self.tolerance, self.speed = (
            joint,
            target,
            tolerance,
            speed,
        )

    def start(self, arm):
        if not link_held(arm):
            raise SourceFailure(
                "Joint motion requires a grasp of the moving link", kind="skill"
            )
        self.grasp_q = float(arm.world[f"{arm.obj}/joints/{arm.asset.joint}/position"])
        self.reference = self.grasp_q
        self.origin = arm.tcp.copy()

    def advance(self, arm):
        q = float(arm.world[f"{arm.obj}/joints/{arm.asset.joint}/position"])
        arm.command[arm.grip_slice] = arm.closed
        if abs(self.target - q) < self.tolerance:
            return True
        parent = arm.world[f"{arm.obj}/links/{arm.asset.root_body}/pose_world"]
        frame = transform(parent, self.joint["pose_parent"])
        self.reference += np.clip(
            self.target - self.reference, -self.speed * arm.dt, self.speed * arm.dt
        )
        desired = joint_step(
            self.origin,
            frame,
            self.joint["axis"],
            self.reference - self.grasp_q,
            self.joint["type"],
        )
        arm.command[arm.arm_slice] = arm.planner.cartesian_step(
            arm.observation, arm.world, desired
        )
        return False


class ArticulationExpert(ActionExpert):
    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        self.side, self.obj, self.asset = self.arm.side, self.arm.obj, self.arm.asset
        if collection.deployment.arms[self.side].asset != PANDA_ASSET:
            raise ValueError(
                "Articulation grasp manipulation currently requires the reviewed Panda fingers"
            )
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
        if not self.joint["limits"][0] <= self.target <= self.joint["limits"][1]:
            raise ValueError("Joint target exceeds USD joint limits")

    def _tcp(self, body, offset=0.0):
        contact = transform(body, self.contact_pose)
        rotation = Rotation.from_quat(contact[3:].copy())
        # Grasp-frame +Z is the tool approach direction.
        point = contact[:3] - rotation.apply([0, 0, offset])
        return np.r_[
            point - rotation.apply(self.planner.profile.tcp_to_grasp),
            rotation.as_quat(),
        ]

    def routine(self):
        arm = self.arm
        for name, offset in (("approach", 0.065), ("grasp_pose", 0.0)):
            body = arm.world[f"{arm.obj}/links/{arm.asset.moving_body}/pose_world"]
            yield Move(
                self._tcp(body, offset), name=name, allow_object_contact=offset == 0
            )
        yield CloseGripper(
            stable_samples=3,
            min_time=0,
            timeout=20,
            contact=link_held,
        )
        yield MoveJoint(
            self.joint,
            self.target,
            min(self.precision, self.collection.task.parameters["position_tolerance"]),
            self.speed,
        )
        yield Release(timeout=20, released=lambda arm: not link_held(arm))
