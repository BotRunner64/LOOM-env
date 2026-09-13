"""One logical left/right interface for two fixed-base Isaac Lab articulations.

Import after AppLauncher. The caller owns the physics clock and rendering.
"""

from pathlib import Path

import numpy as np
import torch
from pxr import Usd, UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim.utils.stage import get_current_stage

from loom_env.embodiments.assets import (
    PANDA_ASSET,
    PANDA_USD,
    PANDA_VERSION,
    MODELS,
    SOURCES,
    preparation_version,
    model_name,
    converted_usd,
    prepared_urdf,
    joint_definitions,
    verify_asset,
    validate_visuals,
)
from loom_env.specs.config import ARMS, DeploymentSpec


def spawn_robot(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Enable contact reports on nested URDF bodies before sensor initialization."""
    prim = sim_utils.spawn_from_usd(
        prim_path, cfg, translation=translation, orientation=orientation, **kwargs
    )
    if cfg.activate_contact_sensors:
        stage = get_current_stage()
        for path in sim_utils.find_matching_prim_paths(prim_path):
            for body in Usd.PrimRange(stage.GetPrimAtPath(path)):
                if body.HasAPI(UsdPhysics.RigidBodyAPI):
                    sim_utils.activate_contact_sensors(str(body.GetPath()))
    return prim


def articulation_config(arm, asset_root):
    """Resolve a supported asset and its simulation actuator configuration."""
    if arm.asset == PANDA_ASSET:
        from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG

        cfg = FRANKA_PANDA_HIGH_PD_CFG.copy()
        cfg.spawn.usd_path = PANDA_USD
        version = PANDA_VERSION
    else:
        name = model_name(arm.asset)
        model = MODELS[name]
        joints = joint_definitions(prepared_urdf(asset_root, name))
        actuators = {}
        for group, names, gains in (
            ("arm", arm.joint_names, model["arm_gains"]),
            ("gripper", arm.gripper.joint_names, model["gripper_gains"]),
        ):
            actuators[group] = ImplicitActuatorCfg(
                joint_names_expr=list(names),
                effort_limit_sim={
                    n: min(
                        float(joints[n].find("limit").get("effort")),
                        model.get(f"{group}_effort_limit", float("inf")),
                    )
                    for n in names
                },
                armature=model.get(f"{group}_armature", 0.0),
                velocity_limit_sim={
                    n: min(
                        float(joints[n].find("limit").get("velocity")),
                        3.0 if group == "arm" else 1.0,
                    )
                    for n in names
                },
                stiffness={
                    n: gains[0] if joints[n].find("mimic") is None else 0.0
                    for n in names
                },
                damping={
                    n: gains[1] if joints[n].find("mimic") is None else 0.0
                    for n in names
                },
            )
        cfg = ArticulationCfg(
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(converted_usd(asset_root, name).resolve()),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True, max_depenetration_velocity=5.0
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
            ),
            actuators=actuators,
            soft_joint_pos_limit_factor=1.0,
        )
        version = f"{model['source']}-{SOURCES[model['source']]['revision']}:isaac-import-v{preparation_version(name)}"
    cfg.spawn.func = spawn_robot
    cfg.init_state.pos = arm.base_pose[:3]
    cfg.init_state.rot = arm.base_pose[3:]
    opened = np.array([high for _, high in arm.gripper.command_limits])
    fingers = np.asarray(arm.gripper.joint_map) @ opened + arm.gripper.joint_offset
    cfg.init_state.joint_pos = {
        **dict(zip(arm.joint_names, arm.initial_positions)),
        **dict(zip(arm.gripper.joint_names, fingers)),
    }
    return cfg, version


class DualArmArticulation:
    """Map native joints to the deployment protocol without padding or clipping."""

    def __init__(
        self, deployment: DeploymentSpec, asset_root: str | Path, *, robots=None
    ):
        self.deployment = deployment
        self.robots = {}
        self.joint_ids, self.gripper_ids, self.tcp_ids = {}, {}, {}
        self.asset_versions = {}
        self.usd_paths = {}
        self.controller_parameters = {}
        self.model_checks = {}
        self.asset_manifests = {}
        for asset in dict.fromkeys(arm.asset for arm in deployment.arms.values()):
            if asset != PANDA_ASSET:
                self.asset_manifests[asset] = verify_asset(
                    asset_root, model_name(asset)
                )
        for side in ARMS:
            arm = deployment.arms[side]
            cfg, version = articulation_config(arm, asset_root)
            cfg.prim_path = f"/World/{side}_robot"
            self.robots[side] = Articulation(cfg) if robots is None else robots[side]
            cfg = self.robots[side].cfg
            # URDF importer 3.0 nests rigid links. Lab's spawn helper stops at
            # the first rigid body, so apply the same properties to every link.
            prim = get_current_stage().GetPrimAtPath(
                sim_utils.find_matching_prim_paths(cfg.prim_path)[0]
            )
            for body in Usd.PrimRange(prim):
                if body.HasAPI(UsdPhysics.RigidBodyAPI):
                    sim_utils.modify_rigid_body_properties(
                        str(body.GetPath()), cfg.spawn.rigid_props
                    )
            groups = (
                ()
                if arm.asset == PANDA_ASSET
                else MODELS[model_name(arm.asset)].get("collision_groups", ())
            )
            for group in groups:
                bodies = {
                    body.GetName(): body
                    for body in Usd.PrimRange(prim)
                    if body.HasAPI(UsdPhysics.RigidBodyAPI) and body.GetName() in group
                }
                if set(bodies) != set(group):
                    raise ValueError(
                        f"Missing collision-filter bodies: {set(group) - set(bodies)}"
                    )
                for body in bodies.values():
                    relation = UsdPhysics.FilteredPairsAPI.Apply(
                        body
                    ).CreateFilteredPairsRel()
                    for other in bodies.values():
                        if other != body:
                            relation.AddTarget(other.GetPath())
            self.asset_versions[arm.asset] = version
            self.usd_paths[side] = cfg.spawn.usd_path
            self.controller_parameters[side] = {
                name: {
                    "joint_names": actuator.joint_names_expr,
                    "stiffness": actuator.stiffness,
                    "damping": actuator.damping,
                    "armature": actuator.armature,
                    "effort_limit": actuator.effort_limit_sim,
                    "velocity_limit": actuator.velocity_limit_sim,
                }
                for name, actuator in cfg.actuators.items()
            }

    def initialize(self):
        """Validate the actual imported model after the simulator's first reset."""
        for side, robot in self.robots.items():
            arm = self.deployment.arms[side]
            root = get_current_stage().GetPrimAtPath(
                sim_utils.find_matching_prim_paths(robot.cfg.prim_path)[0]
            )
            colliders = sum(
                prim.HasAPI(UsdPhysics.CollisionAPI)
                for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
            )
            if not robot.is_fixed_base or colliders == 0:
                raise ValueError(f"{side} requires a fixed base and collision geometry")
            self.model_checks[side] = {
                "fixed_base": robot.is_fixed_base,
                "collision_shapes": colliders,
                "body_count": robot.num_bodies,
                "visual_meshes_per_body": validate_visuals(root),
                "joint_count": robot.num_joints,
                "collision_exclusion_groups": []
                if arm.asset == PANDA_ASSET
                else MODELS[model_name(arm.asset)].get("collision_groups", ()),
            }
            arm_ids, arm_names = robot.find_joints(arm.joint_names, preserve_order=True)
            finger_ids, finger_names = robot.find_joints(
                arm.gripper.joint_names, preserve_order=True
            )
            body_ids, _ = robot.find_bodies(arm.tcp_frame, preserve_order=True)
            if (
                tuple(arm_names) != arm.joint_names
                or tuple(finger_names) != arm.gripper.joint_names
                or len(body_ids) != 1
                or set(arm_ids + finger_ids) != set(range(robot.num_joints))
            ):
                raise ValueError(f"Unexpected {side} joint/gripper/TCP mapping")
            actual_limits = robot.data.joint_pos_limits.torch[0].cpu().numpy()
            if not np.allclose(actual_limits[arm_ids], arm.joint_limits, atol=1e-4):
                raise ValueError(
                    f"{side} USD joint limits disagree with the deployment"
                )
            # Extremes of the affine map, including negative mimic coefficients.
            matrix = np.asarray(arm.gripper.joint_map)
            bounds = np.asarray(arm.gripper.command_limits)
            a, b = matrix * bounds[:, 0], matrix * bounds[:, 1]
            low = np.minimum(a, b).sum(axis=1) + arm.gripper.joint_offset
            high = np.maximum(a, b).sum(axis=1) + arm.gripper.joint_offset
            if np.any(low < actual_limits[finger_ids, 0] - 1e-5) or np.any(
                high > actual_limits[finger_ids, 1] + 1e-5
            ):
                raise ValueError(f"{side} gripper commands exceed USD joint limits")
            self.joint_ids[side] = arm_ids
            self.gripper_ids[side] = finger_ids
            self.tcp_ids[side] = body_ids[0]

    def reset(self, command):
        """Write measured state, zero velocities and controller targets explicitly."""
        command = self.deployment.validate_action(command)
        for side, robot in self.robots.items():
            robot.reset()
            q = robot.data.default_joint_pos.torch.clone()
            q[:, self.joint_ids[side]] = self._tensor(
                robot, command[self.deployment.action_slices[f"{side}/arm"]]
            )
            q[:, self.gripper_ids[side]] = self._tensor(
                robot, self._gripper_target(side, command)
            )
            velocity = torch.zeros_like(q)
            robot.write_joint_state_to_sim_index(position=q, velocity=velocity)
            robot.set_joint_velocity_target_index(target=velocity)
        self.submit(command)

    @staticmethod
    def _tensor(robot, values):
        return torch.tensor(values[None], dtype=torch.float32, device=robot.device)

    def _gripper_target(self, side, command):
        grip = self.deployment.arms[side].gripper
        return (
            np.asarray(grip.joint_map)
            @ command[self.deployment.action_slices[f"{side}/gripper"]]
            + grip.joint_offset
        )

    def submit(self, command):
        # Validate both sides before changing either robot's targets.
        command = self.deployment.validate_action(command)
        for side, robot in self.robots.items():
            robot.set_joint_position_target_index(
                target=self._tensor(
                    robot, command[self.deployment.action_slices[f"{side}/arm"]]
                ),
                joint_ids=self.joint_ids[side],
            )
            robot.set_joint_position_target_index(
                target=self._tensor(robot, self._gripper_target(side, command)),
                joint_ids=self.gripper_ids[side],
            )

    def write_data_to_sim(self):
        for robot in self.robots.values():
            robot.write_data_to_sim()

    def update(self, dt):
        for robot in self.robots.values():
            robot.update(dt)

    def observe(self):
        values = {}
        for side, robot in self.robots.items():
            q = robot.data.joint_pos.torch[0].cpu().numpy()
            dq = robot.data.joint_vel.torch[0].cpu().numpy()
            for key, value in (
                ("joint_position", q[self.joint_ids[side]]),
                ("joint_velocity", dq[self.joint_ids[side]]),
                ("gripper_position", q[self.gripper_ids[side]]),
                ("gripper_velocity", dq[self.gripper_ids[side]]),
                (
                    "tcp_pose_world",
                    robot.data.body_link_pose_w.torch[0, self.tcp_ids[side]]
                    .cpu()
                    .numpy(),
                ),
            ):
                values[f"robot/{side}/{key}"] = value.copy()
        return values
