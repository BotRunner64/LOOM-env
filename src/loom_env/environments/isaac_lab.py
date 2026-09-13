"""Asset-based ManagerBasedEnv with an explicit LOOM Runner boundary.

Import only after AppLauncher. Native managers own control, observations and
step hooks; EpisodeWriter remains the only on-disk recording implementation.
"""

import importlib.metadata
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch
from pxr import Usd, UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import (
    ActionTerm,
    ActionTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RecorderTerm,
    RecorderTermCfg,
)
from isaaclab.managers.recorder_manager import RecorderManagerBaseCfg, DatasetExportMode
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab_physx.sensors import ContactSensorCfg

from loom_env.embodiments.cameras import CameraMounts, camera_config
from loom_env.embodiments.manipulation import manipulation_profile
from loom_env.embodiments.isaac_lab import DualArmArticulation, articulation_config
from loom_env.assets.catalog import (
    asset_definition,
    load_prepared,
    prepared_directory,
    sha256,
)
from loom_env.embodiments.commands import gripper_command, initial_command
from loom_env.embodiments.contacts import opposing_contacts
from loom_env.scenes.isaac_lab import instance_config, simulation_config
from loom_env.scenes.workspace import (
    sample_objects,
    dynamic_names,
    transform,
    workspace,
    corners,
)
from loom_env.specs.config import ARMS, EpisodeSpec, plain
from loom_env.specs.episode import Frame, Observation, Transition, observation_shapes


def _tree_map(value, func):
    return {
        k: _tree_map(v, func) if isinstance(v, dict) else func(v)
        for k, v in value.items()
    }


class DualArmPositionAction(ActionTerm):
    """Absolute native joint targets for both arms and their affine grippers."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._actions = torch.tensor(
            initial_command(env.collection.deployment)[None],
            device=env.device,
            dtype=torch.float32,
        )

    @property
    def action_dim(self):
        return self._env.collection.deployment.action_dim

    @property
    def raw_actions(self):
        return self._actions

    @property
    def processed_actions(self):
        return self._actions

    def process_actions(self, actions):
        self._env.collection.deployment.validate_action(actions[0].cpu().numpy())
        self._actions.copy_(actions)

    def apply_actions(self):
        self._env.robot.submit(self._actions[0].cpu().numpy())


@configclass
class PositionActionsCfg:
    position: ActionTermCfg = ActionTermCfg(
        class_type=DualArmPositionAction, asset_name="left"
    )


class TransitionRecorder(RecorderTerm):
    """Capture one owned transition at native pre/post-step hooks, without I/O."""

    def record_pre_step(self):
        self._env.applied_control = (
            self._env.action_manager.get_term("position")
            .processed_actions[0]
            .cpu()
            .numpy()
            .copy()
        )
        return None, None

    def record_post_step(self):
        self._env.recorded_frame = self._env.frame()
        return None, None


def policy_signal(env, key):
    if key.startswith("robot/"):
        _, side, field = key.split("/")
        data = env.robot.robots[side].data
        if field == "tcp_pose_world":
            return data.body_link_pose_w.torch[:, env.robot.tcp_ids[side]]
        ids = (
            env.robot.gripper_ids[side]
            if field.startswith("gripper")
            else env.robot.joint_ids[side]
        )
        value = data.joint_pos if field.endswith("position") else data.joint_vel
        return value.torch[:, ids]
    _, name, field = key.split("/")
    camera_spec = next(c for c in env.collection.deployment.cameras if c.name == name)
    capture_step = (
        env.control_step // camera_spec.period_steps * camera_spec.period_steps
    )
    if field == "rgb":
        if env.camera_sync_step != env.control_step:
            env.camera_mounts.update(env.sim, warmup=env.control_step == 0)
            env.camera_sync_step = env.control_step
        camera = env.scene[f"camera_{name}"]
        if env.camera_steps.get(name) != capture_step:
            camera.update(env.step_dt, force_recompute=True)
            env.camera_images[name] = camera.data.output["rgb"].torch[..., :3].clone()
            env.camera_poses[name] = np.r_[
                camera.data.pos_w.torch[0].cpu().numpy(),
                camera.data.quat_w_opengl.torch[0].cpu().numpy(),
            ]
            env.camera_steps[name] = capture_step
        return env.camera_images[name]
    if field == "valid":
        return torch.ones(1, device=env.device, dtype=torch.bool)
    return torch.tensor(
        [capture_step * env.step_dt], device=env.device, dtype=torch.float64
    )


def environment_config(collection, asset_root):
    candidates = sample_objects(collection.scene, 0)
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=3.0, replicate_physics=False)
    prefix = "{ENV_REGEX_NS}"
    contact_paths = {}
    for side in ARMS:
        arm = collection.deployment.arms[side]
        profile = manipulation_profile(arm)
        cfg, _ = articulation_config(arm, asset_root)
        if arm.asset not in contact_paths:
            stage = Usd.Stage.Open(cfg.spawn.usd_path)
            root = stage.GetDefaultPrim()
            paths = {}
            for prim in Usd.PrimRange(root):
                if prim.GetName() in profile.finger_bodies and prim.HasAPI(
                    UsdPhysics.RigidBodyAPI
                ):
                    if prim.GetName() in paths:
                        raise ValueError("Ambiguous contact body name in robot asset")
                    paths[prim.GetName()] = str(
                        prim.GetPath().MakeRelativePath(root.GetPath())
                    )
            if set(paths) != set(profile.finger_bodies):
                raise ValueError("Manipulation contact bodies missing from robot asset")
            contact_paths[arm.asset] = [paths[name] for name in profile.finger_bodies]
        cfg.prim_path = f"{prefix}/{side}_robot"
        cfg.spawn.activate_contact_sensors = True
        setattr(scene, side, cfg)
        for finger, path in zip(("left", "right"), contact_paths[arm.asset]):
            setattr(
                scene,
                f"contact_{side}_{finger}",
                ContactSensorCfg(
                    prim_path=f"{prefix}/{side}_robot/{path}",
                    filter_prim_paths_expr=[
                        f"{prefix}/object_{name}"
                        for name in dynamic_names(collection.scene)
                    ],
                    max_contact_data_count_per_prim=64,
                ),
            )
    for name, candidate in candidates.items():
        setattr(
            scene,
            f"object_{name}",
            instance_config(
                collection.scene.objects[name],
                f"{prefix}/object_{name}",
                candidate,
                asset_root,
            ),
        )
    scene.light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=600)
    )
    scene.ground = AssetBaseCfg(
        prim_path="/World/Ground", spawn=sim_utils.GroundPlaneCfg()
    )
    for camera in collection.deployment.cameras:
        setattr(
            scene,
            f"camera_{camera.name}",
            camera_config(camera),
        )
    observations = ObservationGroupCfg(concatenate_terms=False, enable_corruption=False)
    for key in observation_shapes(collection.deployment):
        setattr(
            observations,
            key,
            ObservationTermCfg(func=policy_signal, params={"key": key}),
        )
    recorders = RecorderManagerBaseCfg(
        dataset_export_mode=DatasetExportMode.EXPORT_NONE,
        export_in_record_pre_reset=False,
    )
    recorders.transition = RecorderTermCfg(class_type=TransitionRecorder)
    return ManagerBasedEnvCfg(
        scene=scene,
        decimation=collection.deployment.decimation,
        sim=simulation_config(collection.deployment),
        actions=PositionActionsCfg(),
        observations={"policy": observations},
        recorders=recorders,
        seed=0,
        ui_window_class_type=None,
        num_rerenders_on_reset=2,
    )


class ManipulationEnvironment(ManagerBasedEnv):
    """One physical scene, no automatic episode termination or reset."""

    def __init__(self, collection, asset_root=Path(".cache/assets")):
        self.collection = collection
        self.asset_root = asset_root
        self.object_names = dynamic_names(collection.scene)
        self.instance_poses = sample_objects(collection.scene, 0)
        self.asset_manifests = {
            obj["asset"]: load_prepared(asset_root, obj["asset"])
            for obj in collection.scene.objects.values()
        }
        self.scene_asset_versions = {
            key: sha256(prepared_directory(asset_root, key) / "asset.json")
            for key in self.asset_manifests
        }
        self.control_step = 0
        self.camera_steps, self.camera_images, self.camera_poses = {}, {}, {}
        self.camera_sync_step = None
        self.recorded_frame = None
        super().__init__(environment_config(collection, asset_root))
        # Cold PhysX/Fabric initialization needs one actual step before reset_to
        # can refresh articulation visuals. This is outside every Episode;
        # reset_episode restores the complete recorded state afterwards.
        self.robot.reset(initial_command(collection.deployment))
        self.scene.write_data_to_sim()
        self.sim.step(render=False)
        self.scene.update(self.physics_dt)

    def load_managers(self):
        self.robot = DualArmArticulation(
            self.collection.deployment,
            self.asset_root,
            robots={side: self.scene[side] for side in ARMS},
        )
        self.robot.initialize()
        self.camera_mounts = CameraMounts(
            self.collection.deployment,
            {
                c.name: self.scene[f"camera_{c.name}"]
                for c in self.collection.deployment.cameras
            },
            self.robot.robots,
            self.asset_root,
        )
        super().load_managers()

    def world_state(self):
        world = {}
        for index, name in enumerate(self.object_names):
            data = self.scene[f"object_{name}"].data
            pose = data.root_pose_w.torch[0].cpu().numpy().copy()
            velocity = data.root_vel_w.torch[0].cpu().numpy().copy()
            contacts, grasped = [], []
            for side in ARMS:
                forces = np.stack(
                    [
                        self.scene[f"contact_{side}_{finger}"]
                        .data.force_matrix_w.torch[0, 0, index]
                        .cpu()
                        .numpy()
                        for finger in ("left", "right")
                    ]
                )
                robot_data = self.robot.robots[side].data
                fingers = (
                    robot_data.joint_pos.torch[0, self.robot.gripper_ids[side]]
                    .cpu()
                    .numpy()
                )
                contacts.append(forces)
                gripper = self.collection.deployment.arms[side].gripper
                grasped.append(
                    opposing_contacts(
                        forces,
                        gripper_command(gripper, fingers),
                        gripper.command_limits[0],
                    )
                )
            world.update(
                {
                    f"{name}/pose_world": pose,
                    f"{name}/velocity_world": velocity,
                    f"{name}/grasped_by": np.asarray(grasped, dtype=bool),
                    f"{name}/finger_contact_forces_world": np.asarray(contacts),
                }
            )
        for name, obj in self.collection.scene.objects.items():
            if obj["static"]:
                world[f"{name}/pose_world"] = self.instance_poses[name].copy()
            asset = asset_definition(obj["asset"])
            if asset.interior is not None:
                world[f"{name}/region_pose_world"] = transform(
                    world[f"{name}/pose_world"], [*asset.interior[0], 0, 0, 0, 1]
                )
        world.update(
            {
                f"cameras/{camera.name}/pose_world": self.camera_poses[camera.name]
                for camera in self.collection.deployment.cameras
            }
        )
        return world

    def frame(self):
        values = {
            key: value[0].cpu().numpy() for key, value in self.obs_buf["policy"].items()
        }
        return Frame(
            Observation(self.control_step * self.step_dt, values), self.world_state()
        )

    def step(self, action):
        command = self.collection.deployment.validate_action(action)
        self.control_step += 1
        self.recorded_frame = None
        super().step(
            torch.tensor(command[None], device=self.device, dtype=torch.float32)
        )
        if self.recorded_frame is None:
            raise RuntimeError(
                "Native post-step recorder did not capture the transition"
            )
        return Transition(self.recorded_frame, self.applied_control.copy())

    def reset_episode(self, spec):
        for field in ("deployment", "scene"):
            if plain(getattr(spec.collection, field)) != plain(
                getattr(self.collection, field)
            ):
                raise ValueError(f"Episode {field} differs from the instantiated scene")
        for asset_id, version in self.scene_asset_versions.items():
            if spec.asset_versions[asset_id] != version:
                raise ValueError(f"Episode asset version differs: {asset_id}")
        if set(spec.initial_state) != {"scene", "control_targets"}:
            raise ValueError("Expected a measured scene snapshot and control targets")
        self.control_step = self._sim_step_counter = 0
        self.camera_steps.clear()
        self.camera_sync_step = None
        state = _tree_map(
            plain(spec.initial_state["scene"]),
            lambda v: torch.tensor(v, device=self.device, dtype=torch.float32),
        )
        self.reset_to(state, env_ids=None, seed=spec.seed)
        command = np.array(spec.initial_state["control_targets"])
        self.action_manager.process_action(
            torch.tensor(command[None], device=self.device, dtype=torch.float32)
        )
        self.robot.submit(command)
        for robot in self.robot.robots.values():
            robot.set_joint_velocity_target_index(
                target=torch.zeros_like(robot.data.joint_vel.torch)
            )
        self.scene.write_data_to_sim()
        return self.frame()

    def resolve_episode(self, episode_id, seed, *, provenance=None):
        """Validate initial robot/support positions, then freeze measured state."""
        self.control_step = self._sim_step_counter = 0
        self.camera_steps.clear()
        self.camera_sync_step = None
        self.reset(seed=seed)
        command = initial_command(self.collection.deployment)
        self.robot.reset(command)
        candidates = sample_objects(self.collection.scene, seed)
        for name in self.object_names:
            candidate = candidates[name]
            obj = self.scene[f"object_{name}"]
            obj.write_root_pose_to_sim_index(
                root_pose=torch.tensor(
                    candidate[None],
                    device=self.device,
                    dtype=torch.float32,
                )
            )
            obj.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros((1, 6), device=self.device)
            )
        support, _ = workspace(self.collection.scene)
        ready_steps = 0
        for _ in range(round(5.0 / self.step_dt)):
            frame = self.step(command).frame
            joint_errors = {
                side: float(
                    np.max(
                        np.abs(
                            frame.observation.values[f"robot/{side}/joint_position"]
                            - self.collection.deployment.arms[side].initial_positions
                        )
                    )
                )
                for side in ARMS
            }
            support_errors = {}
            for name in self.object_names:
                measured = frame.world_state[f"{name}/pose_world"]
                asset = asset_definition(self.collection.scene.objects[name]["asset"])
                bottom = (
                    Rotation.from_quat(measured[3:]).apply(corners(asset))
                    + measured[:3]
                )[:, 2].min()
                support_errors[name] = {
                    "xy_m": float(np.linalg.norm(measured[:2] - candidates[name][:2])),
                    "height_m": float(abs(bottom - support[2])),
                }
            ready = all(error < 0.003 for error in joint_errors.values()) and all(
                error["xy_m"] <= 0.005 and error["height_m"] <= 0.003
                for error in support_errors.values()
            )
            ready_steps = ready_steps + 1 if ready else 0
            if ready_steps * self.step_dt >= 0.25:
                break
        else:
            raise RuntimeError(
                "Candidate did not reach its initial robot/support positions within "
                f"five seconds: joint_errors_rad={joint_errors}, "
                f"support_errors={support_errors}"
            )
        state = _tree_map(self.scene.get_state(), lambda v: v.cpu().tolist())
        return EpisodeSpec(
            id=episode_id,
            collection=self.collection,
            seed=seed,
            initial_state={"scene": state, "control_targets": command.tolist()},
            sampled_parameters={
                "object_candidates": {
                    name: value.tolist() for name, value in candidates.items()
                },
                "initialization_steps": self.control_step,
                "controllers": self.robot.controller_parameters,
                "manipulation_profiles": {
                    side: plain(manipulation_profile(arm))
                    for side, arm in self.collection.deployment.arms.items()
                },
                "gravity_compensation": True,
                "external_forces_every_iteration": self.cfg.sim.physics.enable_external_forces_every_iteration,
                "camera_mounts": self.camera_mounts.resolved,
                "camera_intrinsics": {
                    camera.name: self.scene[f"camera_{camera.name}"]
                    .data.intrinsic_matrices.torch[0]
                    .cpu()
                    .tolist()
                    for camera in self.collection.deployment.cameras
                },
                "rendering": {
                    "dome_light_intensity": self.scene.cfg.light.spawn.intensity,
                    "ambient_light_intensity": self.cfg.sim.render.ambient_light_intensity,
                },
            },
            asset_versions={
                **self.robot.asset_versions,
                **self.scene_asset_versions,
            },
            runtime_versions={
                name: importlib.metadata.version(name)
                for name in (
                    "loom-env",
                    "isaaclab",
                    "isaaclab-physx",
                    "isaaclab-assets",
                    "isaacsim",
                    "torch",
                    "nvidia-curobo",
                )
            },
            provenance={
                "source": "simulation",
                "runtime": "ManagerBasedEnv",
                "physics_backend": "PhysX",
                "resolved_robot_usd": self.robot.usd_paths,
                "model_checks": self.robot.model_checks,
                "scene_assets": self.asset_manifests,
                **(provenance or {}),
            },
        )
