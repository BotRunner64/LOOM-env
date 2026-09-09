"""Single-table ManagerBasedEnv with an explicit LOOM Runner boundary.

Import only after AppLauncher. Native managers own control, observations and
step hooks; EpisodeWriter remains the only on-disk recording implementation.
"""

import importlib.metadata
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
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
from isaaclab.sensors import CameraCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.renderers import IsaacRtxRendererCfg
from isaaclab_physx.sensors import ContactSensorCfg

from loom_env.embodiments.assets import PANDA_ASSET
from loom_env.embodiments.isaac_lab import DualArmArticulation, articulation_config
from loom_env.environments.tabletop import (
    grasp_evidence,
    initial_command,
    sample_cube,
    tabletop_geometry,
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
        camera = env.scene[f"camera_{name}"]
        if env.camera_steps.get(name) != capture_step:
            camera.update(env.step_dt, force_recompute=True)
            env.camera_images[name] = camera.data.output["rgb"].torch[..., :3].clone()
            env.camera_steps[name] = capture_step
        return env.camera_images[name]
    if field == "valid":
        return torch.ones(1, device=env.device, dtype=torch.bool)
    return torch.tensor(
        [capture_step * env.step_dt], device=env.device, dtype=torch.float64
    )


def environment_config(collection, asset_root):
    if any(arm.asset != PANDA_ASSET for arm in collection.deployment.arms.values()):
        raise ValueError(
            "The first contact-based place environment supports dual Panda"
        )
    if collection.task.id != "put_cube_in_container":
        raise ValueError("Tabletop environment requires put_cube_in_container")
    geometry = tabletop_geometry(collection)
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=3.0, replicate_physics=False)
    prefix = "{ENV_REGEX_NS}"
    for side in ARMS:
        cfg, _ = articulation_config(collection.deployment.arms[side], asset_root)
        cfg.prim_path = f"{prefix}/{side}_robot"
        cfg.spawn.activate_contact_sensors = True
        setattr(scene, side, cfg)
        for finger in ("left", "right"):
            setattr(
                scene,
                f"contact_{side}_{finger}",
                ContactSensorCfg(
                    prim_path=f"{prefix}/{side}_robot/panda_{finger}finger",
                    filter_prim_paths_expr=[f"{prefix}/cube"],
                    max_contact_data_count_per_prim=64,
                ),
            )
    for name, box in geometry.items():
        setattr(
            scene,
            name,
            AssetBaseCfg(
                prim_path=f"{prefix}/{name}",
                spawn=sim_utils.CuboidCfg(
                    size=tuple(box["size"]),
                    collision_props=sim_utils.CollisionPropertiesCfg(
                        contact_offset=0.002, rest_offset=0.0
                    ),
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.43, 0.47, 0.51)
                        if name == "table"
                        else (0.16, 0.40, 0.30)
                    ),
                ),
                init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(box["position"])),
            ),
        )
    scene.cube = RigidObjectCfg(
        prim_path=f"{prefix}/cube",
        spawn=sim_utils.CuboidCfg(
            size=tuple(collection.task.parameters["object_size"]),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
                max_depenetration_velocity=1.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.002, rest_offset=0.0
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=1.0, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.92, 0.16, 0.10)
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=tuple(sample_cube(collection, 0))
        ),
    )
    scene.light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=600)
    )
    scene.ground = AssetBaseCfg(
        prim_path="/World/Ground", spawn=sim_utils.GroundPlaneCfg()
    )
    for camera in collection.deployment.cameras:
        if camera.parent_frame != "world":
            raise ValueError("Tabletop cameras currently require parent_frame: world")
        setattr(
            scene,
            f"camera_{camera.name}",
            CameraCfg(
                prim_path=f"/World/Camera_{camera.name}",
                width=camera.width,
                height=camera.height,
                offset=CameraCfg.OffsetCfg(
                    pos=camera.pose[:3], rot=camera.pose[3:], convention="opengl"
                ),
                spawn=sim_utils.PinholeCameraCfg(
                    focal_length=22, horizontal_aperture=24, clipping_range=(0.05, 20)
                ),
                data_types=["rgb"],
                renderer_cfg=IsaacRtxRendererCfg(),
            ),
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
        sim=sim_utils.SimulationCfg(
            dt=collection.deployment.physics_dt,
            render_interval=collection.deployment.decimation,
            device="cuda:0",
            physics=PhysxCfg(),
            render=sim_utils.RenderCfg(ambient_light_intensity=0.3),
        ),
        actions=PositionActionsCfg(),
        observations={"policy": observations},
        recorders=recorders,
        seed=0,
        ui_window_class_type=None,
        num_rerenders_on_reset=2,
    )


class TabletopEnvironment(ManagerBasedEnv):
    """One physical scene, no automatic episode termination or reset."""

    def __init__(self, collection, asset_root=Path(".cache/assets")):
        self.collection = collection
        self.asset_root = asset_root
        self.control_step = 0
        self.camera_steps, self.camera_images = {}, {}
        self.recorded_frame = None
        super().__init__(environment_config(collection, asset_root))

    def load_managers(self):
        self.robot = DualArmArticulation(
            self.collection.deployment,
            self.asset_root,
            robots={side: self.scene[side] for side in ARMS},
        )
        self.robot.initialize()
        super().load_managers()

    def world_state(self):
        pose = self.scene["cube"].data.root_pose_w.torch[0].cpu().numpy().copy()
        velocity = self.scene["cube"].data.root_vel_w.torch[0].cpu().numpy().copy()
        contacts, grasped = [], []
        for side in ARMS:
            forces = np.stack(
                [
                    self.scene[f"contact_{side}_{finger}"]
                    .data.force_matrix_w.torch[0, 0, 0]
                    .cpu()
                    .numpy()
                    for finger in ("left", "right")
                ]
            )
            data = self.robot.robots[side].data
            hand = (
                data.body_link_pose_w.torch[0, self.robot.tcp_ids[side]].cpu().numpy()
            )
            local = Rotation.from_quat(hand[3:]).inv().apply(pose[:3] - hand[:3])
            fingers = (
                data.joint_pos.torch[0, self.robot.gripper_ids[side]].cpu().numpy()
            )
            contacts.append(forces)
            grasped.append(
                grasp_evidence(
                    forces,
                    fingers,
                    local,
                    self.collection.task.parameters["object_size"],
                )
            )
        obj = self.collection.role_bindings["target_object"]
        receptacle = self.collection.role_bindings["container"]
        return {
            f"{obj}/pose_world": pose,
            f"{obj}/velocity_world": velocity,
            f"{obj}/grasped_by": np.asarray(grasped, dtype=bool),
            f"{obj}/finger_contact_forces_world": np.asarray(contacts),
            f"{receptacle}/region_pose_world": np.r_[
                self.collection.scene.parameters["container_position"],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }

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
        for field in ("task", "deployment", "scene", "role_bindings"):
            if plain(getattr(spec.collection, field)) != plain(
                getattr(self.collection, field)
            ):
                raise ValueError(f"Episode {field} differs from the instantiated scene")
        if set(spec.initial_state) != {"scene", "control_targets"}:
            raise ValueError(
                "Expected a measured tabletop scene snapshot and control targets"
            )
        self.control_step = self._sim_step_counter = 0
        self.camera_steps.clear()
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
        """Settle a candidate, validate it, then freeze measured simulator state."""
        self.control_step = self._sim_step_counter = 0
        self.camera_steps.clear()
        self.reset(seed=seed)
        command = initial_command(self.collection.deployment)
        self.robot.reset(command)
        candidate = sample_cube(self.collection, seed)
        self.scene["cube"].write_root_pose_to_sim_index(
            root_pose=torch.tensor(
                np.r_[candidate, [0, 0, 0, 1]][None],
                device=self.device,
                dtype=torch.float32,
            )
        )
        self.scene["cube"].write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), device=self.device)
        )
        stable = 0
        for _ in range(100):
            frame = self.step(command).frame
            robot_stable = all(
                np.max(
                    np.abs(
                        frame.observation.values[f"robot/{side}/joint_position"]
                        - self.collection.deployment.arms[side].initial_positions
                    )
                )
                < 0.003
                and np.max(
                    np.abs(frame.observation.values[f"robot/{side}/joint_velocity"])
                )
                < 0.01
                for side in ARMS
            )
            obj = self.collection.role_bindings["target_object"]
            cube_stable = (
                np.linalg.norm(frame.world_state[f"{obj}/velocity_world"]) < 0.01
            )
            stable = stable + 1 if robot_stable and cube_stable else 0
            if stable >= 5:
                break
        else:
            raise RuntimeError("Candidate failed to settle within five seconds")
        if (
            np.linalg.norm(frame.world_state[f"{obj}/pose_world"][:2] - candidate[:2])
            > 0.005
            or abs(
                frame.world_state[f"{obj}/pose_world"][2]
                - self.collection.scene.parameters["table_height"]
                - self.collection.task.parameters["object_size"][2] / 2
            )
            > 0.002
        ):
            raise RuntimeError("Candidate moved outside its accepted support pose")
        state = _tree_map(self.scene.get_state(), lambda v: v.cpu().tolist())
        return EpisodeSpec(
            id=episode_id,
            collection=self.collection,
            seed=seed,
            initial_state={"scene": state, "control_targets": command.tolist()},
            sampled_parameters={
                "cube_candidate": candidate.tolist(),
                "settling_steps": self.control_step,
                "controllers": self.robot.controller_parameters,
                "gravity_compensation": True,
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
                "primitive:cube": "tabletop-v1",
                "primitive:open_box": "tabletop-v1",
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
                **(provenance or {}),
            },
        )
