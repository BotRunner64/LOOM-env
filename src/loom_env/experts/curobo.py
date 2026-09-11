"""cuRobo 0.8 pose planning in the selected Panda's base frame."""

from itertools import product
from copy import copy
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from curobo._src.collision.attachment_manager import AttachmentManager
from curobo.config_io import load_yaml
from curobo.content import get_robot_configs_path
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.scene import Cuboid, Mesh, Scene
from curobo.types import GoalToolPose, JointState, Pose

from loom_env.assets.catalog import asset_definition, collision_mesh
from loom_env.runtime.runner import SourceFailure


class PandaPlanner:
    def __init__(self, deployment, scene, side, target_object, asset_root):
        self.deployment, self.scene, self.side = deployment, scene, side
        self.target_object = target_object
        self.arm = deployment.arms[side]
        self.meshes = {}
        for name, obj in scene.objects.items():
            vertices, faces = collision_mesh(asset_root, obj["asset"])
            self.meshes[name] = Mesh(
                name=name,
                vertices=vertices.tolist(),
                faces=faces.tolist(),
                pose=[0, 0, 0, 1, 0, 0, 0],
            )
        self.base = np.asarray(self.arm.base_pose)
        self.rotation = Rotation.from_quat(self.base[3:])
        robot = load_yaml(str(Path(get_robot_configs_path()) / "franka.yml"))
        robot["robot_cfg"]["kinematics"]["extra_collision_spheres"] = {
            "attached_object": 8
        }
        cfg = MotionPlannerCfg.create(
            robot=robot,
            collision_cache={"obb": 128, "mesh": 16},
            num_ik_seeds=32,
            num_trajopt_seeds=4,
            random_seed=0,
            position_tolerance=0.002,
            orientation_tolerance=0.025,
            optimizer_collision_activation_distance=0.005,
        )
        # Execute each 25 ms planned sample over a 50 ms control interval.
        # Time scaling slows the complete collision-checked path uniformly.
        cfg.trajopt_solver_config.interpolation_dt = deployment.control_dt / 2
        self.planner = MotionPlanner(cfg)
        # The pinned 0.8 high-level property refers to a missing solver field.
        # Instantiate its native manager against the shared kinematics params.
        self.attachment = AttachmentManager(self.planner.kinematics)
        if (
            self.planner.ik_solver.kinematics.config.kinematics_config
            is not self.planner.kinematics.config.kinematics_config
        ):
            raise RuntimeError("cuRobo solvers must share attachment geometry")
        self.holding_model = Kinematics(
            KinematicsCfg.from_robot_yaml_file("franka.yml", tool_frames=["panda_hand"])
        )
        self.attached = False

    def _state(self, q):
        return JointState.from_position(
            torch.tensor(np.asarray(q)[None], device="cuda:0", dtype=torch.float32),
            joint_names=list(self.arm.joint_names),
        )

    def _local_pose(self, world):
        position = self.rotation.inv().apply(np.asarray(world[:3]) - self.base[:3])
        rotation = self.rotation.inv() * Rotation.from_quat(world[3:])
        return np.r_[position, rotation.as_quat()]

    def _box(self, name, size, world_pose):
        local = self._local_pose(world_pose)
        return Cuboid(
            name=name, dims=list(size), pose=local[[0, 1, 2, 6, 3, 4, 5]].tolist()
        )

    def update_world(self, observation, truth, *, allow_object_contact):
        boxes = []
        other = "left" if self.side == "right" else "right"
        arm = self.deployment.arms[other]
        held = self.holding_model.compute_kinematics(
            self._state(observation.values[f"robot/{other}/joint_position"])
        )
        spheres = held.robot_spheres.detach().cpu().numpy().reshape(-1, 4)
        rotation = Rotation.from_quat(arm.base_pose[3:])
        for i, sphere in enumerate(spheres):
            if sphere[3] <= 0:
                continue
            center = rotation.apply(sphere[:3]) + arm.base_pose[:3]
            # Conservative boxes enclosing the held arm's collision spheres.
            boxes.append(
                self._box(f"held_{i}", [2 * sphere[3]] * 3, np.r_[center, [0, 0, 0, 1]])
            )
        meshes = []
        for name in self.scene.objects:
            if name == self.target_object and (allow_object_contact or self.attached):
                continue
            mesh = copy(self.meshes[name])
            local = self._local_pose(truth[f"{name}/pose_world"])
            mesh.pose = local[[0, 1, 2, 6, 3, 4, 5]].tolist()
            meshes.append(mesh)
        self.planner.update_world(Scene(cuboid=boxes, mesh=meshes))
        return len(boxes) + len(meshes)

    def attach(self, observation, truth):
        """Planning geometry only; the simulator object remains a free rigid body."""
        obj = self.target_object
        low, high = np.asarray(
            asset_definition(self.scene.objects[obj]["asset"]).bounds
        )
        size = high - low
        centers = (low + high) / 2 + np.asarray(
            list(product((-1, 1), repeat=3))
        ) * size / 4
        # Each sphere covers one octant of the object envelope, including its corners.
        spheres = np.c_[centers, np.full(8, np.linalg.norm(size / 4))]
        self.attachment.update(
            torch.tensor(spheres, device="cuda:0", dtype=torch.float32),
            self._state(observation.values[f"robot/{self.side}/joint_position"]),
            world_objects_pose_offset=Pose.from_list(
                self._local_pose(truth[f"{obj}/pose_world"]).tolist(), q_xyzw=True
            ),
        )
        self.attached = True

    def detach(self):
        self.attachment.detach()
        self.attached = False

    def plan(self, observation, truth, goal_world, *, allow_object_contact=False):
        obstacle_count = self.update_world(
            observation, truth, allow_object_contact=allow_object_contact
        )
        goal = GoalToolPose.from_poses(
            {
                self.arm.tcp_frame: Pose.from_list(
                    self._local_pose(goal_world).tolist(), q_xyzw=True
                )
            },
            ordered_tool_frames=[self.arm.tcp_frame],
        )
        start = time.perf_counter()
        result = self.planner.plan_pose(
            goal,
            self._state(observation.values[f"robot/{self.side}/joint_position"]),
            max_attempts=5,
        )
        elapsed = time.perf_counter() - start
        if result is None or not bool(result.success.all().item()):
            raise SourceFailure(
                f"cuRobo failed to reach {np.asarray(goal_world).tolist()}",
                kind="planning",
            )
        trajectory = (
            result.get_interpolated_plan()
            .reorder(list(self.arm.joint_names))
            .position.detach()
            .cpu()
            .numpy()
            .reshape(-1, len(self.arm.joint_names))
        )
        if len(trajectory) < 2 or not np.isfinite(trajectory).all():
            raise SourceFailure("Invalid cuRobo trajectory", kind="planning")
        return trajectory[1:].copy(), {
            "success": True,
            "goal_pose_world": np.asarray(goal_world).tolist(),
            "waypoints": len(trajectory) - 1,
            "planning_seconds": elapsed,
            "obstacle_count": obstacle_count,
            "held_arm_collision": True,
            "self_collision": True,
            "attached_object": self.attached,
            "target_contact_allowed": allow_object_contact,
            "time_scale": 2.0,
        }
