"""cuRobo 0.8 pose planning in the selected Panda's base frame."""

from itertools import product
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
from curobo.scene import Cuboid, Scene
from curobo.types import GoalToolPose, JointState, Pose

from loom_env.environments.tabletop import tabletop_geometry
from loom_env.runtime.runner import SourceFailure


class PandaPlanner:
    def __init__(self, collection, side):
        self.collection, self.side = collection, side
        self.arm = collection.deployment.arms[side]
        self.base = np.asarray(self.arm.base_pose)
        self.rotation = Rotation.from_quat(self.base[3:])
        robot = load_yaml(str(Path(get_robot_configs_path()) / "franka.yml"))
        robot["robot_cfg"]["kinematics"]["extra_collision_spheres"] = {
            "attached_object": 8
        }
        cfg = MotionPlannerCfg.create(
            robot=robot,
            collision_cache={"obb": 128},
            num_ik_seeds=32,
            num_trajopt_seeds=4,
            random_seed=0,
            position_tolerance=0.002,
            orientation_tolerance=0.025,
            optimizer_collision_activation_distance=0.005,
        )
        # Execute each 25 ms planned sample over a 50 ms control interval.
        # Time scaling slows the complete collision-checked path uniformly.
        cfg.trajopt_solver_config.interpolation_dt = (
            collection.deployment.control_dt / 2
        )
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
        boxes = [
            self._box(name, box["size"], np.r_[box["position"], [0, 0, 0, 1]])
            for name, box in tabletop_geometry(self.collection).items()
        ]
        other = "left" if self.side == "right" else "right"
        arm = self.collection.deployment.arms[other]
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
        target = self.collection.role_bindings["target_object"]
        for name, obj in self.collection.scene.objects.items():
            if obj["asset"] != "primitive:cube":
                continue
            if name == target and (allow_object_contact or self.attached):
                continue
            boxes.append(
                self._box(f"object_{name}", obj["size"], truth[f"{name}/pose_world"])
            )
        self.planner.update_world(Scene(cuboid=boxes))
        return len(boxes)

    def attach(self, observation, truth):
        """Planning geometry only; the simulator object remains a free rigid body."""
        obj = self.collection.role_bindings["target_object"]
        size = np.asarray(self.collection.scene.objects[obj]["size"])
        centers = np.asarray(list(product((-1, 1), repeat=3))) * size / 4
        # Each sphere covers one octant of the cube, including its corners.
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
