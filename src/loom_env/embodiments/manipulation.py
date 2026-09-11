"""Reviewed grasp geometry and native planner models for supported embodiments."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from loom_env.embodiments.assets import (
    PANDA_ASSET,
    MODELS,
    model_name,
    prepared_urdf,
    sha256,
    source_urdf,
    verify_asset,
)


@dataclass(frozen=True)
class ParallelJaw:
    tcp_frame: str
    finger_bodies: tuple[str, str]
    tcp_to_grasp: tuple[float, float, float]
    # Desired tool orientation relative to the installed robot base, xyzw.
    grasp_rotation: tuple[float, float, float, float]
    # Fixed mounting contact; omit only from the moving arm collision model.
    mounting_contact_bodies: tuple[str, ...] = ()


PROFILES = {
    PANDA_ASSET: ParallelJaw(
        "panda_hand",
        ("panda_leftfinger", "panda_rightfinger"),
        (0.0, 0.0, 0.1034),
        (1.0, 0.0, 0.0, 0.0),
    ),
    "robotwin:piper": ParallelJaw(
        "link6",
        ("link7", "link8"),
        # Source CAD finger ends near local Z=0.14 m; target that contact region.
        (0.0, 0.0, 0.14),
        # Pitch down toward the object while preserving the initial +Y finger order.
        # Do not roll the palm over merely because the two jaw contacts are symmetric.
        (0.0, 0.9659258262890683, 0.0, 0.25881904510252074),
        ("base_link",),
    ),
}
PIPER_PLANNING_FILES = {
    "curobo_tmp.yml": "635fb098c64ab1e274039f19bf51a710ed4d2628ce819b989387f78277198067",
    "collision_piper.yml": "61b272ee694afeff9500a35a03ba7897b00e0a47d3fce704efbc8b0727ee7843",
}


def manipulation_profile(arm):
    try:
        profile = PROFILES[arm.asset]
    except KeyError as error:
        raise ValueError(f"No manipulation profile for {arm.asset}") from error
    if arm.tcp_frame != profile.tcp_frame:
        raise ValueError("Manipulation profile and deployment TCP disagree")
    if arm.gripper.unit != "m" or len(arm.gripper.command_names) != 1:
        raise ValueError(
            "Parallel-jaw manipulation requires one linear opening command"
        )
    return profile


def planner_robot(arm, asset_root):
    """Build cuRobo 0.8 configuration from pinned model data and the deployment."""
    profile = manipulation_profile(arm)
    if arm.asset == PANDA_ASSET:
        from curobo.content import get_robot_configs_path

        path = Path(get_robot_configs_path()) / "franka.yml"
        robot = yaml.safe_load(path.read_text())
    elif arm.asset == "robotwin:piper":
        name = model_name(arm.asset)
        verify_asset(asset_root, name)
        directory = source_urdf(asset_root, name).parent
        for filename, expected in PIPER_PLANNING_FILES.items():
            if sha256(directory / filename) != expected:
                raise ValueError(f"Piper planning source changed: {filename}")
        source = yaml.safe_load((directory / "curobo_tmp.yml").read_text())[
            "robot_cfg"
        ]["kinematics"]
        # Select the source geometry fields explicitly. The source uses cuRobo 0.7
        # names and has inconsistent cspace vector lengths; build native 0.8 data.
        kin = {
            key: source[key]
            for key in (
                "collision_link_names",
                "collision_sphere_buffer",
                "self_collision_ignore",
                "self_collision_buffer",
                "mesh_link_names",
            )
        }
        opened = np.array([high for _, high in arm.gripper.command_limits])
        fingers = np.asarray(arm.gripper.joint_map) @ opened + arm.gripper.joint_offset
        joints = [*arm.joint_names, *arm.gripper.joint_names]
        kin.update(
            format_version=2.0,
            urdf_path=str(prepared_urdf(asset_root, name).resolve()),
            asset_root_path=str(directory.resolve()),
            base_link=MODELS[name]["base"],
            tool_frames=[arm.tcp_frame],
            collision_spheres=yaml.safe_load(
                (directory / "collision_piper.yml").read_text()
            )["collision_spheres"],
            lock_joints=dict(zip(arm.gripper.joint_names, fingers.tolist())),
            cspace={
                "joint_names": joints,
                "default_joint_position": [*arm.initial_positions, *fingers.tolist()],
                "null_space_weight": [1.0] * len(joints),
                "cspace_distance_weight": [1.0] * len(joints),
                "max_acceleration": source["cspace"]["max_acceleration"],
                "max_jerk": source["cspace"]["max_jerk"],
            },
            extra_links={
                "attached_object": {
                    "parent_link_name": arm.tcp_frame,
                    "link_name": "attached_object",
                    "joint_name": "attach_joint",
                    "joint_type": "FIXED",
                    "fixed_transform": [0, 0, 0, 1, 0, 0, 0],
                }
            },
        )
        # The source's first right-finger sphere is entirely outside the CAD:
        # centre Y=-0.031 m, radius=0.01 m; mesh minimum Y=-0.004968 m.
        # Remove that false obstacle; retain the three spheres on the actual jaw.
        kin["collision_spheres"]["link8"].pop(0)
        kin["collision_link_names"].append("attached_object")
        kin["self_collision_ignore"]["attached_object"] = [
            profile.tcp_frame,
            *profile.finger_bodies,
        ]
        kin["self_collision_buffer"]["attached_object"] = 0.0
        robot = {"robot_cfg": {"kinematics": kin}}
    else:
        raise ValueError(f"No planner model for {arm.asset}")
    robot["robot_cfg"]["kinematics"]["extra_collision_spheres"] = {"attached_object": 8}
    return robot
