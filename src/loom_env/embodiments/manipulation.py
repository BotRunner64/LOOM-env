"""Reviewed grasp geometry and native planner models for supported embodiments."""

from dataclasses import dataclass
from itertools import combinations
import json
import xml.etree.ElementTree as ET
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
    "robotwin:x5": ParallelJaw(
        "link6",
        ("link7", "link8"),
        (0.145, 0.0, 0.0),
        (0.0, 0.7071067811865476, 0.0, 0.7071067811865476),
        ("base_link",),
    ),
    "robotwin:ur5-wsg": ParallelJaw(
        "wrist_3_link",
        ("gripper_left", "gripper_right"),
        (0.0, 0.222, 0.0),
        (-0.5, -0.5, 0.5, 0.5),
        ("base_link",),
    ),
    "i2rt:yam-v1": ParallelJaw(
        "gripper",
        ("tip_left", "tip_right"),
        (0.0, 0.0, -0.135),
        (0.0, 0.0, 1.0, 0.0),
        ("base",),
    ),
    "maniskill:xarm6-robotiq": ParallelJaw(
        "link6",
        ("left_inner_finger", "right_inner_finger"),
        (0.0, 0.0, 0.152),
        (1.0, 0.0, 0.0, 0.0),
        ("link_base",),
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
    if len(arm.gripper.command_names) != 1:
        raise ValueError("Parallel-jaw manipulation requires one opening command")
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
        robot = _urdf_planner_robot(arm, asset_root)
    robot["robot_cfg"]["kinematics"]["extra_collision_spheres"] = {"attached_object": 8}
    return robot


def _urdf_planner_robot(arm, asset_root):
    """Use mesh-fitted geometry and the same structural exclusions as physics."""
    name = model_name(arm.asset)
    verify_asset(asset_root, name)
    urdf = prepared_urdf(asset_root, name)
    path = urdf.parent / "planning.json"
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {path}; run scripts/prepare_planning.py {name}"
        )
    data = json.loads(path.read_text())
    if data["urdf_sha256"] != sha256(urdf):
        raise ValueError(f"Stale planning geometry: {path}")
    spheres = data["collision_spheres"]
    tree = ET.parse(urdf).getroot()
    parents = {j.find("child").get("link"): j for j in tree.findall("joint")}

    def body(link):
        while link in parents and parents[link].get("type") == "fixed":
            link = parents[link].find("parent").get("link")
        return link

    adjacent = {
        frozenset(
            (body(j.find("parent").get("link")), body(j.find("child").get("link")))
        )
        for j in parents.values()
    }
    for group in MODELS[name].get("collision_groups", ()):
        adjacent.update(frozenset(pair) for pair in combinations(group, 2))
    ignore = {link: [] for link in spheres}
    for a, b in combinations(spheres, 2):
        if body(a) == body(b) or frozenset((body(a), body(b))) in adjacent:
            ignore[a].append(b)
    # The attached object intentionally touches the gripper; retain arm collisions.
    gripper_links = []
    for link in spheres:
        ancestor = link
        while ancestor in parents and ancestor != arm.tcp_frame:
            ancestor = parents[ancestor].find("parent").get("link")
        if ancestor == arm.tcp_frame:
            gripper_links.append(link)
    ignore["attached_object"] = gripper_links
    opened = np.array([high for _, high in arm.gripper.command_limits])
    fingers = np.asarray(arm.gripper.joint_map) @ opened + arm.gripper.joint_offset
    independent = {
        j.get("name") for j in tree.findall("joint") if j.find("mimic") is None
    }
    locked = {
        joint: float(q)
        for joint, q in zip(arm.gripper.joint_names, fingers)
        if joint in independent
    }
    joints = [*arm.joint_names, *locked]
    return {
        "robot_cfg": {
            "kinematics": {
                "format_version": 2.0,
                "urdf_path": str(urdf.resolve()),
                "asset_root_path": str(urdf.parent.resolve()),
                "base_link": MODELS[name]["base"],
                "tool_frames": [arm.tcp_frame],
                "collision_link_names": [*spheres, "attached_object"],
                "collision_spheres": spheres,
                "collision_sphere_buffer": 0.0,
                "self_collision_ignore": ignore,
                "self_collision_buffer": {link: 0.0 for link in ignore},
                "mesh_link_names": list(spheres),
                "lock_joints": locked,
                "cspace": {
                    "joint_names": joints,
                    "default_joint_position": [
                        *arm.initial_positions,
                        *locked.values(),
                    ],
                    "null_space_weight": [1.0] * len(joints),
                    "cspace_distance_weight": [1.0] * len(joints),
                    "max_acceleration": 15.0,
                    "max_jerk": 500.0,
                },
                "extra_links": {
                    "attached_object": {
                        "parent_link_name": arm.tcp_frame,
                        "link_name": "attached_object",
                        "joint_name": "attach_joint",
                        "joint_type": "FIXED",
                        "fixed_transform": [0, 0, 0, 1, 0, 0, 0],
                    }
                },
            }
        }
    }
