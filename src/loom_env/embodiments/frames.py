"""Resolve optical mounts through URDF fixed frames, without a simulator."""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.embodiments.assets import PANDA_ASSET, model_name, prepared_urdf


def resolve_fixed_frame(urdf, frame, body_names):
    """Return the nearest retained body and its pose of a URDF frame (xyzw).

    Only fixed joints may be collapsed. A missing moving body is an error, not
    permission to freeze a joint at its zero position.
    """
    root = ET.parse(urdf).getroot()
    links = {link.get("name") for link in root.findall("link")}
    joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    position, rotation = np.zeros(3), Rotation.identity()
    visited = set()
    while True:
        if frame not in links:
            raise ValueError(f"Mount frame not found in URDF: {frame}")
        if frame in body_names:
            return frame, np.r_[position, rotation.as_quat()]
        if frame in visited:
            raise ValueError(f"Cycle in mount frame chain: {frame}")
        visited.add(frame)
        joint = joints.get(frame)
        if joint is None:
            raise ValueError(f"Mount frame has no retained ancestor body: {frame}")
        if joint.get("type") != "fixed":
            raise ValueError(
                f"Mount chain crosses untracked moving joint: {joint.get('name')}"
            )
        origin = joint.find("origin")
        xyz = "0 0 0" if origin is None else origin.get("xyz", "0 0 0")
        rpy = "0 0 0" if origin is None else origin.get("rpy", "0 0 0")
        parent_rotation = Rotation.from_euler("xyz", [float(v) for v in rpy.split()])
        position = parent_rotation.apply(position) + np.array(
            [float(v) for v in xyz.split()]
        )
        rotation = parent_rotation * rotation
        frame = joint.find("parent").get("link")


def resolve_camera_mount(spec, arm, body_names, asset_root):
    """Resolve a named asset frame plus explicit OpenGL optical offset to a body.

    Existing body frames need no URDF, including Panda's native USD bodies.
    URDF assets are verified by the robot loader before calling this function.
    """
    _, frame = spec.parent_frame.split("/")
    if frame in body_names:
        return frame, np.asarray(spec.pose)
    if arm.asset == PANDA_ASSET:
        raise ValueError(
            f"Mount frame not found in native USD bodies: {spec.parent_frame}"
        )
    body, pose = resolve_fixed_frame(
        prepared_urdf(asset_root, model_name(arm.asset)), frame, body_names
    )
    rotation = Rotation.from_quat(pose[3:])
    return body, np.r_[
        pose[:3] + rotation.apply(spec.pose[:3]),
        (rotation * Rotation.from_quat(spec.pose[3:])).as_quat(),
    ]
