"""One asset workspace, explicit poses, and bounded layout sampling."""

from itertools import product

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.specs.config import pose, vector


def transform(parent, child):
    parent, child = np.asarray(parent), np.asarray(child)
    rotation = Rotation.from_quat(parent[3:].copy())
    return np.r_[
        parent[:3] + rotation.apply(child[:3]),
        (rotation * Rotation.from_quat(child[3:].copy())).as_quat(),
    ]


def corners(asset):
    low, high = np.asarray(asset.bounds)
    return np.array(list(product(*zip(low, high))))


def workspace(scene):
    if set(scene.parameters) != {"workspace_pose", "workspace_size"}:
        raise ValueError("Scene requires workspace_pose and workspace_size")
    frame = np.asarray(pose(scene.parameters["workspace_pose"]))
    size = np.asarray(vector(scene.parameters["workspace_size"], 2, "workspace_size"))
    if np.any(size <= 0):
        raise ValueError("Workspace dimensions must be positive")
    # Current sampler uses a horizontal support plane. Tilt needs a new sampler.
    if not np.allclose(Rotation.from_quat(frame[3:]).apply([0, 0, 1]), [0, 0, 1]):
        raise ValueError("Workspace support plane must be horizontal")
    supports = []
    for name, obj in scene.objects.items():
        asset = asset_definition(obj["asset"])
        if obj["category"] != asset.category:
            raise ValueError(f"Asset category mismatch: {name}")
        required = {"asset", "category", "description", "pose", "static"}
        optional = {"position_min", "position_max"}
        if set(obj) - required - optional or not required <= set(obj):
            raise ValueError(f"Unsupported scene object fields: {name}")
        pose(obj["pose"])
        if type(obj["static"]) is not bool:
            raise ValueError("Object static must be boolean")
        if not isinstance(obj["description"], str) or not obj["description"].strip():
            raise ValueError("Object description is required")
        if asset.category == "workspace":
            supports.append(name)
            if not obj["static"] or not np.allclose(obj["pose"], [0, 0, 0, 0, 0, 0, 1]):
                raise ValueError(
                    "Workspace asset must be static at the workspace origin"
                )
            if np.any(
                size / 2
                > np.minimum(
                    -np.array(asset.bounds[0])[:2], np.array(asset.bounds[1])[:2]
                )
            ):
                raise ValueError("Workspace region extends outside the support asset")
        if ("position_min" in obj) != ("position_max" in obj):
            raise ValueError("Sampling bounds must be supplied together")
        if "position_min" in obj:
            lo = np.array(vector(obj["position_min"], 2, "position_min"))
            hi = np.array(vector(obj["position_max"], 2, "position_max"))
            if obj["static"] or np.any(hi < lo):
                raise ValueError("Invalid sampling range")
        if obj["static"] == asset.dynamic:
            raise ValueError(f"Scene cannot override source rigid-body mode: {name}")
    if len(supports) != 1:
        raise ValueError("Scene requires exactly one workspace asset")
    return frame, size


def sample_objects(scene, seed):
    """Return world poses for every instance, independent of task and bindings."""
    frame, size = workspace(scene)
    rng = np.random.default_rng(seed)
    poses, occupied = {}, []
    ordered = sorted(scene.objects, key=lambda n: (not scene.objects[n]["static"], n))
    for name in ordered:
        obj = scene.objects[name]
        asset = asset_definition(obj["asset"])
        local = np.array(obj["pose"], dtype=float)
        points = Rotation.from_quat(local[3:]).apply(corners(asset))
        low, high = points.min(axis=0), points.max(axis=0)
        if asset.category == "workspace":
            poses[name] = transform(frame, local)
            continue
        for _ in range(100):
            if "position_min" in obj:
                local[:2] = rng.uniform(obj["position_min"], obj["position_max"])
            lower, upper = low + local[:3], high + local[:3]
            if np.any(lower[:2] < -size / 2) or np.any(upper[:2] > size / 2):
                raise ValueError(f"Object extends outside workspace: {name}")
            if not -0.0001 <= lower[2] <= 0.01:
                raise ValueError(f"Object is not supported by workspace: {name}")
            if all(
                np.any(upper[:2] + 0.02 < a) or np.any(lower[:2] - 0.02 > b)
                for a, b in occupied
            ):
                occupied.append((lower[:2].copy(), upper[:2].copy()))
                poses[name] = transform(frame, local)
                break
        else:
            raise ValueError(f"Could not sample separated object: {name}")
    return poses


def dynamic_names(scene):
    return tuple(
        sorted(name for name, obj in scene.objects.items() if not obj["static"])
    )


def maximum_point_speed(asset, world_pose, velocity):
    """Conservative envelope speed in m/s, including rotation about the body origin."""
    velocity = np.asarray(vector(velocity, 6, "velocity"))
    offsets = Rotation.from_quat(np.asarray(world_pose[3:]).copy()).apply(
        corners(asset)
    )
    return float(
        np.linalg.norm(velocity[:3] + np.cross(velocity[3:], offsets), axis=1).max()
    )
