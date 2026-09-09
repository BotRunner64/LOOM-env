"""Shared primitive geometry and candidate sampling for the first place task."""

import numpy as np


def tabletop_geometry(collection):
    """Return the exact boxes used both by PhysX and the motion planner."""
    p = collection.scene.parameters
    expected = {
        "table_height",
        "cube_position_min",
        "cube_position_max",
        "container_position",
    }
    if set(p) != expected:
        raise ValueError(
            "Tabletop scene parameters must specify exactly "
            + ", ".join(sorted(expected))
        )
    assets = {"target_object": "primitive:cube", "container": "primitive:open_box"}
    if set(collection.role_bindings) != set(assets) or set(
        collection.scene.objects
    ) != set(collection.role_bindings.values()):
        raise ValueError(
            "Tabletop scene requires exactly a target object and a container"
        )
    for role, asset in assets.items():
        obj = collection.scene.objects[collection.role_bindings[role]]
        if set(obj) != {"asset", "category"} or obj["asset"] != asset:
            raise ValueError(f"Unsupported tabletop object for {role}: {obj}")
    height = float(p["table_height"])
    sx, sy, sz = collection.task.parameters["region_size"]
    cx, cy, cz = p["container_position"]  # center of the internal target region
    wall = 0.01
    floor = cz - sz / 2
    if not np.isclose(floor - wall, height):
        raise ValueError("Container floor must rest on the table")
    boxes = {
        "table": {"size": [1.6, 1.8, 0.08], "position": [0.45, 0, height - 0.04]},
        "container_base": {
            "size": [sx + 2 * wall, sy + 2 * wall, wall],
            "position": [cx, cy, floor - wall / 2],
        },
    }
    for name, size, offset in (
        ("xm", [wall, sy + 2 * wall, sz], [-(sx + wall) / 2, 0, 0]),
        ("xp", [wall, sy + 2 * wall, sz], [(sx + wall) / 2, 0, 0]),
        ("ym", [sx, wall, sz], [0, -(sy + wall) / 2, 0]),
        ("yp", [sx, wall, sz], [0, (sy + wall) / 2, 0]),
    ):
        boxes[f"container_{name}"] = {
            "size": size,
            "position": (np.array([cx, cy, cz]) + offset).tolist(),
        }
    return boxes


def sample_cube(collection, seed):
    """Sample reproducibly; reject overlaps before entering the simulator."""
    p = collection.scene.parameters
    low, high = np.asarray(p["cube_position_min"]), np.asarray(p["cube_position_max"])
    if low.shape != (3,) or high.shape != (3,) or np.any(high < low):
        raise ValueError("Invalid cube sampling bounds")
    half = np.asarray(collection.task.parameters["object_size"]) / 2
    center = np.asarray(p["container_position"])
    outer = np.asarray(collection.task.parameters["region_size"]) / 2 + 0.01
    rng = np.random.default_rng(seed)
    for _ in range(100):
        position = rng.uniform(low, high)
        if position[2] - half[2] < p["table_height"] - 1e-8:
            raise ValueError("Cube sampling bounds penetrate the table")
        if np.any(np.abs(position[:2] - center[:2]) > outer[:2] + half[:2] + 0.02):
            return position
    raise ValueError("Could not sample a cube separated from the container")


def initial_command(deployment):
    return np.concatenate(
        [
            np.r_[arm.initial_positions, [hi for _, hi in arm.gripper.command_limits]]
            for arm in (deployment.arms["left"], deployment.arms["right"])
        ]
    )


def grasp_evidence(forces, finger_positions, object_in_hand, object_size):
    """Opposing measured finger contacts, finite aperture and cube between pads.

    Forces are the two finger-to-cube normal forces in world coordinates. No
    commanded gripper value or expert stage enters this predicate.
    """
    forces = np.asarray(forces)
    magnitudes = np.linalg.norm(forces, axis=1)
    opposing = np.dot(forces[0], forces[1]) < -0.5 * np.prod(magnitudes)
    width = float(np.sum(finger_positions))
    local = np.asarray(object_in_hand)
    between = abs(local[0]) < object_size[0] / 2 + 0.01 and abs(local[1]) < 0.015
    depth = 0.065 < local[2] < 0.125
    return bool(
        np.all(magnitudes > 0.2)
        and opposing
        and 0.015 < width < 0.065
        and between
        and depth
    )
