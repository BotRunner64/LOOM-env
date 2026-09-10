"""Shared tabletop geometry, object sampling and physical grasp evidence."""

import numpy as np

from loom_env.specs.config import vector


TABLE_SIZE = (1.6, 1.8, 0.08)
TABLE_CENTER_XY = (0.45, 0.0)
WALL = 0.01


def tabletop_geometry(collection):
    """Static boxes shared by PhysX, previews and the collision planner.

    Object dimensions, colors and sampling bounds belong to the scene. Role
    bindings only select the object and receptacle used by a particular task.
    """
    if set(collection.scene.parameters) != {"table_height"}:
        raise ValueError("Tabletop scene parameters must specify exactly table_height")
    height = float(collection.scene.parameters["table_height"])
    if not np.isfinite(height) or height <= 0:
        raise ValueError("Table height must be finite and positive")
    if set(collection.role_bindings) != {"target_object", "container"}:
        raise ValueError("Tabletop requires target_object and container roles")
    boxes = {
        "table": {
            "size": TABLE_SIZE,
            "position": [*TABLE_CENTER_XY, height - TABLE_SIZE[2] / 2],
            "color": [0.43, 0.47, 0.51],
        }
    }
    footprints = []
    common = {"asset", "category", "description", "size", "color"}
    for name, obj in collection.scene.objects.items():
        is_cube = obj["asset"] == "primitive:cube"
        is_container = obj["asset"] == "primitive:open_box"
        expected = common | (
            {"position_min", "position_max"} if is_cube else {"position"}
        )
        if not (is_cube or is_container) or set(obj) != expected:
            raise ValueError(f"Unsupported tabletop object fields: {name}")
        if obj["category"] != ("graspable_object" if is_cube else "receptacle"):
            raise ValueError(f"Incorrect tabletop object category: {name}")
        if not isinstance(obj["description"], str) or not obj["description"].strip():
            raise ValueError(f"Object description is required: {name}")
        size = np.asarray(vector(obj["size"], 3, f"{name} size"))
        color = np.asarray(vector(obj["color"], 3, f"{name} color"))
        if np.any(size <= 0) or np.any((color < 0) | (color > 1)):
            raise ValueError(f"Invalid size or color: {name}")
        if is_cube:
            low = np.asarray(vector(obj["position_min"], 3, f"{name} position_min"))
            high = np.asarray(vector(obj["position_max"], 3, f"{name} position_max"))
            if np.any(high < low) or low[2] - size[2] / 2 < height - 1e-8:
                raise ValueError(f"Invalid or unsupported sampling bounds: {name}")
            for bound in (low, high):
                _on_table(name, bound, size)
            continue
        center = np.asarray(vector(obj["position"], 3, f"{name} position"))
        sx, sy, sz = size
        floor = center[2] - sz / 2
        if not np.isclose(floor - WALL, height):
            raise ValueError(f"Container floor must rest on the table: {name}")
        outer = size + [2 * WALL, 2 * WALL, 0]
        _on_table(name, center, outer)
        if any(
            np.all(np.abs(center[:2] - c[:2]) < (outer[:2] + s[:2]) / 2)
            for c, s in footprints
        ):
            raise ValueError("Containers must not overlap")
        footprints.append((center, outer))
        for part, dims, position in (
            (
                "base",
                [sx + 2 * WALL, sy + 2 * WALL, WALL],
                [center[0], center[1], floor - WALL / 2],
            ),
            ("xm", [WALL, sy + 2 * WALL, sz], center + [-(sx + WALL) / 2, 0, 0]),
            ("xp", [WALL, sy + 2 * WALL, sz], center + [(sx + WALL) / 2, 0, 0]),
            ("ym", [sx, WALL, sz], center + [0, -(sy + WALL) / 2, 0]),
            ("yp", [sx, WALL, sz], center + [0, (sy + WALL) / 2, 0]),
        ):
            key = f"{name}_{part}"
            if key in boxes:
                raise ValueError(f"Duplicate geometry name: {key}")
            boxes[key] = {
                "size": list(dims),
                "position": list(position),
                "color": list(color),
            }
    for name, obj in collection.scene.objects.items():
        if obj["asset"] == "primitive:cube" and name in boxes:
            raise ValueError(f"Object name conflicts with static geometry: {name}")
    return boxes


def _on_table(name, position, size):
    if np.any(
        np.abs(np.asarray(position)[:2] - TABLE_CENTER_XY) + np.asarray(size)[:2] / 2
        > np.asarray(TABLE_SIZE)[:2] / 2 + 1e-8
    ):
        raise ValueError(f"Object extends outside the tabletop: {name}")


def sample_objects(collection, seed):
    """Sample every free object deterministically, independently of task bindings."""
    tabletop_geometry(collection)
    rng = np.random.default_rng(seed)
    occupied = [
        (np.asarray(obj["position"]), np.asarray(obj["size"]) + [2 * WALL, 2 * WALL, 0])
        for obj in collection.scene.objects.values()
        if obj["asset"] == "primitive:open_box"
    ]
    candidates = {}
    for name, obj in sorted(collection.scene.objects.items()):
        if obj["asset"] != "primitive:cube":
            continue
        size = np.asarray(obj["size"])
        for _ in range(100):
            position = rng.uniform(obj["position_min"], obj["position_max"])
            if all(
                np.any(
                    np.abs(position[:2] - center[:2])
                    > (size[:2] + other[:2]) / 2 + 0.02
                )
                for center, other in occupied
            ):
                candidates[name] = position
                occupied.append((position, size))
                break
        else:
            raise ValueError(f"Could not sample separated tabletop object: {name}")
    return candidates


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
