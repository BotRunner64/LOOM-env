"""The first supported deployment uses absolute joint-position commands.

Configuration is immutable after parsing. Poses are xyz + quaternion xyzw in
metres; command vectors are ordered left arm, left gripper, right arm, right
gripper. No implicit normalization, clipping, or joint-name sorting is allowed.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, fields, is_dataclass
import math
from pathlib import Path
import re
from string import Formatter
from types import MappingProxyType
from typing import Any

import numpy as np
import yaml

SCHEMA_VERSION = 1
ARMS = ("left", "right")


def plain(value: Any) -> Any:
    """Convert protocol values to independent JSON-compatible containers."""
    if is_dataclass(value) and not isinstance(value, type):
        return {
            field.name: plain(getattr(value, field.name)) for field in fields(value)
        }
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def frozen_json(value: Any) -> Any:
    """Reject non-JSON data and prevent mutable metadata escaping a spec."""
    if isinstance(value, Mapping):
        if not all(isinstance(key, str) for key in value):
            raise ValueError("Metadata keys must be strings")
        return MappingProxyType({key: frozen_json(item) for key, item in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(frozen_json(item) for item in value)
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float and math.isfinite(value):
        return value
    raise ValueError(f"Not finite JSON data: {type(value).__name__}")


def identifier(value: str) -> None:
    if not isinstance(value, str) or not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_.-]*", value
    ):
        raise ValueError(f"Invalid identifier: {value!r}")


def names(values: Any) -> tuple[str, ...]:
    if isinstance(values, str):
        raise ValueError("Expected a list of names")
    result = tuple(values)
    if not result or len(set(result)) != len(result):
        raise ValueError("Names must be nonempty and unique")
    for name in result:
        identifier(name)
    return result


def vector(value: Any, size: int, label: str) -> tuple[float, ...]:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{label} must contain {size} finite numbers")
    return tuple(float(item) for item in array)


def pose(value: Any) -> tuple[float, ...]:
    result = vector(value, 7, "Pose (xyz + xyzw)")
    if not math.isclose(sum(x * x for x in result[3:]), 1.0, abs_tol=1e-6):
        raise ValueError("Pose quaternion must have unit length")
    return result


def limits(value: Any, size: int) -> tuple[tuple[float, float], ...]:
    array = np.asarray(value, dtype=np.float64)
    if (
        array.shape != (size, 2)
        or not np.isfinite(array).all()
        or np.any(array[:, 0] >= array[:, 1])
    ):
        raise ValueError(f"Expected {size} finite, increasing limit pairs")
    return tuple(tuple(float(x) for x in row) for row in array)


@dataclass(frozen=True)
class GripperSpec:
    joint_names: tuple[str, ...]
    command_names: tuple[str, ...]
    command_limits: tuple[tuple[float, float], ...]
    unit: str
    # physical_joint_targets = joint_map @ command + joint_offset
    joint_map: tuple[tuple[float, ...], ...]
    joint_offset: tuple[float, ...]

    def __post_init__(self):
        object.__setattr__(self, "joint_names", names(self.joint_names))
        object.__setattr__(self, "command_names", names(self.command_names))
        object.__setattr__(
            self, "command_limits", limits(self.command_limits, len(self.command_names))
        )
        if self.unit not in {"m", "rad"}:
            raise ValueError("Gripper unit must be m or rad")
        matrix = np.asarray(self.joint_map, dtype=np.float64)
        if matrix.shape != (len(self.joint_names), len(self.command_names)) or not (
            np.isfinite(matrix).all()
        ):
            raise ValueError("Gripper joint_map dimensions do not match its names")
        object.__setattr__(
            self, "joint_map", tuple(tuple(row) for row in matrix.tolist())
        )
        object.__setattr__(
            self,
            "joint_offset",
            vector(self.joint_offset, len(self.joint_names), "Offset"),
        )


@dataclass(frozen=True)
class ArmSpec:
    asset: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...]
    initial_positions: tuple[float, ...]
    base_pose: tuple[float, ...]
    tcp_frame: str
    gripper: GripperSpec

    def __post_init__(self):
        if not isinstance(self.asset, str) or not self.asset.strip():
            raise ValueError("Arm asset reference is required")
        object.__setattr__(self, "joint_names", names(self.joint_names))
        if len(self.joint_names) not in {6, 7}:
            raise ValueError(
                "Each arm must have 6 or 7 arm joints, excluding the gripper"
            )
        object.__setattr__(
            self, "joint_limits", limits(self.joint_limits, len(self.joint_names))
        )
        object.__setattr__(
            self,
            "initial_positions",
            vector(self.initial_positions, len(self.joint_names), "Initial positions"),
        )
        if any(
            not low <= q <= high
            for q, (low, high) in zip(self.initial_positions, self.joint_limits)
        ):
            raise ValueError("Initial arm positions exceed joint limits")
        object.__setattr__(self, "base_pose", pose(self.base_pose))
        identifier(self.tcp_frame)
        if not isinstance(self.gripper, GripperSpec):
            raise TypeError("Arm gripper must be a GripperSpec")
        if set(self.joint_names) & set(self.gripper.joint_names):
            raise ValueError("Arm and gripper joint names must be disjoint")


@dataclass(frozen=True)
class CameraSpec:
    """OpenGL optical pose in a world, body, or fixed URDF parent frame."""

    name: str
    width: int
    height: int
    period_steps: int
    parent_frame: str
    pose: tuple[float, ...]
    focal_length: float = 22.0
    horizontal_aperture: float = 24.0
    clipping_range: tuple[float, float] = (0.01, 20.0)

    def __post_init__(self):
        identifier(self.name)
        for value in (self.width, self.height, self.period_steps):
            if type(value) is not int or value <= 0:
                raise ValueError(
                    "Camera dimensions and period_steps must be positive ints"
                )
        if not isinstance(self.parent_frame, str):
            raise ValueError("Camera parent_frame must be a string")
        if self.parent_frame != "world":
            parts = self.parent_frame.split("/")
            if len(parts) != 2 or parts[0] not in ARMS:
                raise ValueError(
                    "Camera parent_frame must be world or left/right/<frame>"
                )
            identifier(parts[1])
        object.__setattr__(self, "pose", pose(self.pose))
        for value in (self.focal_length, self.horizontal_aperture):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Camera focal length and aperture must be positive")
        clip = tuple(self.clipping_range)
        if (
            len(clip) != 2
            or not all(math.isfinite(v) for v in clip)
            or not 0 < clip[0] < clip[1]
        ):
            raise ValueError("Camera clipping range must satisfy 0 < near < far")
        object.__setattr__(self, "clipping_range", clip)


@dataclass(frozen=True)
class DeploymentSpec:
    id: str
    arms: Mapping[str, ArmSpec]
    control_dt: float
    physics_dt: float
    capabilities: tuple[str, ...]
    cameras: tuple[CameraSpec, ...] = ()
    control_type: str = "joint_position"

    def __post_init__(self):
        identifier(self.id)
        if set(self.arms) != set(ARMS) or not all(
            isinstance(arm, ArmSpec) for arm in self.arms.values()
        ):
            raise ValueError("Deployment requires exactly left and right ArmSpecs")
        object.__setattr__(self, "arms", MappingProxyType(dict(self.arms)))
        if self.control_type != "joint_position":
            raise ValueError("Only absolute joint_position control is implemented")
        if not all(
            math.isfinite(dt) and dt > 0 for dt in (self.control_dt, self.physics_dt)
        ) or not math.isclose(self.control_dt / self.physics_dt, self.decimation):
            raise ValueError(
                "control_dt must be a positive integer multiple of physics_dt"
            )
        object.__setattr__(self, "capabilities", names(self.capabilities))
        object.__setattr__(self, "cameras", tuple(self.cameras))
        if len({camera.name for camera in self.cameras}) != len(self.cameras):
            raise ValueError("Camera names must be unique")

    @property
    def decimation(self) -> int:
        return round(self.control_dt / self.physics_dt)

    @property
    def action_slices(self) -> dict[str, slice]:
        result = {}
        offset = 0
        for side in ARMS:
            arm = self.arms[side]
            for field, size in (
                ("arm", len(arm.joint_names)),
                ("gripper", len(arm.gripper.command_names)),
            ):
                result[f"{side}/{field}"] = slice(offset, offset + size)
                offset += size
        return result

    @property
    def action_dim(self) -> int:
        return sum(part.stop - part.start for part in self.action_slices.values())

    def validate_action(self, value: Any) -> np.ndarray:
        action = np.array(value, dtype=np.float64, copy=True)
        if action.shape != (self.action_dim,) or not np.isfinite(action).all():
            raise ValueError(f"Action must have {self.action_dim} finite values")
        for key, part in self.action_slices.items():
            side, field = key.split("/")
            arm = self.arms[side]
            bounds = np.asarray(
                arm.joint_limits if field == "arm" else arm.gripper.command_limits
            )
            if np.any(action[part] < bounds[:, 0]) or np.any(
                action[part] > bounds[:, 1]
            ):
                raise ValueError(
                    f"Action exceeds {key} limits; implicit clipping is disabled"
                )
        return action

    def action_descriptor(self) -> dict[str, Any]:
        return {
            "control_type": self.control_type,
            "semantics": "absolute",
            "reference_frame": "joint",
            "rotation_representation": None,
            "normalization": "none",
            "clipping": "reject",
            "control_dt": self.control_dt,
            "dimension": self.action_dim,
            "fields": {
                key: {
                    "slice": [part.start, part.stop],
                    "names": list(
                        self.arms[key.split("/")[0]].joint_names
                        if key.endswith("/arm")
                        else self.arms[key.split("/")[0]].gripper.command_names
                    ),
                    "unit": (
                        "rad"
                        if key.endswith("/arm")
                        else self.arms[key.split("/")[0]].gripper.unit
                    ),
                }
                for key, part in self.action_slices.items()
            },
        }


@dataclass(frozen=True)
class TaskSpec:
    id: str
    instruction: str
    roles: Mapping[str, str]
    required_capabilities: tuple[str, ...]
    parameters: Mapping[str, Any]

    def __post_init__(self):
        identifier(self.id)
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("Task instruction is required")
        if not self.roles:
            raise ValueError("Task object roles are required")
        for role, category in self.roles.items():
            identifier(role)
            identifier(category)
        object.__setattr__(self, "roles", frozen_json(self.roles))
        object.__setattr__(self, "parameters", frozen_json(self.parameters))
        object.__setattr__(
            self, "required_capabilities", names(self.required_capabilities)
        )


@dataclass(frozen=True)
class SceneSpec:
    id: str
    # Scene-specific distributions, interpreted by the concrete sampler.
    objects: Mapping[str, Any]
    parameters: Mapping[str, Any]

    def __post_init__(self):
        identifier(self.id)
        if not self.objects:
            raise ValueError("Scene objects are required")
        for name, obj in self.objects.items():
            identifier(name)
            if not isinstance(obj, Mapping) or not {"asset", "category"} <= obj.keys():
                raise ValueError(f"Scene object {name} requires asset and category")
            identifier(obj["category"])
            if not isinstance(obj["asset"], str) or not obj["asset"]:
                raise ValueError(f"Scene object {name} requires an asset reference")
        object.__setattr__(self, "objects", frozen_json(self.objects))
        object.__setattr__(self, "parameters", frozen_json(self.parameters))


@dataclass(frozen=True)
class CollectionSpec:
    task: TaskSpec
    deployment: DeploymentSpec
    scene: SceneSpec
    role_bindings: Mapping[str, str]
    arm_roles: Mapping[str, str]
    max_steps: int

    def __post_init__(self):
        if type(self.max_steps) is not int or self.max_steps <= 0:
            raise ValueError("max_steps must be a positive integer")
        if set(self.role_bindings) != set(self.task.roles):
            raise ValueError("Every task object role must be bound exactly once")
        for role, obj in self.role_bindings.items():
            if obj not in self.scene.objects:
                raise ValueError(f"Unknown scene object for role {role}: {obj}")
            if self.scene.objects[obj]["category"] != self.task.roles[role]:
                raise ValueError(
                    f"Scene object category does not match task role {role}"
                )
        missing = set(self.task.required_capabilities) - set(
            self.deployment.capabilities
        )
        if missing:
            raise ValueError(f"Deployment lacks capabilities: {sorted(missing)}")
        if not self.arm_roles or any(
            side not in ARMS for side in self.arm_roles.values()
        ):
            raise ValueError("Operation roles must bind to left or right")
        for role in self.arm_roles:
            identifier(role)
        object.__setattr__(self, "role_bindings", frozen_json(self.role_bindings))
        object.__setattr__(self, "arm_roles", frozen_json(self.arm_roles))
        # Templates reference role descriptions only; no attribute/index access.
        _ = self.instruction

    @property
    def instruction(self) -> str:
        descriptions = {}
        for _, field, format_spec, conversion in Formatter().parse(
            self.task.instruction
        ):
            if field is None:
                continue
            if field not in self.role_bindings or format_spec or conversion:
                raise ValueError(f"Invalid instruction role placeholder: {field}")
            obj = self.scene.objects[self.role_bindings[field]]
            description = obj.get("description")
            if not isinstance(description, str) or not description.strip():
                raise ValueError(
                    f"Instruction role {field} requires an object description"
                )
            descriptions[field] = description
        return self.task.instruction.format_map(descriptions)


@dataclass(frozen=True)
class EpisodeSpec:
    id: str
    collection: CollectionSpec
    seed: int
    # Measured, accepted initial state, after simulation stabilization.
    initial_state: Mapping[str, Any]
    sampled_parameters: Mapping[str, Any]
    asset_versions: Mapping[str, str]
    runtime_versions: Mapping[str, str]
    provenance: Mapping[str, Any]
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self):
        identifier(self.id)
        if (
            type(self.schema_version) is not int
            or self.schema_version != SCHEMA_VERSION
        ):
            raise ValueError(f"Unsupported schema_version: {self.schema_version}")
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("seed must be an unsigned 32-bit integer")
        if not self.initial_state or not self.runtime_versions:
            raise ValueError("Accepted initial state and runtime versions are required")
        assets = {arm.asset for arm in self.collection.deployment.arms.values()} | {
            obj["asset"] for obj in self.collection.scene.objects.values()
        }
        if not assets <= self.asset_versions.keys():
            raise ValueError("Every resolved asset must have an explicit version")
        for versions in (self.asset_versions, self.runtime_versions):
            if any(not isinstance(v, str) or not v.strip() for v in versions.values()):
                raise ValueError("Version values must be nonempty strings")
        for name in (
            "initial_state",
            "sampled_parameters",
            "asset_versions",
            "runtime_versions",
            "provenance",
        ):
            object.__setattr__(self, name, frozen_json(getattr(self, name)))


def deployment_from_dict(value: Mapping[str, Any]) -> DeploymentSpec:
    value = dict(value)
    value["arms"] = {
        side: ArmSpec(**{**arm, "gripper": GripperSpec(**arm["gripper"])})
        for side, arm in value["arms"].items()
    }
    value["cameras"] = tuple(CameraSpec(**cam) for cam in value.get("cameras", ()))
    return DeploymentSpec(**value)


def collection_from_dict(value: Mapping[str, Any]) -> CollectionSpec:
    return CollectionSpec(
        **{
            **value,
            "task": TaskSpec(**value["task"]),
            "deployment": deployment_from_dict(value["deployment"]),
            "scene": SceneSpec(**value["scene"]),
        }
    )


def episode_from_dict(value: Mapping[str, Any]) -> EpisodeSpec:
    return EpisodeSpec(
        **{
            **value,
            "collection": collection_from_dict(value["collection"]),
        }
    )


class _ConfigLoader(yaml.SafeLoader):
    """Safe YAML with duplicate keys rejected instead of silently overwritten."""


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ValueError(f"YAML keys must be unique strings: {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_ConfigLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _read_mapping(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.load(stream, Loader=_ConfigLoader)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    return value


def load_deployment(path: str | Path) -> DeploymentSpec:
    """Load one deployment preset without requiring simulator assets."""
    path = Path(path)
    try:
        return deployment_from_dict(_read_mapping(path))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid deployment config {path}: {error}") from error


def load_scene(path: str | Path) -> SceneSpec:
    """Load one scene preset without requiring simulator assets."""
    path = Path(path)
    try:
        return SceneSpec(**_read_mapping(path))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid scene config {path}: {error}") from error


def load_collection(path: str | Path) -> CollectionSpec:
    """Resolve one level of task/deployment/scene references relative to this file."""
    path = Path(path)
    value = _read_mapping(path)
    for key in ("task", "deployment", "scene"):
        if isinstance(value.get(key), str):
            value[key] = _read_mapping(path.parent / value[key])
    try:
        return collection_from_dict(value)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid collection config {path}: {error}") from error
