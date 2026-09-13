"""One immutable episode: HDF5 states, per-camera MP4, and a JSON manifest.

Only ``episodes/<id>/manifest.json`` is discoverable. Interrupted attempts stay
in ``.incomplete/<id>``. Directory rename publishes data and metadata together.
Each worker must use a unique episode id; no shared HDF5 writer is needed.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

import imageio.v2 as imageio
import h5py
import numpy as np

from loom_env.data.camera_video import (
    CameraVideoWriter,
    VIDEO_ENCODING,
    sha256,
    video_path,
)
from loom_env.specs.config import SCHEMA_VERSION, EpisodeSpec, episode_from_dict, plain
from loom_env.specs.episode import (
    Event,
    Frame,
    Observation,
    Outcome,
    observation_shapes,
)


def _write_json(path: Path, value: Any) -> None:
    # Callers use a private staging directory; the final directory is immutable.
    with path.open("w", encoding="utf-8") as stream:
        json.dump(plain(value), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def _datasets(group: h5py.Group) -> dict[str, h5py.Dataset]:
    result = {}

    def visit(name, obj):
        if isinstance(obj, h5py.Dataset):
            result[name] = obj

    group.visititems(visit)
    return result


def _observations(group: h5py.Group, timestamps: np.ndarray) -> Iterator[Observation]:
    datasets = _datasets(group)
    batch_size = 64
    for start in range(0, len(timestamps), batch_size):
        batch = {
            key: dataset[start : start + batch_size]
            for key, dataset in datasets.items()
        }
        for offset, timestamp in enumerate(timestamps[start : start + batch_size]):
            yield Observation(
                float(timestamp), {key: values[offset] for key, values in batch.items()}
            )


def _video_observations(path, manifest, group, timestamps):
    """Reassemble observations without holding an episode's images in memory."""
    deployment = episode_from_dict(manifest["spec"]).collection.deployment
    expected = {camera.name for camera in deployment.cameras}
    if set(manifest.get("camera_videos", {})) != expected:
        raise ValueError("Camera video descriptors do not match deployment")
    with ExitStack() as stack:
        readers = {}
        for camera in deployment.cameras:
            descriptor = manifest["camera_videos"][camera.name]
            relative = video_path(camera.name)
            if (
                descriptor.get("path") != relative
                or descriptor.get("num_frames") != len(timestamps)
                or descriptor.get("fps") != 1 / deployment.control_dt
                or any(descriptor.get(k) != v for k, v in VIDEO_ENCODING.items())
            ):
                raise ValueError(f"Invalid camera video descriptor: {camera.name}")
            file = path / relative
            if not file.is_file() or sha256(file) != descriptor.get("sha256"):
                raise ValueError(f"Missing or corrupted camera video: {camera.name}")
            reader = stack.enter_context(imageio.get_reader(file, format="FFMPEG"))
            metadata = reader.get_meta_data()
            if metadata["size"] != (camera.width, camera.height) or not math.isclose(
                metadata["fps"], descriptor["fps"], abs_tol=0.01
            ):
                raise ValueError(
                    f"Camera video dimensions or frame rate disagree: {camera.name}"
                )
            readers[camera.name] = iter(reader)
        for observation in _observations(group, timestamps):
            values = dict(observation.values)
            for name, reader in readers.items():
                try:
                    values[f"cameras/{name}/rgb"] = next(reader)
                except StopIteration as error:
                    raise ValueError(
                        f"Camera video has too few frames: {name}"
                    ) from error
            yield Observation(observation.timestamp, values)
        for name, reader in readers.items():
            if next(reader, None) is not None:
                raise ValueError(f"Camera video has too many frames: {name}")


class EpisodeWriter:
    """Stream completed transitions; an aborted writer can never commit."""

    def __init__(self, root: str | Path, spec: EpisodeSpec):
        self.root = Path(root)
        self.spec = spec
        self.steps = 0
        self._state = "created"
        self._file = None
        self._events: list[Event] = []
        self._videos: dict[str, CameraVideoWriter] = {}
        self.path = self.root / ".incomplete" / spec.id
        self.destination = self.root / "episodes" / spec.id
        if self.destination.exists():
            raise FileExistsError(f"Episode already exists: {spec.id}")
        self.path.mkdir(parents=True, exist_ok=False)
        try:
            _write_json(self.path / "attempt.json", {"spec": spec, "state": "created"})
            self._file = h5py.File(self.path / "trajectory.hdf5", "x")
            self._file.attrs["loom_schema_version"] = SCHEMA_VERSION
            self._file.attrs["complete"] = False
            self._demo = self._file.create_group("data/demo_0")
            self._demo.attrs["seed"] = spec.seed
            self._demo.create_group("observations")
            self._demo.create_group("world_state")
            self._demo.create_dataset(
                "timestamps", shape=(0,), maxshape=(None,), dtype="float64"
            )
            for group in ("input_actions", "applied_control_targets"):
                for key, part in spec.collection.deployment.action_slices.items():
                    self._demo.create_dataset(
                        f"{group}/{key}",
                        shape=(0, part.stop - part.start),
                        maxshape=(None, part.stop - part.start),
                        dtype="float64",
                    )
        except BaseException:
            self.close()
            raise

    def _require(self, state: str) -> None:
        if self._state != state or self._file is None:
            raise RuntimeError(f"Writer is {self._state}; expected {state}")

    def _append(self, key: str, value: Any) -> None:
        value = np.asarray(value)
        if key not in self._demo:
            self._demo.create_dataset(
                key,
                shape=(0, *value.shape),
                maxshape=(None, *value.shape),
                dtype=value.dtype,
                compression="gzip",
                shuffle=True,
                # A streamed image must not rewrite a multi-frame chunk.
                chunks=(1, *value.shape) if value.ndim >= 2 else True,
            )
        dataset = self._demo[key]
        if dataset.shape[1:] != value.shape or dataset.dtype != value.dtype:
            raise ValueError(f"Array shape or dtype changed during episode: {key}")
        index = len(dataset)
        dataset.resize(index + 1, axis=0)
        dataset[index] = value

    def _frame(self, frame: Frame) -> None:
        for camera in self.spec.collection.deployment.cameras:
            if camera.name not in self._videos:
                self._videos[camera.name] = CameraVideoWriter(
                    self.path / video_path(camera.name),
                    camera,
                    self.spec.collection.deployment.control_dt,
                )
            self._videos[camera.name].append(
                frame.observation.values[f"cameras/{camera.name}/rgb"]
            )
        self._append("timestamps", np.float64(frame.observation.timestamp))
        for group, values in (
            ("observations", frame.observation.values),
            ("world_state", frame.world_state),
        ):
            for key, value in values.items():
                if group == "observations" and key.endswith("/rgb"):
                    continue
                self._append(f"{group}/{key}", value)

    def begin(self, initial: Frame) -> None:
        self._require("created")
        initial.observation.validate(self.spec.collection.deployment)
        if not math.isclose(initial.observation.timestamp, 0.0, abs_tol=1e-9):
            raise ValueError("Initial observation must be at episode time zero")
        try:
            self._frame(initial)
            self._world_keys = set(initial.world_state)
            self._file.flush()
            self._state = "recording"
        except BaseException as error:
            self.abort(f"initial_write: {type(error).__name__}: {error}")
            raise

    def append(self, action: Any, applied_control: Any, frame: Frame) -> None:
        self._require("recording")
        deployment = self.spec.collection.deployment
        action = deployment.validate_action(action)
        applied_control = deployment.validate_action(applied_control)
        frame.observation.validate(deployment)
        expected_time = (self.steps + 1) * deployment.control_dt
        if not math.isclose(
            frame.observation.timestamp, expected_time, rel_tol=1e-7, abs_tol=1e-9
        ):
            raise ValueError("Observation timestamp does not match the control clock")
        if set(frame.world_state) != self._world_keys:
            raise ValueError("World-state fields changed during episode")
        try:
            for group, values in (
                ("input_actions", action),
                ("applied_control_targets", applied_control),
            ):
                for key, part in deployment.action_slices.items():
                    self._append(f"{group}/{key}", values[part])
            self._frame(frame)
            self._file.flush()
            self.steps += 1
        except BaseException as error:
            self.abort(f"transition_write: {type(error).__name__}: {error}")
            raise

    def event(self, event: Event) -> None:
        self._require("recording")
        if event.step > self.steps:
            raise ValueError("Event references an unrecorded observation")
        self._events.append(event)
        # Events are also retained if a later transition is interrupted.
        with (self.path / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(
                json.dumps(plain(event), ensure_ascii=False, allow_nan=False) + "\n"
            )

    def finish(self, outcome: Outcome) -> Path:
        self._require("recording")
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "id": self.spec.id,
            "complete": True,
            "num_steps": self.steps,
            "spec": self.spec,
            "action_descriptor": self.spec.collection.deployment.action_descriptor(),
            "observation_descriptor": observation_shapes(
                self.spec.collection.deployment
            ),
            "outcome": outcome,
            "events": self._events,
        }
        try:
            for video in self._videos.values():
                video.close()
            manifest["camera_videos"] = {
                camera.name: {
                    "path": video_path(camera.name),
                    "num_frames": self.steps + 1,
                    "fps": 1 / self.spec.collection.deployment.control_dt,
                    **VIDEO_ENCODING,
                    "sha256": sha256(self.path / video_path(camera.name)),
                }
                for camera in self.spec.collection.deployment.cameras
            }
            self._demo.attrs["num_samples"] = self.steps
            self._demo.attrs["success"] = outcome.code == "success"
            self._file.attrs["complete"] = True
            self._file.flush()
            self._file.close()
            self._file = None
            _write_json(self.path / "manifest.json", manifest)
            validate_episode(self.path)
            self.destination.parent.mkdir(parents=True, exist_ok=True)
            # Both directories are on the same filesystem. A completed episode
            # contains files, so POSIX rename cannot replace it with another dir.
            (self.path / "attempt.json").unlink()
            (self.path / "events.jsonl").unlink(missing_ok=True)
            self.path.rename(self.destination)
            self._state = "committed"
            return self.destination
        except BaseException as error:
            self.abort(f"commit: {type(error).__name__}: {error}")
            raise

    def abort(self, reason: str) -> str | None:
        """Best-effort cleanup; report cleanup I/O errors without hiding the cause."""
        if self._state in {"committed", "aborted"}:
            return None
        errors = []
        try:
            self.close()
        except OSError as error:
            errors.append(f"close: {error}")
        self._state = "aborted"
        try:
            _write_json(
                self.path / "attempt.json",
                {
                    "spec": self.spec,
                    "state": "aborted",
                    "reason": reason,
                    "last_complete_step": self.steps,
                },
            )
        except OSError as error:
            errors.append(f"attempt_log: {error}")
        return "; ".join(errors) or None

    def close(self) -> None:
        """Close resources without publishing; safe for interruption cleanup."""
        for video in self._videos.values():
            video.abort()
        if self._file is not None:
            try:
                self._file.close()
            finally:
                self._file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._state != "committed":
            self.abort(
                f"{exc_type.__name__}: {exc_value}" if exc_type else "not_finished"
            )


def read_manifest(path: str | Path) -> dict[str, Any]:
    with (Path(path) / "manifest.json").open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    if (
        type(manifest.get("schema_version")) is not int
        or manifest["schema_version"] != SCHEMA_VERSION
        or manifest.get("complete") is not True
    ):
        raise ValueError("Episode is incomplete or uses an unsupported schema")
    return manifest


def validate_episode(path: str | Path) -> dict[str, Any]:
    """Validate metadata, T/T+1 alignment, command limits and sensor timestamps."""
    path = Path(path)
    manifest = read_manifest(path)
    spec = episode_from_dict(manifest["spec"])
    deployment = spec.collection.deployment
    steps = manifest["num_steps"]
    if type(steps) is not int or not 0 <= steps <= spec.collection.max_steps:
        raise ValueError("Invalid episode step count")
    if manifest["id"] != spec.id or path.name != spec.id:
        raise ValueError("Episode ids disagree")
    if manifest["action_descriptor"] != deployment.action_descriptor() or (
        manifest["observation_descriptor"] != plain(observation_shapes(deployment))
    ):
        raise ValueError("Descriptors disagree with the resolved deployment")
    outcome = Outcome(**manifest["outcome"])
    for value in manifest["events"]:
        if Event(**value).step > steps:
            raise ValueError("Event references an observation beyond the episode")
    with h5py.File(path / "trajectory.hdf5", "r") as stream:
        if stream.attrs.get(
            "loom_schema_version"
        ) != SCHEMA_VERSION or not stream.attrs.get("complete", False):
            raise ValueError("HDF5 schema or completion marker is invalid")
        demo = stream["data/demo_0"]
        if set(demo) != {
            "timestamps",
            "observations",
            "world_state",
            "input_actions",
            "applied_control_targets",
        }:
            raise ValueError("HDF5 episode groups do not match the schema")
        if (
            demo.attrs["num_samples"] != steps
            or bool(demo.attrs["success"]) != (outcome.code == "success")
            or demo.attrs["seed"] != spec.seed
        ):
            raise ValueError("HDF5 attributes disagree with the manifest")
        timestamps = demo["timestamps"][:]
        if timestamps.shape != (steps + 1,) or not np.allclose(
            timestamps,
            np.arange(steps + 1) * deployment.control_dt,
            rtol=1e-7,
            atol=1e-9,
        ):
            raise ValueError("Expected T+1 observations on the control time axis")
        obs = _datasets(demo["observations"])
        if set(obs) != {
            key for key in observation_shapes(deployment) if not key.endswith("/rgb")
        }:
            raise ValueError("Recorded observations do not match deployment")
        for key, dataset in _datasets(demo).items():
            length = (
                steps
                if key.startswith(("input_actions/", "applied_control_targets/"))
                else steps + 1
            )
            if dataset.ndim == 0 or len(dataset) != length:
                raise ValueError(f"Invalid time dimension for {key}")
            if dataset.dtype.kind not in "biuf":
                raise ValueError(f"Non-numeric dataset: {key}")
            # Bound memory usage even for camera recordings.
            for start in range(0, length, 64):
                if not np.isfinite(dataset[start : start + 64]).all():
                    raise ValueError(f"Non-finite values in {key}")
        for observation in _video_observations(
            path, manifest, demo["observations"], timestamps
        ):
            observation.validate(deployment)
        for group in ("input_actions", "applied_control_targets"):
            datasets = _datasets(demo[group])
            if set(datasets) != set(deployment.action_slices):
                raise ValueError(f"Invalid action fields in {group}")
            for key, part in deployment.action_slices.items():
                if datasets[key].shape != (steps, part.stop - part.start):
                    raise ValueError(f"Invalid action dimension: {group}/{key}")
            for step in range(steps):
                deployment.validate_action(
                    np.concatenate(
                        [datasets[key][step] for key in deployment.action_slices]
                    )
                )
    return manifest


class EpisodeReader:
    """Offline access; model observations and simulator truth have separate APIs."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        if (
            self.path.parent.name == ".incomplete"
            or (self.path / "attempt.json").exists()
        ):
            raise ValueError("Uncommitted attempts cannot be opened as episodes")
        self.manifest = validate_episode(self.path)
        self.spec = episode_from_dict(self.manifest["spec"])
        self._file = h5py.File(self.path / "trajectory.hdf5", "r")
        self._demo = self._file["data/demo_0"]
        self._video_readers = {}

    def __len__(self) -> int:
        return self.manifest["num_steps"]

    def observation(self, step: int) -> Observation:
        if not 0 <= step <= len(self):
            raise IndexError(step)
        values = {
            key: value[step]
            for key, value in _datasets(self._demo["observations"]).items()
        }
        for camera in self.spec.collection.deployment.cameras:
            if camera.name not in self._video_readers:
                self._video_readers[camera.name] = imageio.get_reader(
                    self.path / video_path(camera.name), format="FFMPEG"
                )
            values[f"cameras/{camera.name}/rgb"] = self._video_readers[
                camera.name
            ].get_data(step)
        return Observation(float(self._demo["timestamps"][step]), values)

    def observations(self) -> Iterator[Observation]:
        """Iterate in control-time order with bounded batch reads for video export."""
        yield from _video_observations(
            self.path,
            self.manifest,
            self._demo["observations"],
            self._demo["timestamps"][:],
        )

    def world_state(self, step: int) -> dict[str, np.ndarray]:
        if not 0 <= step <= len(self):
            raise IndexError(step)
        return {
            key: value[step]
            for key, value in _datasets(self._demo["world_state"]).items()
        }

    def action(self, step: int, *, applied: bool = False) -> np.ndarray:
        if not 0 <= step < len(self):
            raise IndexError(step)
        group = "applied_control_targets" if applied else "input_actions"
        return np.concatenate(
            [
                self._demo[f"{group}/{key}"][step]
                for key in self.spec.collection.deployment.action_slices
            ]
        )

    def close(self):
        for reader in self._video_readers.values():
            reader.close()
        self._video_readers.clear()
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def iter_episodes(root: str | Path) -> Iterator[Path]:
    """Discover only committed, validated episodes; corruption fails explicitly."""
    for manifest in sorted((Path(root) / "episodes").glob("*/manifest.json")):
        validate_episode(manifest.parent)
        yield manifest.parent


def build_index(root: str | Path, destination: str | Path) -> int:
    """Rebuild a derived JSONL index; the episode manifests remain authoritative."""
    root, destination = Path(root), Path(destination)
    if not root.is_dir():
        raise FileNotFoundError(f"Episode root does not exist: {root}")
    if destination.resolve().is_relative_to((root / "episodes").resolve()) or (
        destination.resolve().is_relative_to((root / ".incomplete").resolve())
    ):
        raise ValueError("An index cannot overwrite episode or attempt contents")
    destination.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=destination.parent, delete=False
        ) as stream:
            temporary = Path(stream.name)
            for path in iter_episodes(root):
                manifest = read_manifest(path)
                collection = manifest["spec"]["collection"]
                row = {
                    "id": manifest["id"],
                    "path": str(path.relative_to(root)),
                    "task_id": collection["task"]["id"],
                    "deployment_id": collection["deployment"]["id"],
                    "num_steps": manifest["num_steps"],
                    "outcome": manifest["outcome"]["code"],
                }
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
                count += 1
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return count
