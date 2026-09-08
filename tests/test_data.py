from dataclasses import replace
import json

import h5py
import numpy as np
import pytest

from loom_env.data.episodes import (
    EpisodeReader,
    EpisodeWriter,
    build_index,
    iter_episodes,
    validate_episode,
)
from loom_env.specs.config import CameraSpec
from loom_env.specs.episode import Outcome


def test_roundtrip_alignment_terminal_frame_and_camera_time(
    tmp_path, spec, frame_factory, action
):
    camera = CameraSpec("front", 8, 6, 2, "world", (0, 0, 1, 0, 0, 0, 1))
    spec = replace(
        spec,
        collection=replace(
            spec.collection,
            deployment=replace(spec.collection.deployment, cameras=(camera,)),
        ),
    )
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        for step in range(1, 4):
            target = action.copy()
            target[0] += 0.01
            writer.append(
                action, target, frame_factory(spec, step, position=(step, 0.0, 0.8))
            )
        path = writer.finish(Outcome("timeout", "budget"))
    with EpisodeReader(path) as episode:
        assert len(episode) == 3
        sequential = list(episode.observations())
        assert [obs.timestamp for obs in sequential] == pytest.approx(
            [0, 0.05, 0.10, 0.15]
        )
        for step, obs in enumerate(sequential):
            for key, value in episode.observation(step).values.items():
                np.testing.assert_array_equal(obs.values[key], value)
        assert episode.observation(3).timestamp == pytest.approx(0.15)
        np.testing.assert_array_equal(
            episode.world_state(3)["cube/pose_world"][:3], [3, 0, 0.8]
        )
        assert episode.observation(3).values[
            "cameras/front/timestamp"
        ] == pytest.approx(0.1)
        assert not episode.observation(3).values["cameras/front/valid"]
        assert episode.action(0, applied=True)[0] != episode.action(0)[0]
        assert episode.manifest["spec"]["seed"] == 7
        with pytest.raises(IndexError):
            episode.action(3)
    assert list(iter_episodes(tmp_path)) == [path]
    assert build_index(tmp_path, tmp_path / "index.jsonl") == 1
    assert json.loads((tmp_path / "index.jsonl").read_text())["num_steps"] == 3
    with pytest.raises(FileExistsError):
        EpisodeWriter(tmp_path, spec)


def test_interruption_remains_discoverable_but_unpublished(
    tmp_path, spec, frame_factory, action
):
    with pytest.raises(KeyboardInterrupt):
        with EpisodeWriter(tmp_path, spec) as writer:
            writer.begin(frame_factory(spec))
            writer.append(action, action, frame_factory(spec, 1))
            raise KeyboardInterrupt
    assert list(iter_episodes(tmp_path)) == []
    attempt = json.loads(
        (tmp_path / ".incomplete" / spec.id / "attempt.json").read_text()
    )
    assert attempt["state"] == "aborted" and attempt["last_complete_step"] == 1
    assert "KeyboardInterrupt" in attempt["reason"]
    with pytest.raises(RuntimeError):
        writer.finish(Outcome("success", "bad"))


def test_partial_hdf5_write_cannot_commit(
    tmp_path, spec, frame_factory, action, monkeypatch
):
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        original = writer._append

        def broken(key, value):
            original(key, value)
            if key == "input_actions/left/gripper":
                raise OSError("disk full")

        monkeypatch.setattr(writer, "_append", broken)
        with pytest.raises(OSError, match="disk full"):
            writer.append(action, action, frame_factory(spec, 1))
        with pytest.raises(RuntimeError):
            writer.finish(Outcome("success", "bad"))
    assert list(iter_episodes(tmp_path)) == []


def test_publish_failure_keeps_index_unmodified(
    tmp_path, spec, frame_factory, monkeypatch
):
    from pathlib import Path

    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))

        def fail_rename(*args):
            raise OSError("publish failed")

        monkeypatch.setattr(Path, "rename", fail_rename)
        with pytest.raises(OSError):
            writer.finish(Outcome("runtime_error", "source unavailable"))
    assert (tmp_path / ".incomplete" / spec.id / "manifest.json").exists()
    assert list(iter_episodes(tmp_path)) == []


@pytest.mark.parametrize(
    "corruption", ["alignment", "nan", "action", "camera_time", "descriptor"]
)
def test_corruption_is_rejected_before_indexing(
    tmp_path, spec, frame_factory, action, corruption
):
    camera = CameraSpec("front", 8, 6, 1, "world", (0, 0, 1, 0, 0, 0, 1))
    spec = replace(
        spec,
        collection=replace(
            spec.collection,
            deployment=replace(spec.collection.deployment, cameras=(camera,)),
        ),
    )
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        writer.append(action, action, frame_factory(spec, 1))
        path = writer.finish(Outcome("success", "test"))
    if corruption == "descriptor":
        manifest = json.loads((path / "manifest.json").read_text())
        manifest["action_descriptor"]["dimension"] = 14
        (path / "manifest.json").write_text(json.dumps(manifest))
    else:
        with h5py.File(path / "trajectory.hdf5", "r+") as stream:
            demo = stream["data/demo_0"]
            if corruption == "alignment":
                demo["timestamps"].resize(1, axis=0)
            elif corruption == "nan":
                demo["world_state/cube/pose_world"][0, 0] = np.nan
            elif corruption == "action":
                demo["input_actions/left/arm"][0, 0] = 100
            else:
                demo["observations/cameras/front/timestamp"][1] = 1.0
    with pytest.raises(ValueError):
        validate_episode(path)
    index = tmp_path / "index.jsonl"
    index.write_text("previous index\n")
    with pytest.raises(ValueError):
        build_index(tmp_path, index)
    assert index.read_text() == "previous index\n"


def test_index_cannot_overwrite_episode_and_attempt_data(tmp_path, spec, frame_factory):
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        path = writer.finish(Outcome("runtime_error", "source unavailable"))
    for target in (path / "manifest.json", tmp_path / ".incomplete" / "index"):
        with pytest.raises(ValueError, match="overwrite"):
            build_index(tmp_path, target)
    assert set(p.name for p in path.iterdir()) == {"manifest.json", "trajectory.hdf5"}
    validate_episode(path)


def test_unpublished_manifest_is_not_readable(
    tmp_path, spec, frame_factory, monkeypatch
):
    from pathlib import Path

    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))

        def fail_rename(*args):
            raise OSError("rename interrupted")

        monkeypatch.setattr(Path, "rename", fail_rename)
        with pytest.raises(OSError):
            writer.finish(Outcome("runtime_error", "source unavailable"))
    with pytest.raises(ValueError, match="Uncommitted"):
        EpisodeReader(writer.path)
