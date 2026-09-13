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
    assert set(p.name for p in path.iterdir()) == {
        "manifest.json",
        "trajectory.hdf5",
        "cameras",
    }
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


def test_no_camera_episode(tmp_path, spec, frame_factory):
    spec = replace(
        spec,
        collection=replace(
            spec.collection,
            deployment=replace(spec.collection.deployment, cameras=()),
        ),
    )
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        path = writer.finish(Outcome("success", "test"))
    with EpisodeReader(path) as episode:
        assert episode.manifest["camera_videos"] == {}
        assert len(list(episode.observations())) == 1
        assert not (path / "cameras").exists()


def test_video_random_access_across_keyframes(tmp_path, spec, frame_factory, action):
    camera = CameraSpec("front", 65, 49, 2, "world", (0, 0, 1, 0, 0, 0, 1))
    spec = replace(
        spec,
        collection=replace(
            spec.collection,
            max_steps=44,
            deployment=replace(spec.collection.deployment, cameras=(camera,)),
        ),
    )
    originals = []
    with EpisodeWriter(tmp_path, spec) as writer:
        for step in range(45):
            frame = frame_factory(spec, step)
            values = dict(frame.observation.values)
            # Repeat images on non-capture ticks, as the real camera does.
            color = (30 + (step // 2) * 8, 180 - (step // 2) * 5, 50)
            rgb = np.full((49, 65, 3), color, dtype=np.uint8)
            values["cameras/front/rgb"] = rgb
            originals.append(rgb)
            frame = replace(
                frame, observation=replace(frame.observation, values=values)
            )
            if step == 0:
                writer.begin(frame)
            else:
                writer.append(action, action, frame)
        path = writer.finish(Outcome("timeout", "test"))
    with h5py.File(path / "trajectory.hdf5") as f:
        assert "rgb" not in f["data/demo_0/observations/cameras/front"]
    with EpisodeReader(path) as episode:
        sequential = list(episode.observations())
        assert len(sequential) == 45
        for step in (44, 0, 21, 19, 40, 1, 43):
            obs = episode.observation(step)
            rgb = obs.values["cameras/front/rgb"]
            np.testing.assert_array_equal(
                rgb, sequential[step].values["cameras/front/rgb"]
            )
            np.testing.assert_allclose(rgb, originals[step], atol=5)
            assert bool(obs.values["cameras/front/valid"]) == (step % 2 == 0)
            assert obs.values["cameras/front/timestamp"] == pytest.approx(
                (step // 2) * 0.1
            )


@pytest.mark.parametrize(
    "corruption", ["missing", "bytes", "frames", "path", "short_video"]
)
def test_camera_video_corruption(tmp_path, spec, frame_factory, action, corruption):
    from loom_env.data.camera_video import CameraVideoWriter, sha256

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
    video = path / "cameras/front.mp4"
    manifest = json.loads((path / "manifest.json").read_text())
    if corruption == "missing":
        video.unlink()
    elif corruption == "bytes":
        data = bytearray(video.read_bytes())
        data[len(data) // 2] ^= 1
        video.write_bytes(data)
    elif corruption == "frames":
        manifest["camera_videos"]["front"]["num_frames"] = 1
    elif corruption == "path":
        manifest["camera_videos"]["front"]["path"] = "../../outside.mp4"
    else:
        video.unlink()
        encoder = CameraVideoWriter(video, camera, 0.05)
        encoder.append(np.zeros((6, 8, 3), dtype=np.uint8))
        encoder.close()
        manifest["camera_videos"]["front"]["sha256"] = sha256(video)
    (path / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        validate_episode(path)
    with pytest.raises(ValueError):
        list(iter_episodes(tmp_path))


def test_encoder_exit_failure_cannot_publish(
    tmp_path, spec, frame_factory, monkeypatch
):
    import loom_env.data.camera_video as video

    # A real subprocess exits unsuccessfully, including after a buffered write.
    monkeypatch.setattr(video.imageio_ffmpeg, "get_ffmpeg_exe", lambda: "/bin/false")
    with pytest.raises(OSError):
        with EpisodeWriter(tmp_path, spec) as writer:
            writer.begin(frame_factory(spec))
            writer.finish(Outcome("success", "test"))
    assert list(iter_episodes(tmp_path)) == []
    attempt = json.loads(
        (tmp_path / ".incomplete" / spec.id / "attempt.json").read_text()
    )
    assert attempt["state"] == "aborted"
    assert all(v._process.poll() is not None for v in writer._videos.values())
