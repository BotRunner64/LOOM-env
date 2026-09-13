from dataclasses import replace

import imageio.v2 as imageio
import numpy as np
import pytest

from loom_env.data.episodes import EpisodeReader, EpisodeWriter
from loom_env.data.video import export_video
from loom_env.specs.episode import Outcome


def test_video_encodes_all_cameras_and_terminal_frame(
    tmp_path, spec, frame_factory, action
):
    cameras = tuple(
        replace(camera, width=64, height=48)
        for camera in spec.collection.deployment.cameras
    )
    spec = replace(
        spec,
        collection=replace(
            spec.collection,
            deployment=replace(spec.collection.deployment, cameras=cameras),
        ),
    )
    colors = ((220, 10, 10), (10, 220, 10), (10, 10, 220))
    with EpisodeWriter(tmp_path, spec) as writer:
        for step in range(3):
            frame = frame_factory(spec, step)
            values = dict(frame.observation.values)
            for camera, color in zip(cameras, colors):
                values[f"cameras/{camera.name}/rgb"] = np.full(
                    (camera.height, camera.width, 3), color, dtype=np.uint8
                )
            frame = replace(
                frame, observation=replace(frame.observation, values=values)
            )
            if step == 0:
                writer.begin(frame)
            else:
                writer.append(action, action, frame)
        path = writer.finish(Outcome("timeout", "test"))
    output = export_video(path, tmp_path / "videos" / "test.mp4")
    with imageio.get_reader(output) as reader:
        frames = [np.asarray(frame) for frame in reader.iter_data()]
        assert reader.get_meta_data()["fps"] == pytest.approx(20)
    assert len(frames) == 3
    assert frames[0].shape == (144, 192, 3)
    for index, color in enumerate(colors):
        np.testing.assert_allclose(frames[-1][120, index * 64 + 32], color, atol=8)
    assert output.with_suffix(".png").exists()
    with pytest.raises(ValueError, match="already exists"):
        export_video(path, output)
    with EpisodeReader(path) as episode:
        assert episode.manifest["outcome"]["code"] == "timeout"


def test_failed_encoding_keeps_episode_and_can_retry(
    tmp_path, spec, frame_factory, monkeypatch
):
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        path = writer.finish(Outcome("success", "test"))
    output = tmp_path / "videos" / "retry.mp4"
    original = imageio.get_writer

    def fail(*args, **kwargs):
        args[0].write_bytes(b"partial")
        raise OSError("encoder unavailable")

    with monkeypatch.context() as patch:
        patch.setattr(imageio, "get_writer", fail)
        with pytest.raises(OSError, match="encoder unavailable"):
            export_video(path, output)
    assert not output.exists()
    assert not output.with_name("retry.partial.mp4").exists()
    assert imageio.get_writer is original
    export_video(path, output)
    assert output.exists()


def test_collection_continues_after_video_failure(
    tmp_path, spec, frame_factory, monkeypatch, capsys
):
    import json
    from pathlib import Path
    import runpy
    import sys
    from types import SimpleNamespace

    import loom_env.data.video as video
    import loom_env.runtime.build as build
    import loom_env.runtime.runner as runner

    exits = []
    app = SimpleNamespace(close=lambda **kwargs: exits.append(kwargs["exit_code"]))
    monkeypatch.setitem(
        sys.modules,
        "isaaclab.app",
        SimpleNamespace(AppLauncher=lambda **kwargs: SimpleNamespace(app=app)),
    )
    env = SimpleNamespace(
        resolve_episode=lambda episode_id, seed, **kwargs: replace(
            spec, id=episode_id, seed=seed
        ),
        close=lambda: None,
    )
    source = SimpleNamespace(
        planner=SimpleNamespace(planner=SimpleNamespace(destroy=lambda: None))
    )
    monkeypatch.setattr(build, "create_environment", lambda *args: env)
    monkeypatch.setattr(build, "create_expert", lambda *args: source)

    def run(self, episode_spec, output_dir):
        with EpisodeWriter(output_dir, episode_spec) as writer:
            writer.begin(frame_factory(episode_spec))
            path = writer.finish(Outcome("success", "test"))
        return runner.RunResult(Outcome("success", "test"), 0, path, None)

    monkeypatch.setattr(runner.EpisodeRunner, "run", run)
    original = video.export_video

    def export(path, output):
        if path.name.endswith("0000"):
            raise OSError("test encoding failure")
        return original(path, output)

    monkeypatch.setattr(video, "export_video", export)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "collect.py",
            "--output-dir",
            str(tmp_path),
            "--episode-id",
            "batch",
            "--episodes",
            "2",
        ],
    )
    main = runpy.run_path(
        str(Path(__file__).resolve().parents[1] / "scripts/collect.py")
    )["main"]
    assert main() == 1
    reports = [
        json.loads(line.removeprefix("RESULT "))
        for line in capsys.readouterr().out.splitlines()
        if line.startswith("RESULT ")
    ]
    assert len(reports) == 2
    assert reports[0]["video_error"] == "OSError: test encoding failure"
    assert reports[0]["video_path"] is None
    assert len(reports[0]["camera_video_paths"]) == 3
    assert all(Path(p).exists() for p in reports[0]["camera_video_paths"].values())
    assert Path(reports[0]["episode_path"]).exists()
    assert reports[1]["video_error"] is None
    assert Path(reports[1]["video_path"]).exists()
    assert exits == [1]
