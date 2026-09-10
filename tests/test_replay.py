from dataclasses import replace

import pytest

from loom_env.data.episodes import EpisodeReader, EpisodeWriter
from loom_env.runtime.replay import ReplayComparison, ReplayTask
from loom_env.specs.episode import Outcome, TaskStatus


@pytest.mark.parametrize(
    "change, passed",
    [
        ("none", True),
        ("small_drift", True),
        ("large_drift", False),
        ("initial_drift", False),
        ("missing_terminal", False),
    ],
)
def test_physical_replay_comparison(
    tmp_path, spec, frame_factory, action, change, passed
):
    spec = replace(
        spec,
        collection=replace(
            spec.collection, deployment=replace(spec.collection.deployment, cameras=())
        ),
    )
    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame_factory(spec))
        for step in range(1, 4):
            writer.append(action, action, frame_factory(spec, step))
        path = writer.finish(Outcome("timeout", "test"))
    with EpisodeReader(path) as original:
        comparison = ReplayComparison(original)
        for step in range(3 if change == "missing_terminal" else 4):
            x = 0.5
            if change == "initial_drift" and step == 0:
                x += 0.001
            elif step == 2:
                x += {"small_drift": 0.001, "large_drift": 0.01}.get(change, 0)
            comparison.add(frame_factory(spec, step, position=(x, 0.0, 0.8)), step)
        assert comparison.report()["passed"] is passed


def test_replay_evaluates_predicate_but_executes_terminal_action():
    class AlreadySuccessful:
        def reset(self, world):
            self.calls = 0

        def update(self, world, dt):
            self.calls += 1
            return TaskStatus(Outcome("success", "physical_predicate"))

    physical = AlreadySuccessful()
    replay = ReplayTask(physical, 3)
    replay.reset({})
    assert replay.update({}, 0.05).outcome is None
    assert replay.update({}, 0.05).outcome is None
    assert replay.update({}, 0.05).outcome.code == "success"
    assert physical.calls == 3


def test_replay_detects_drift_of_an_unselected_object(
    tmp_path, spec, frame_factory, action
):
    import numpy as np
    from loom_env.specs.episode import Frame

    objects = {name: dict(obj) for name, obj in spec.collection.scene.objects.items()}
    objects["distractor"] = {**objects["cube"], "description": "蓝色方块"}
    spec = replace(
        spec,
        collection=replace(
            spec.collection,
            scene=replace(spec.collection.scene, objects=objects),
            deployment=replace(spec.collection.deployment, cameras=()),
        ),
    )

    def frame(step, drift=0.0):
        source = frame_factory(spec, step)
        return Frame(
            source.observation,
            {
                **source.world_state,
                "distractor/pose_world": np.array(
                    [0.4 + drift, 0.2, 0.77, 0.0, 0.0, 0.0, 1.0]
                ),
            },
        )

    with EpisodeWriter(tmp_path, spec) as writer:
        writer.begin(frame(0))
        for step in range(1, 4):
            writer.append(action, action, frame(step))
        path = writer.finish(Outcome("timeout", "test"))
    with EpisodeReader(path) as original:
        comparison = ReplayComparison(original)
        for step in range(4):
            comparison.add(frame(step, 0.02 if step == 2 else 0.0), step)
        assert comparison.report()["maximum_errors"]["cube_m"] == 0
        assert comparison.report()["maximum_errors"]["distractor_m"] == pytest.approx(
            0.02
        )
        assert not comparison.report()["passed"]
