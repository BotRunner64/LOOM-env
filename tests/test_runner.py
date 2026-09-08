from dataclasses import replace
import json

import numpy as np
import pytest

from loom_env.data.episodes import EpisodeReader, EpisodeWriter, iter_episodes
from loom_env.runtime.replay import RecordedActions
from loom_env.runtime.runner import EpisodeRunner, SourceFailure
from loom_env.specs.episode import Action, Event, Outcome, TaskStatus, Transition


class TestEnvironment:
    __test__ = False

    def __init__(self, spec, frame_factory, error_step=None):
        self.spec, self.make = spec, frame_factory
        self.step_count = 0
        self.reset_count = 0
        self.error_step = error_step
        self.actions = []

    def reset_episode(self, spec):
        self.reset_count += 1
        self.step_count = 0
        return self.make(spec)

    def step(self, action):
        self.step_count += 1
        if self.step_count == self.error_step:
            raise RuntimeError("physics interrupted")
        self.actions.append(action.copy())
        target = action.copy()
        target[0] += 0.01
        action[:] = 0  # Exercise adapters that reuse their argument buffer.
        return Transition(self.make(self.spec, self.step_count), target)


class TestSource:
    __test__ = False

    def __init__(self, action, error=None):
        self.action, self.error = action, error
        self.calls = 0

    def reset(self, episode_input):
        self.input = episode_input
        self.calls = 0

    def act(self, observation):
        self.calls += 1
        if self.error and self.calls == 2:
            raise self.error
        assert not any("cube" in key for key in observation.values)
        events = (
            (Event("planning", "result", 0, {"success": True}),)
            if self.calls == 1
            else ()
        )
        return Action(self.action, events)


class TestTask:
    __test__ = False

    def __init__(self, stop=2, code="success"):
        self.stop, self.code = stop, code

    def reset(self, initial_state):
        self.steps = 0

    def update(self, world_state, dt):
        assert "cube/pose_world" in world_state
        self.steps += 1
        return (
            TaskStatus(Outcome(self.code, "test_predicate"))
            if self.steps == self.stop
            else TaskStatus()
        )


@pytest.mark.parametrize(
    "stop, code, expected_steps, expected",
    [
        (2, "success", 2, "success"),
        (2, "task_failure", 2, "task_failure"),
        (99, "success", 3, "timeout"),
        (3, "success", 3, "success"),
    ],
)
def test_runner_outcomes_terminal_frame_and_input_isolation(
    tmp_path,
    spec,
    frame_factory,
    action,
    stop,
    code,
    expected_steps,
    expected,
):
    env = TestEnvironment(spec, frame_factory)
    source = TestSource(action)
    result = EpisodeRunner(env, TestTask(stop, code), source).run(spec, tmp_path)
    assert result.outcome.code == expected and result.steps == expected_steps
    assert env.reset_count == 1
    assert not hasattr(source.input, "initial_state")
    assert not hasattr(source.input, "spec")
    with EpisodeReader(result.episode_path) as episode:
        assert episode.observation(expected_steps).timestamp == pytest.approx(
            expected_steps * 0.05
        )
        np.testing.assert_array_equal(episode.action(0), action)
        assert episode.manifest["events"][0]["details"]["success"] is True
        assert episode.manifest["outcome"]["code"] == expected


def test_planner_failure_preserves_prefix_without_task_failure(
    tmp_path, spec, frame_factory, action
):
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory),
        TestTask(99),
        TestSource(action, SourceFailure("no_collision_free_path")),
    ).run(spec, tmp_path)
    assert result.outcome.code == "runtime_error" and result.steps == 1
    with EpisodeReader(result.episode_path) as episode:
        event = episode.manifest["events"][-1]
        assert event["kind"] == "planning" and event["details"]["success"] is False
        assert "no_collision_free_path" in result.outcome.reason


def test_environment_failure_does_not_publish_a_false_terminal_frame(
    tmp_path, spec, frame_factory, action
):
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory, error_step=2),
        TestTask(99),
        TestSource(action),
    ).run(spec, tmp_path)
    assert result.outcome.code == "runtime_error" and result.steps == 1
    assert result.episode_path is None
    assert list(iter_episodes(tmp_path)) == []
    assert (
        "environment_step"
        in json.loads((result.attempt_path / "attempt.json").read_text())["reason"]
    )


def test_recorder_failure_is_structured_and_unpublished(
    tmp_path, spec, frame_factory, action, monkeypatch
):
    def fail_append(*args):
        raise OSError("disk full")

    monkeypatch.setattr(EpisodeWriter, "append", fail_append)
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory), TestTask(), TestSource(action)
    ).run(spec, tmp_path)
    assert result.episode_path is None and "recording" in result.outcome.reason


def test_keyboard_interrupt_propagates_after_cleanup(
    tmp_path, spec, frame_factory, action
):
    with pytest.raises(KeyboardInterrupt):
        EpisodeRunner(
            TestEnvironment(spec, frame_factory),
            TestTask(99),
            TestSource(action, KeyboardInterrupt()),
        ).run(spec, tmp_path)
    assert list(iter_episodes(tmp_path)) == []
    assert (
        "KeyboardInterrupt"
        in (tmp_path / ".incomplete" / spec.id / "attempt.json").read_text()
    )


def test_replay_uses_same_runner_and_exact_recorded_input_actions(
    tmp_path, spec, frame_factory, action
):
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory), TestTask(), TestSource(action)
    ).run(spec, tmp_path)
    with EpisodeReader(result.episode_path) as episode:
        replay_spec = replace(spec, id="replay")
        env = TestEnvironment(replay_spec, frame_factory)
        replay = EpisodeRunner(env, TestTask(), RecordedActions(episode)).run(
            replay_spec, tmp_path
        )
        assert replay.outcome.code == "success"
        for step, actual in enumerate(env.actions):
            np.testing.assert_array_equal(actual, episode.action(step))


def test_invalid_setup_is_not_published(
    tmp_path, spec, frame_factory, action, monkeypatch
):
    env = TestEnvironment(spec, frame_factory)

    def fail_reset(spec):
        raise ValueError("unreachable initial state")

    monkeypatch.setattr(env, "reset_episode", fail_reset)
    result = EpisodeRunner(env, TestTask(), TestSource(action)).run(spec, tmp_path)
    assert result.outcome.code == "invalid_setup"
    assert result.episode_path is None and result.steps == 0


def test_initial_recording_error_is_a_runtime_error(
    tmp_path, spec, frame_factory, action, monkeypatch
):
    def fail_begin(*args):
        raise OSError("disk full")

    monkeypatch.setattr(EpisodeWriter, "begin", fail_begin)
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory), TestTask(), TestSource(action)
    ).run(spec, tmp_path)
    assert result.outcome.code == "runtime_error"
    assert "recording_initial" in result.outcome.reason


def test_cleanup_log_failure_does_not_mask_original_error(
    tmp_path, spec, frame_factory, action, monkeypatch
):
    import loom_env.data.episodes as storage

    original = storage._write_json

    def fail_log(path, value):
        if value.get("state") == "aborted":
            raise OSError("log disk full")
        return original(path, value)

    monkeypatch.setattr(storage, "_write_json", fail_log)
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory, error_step=1),
        TestTask(),
        TestSource(action),
    ).run(spec, tmp_path)
    assert result.outcome.code == "runtime_error" and result.episode_path is None
    assert "physics interrupted" in result.outcome.reason
    assert "log disk full" in result.outcome.reason


def test_output_creation_failure_is_structured(tmp_path, spec, frame_factory, action):
    output = tmp_path / "not-a-directory"
    output.write_text("occupied")
    result = EpisodeRunner(
        TestEnvironment(spec, frame_factory), TestTask(), TestSource(action)
    ).run(spec, output)
    assert result.outcome.code == "runtime_error"
    assert "create_writer" in result.outcome.reason


def test_invalid_action_never_reaches_environment(
    tmp_path, spec, frame_factory, action
):
    invalid = action.copy()
    invalid[0] = 100
    env = TestEnvironment(spec, frame_factory)
    result = EpisodeRunner(env, TestTask(), TestSource(invalid)).run(spec, tmp_path)
    assert result.outcome.code == "runtime_error"
    assert env.step_count == 0
    with EpisodeReader(result.episode_path) as episode:
        assert len(episode) == 0
        assert episode.observation(0).timestamp == 0
