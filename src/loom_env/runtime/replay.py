"""Recorded actions implement the same source interface as a policy."""

from loom_env.data.episodes import EpisodeReader
from loom_env.specs.config import plain
from loom_env.specs.episode import Action, EpisodeInput, Observation


class RecordedActions:
    def __init__(self, episode: EpisodeReader):
        self.episode = episode
        self._step = 0

    def reset(self, episode_input: EpisodeInput) -> None:
        if (
            plain(episode_input.action_descriptor)
            != self.episode.manifest["action_descriptor"]
        ):
            raise ValueError("Replay action layout or control period does not match")
        self._step = 0

    def act(self, observation: Observation) -> Action:
        if self._step >= len(self.episode):
            raise RuntimeError("Recorded actions exhausted before episode termination")
        command = Action(self.episode.action(self._step))
        self._step += 1
        return command
