"""Small structural interfaces with no Isaac Lab, torch, or cuRobo dependency."""

from collections.abc import Mapping
from typing import Protocol

import numpy as np

from loom_env.specs.config import EpisodeSpec
from loom_env.specs.episode import (
    Action,
    EpisodeInput,
    Frame,
    Observation,
    TaskStatus,
    Transition,
)


class ActionSource(Protocol):
    def reset(self, episode_input: EpisodeInput) -> None: ...

    def act(self, observation: Observation) -> Action: ...


class Task(Protocol):
    def reset(self, initial_state: Mapping[str, np.ndarray]) -> None: ...

    def update(
        self, world_state: Mapping[str, np.ndarray], dt: float
    ) -> TaskStatus: ...


class Environment(Protocol):
    def reset_episode(self, spec: EpisodeSpec) -> Frame:
        """Restore an already resolved initial state; exclude stabilization steps."""
        ...

    def step(self, action: np.ndarray) -> Transition:
        """Execute both arms for one control period; never automatically reset."""
        ...
