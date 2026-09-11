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


class ReplayComparison:
    """Compare physical frames at identical control ticks; RGB is not deterministic."""

    def __init__(self, episode):
        self.episode = episode
        self.initial_errors = {}
        self.maximum_errors = {}
        self.frames = 0

    def add(self, frame, step):
        import numpy as np
        from scipy.spatial.transform import Rotation

        expected = self.episode.observation(step)
        truth = self.episode.world_state(step)
        values = frame.observation.values
        errors = {}
        for side in ("left", "right"):
            key = f"robot/{side}/joint_position"
            errors[f"{side}_joint_rad"] = float(
                np.max(np.abs(values[key] - expected.values[key]))
            )
            key = f"robot/{side}/gripper_position"
            errors[f"{side}_gripper_m"] = float(
                np.max(np.abs(values[key] - expected.values[key]))
            )
            key = f"robot/{side}/tcp_pose_world"
            errors[f"{side}_tcp_m"] = float(
                np.linalg.norm(values[key][:3] - expected.values[key][:3])
            )
            errors[f"{side}_tcp_rad"] = float(
                (
                    Rotation.from_quat(values[key][3:].copy()).inv()
                    * Rotation.from_quat(expected.values[key][3:].copy())
                ).magnitude()
            )
        for name, obj in self.episode.spec.collection.scene.objects.items():
            if obj["static"]:
                continue
            actual, desired = (
                frame.world_state[f"{name}/pose_world"],
                truth[f"{name}/pose_world"],
            )
            errors[f"{name}_m"] = float(np.linalg.norm(actual[:3] - desired[:3]))
            errors[f"{name}_rad"] = float(
                (
                    Rotation.from_quat(actual[3:].copy()).inv()
                    * Rotation.from_quat(desired[3:].copy())
                ).magnitude()
            )
        if step == 0:
            self.initial_errors = errors.copy()
        for key, value in errors.items():
            self.maximum_errors[key] = max(self.maximum_errors.get(key, 0), value)
        self.frames += 1

    def report(self):
        # Contact simulation can diverge slightly; limits are in physical units.
        limits = {
            key: 0.03 if key.endswith("_rad") else 0.005 for key in self.maximum_errors
        }
        limits.update(
            {
                "left_gripper_m": 0.002,
                "right_gripper_m": 0.002,
                **{
                    f"{name}_rad": 0.1
                    for name, obj in self.episode.spec.collection.scene.objects.items()
                    if not obj["static"]
                },
            }
        )
        initial_ok = bool(self.initial_errors) and all(
            v < 1e-5 for v in self.initial_errors.values()
        )
        return {
            "passed": initial_ok
            and self.frames == len(self.episode) + 1
            and all(v <= limits[k] for k, v in self.maximum_errors.items()),
            "frames": self.frames,
            "initial_errors": self.initial_errors,
            "maximum_errors": self.maximum_errors,
            "limits": limits,
        }


class ComparingEnvironment:
    def __init__(self, environment, comparison):
        self.environment, self.comparison = environment, comparison

    def reset_episode(self, spec):
        self.step_count = 0
        frame = self.environment.reset_episode(spec)
        self.comparison.add(frame, 0)
        return frame

    def step(self, action):
        transition = self.environment.step(action)
        self.step_count += 1
        self.comparison.add(transition.frame, self.step_count)
        return transition


class ReplayTask:
    """Evaluate the physical predicate but execute the complete recorded action list."""

    def __init__(self, task, steps):
        self.task, self.steps = task, steps

    def reset(self, world_state):
        self.count = 0
        self.task.reset(world_state)

    def update(self, world_state, dt):
        from loom_env.specs.episode import TaskStatus

        self.count += 1
        status = self.task.update(world_state, dt)
        return status if self.count == self.steps else TaskStatus()
