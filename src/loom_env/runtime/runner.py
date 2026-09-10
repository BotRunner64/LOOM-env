"""One synchronous episode loop for experts, policies and recorded actions."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any
from collections.abc import Mapping

from loom_env.data.episodes import EpisodeWriter
from loom_env.specs.config import EpisodeSpec
from loom_env.specs.episode import (
    EpisodeInput,
    Event,
    Outcome,
    observation_shapes,
)

from .protocols import ActionSource, Environment, Task


class SourceFailure(RuntimeError):
    """An expert/planner may attach a reason without claiming task failure."""

    def __init__(self, reason: str, *, kind: str = "planning"):
        super().__init__(reason)
        if kind not in {"planning", "skill", "runtime"}:
            raise ValueError(f"Unknown source failure kind: {kind}")
        self.kind = kind


@dataclass(frozen=True)
class RunResult:
    outcome: Outcome
    steps: int
    episode_path: Path | None
    attempt_path: Path | None


class EpisodeRunner:
    def __init__(self, environment: Environment, task: Task, source: ActionSource):
        self.environment = environment
        self.task = task
        self.source = source

    def run(
        self,
        spec: EpisodeSpec,
        output_dir: str | Path,
        *,
        context: Mapping[str, Any] | None = None,
    ) -> RunResult:
        """Run an already resolved spec. The caller owns environment/app closure.

        A failed source leaves a valid last frame and can be committed as a
        runtime_error. A failed environment step may have partly advanced
        physics: its attempt remains incomplete, even if T/T+1 still aligns.
        KeyboardInterrupt and SystemExit are recorded, then propagated.
        """
        try:
            writer = EpisodeWriter(output_dir, spec)
        except FileExistsError:
            raise  # Duplicate ids must never overwrite a previous attempt.
        except OSError as error:
            attempt = Path(output_dir) / ".incomplete" / spec.id
            return RunResult(
                Outcome(
                    "runtime_error", f"create_writer: {type(error).__name__}: {error}"
                ),
                0,
                None,
                attempt if attempt.is_dir() else None,
            )
        phase = "reset"
        can_commit = False
        try:
            frame = self.environment.reset_episode(spec)
            phase = "recording_initial"
            writer.begin(frame)
            phase = "task_reset"
            self.task.reset(frame.world_state)
            phase = "source_reset"
            can_commit = True
            deployment = spec.collection.deployment
            self.source.reset(
                EpisodeInput(
                    instruction=spec.collection.instruction,
                    action_descriptor=deployment.action_descriptor(),
                    observation_descriptor=observation_shapes(deployment),
                    context={} if context is None else context,
                )
            )
            outcome = None
            for _ in range(spec.collection.max_steps):
                phase = "action_source"
                command = self.source.act(frame.observation)
                action = deployment.validate_action(command.values)
                for event in command.events:
                    writer.event(event)
                phase = "environment_step"
                can_commit = False
                transition = self.environment.step(action.copy())
                phase = "recording"
                writer.append(action, transition.applied_control, transition.frame)
                frame = transition.frame
                can_commit = True
                phase = "task_update"
                status = self.task.update(frame.world_state, deployment.control_dt)
                if status.outcome is not None:
                    outcome = status.outcome
                    break
            if outcome is None:
                outcome = Outcome("timeout", "control_step_budget_exhausted")
            writer.event(
                Event("task", outcome.code, writer.steps, {"reason": outcome.reason})
            )
            phase = "commit"
            can_commit = False
            path = writer.finish(outcome)
            return RunResult(outcome, writer.steps, path, path)
        except (KeyboardInterrupt, SystemExit) as error:
            writer.abort(f"{phase}: {type(error).__name__}")
            raise
        except Exception as error:
            outcome = Outcome(
                "invalid_setup"
                if phase in {"reset", "task_reset"}
                else "runtime_error",
                f"{phase}: {type(error).__name__}: {error}",
            )
            path = None
            if can_commit:
                try:
                    kind = error.kind if isinstance(error, SourceFailure) else "runtime"
                    writer.event(
                        Event(
                            kind,
                            "failed",
                            writer.steps,
                            {
                                "reason": str(error),
                                "phase": phase,
                                "success": False,
                            },
                        )
                    )
                    path = writer.finish(outcome)
                except Exception as commit_error:
                    outcome = Outcome(
                        "runtime_error", f"{outcome.reason}; commit: {commit_error}"
                    )
            if path is None:
                cleanup_error = writer.abort(outcome.reason)
                if cleanup_error:
                    outcome = Outcome(
                        "runtime_error", f"{outcome.reason}; cleanup: {cleanup_error}"
                    )
            return RunResult(outcome, writer.steps, path, path or writer.path)
        finally:
            writer.close()
