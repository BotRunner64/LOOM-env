#!/usr/bin/env python3
"""Verify a successful physical Episode: reset, replay, faults, and arm-role swap."""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import traceback
from unittest.mock import patch

from loom_env.data.episodes import EpisodeReader, EpisodeWriter, iter_episodes
from loom_env.environments.tabletop import initial_command
from loom_env.runtime.replay import (
    ComparingEnvironment,
    RecordedActions,
    ReplayComparison,
    ReplayTask,
)
from loom_env.runtime.runner import EpisodeRunner
from loom_env.specs.config import plain
from loom_env.specs.episode import Action
from loom_env.tasks.place import PlaceTask


class Hold:
    def __init__(self, command):
        self.command = command

    def reset(self, episode_input):
        pass

    def act(self, observation):
        return Action(self.command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, default=Path(".cache/assets"))
    args = parser.parse_args()
    with EpisodeReader(args.episode) as original:
        if original.manifest["outcome"]["code"] != "success":
            parser.error("The reference must be a successful place episode")
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(
            headless=True,
            enable_cameras=bool(original.spec.collection.deployment.cameras),
        )
        report, code = {}, 1
        try:
            from loom_env.environments.isaac_lab import TabletopEnvironment, _tree_map
            from loom_env.experts.curobo import PandaPlanner
            from loom_env.experts.pick_place import PickPlaceExpert

            spec = original.spec
            env = TabletopEnvironment(spec.collection, args.asset_root)
            command = initial_command(spec.collection.deployment)
            env.reset_episode(spec)
            perturbed = command.copy()
            for side in ("left", "right"):
                perturbed[
                    spec.collection.deployment.action_slices[f"{side}/arm"].start
                ] += 0.15
            for _ in range(20):
                env.step(perturbed)
            comparison = ReplayComparison(original)
            comparison.add(env.reset_episode(spec), 0)
            report["reset_after_motion"] = {
                "passed": all(v < 1e-5 for v in comparison.initial_errors.values()),
                "errors": comparison.initial_errors,
            }

            replay_spec = replace(
                spec,
                id=spec.id + "-replay",
                collection=replace(spec.collection, max_steps=len(original)),
                provenance={
                    **spec.provenance,
                    "source": "physical_replay",
                    "replay_of": str(original.path.resolve()),
                },
            )
            comparison = ReplayComparison(original)
            replay = EpisodeRunner(
                ComparingEnvironment(env, comparison),
                ReplayTask(PlaceTask(spec.collection), len(original)),
                RecordedActions(original),
            ).run(replay_spec, args.output_dir)
            report["replay"] = {**comparison.report(), "result": plain(replay)}
            report["replay"]["passed"] &= replay.outcome.code == "success"
            print(
                "CHECK replay " + json.dumps(report["replay"], default=str), flush=True
            )

            timeout_spec = replace(
                spec,
                id=spec.id + "-timeout",
                collection=replace(spec.collection, max_steps=10),
                provenance={
                    **spec.provenance,
                    "source": "verification",
                    "case": "hold_timeout",
                },
            )
            timeout = EpisodeRunner(env, PlaceTask(spec.collection), Hold(command)).run(
                timeout_spec, args.output_dir
            )
            report["timeout"] = {
                "passed": timeout.outcome.code == "timeout" and timeout.steps == 10,
                "result": plain(timeout),
            }

            # A measured initial state just beyond the tabletop, with gravity on.
            # Falling to failure is simulated; no task result or contact is injected.
            state = plain(spec.initial_state)
            state["scene"]["rigid_object"]["cube"]["root_pose"][0][:3] = [
                1.4,
                0.0,
                0.65,
            ]
            failure_spec = replace(
                spec,
                id=spec.id + "-fall",
                initial_state=state,
                provenance={
                    **spec.provenance,
                    "source": "verification",
                    "case": "unsupported_cube_fall",
                },
            )
            env.reset_episode(failure_spec)
            state["scene"] = _tree_map(
                env.scene.get_state(), lambda v: v.cpu().tolist()
            )
            failure_spec = replace(failure_spec, initial_state=state)
            failure = EpisodeRunner(env, PlaceTask(spec.collection), Hold(command)).run(
                failure_spec, args.output_dir
            )
            report["physical_failure"] = {
                "passed": failure.outcome.code == "task_failure",
                "result": plain(failure),
            }

            interrupted_spec = replace(
                spec,
                id=spec.id + "-interrupted",
                provenance={
                    **spec.provenance,
                    "source": "verification",
                    "case": "interrupt_during_frame_write",
                },
            )
            append = EpisodeWriter._append

            def interrupt(writer, key, value):
                if (
                    writer.steps == 1
                    and key
                    == f"world_state/{spec.collection.role_bindings['target_object']}/pose_world"
                ):
                    raise KeyboardInterrupt(
                        "Injected interruption during second frame write"
                    )
                return append(writer, key, value)

            caught = False
            try:
                with patch.object(EpisodeWriter, "_append", interrupt):
                    EpisodeRunner(env, PlaceTask(spec.collection), Hold(command)).run(
                        interrupted_spec, args.output_dir
                    )
            except KeyboardInterrupt:
                caught = True
            attempt = args.output_dir / ".incomplete" / interrupted_spec.id
            report["write_interruption"] = {
                "passed": caught
                and attempt.is_dir()
                and not (args.output_dir / "episodes" / interrupted_spec.id).exists(),
                "attempt": str(attempt),
            }
            list(
                iter_episodes(args.output_dir)
            )  # Verify indexing excludes the partial HDF5 file.

            side = (
                "left"
                if spec.collection.arm_roles["manipulator"] == "right"
                else "right"
            )
            swapped = replace(
                spec,
                id=spec.id + "-swapped",
                collection=replace(spec.collection, arm_roles={"manipulator": side}),
                provenance={
                    **spec.provenance,
                    "case": "arm_role_swap",
                    "paired_initial_state": spec.id,
                },
            )
            planner = PandaPlanner(swapped.collection, side)
            expert = PickPlaceExpert(swapped.collection, planner, env.world_state)
            swapped_result = EpisodeRunner(
                env, PlaceTask(swapped.collection), expert
            ).run(swapped, args.output_dir)
            report["role_swap"] = {
                "passed": swapped_result.outcome.code == "success",
                "arm": side,
                "result": plain(swapped_result),
            }
            planner.planner.destroy()
            report["passed"] = all(case["passed"] for case in report.values())
            code = 0 if report["passed"] else 1
            env.close()
        except Exception:
            traceback.print_exc()
            report["passed"] = False
        finally:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            with (args.output_dir / "verification.json").open("x") as stream:
                json.dump(report, stream, indent=2, default=str)
            print("VERIFICATION " + json.dumps(report, default=str), flush=True)
            launcher.app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
