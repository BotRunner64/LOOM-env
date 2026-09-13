#!/usr/bin/env python3
"""Restore an Episode snapshot and physically replay every saved action."""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import traceback

from loom_env.data.episodes import EpisodeReader
from loom_env.data.video import export_video
from loom_env.runtime.replay import (
    RecordedActions,
    ReplayComparison,
    ComparingEnvironment,
    ReplayTask,
)
from loom_env.runtime.runner import EpisodeRunner
from loom_env.specs.config import plain
from loom_env.tasks import create_task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--episode-id")
    parser.add_argument("--asset-root", type=Path, default=Path(".cache/assets"))
    args = parser.parse_args()
    with EpisodeReader(args.episode) as original:
        if len(original) == 0:
            parser.error("Physical replay requires at least one recorded action")
        spec = replace(
            original.spec,
            id=args.episode_id or original.spec.id + "-replay",
            collection=replace(original.spec.collection, max_steps=len(original)),
            provenance={
                **original.spec.provenance,
                "source": "physical_replay",
                "replay_of": str(original.path.resolve()),
            },
        )
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(
            headless=True, enable_cameras=bool(spec.collection.deployment.cameras)
        )
        code = 1
        try:
            from loom_env.runtime.build import create_environment

            env = create_environment(spec.collection, args.asset_root)
            comparison = ReplayComparison(original)
            result = EpisodeRunner(
                ComparingEnvironment(env, comparison),
                ReplayTask(create_task(spec.collection), len(original)),
                RecordedActions(original),
            ).run(spec, args.output_dir)
            report = comparison.report()
            report["result"] = plain(result)
            report["video_path"] = None
            report["video_error"] = None
            if result.episode_path is not None and spec.collection.deployment.cameras:
                video_path = args.output_dir / "videos" / f"{spec.id}.mp4"
                try:
                    export_video(result.episode_path, video_path)
                    report["video_path"] = str(video_path)
                except Exception as error:
                    report["video_error"] = f"{type(error).__name__}: {error}"
                    traceback.print_exc()
            report["outcome_matches"] = (
                result.outcome.code == original.manifest["outcome"]["code"]
            )
            report["passed"] &= report["outcome_matches"]
            report["passed"] &= report["video_error"] is None
            path = args.output_dir / f"{spec.id}-comparison.json"
            with path.open("x") as stream:
                json.dump(report, stream, indent=2, default=str)
            print("REPLAY " + json.dumps(report, default=str), flush=True)
            code = 0 if report["passed"] else 1
            env.close()
        except Exception:
            traceback.print_exc()
        finally:
            launcher.app.close(exit_code=code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
