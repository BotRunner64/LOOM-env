#!/usr/bin/env python3
"""Collect physical manipulation episodes with the shared LOOM Runner."""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path
import traceback

from loom_env.specs.config import load_collection, plain

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--collection", type=Path, default=ROOT / "configs/collection/pick_place.yaml"
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/manipulation")
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    parser.add_argument(
        "--episode-id",
        default="episode-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"),
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--arm", choices=("left", "right"))
    parser.add_argument("--max-steps", type=int)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes must be positive")
    collection = load_collection(args.collection)
    if args.arm:
        collection = replace(collection, arm_roles={"manipulator": args.arm})
    if args.max_steps is not None:
        collection = replace(collection, max_steps=args.max_steps)
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(
        headless=True, enable_cameras=bool(collection.deployment.cameras)
    )
    code = 0
    env = source = None
    try:
        from loom_env.runtime.build import create_environment, create_expert
        from loom_env.runtime.runner import EpisodeRunner
        from loom_env.tasks import create_task

        env = create_environment(collection, args.asset_root)
        source = create_expert(collection, env, args.asset_root)
        runner = EpisodeRunner(env, create_task(collection), source)
        for i in range(args.episodes):
            episode_id = (
                args.episode_id if args.episodes == 1 else f"{args.episode_id}-{i:04d}"
            )
            spec = env.resolve_episode(
                episode_id,
                args.seed + i,
                provenance={"expert": type(source).__name__, "planner": "cuRobo"},
            )
            result = runner.run(spec, args.output_dir)
            print(
                "RESULT " + json.dumps(plain(result), ensure_ascii=False, default=str),
                flush=True,
            )
            if result.outcome.code != "success":
                code = 1
    except Exception:
        traceback.print_exc()
        code = 1
    finally:
        if source is not None:
            source.planner.planner.destroy()
        if env is not None:
            env.close()
        launcher.app.close(exit_code=code)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
