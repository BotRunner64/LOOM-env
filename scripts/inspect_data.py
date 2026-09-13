#!/usr/bin/env python3
"""Validate configuration/episodes or rebuild an offline episode index."""

import argparse
import json
from pathlib import Path

from loom_env.data.episodes import build_index, validate_episode
from loom_env.specs.config import load_collection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    config = commands.add_parser(
        "config", help="Resolve and validate a collection YAML"
    )
    config.add_argument("path", type=Path)
    episode = commands.add_parser(
        "episode", help="Validate one committed episode directory"
    )
    episode.add_argument("path", type=Path)
    index = commands.add_parser(
        "index", help="Validate episodes and rebuild a JSONL index"
    )
    index.add_argument("root", type=Path)
    index.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "config":
            spec = load_collection(args.path)
            from loom_env.tasks import create_task
            from loom_env.scenes.workspace import sample_objects

            create_task(spec)
            sample_objects(spec.scene, 0)
            result = {
                "task": spec.task.id,
                "instruction": spec.instruction,
                "deployment": spec.deployment.id,
                "scene": spec.scene.id,
                "max_steps": spec.max_steps,
                "action_descriptor": spec.deployment.action_descriptor(),
            }
        elif args.command == "episode":
            manifest = validate_episode(args.path)
            result = {
                key: manifest[key]
                for key in (
                    "schema_version",
                    "id",
                    "num_steps",
                    "outcome",
                    "camera_videos",
                )
            }
        else:
            result = {
                "episodes": build_index(args.root, args.output),
                "index": str(args.output),
            }
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.exit(1, f"{error}\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
