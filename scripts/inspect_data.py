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
            from loom_env.scenes.workspace import sample_objects
            from loom_env.tasks import create_task

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
            if manifest["spec"]["collection"]["task"]["id"] == "handover_object":
                import numpy as np

                from loom_env.data.episodes import EpisodeReader
                from loom_env.tasks import create_task

                with EpisodeReader(args.path) as reader:
                    task = create_task(reader.spec.collection)
                    task.reset(reader.world_state(0))
                    transitions = []
                    counts = {"giver_only": 0, "both": 0, "receiver_only": 0}
                    recomputed = None
                    for k in range(1, len(reader) + 1):
                        world = reader.world_state(k)
                        held = world[f"{task.object_name}/grasped_by"]
                        giver, receiver = held[task.giver], held[task.receiver]
                        counts["giver_only"] += int(giver and not receiver)
                        counts["both"] += int(giver and receiver)
                        counts["receiver_only"] += int(receiver and not giver)
                        phase = task.phase
                        status = task.update(
                            world, reader.spec.collection.deployment.control_dt
                        )
                        if phase != task.phase:
                            transitions.append({"step": k, "phase": task.phase})
                        if status.outcome is not None:
                            recomputed = {
                                "step": k,
                                "code": status.outcome.code,
                                "reason": status.outcome.reason,
                            }
                            break
                    final = reader.world_state(len(reader))
                    position = final[f"{task.object_name}/pose_world"][:3]
                    velocity = final[f"{task.object_name}/velocity_world"]
                    points, _ = task._state(final)
                    result["handover_metrics"] = {
                        "grasp_geometry": next(
                            (
                                event["details"]
                                for event in manifest["events"]
                                if event["name"] == "handover_grasp_geometry"
                            ),
                            None,
                        ),
                        "phase_transitions": transitions,
                        "grasp_samples": counts,
                        "recomputed_outcome": recomputed,
                        "final_grasped_by": final[
                            f"{task.object_name}/grasped_by"
                        ].tolist(),
                        "upward_displacement_m": None
                        if task.exchange_position is None
                        else float(position[2] - task.exchange_position[2]),
                        "final_clearance_m": float(points[:, 2].min() - task.support[2]),
                        "final_linear_speed_m_s": float(np.linalg.norm(velocity[:3])),
                        "final_angular_speed_rad_s": float(np.linalg.norm(velocity[3:])),
                        "verified_stable_time_s": task.elapsed
                        if task.phase == "receiver"
                        else 0.0,
                        "final_giver_contact_force_n": float(
                            np.linalg.norm(
                                final[
                                    f"{task.object_name}/finger_contact_forces_world"
                                ][task.giver],
                                axis=1,
                            ).max()
                        ),
                    }
            if manifest["spec"]["collection"]["task"]["id"] in {
                "push_object",
                "push_into_region",
            }:
                import numpy as np

                from loom_env.data.episodes import EpisodeReader
                from loom_env.tasks import create_task

                with EpisodeReader(args.path) as reader:
                    task = create_task(reader.spec.collection)
                    samples = [
                        task.metrics(reader.world_state(k))
                        for k in range(len(reader) + 1)
                    ]
                    other_displacement = {}
                    for name, obj in reader.spec.collection.scene.objects.items():
                        if obj["static"] or name == task.object_name:
                            continue
                        positions = np.asarray(
                            [
                                reader.world_state(k)[f"{name}/pose_world"][:2]
                                for k in range(len(reader) + 1)
                            ]
                        )
                        other_displacement[name] = float(
                            np.linalg.norm(positions - positions[0], axis=1).max()
                        )
                    result["push_metrics"] = {
                        "target_position_world": task.target_position_world.tolist(),
                        "non_target_max_xy_displacement_m": other_displacement,
                        "goal_radius_m": task.parameters["position_tolerance"],
                        "final_position_error_m": samples[-1]["position_error"],
                        "max_clearance_m": max(m["clearance"] for m in samples),
                        "min_clearance_m": min(m["clearance"] for m in samples),
                        "grasped_samples": sum(m["grasped"] for m in samples),
                        "contact_samples": sum(
                            m["contact_force"] > task.parameters["contact_force"]
                            for m in samples
                        ),
                        "sample_period_s": reader.spec.collection.deployment.control_dt,
                    }
                    if reader.spec.collection.task.id == "push_into_region":
                        result["push_metrics"].pop("goal_radius_m")
                        result["push_metrics"].update(
                            region_pose_world=task.region_pose.tolist(),
                            region_bounds_xy_m=task.region_bounds.tolist(),
                            final_region_margin_m=samples[-1]["region_margin"],
                            max_tilt_deg=float(
                                np.degrees(max(m["tilt"] for m in samples))
                            ),
                        )
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
