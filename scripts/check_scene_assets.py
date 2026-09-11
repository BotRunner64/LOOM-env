#!/usr/bin/env python3
"""Validate a prepared scene's physical rest state and save actual camera images."""

import argparse
import json
from pathlib import Path
import traceback

from loom_env.specs.config import load_collection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--collection", type=Path, default=Path("configs/collection/pick_place.yaml")
    )
    parser.add_argument("--asset-root", type=Path, default=Path(".cache/assets"))
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    collection = load_collection(args.collection)
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(
        headless=True, enable_cameras=bool(collection.deployment.cameras)
    )
    env = None
    report = {"passed": False}
    try:
        from PIL import Image
        import torch
        from loom_env.assets.catalog import asset_definition
        from loom_env.embodiments.commands import initial_command
        from loom_env.tasks import create_task
        from loom_env.runtime.build import create_environment

        env = create_environment(collection, args.asset_root)
        try:
            spec = env.resolve_episode("asset-health", 0)
            report["passed"] = True
            report["asset_versions"] = dict(spec.asset_versions)
        except Exception as error:
            report["error"] = str(error)
        frame = env.frame()
        report["world_state"] = {k: v.tolist() for k, v in frame.world_state.items()}
        report["robot"] = {
            k: v.tolist()
            for k, v in frame.observation.values.items()
            if k.startswith("robot/")
        }
        for camera in collection.deployment.cameras:
            Image.fromarray(
                frame.observation.values[f"cameras/{camera.name}/rgb"]
            ).save(args.output_dir / f"{camera.name}.png")
        if report["passed"] and collection.task.id == "put_object_in_container":
            # A real release into the cavity checks the collider, not just USD metadata.
            target = collection.role_bindings["target_object"]
            container = collection.role_bindings["container"]
            asset = asset_definition(collection.scene.objects[target]["asset"])
            receptacle = asset_definition(collection.scene.objects[container]["asset"])
            drop_pose = frame.world_state[f"{container}/region_pose_world"].copy()
            drop_pose[2] += receptacle.interior[1][2] / 2 - asset.bounds[0][2] + 0.03
            body = env.scene[f"object_{target}"]
            body.write_root_pose_to_sim_index(
                root_pose=torch.tensor(
                    drop_pose[None], device=env.device, dtype=torch.float32
                )
            )
            body.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros((1, 6), device=env.device)
            )
            task = create_task(collection)
            task.reset(frame.world_state)
            outcome = None
            for _ in range(100):
                frame = env.step(initial_command(collection.deployment)).frame
                outcome = task.update(
                    frame.world_state, collection.deployment.control_dt
                ).outcome
                if outcome is not None:
                    break
            report["container_release"] = {
                "passed": outcome is not None and outcome.code == "success",
                "final_pose": frame.world_state[f"{target}/pose_world"].tolist(),
                "final_velocity": frame.world_state[
                    f"{target}/velocity_world"
                ].tolist(),
                "reason": (
                    outcome.reason if outcome else "did_not_remain_inside_released"
                ),
                "container_pose": frame.world_state[f"{container}/pose_world"].tolist(),
                "container_velocity": frame.world_state[
                    f"{container}/velocity_world"
                ].tolist(),
            }
            report["passed"] &= report["container_release"]["passed"]
            for camera in collection.deployment.cameras:
                Image.fromarray(
                    frame.observation.values[f"cameras/{camera.name}/rgb"]
                ).save(args.output_dir / f"{camera.name}-container.png")
    except Exception:
        report["passed"] = False
        report["error"] = traceback.format_exc()
    finally:
        (args.output_dir / "validation.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print("ASSET_VALIDATION " + json.dumps(report), flush=True)
        if env is not None:
            env.close()
        launcher.app.close(exit_code=0 if report["passed"] else 1)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
