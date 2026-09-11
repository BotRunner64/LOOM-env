#!/usr/bin/env python3
"""Record the first five seconds of a deployment holding its initial targets."""

import argparse
from dataclasses import replace
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from loom_env.embodiments.commands import initial_command
from loom_env.scenes.workspace import sample_objects
from loom_env.specs.config import (
    CameraSpec,
    load_collection,
    load_deployment,
    load_scene,
    plain,
)


def look_at(eye, target):
    eye, target = np.asarray(eye, dtype=float), np.asarray(target, dtype=float)
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return tuple(
        np.r_[
            eye, Rotation.from_matrix(np.column_stack((right, up, -forward))).as_quat()
        ]
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deployment", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, default=Path(".cache/assets"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    deployment = load_deployment(args.deployment)
    cameras = (
        next(c for c in deployment.cameras if c.name == "front"),
        CameraSpec(
            "side_close",
            640,
            480,
            1,
            "world",
            look_at([-0.1, -1.4, 0.85], [-0.08, 0, 0.78]),
        ),
        CameraSpec(
            "rear_close",
            640,
            480,
            1,
            "world",
            look_at([-1.25, -0.5, 1.05], [-0.05, 0, 0.8]),
        ),
    )
    collection = replace(
        load_collection("configs/collection/pick_place.yaml"),
        deployment=replace(deployment, cameras=cameras),
        scene=load_scene(args.scene),
    )
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, enable_cameras=True)
    env = None
    try:
        import torch
        import warp as wp
        from pxr import UsdPhysics
        from isaaclab.sim.utils.stage import get_current_stage
        from loom_env.runtime.build import create_environment

        env = create_environment(collection, args.asset_root)
        # GPU PhysX does not provide reliable CPU contact callbacks. Record
        # tensor net forces per body; these do not identify the other collider.
        physics_view = env.scene["contact_right_left"]._physics_sim_view
        contact_views = {
            str(prim.GetPath()): physics_view.create_rigid_contact_view(
                str(prim.GetPath()), max_contact_data_count=128
            )
            for prim in get_current_stage().Traverse()
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and "robot" in str(prim.GetPath())
        }
        env.reset(seed=0)
        env.control_step = env._sim_step_counter = 0
        env.camera_steps.clear()
        env.camera_sync_step = None
        command = initial_command(deployment)
        env.robot.reset(command)
        for name, pose in sample_objects(collection.scene, 0).items():
            if name not in env.object_names:
                continue
            obj = env.scene[f"object_{name}"]
            obj.write_root_pose_to_sim_index(
                root_pose=torch.tensor(
                    pose[None], device=env.device, dtype=torch.float32
                )
            )
            obj.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros((1, 6), device=env.device)
            )
        env.scene.write_data_to_sim()
        env.sim.forward()
        env.scene.update(env.physics_dt)
        env.sim.render()
        env.obs_buf = env.observation_manager.compute()
        report = {"collection": plain(collection), "playback_speed": 0.5, "frames": []}
        with imageio.get_writer(
            args.output_dir / "initialization.mp4",
            format="FFMPEG",
            fps=10,
            codec="libx264",
            quality=8,
            macro_block_size=2,
            ffmpeg_params=["-movflags", "+faststart"],
        ) as writer:
            for step in range(101):
                if step:
                    frame = env.step(command).frame
                else:
                    frame = env.frame()
                errors = {
                    side: float(
                        np.max(
                            np.abs(
                                frame.observation.values[f"robot/{side}/joint_position"]
                                - deployment.arms[side].initial_positions
                            )
                        )
                    )
                    for side in ("left", "right")
                }
                row = {
                    "step": step,
                    "time": step * deployment.control_dt,
                    "max_joint_error_rad": errors,
                    "object_poses_world": {
                        name: frame.world_state[f"{name}/pose_world"].tolist()
                        for name in env.object_names
                    },
                    "net_contact_forces_n": {
                        path: wp.to_torch(
                            view.get_net_contact_forces(dt=env.physics_dt)
                        )
                        .cpu()
                        .numpy()
                        .reshape(3)
                        .tolist()
                        for path, view in contact_views.items()
                    }
                    if step
                    else None,
                    "contact_sampling": "last physics substep; no contact pairs",
                    "joint_positions": {
                        side: frame.observation.values[
                            f"robot/{side}/joint_position"
                        ].tolist()
                        for side in ("left", "right")
                    },
                }
                report["frames"].append(row)
                max_force = max(
                    (
                        np.linalg.norm(v)
                        for v in (row["net_contact_forces_n"] or {}).values()
                    ),
                    default=0.0,
                )
                canvas = Image.new("RGB", (1920, 600), "#111827")
                draw = ImageDraw.Draw(canvas)
                draw.text(
                    (12, 8),
                    f"{deployment.id} | Initial targets held constant | 0.5x playback",
                    fill="white",
                    font_size=25,
                )
                draw.text(
                    (12, 43),
                    f"t={row['time']:.2f}s | max joint deviation: left={np.degrees(errors['left']):.1f} deg, right={np.degrees(errors['right']):.1f} deg | max net contact={max_force:.1f} N",
                    fill="#90d9ec",
                    font_size=22,
                )
                for i, camera in enumerate(cameras):
                    canvas.paste(
                        Image.fromarray(
                            frame.observation.values[f"cameras/{camera.name}/rgb"]
                        ),
                        (i * 640, 120),
                    )
                    draw.text(
                        (i * 640 + 12, 89), camera.name, fill="white", font_size=22
                    )
                if step in (0, 1, 5, 20, 100):
                    canvas.save(args.output_dir / f"frame-{step:03d}.jpg")
                for _ in range(10 if step in (0, 100) else 1):
                    writer.append_data(np.asarray(canvas))
                if step % 20 == 0:
                    print(
                        "FRAME",
                        step,
                        errors,
                        "max net contact N",
                        max_force,
                        flush=True,
                    )
        try:
            spec = env.resolve_episode("initialization-validation", 0)
            report["initialization"] = {
                "passed": True,
                "asset_versions": dict(spec.asset_versions),
            }
        except RuntimeError as error:
            report["initialization"] = {"passed": False, "error": str(error)}
        (args.output_dir / "measurements.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print("VIDEO", args.output_dir / "initialization.mp4", flush=True)
    except Exception:
        import traceback

        traceback.print_exc()
        raise
    finally:
        if env is not None:
            env.close()
        launcher.app.close()


if __name__ == "__main__":
    main()
