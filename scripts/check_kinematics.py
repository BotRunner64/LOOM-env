#!/usr/bin/env python3
"""Compare cuRobo FK against recorded TCP and optical poses, without Isaac Sim."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.data.episodes import EpisodeReader
from loom_env.embodiments.assets import (
    PANDA_ASSET,
    MODELS,
    model_name,
    prepared_urdf,
    verify_asset,
)
from loom_env.specs.config import ARMS
from loom_env.embodiments.commands import gripper_mapping_error

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.episode.resolve()):
        parser.error("Check output must be outside the immutable episode directory")

    import torch
    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo.types import JointState

    with EpisodeReader(args.episode) as episode:
        deployment = episode.spec.collection.deployment
        samples = []
        camera_samples = {
            c.name: {"poses": [], "ticks": []} for c in deployment.cameras
        }
        for step, observation in enumerate(episode.observations()):
            samples.append(
                {k: v for k, v in observation.values.items() if k.startswith("robot/")}
            )
            world = episode.world_state(step)
            for camera in deployment.cameras:
                prefix = f"cameras/{camera.name}"
                timestamp = float(observation.values[f"{prefix}/timestamp"])
                tick = round(timestamp / deployment.control_dt)
                if (
                    tick != step // camera.period_steps * camera.period_steps
                    or abs(timestamp - tick * deployment.control_dt) > 1e-6
                ):
                    raise ValueError(
                        f"Camera timestamp misaligned: {camera.name}, step {step}"
                    )
                camera_samples[camera.name]["ticks"].append(tick)
                camera_samples[camera.name]["poses"].append(
                    world[f"{prefix}/pose_world"]
                )
    report = {
        "episode": str(args.episode),
        "samples": len(samples),
        "arms": {},
        "cameras": {},
    }

    def compare_camera(camera, position, orientation):
        measured = np.stack(camera_samples[camera.name]["poses"])
        position_error = np.linalg.norm(position - measured[:, :3], axis=1)
        angle_error = (
            orientation.inv() * Rotation.from_quat(measured[:, 3:])
        ).magnitude()
        report["cameras"][camera.name] = {
            "parent_frame": camera.parent_frame,
            "max_position_error_m": float(position_error.max()),
            "max_orientation_error_rad": float(angle_error.max()),
            "passed": bool(
                np.all(position_error < 1e-4) and np.all(angle_error < 1e-3)
            ),
        }

    for side in ARMS:
        arm = deployment.arms[side]
        cameras = [
            c for c in deployment.cameras if c.parent_frame.startswith(f"{side}/")
        ]
        tool_frames = list(
            dict.fromkeys(
                [arm.tcp_frame] + [c.parent_frame.split("/")[1] for c in cameras]
            )
        )
        if arm.asset == PANDA_ASSET:
            config = KinematicsCfg.from_robot_yaml_file(
                "franka.yml", tool_frames=tool_frames
            )
        else:
            name = model_name(arm.asset)
            verify_asset(args.asset_root, name)
            config = KinematicsCfg.from_basic_urdf(
                str(prepared_urdf(args.asset_root, name).resolve()),
                MODELS[name]["base"],
                tool_frames,
            )
        robot = Kinematics(config)
        if tuple(robot.joint_names) != arm.joint_names:
            raise ValueError(f"cuRobo joint order mismatch: {robot.joint_names}")
        q = torch.tensor(
            np.stack([v[f"robot/{side}/joint_position"] for v in samples]),
            dtype=torch.float32,
            device="cuda",
        )
        state = robot.compute_kinematics(
            JointState.from_position(q, joint_names=list(arm.joint_names))
        )

        def world_pose(frame):
            pose = state.tool_poses.get_link_pose(frame)
            position = pose.position.detach().cpu().numpy().reshape(-1, 3)
            # cuRobo uses wxyz; LOOM and installed Isaac Lab use xyzw.
            quaternion = (
                pose.quaternion.detach().cpu().numpy().reshape(-1, 4)[:, [1, 2, 3, 0]]
            )
            base = Rotation.from_quat(arm.base_pose[3:])
            return base.apply(position) + arm.base_pose[:3], base * Rotation.from_quat(
                quaternion
            )

        position, orientation = world_pose(arm.tcp_frame)
        for camera in cameras:
            parent_position, parent_rotation = world_pose(
                camera.parent_frame.split("/")[1]
            )
            ticks = camera_samples[camera.name]["ticks"]
            parent_position, parent_rotation = (
                parent_position[ticks],
                parent_rotation[ticks],
            )
            compare_camera(
                camera,
                parent_position + parent_rotation.apply(camera.pose[:3]),
                parent_rotation * Rotation.from_quat(camera.pose[3:]),
            )
        measured = np.stack([v[f"robot/{side}/tcp_pose_world"] for v in samples])
        position_error = np.linalg.norm(position - measured[:, :3], axis=1)
        angle_error = (
            orientation.inv() * Rotation.from_quat(measured[:, 3:])
        ).magnitude()
        fingers = np.stack([v[f"robot/{side}/gripper_position"] for v in samples])
        linkage_error = float(gripper_mapping_error(arm.gripper, fingers))
        linkage_tolerance = 0.0005 if arm.gripper.unit == "m" else 0.003
        report["arms"][side] = {
            "gripper_mapping_error": linkage_error,
            "gripper_mapping_tolerance": linkage_tolerance,
            "gripper_joint_unit": arm.gripper.unit,
            "gripper_each_joint_excursion": np.ptp(fingers, axis=0).tolist(),
            "max_position_error_m": float(position_error.max()),
            "max_orientation_error_rad": float(angle_error.max()),
            "passed": bool(
                np.all(position_error < 1e-4)
                and np.all(angle_error < 1e-3)
                and linkage_error < linkage_tolerance
            ),
        }
    for camera in deployment.cameras:
        if camera.parent_frame == "world":
            compare_camera(
                camera, np.asarray(camera.pose[:3]), Rotation.from_quat(camera.pose[3:])
            )
    report["criteria"] = {"position_m": 1e-4, "orientation_rad": 1e-3}
    report["scope"] = (
        "Arm FK, gripper linkage and camera extrinsics at their sample timestamps; "
        "visual quality, collision-aware planning and grasp execution are not tested"
    )
    report["passed"] = all(
        v["passed"] for group in ("arms", "cameras") for v in report[group].values()
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
