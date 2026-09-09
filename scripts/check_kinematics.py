#!/usr/bin/env python3
"""Compare cuRobo FK against every recorded measured TCP pose, without Isaac Sim."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.data.episodes import EpisodeReader
from loom_env.embodiments.assets import PANDA_ASSET, PIPER_ASSET, piper_source_dir
from loom_env.specs.config import ARMS

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
        samples = [
            {k: v for k, v in observation.values.items() if k.startswith("robot/")}
            for observation in episode.observations()
        ]
    report = {"episode": str(args.episode), "samples": len(samples), "arms": {}}
    for side in ARMS:
        arm = deployment.arms[side]
        if arm.asset == PIPER_ASSET:
            config = KinematicsCfg.from_basic_urdf(
                str((piper_source_dir(args.asset_root) / "piper.urdf").resolve()),
                "base_link",
                [arm.tcp_frame],
            )
        elif arm.asset == PANDA_ASSET:
            config = KinematicsCfg.from_robot_yaml_file(
                "franka.yml", tool_frames=[arm.tcp_frame]
            )
        else:
            raise ValueError(f"Unsupported robot asset: {arm.asset}")
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
        pose = state.tool_poses.get_link_pose(arm.tcp_frame)
        position = pose.position.detach().cpu().numpy().reshape(-1, 3)
        # cuRobo uses wxyz; the LOOM protocol and installed Isaac Lab use xyzw.
        quaternion = (
            pose.quaternion.detach().cpu().numpy().reshape(-1, 4)[:, [1, 2, 3, 0]]
        )
        base = Rotation.from_quat(arm.base_pose[3:])
        position = base.apply(position) + np.array(arm.base_pose[:3])
        orientation = base * Rotation.from_quat(quaternion)
        measured = np.stack([v[f"robot/{side}/tcp_pose_world"] for v in samples])
        position_error = np.linalg.norm(position - measured[:, :3], axis=1)
        angle_error = (
            orientation.inv() * Rotation.from_quat(measured[:, 3:])
        ).magnitude()
        report["arms"][side] = {
            "max_position_error_m": float(position_error.max()),
            "max_orientation_error_rad": float(angle_error.max()),
            "passed": bool(
                np.all(position_error < 1e-4) and np.all(angle_error < 1e-3)
            ),
        }
    report["criteria"] = {"position_m": 1e-4, "orientation_rad": 1e-3}
    report["scope"] = (
        "FK only; collision-aware planning and grasp execution are not tested"
    )
    report["passed"] = all(v["passed"] for v in report["arms"].values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
