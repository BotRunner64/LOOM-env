#!/usr/bin/env python3
"""Record a real dual-arm joint-motion diagnostic with the LOOM Runner.

This uses Isaac Lab's low-level SimulationContext, as in its articulation
tutorial. It is a motion/rendering diagnostic, not the planned ManagerBasedEnv
pick-and-place implementation. No cuRobo planner or grasp expert is used.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import importlib.metadata
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np

from loom_env.runtime.runner import EpisodeRunner
from loom_env.runtime.motion import DURATION, MotionCheck, MotionSource, gripper_command
from loom_env.specs.config import (
    ARMS,
    CameraSpec,
    EpisodeSpec,
    TaskSpec,
    load_collection,
    load_deployment,
)
from loom_env.specs.episode import (
    Frame,
    Observation,
    Transition,
)

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "preview")
    parser.add_argument(
        "--episode-id",
        default="motion-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"),
    )
    parser.add_argument(
        "--deployment", type=Path, default=ROOT / "configs/deployments/dual_panda.yaml"
    )
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    args = parser.parse_args()
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
        parser.error(
            "Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's Omniverse EULA"
        )

    collection = load_collection(ROOT / "configs/collection/pick_place.yaml")
    collection = replace(collection, deployment=load_deployment(args.deployment))
    deployment = collection.deployment
    source = MotionSource(deployment)

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, enable_cameras=True)
    app = launcher.app
    try:
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import RigidObject, RigidObjectCfg
        from isaaclab.sensors import Camera, CameraCfg
        from loom_env.embodiments.isaac_lab import DualArmArticulation
        from isaaclab_physx.physics import PhysxCfg
        from isaaclab_physx.renderers import IsaacRtxRendererCfg

        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(
                dt=deployment.physics_dt,
                device="cuda:0",
                physics=PhysxCfg(),
            )
        )

        def block(path, size, position, color, *, collision=True):
            cfg = sim_utils.CuboidCfg(
                size=size,
                collision_props=sim_utils.CollisionPropertiesCfg()
                if collision
                else None,
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=color, roughness=0.6
                ),
            )
            cfg.func(path, cfg, translation=position)

        block("/World/Floor", (8, 8, 0.1), (0, 0, -0.05), (0.12, 0.15, 0.19))
        block("/World/Table", (1.6, 1.8, 0.08), (0.45, 0, 0.71), (0.43, 0.47, 0.51))
        light = sim_utils.DomeLightCfg(intensity=2200.0)
        light.func("/World/Light", light)
        robot = DualArmArticulation(deployment, args.asset_root)
        for side, color in (("left", (0.04, 0.6, 0.8)), ("right", (0.95, 0.48, 0.06))):
            arm = deployment.arms[side]
            block(
                f"/World/{side}_marker",
                (0.25, 0.25, 0.008),
                (arm.base_pose[0], arm.base_pose[1], 0.754),
                color,
                collision=False,
            )
        cube = RigidObject(
            RigidObjectCfg(
                prim_path="/World/Cube",
                spawn=sim_utils.CuboidCfg(
                    size=collection.task.parameters["object_size"],
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.92, 0.16, 0.10)
                    ),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.58, -0.06, 0.79)),
            )
        )
        # A local geometric container provides scene context; nothing is grasped.
        sx, sy, sz = collection.task.parameters["region_size"]
        cx, cy, floor_z, wall = 0.60, 0.18, 0.75, 0.01
        block(
            "/World/Container/base",
            (sx + 2 * wall, sy + 2 * wall, wall),
            (cx, cy, floor_z + wall / 2),
            (0.16, 0.40, 0.30),
        )
        for index, (size, offset) in enumerate(
            (
                ((wall, sy + 2 * wall, sz), (-(sx + wall) / 2, 0)),
                ((wall, sy + 2 * wall, sz), ((sx + wall) / 2, 0)),
                ((sx, wall, sz), (0, -(sy + wall) / 2)),
                ((sx, wall, sz), (0, (sy + wall) / 2)),
            )
        ):
            block(
                f"/World/Container/wall{index}",
                size,
                (cx + offset[0], cy + offset[1], floor_z + wall + sz / 2),
                (0.16, 0.40, 0.30),
            )

        camera = Camera(
            CameraCfg(
                prim_path="/World/Camera",
                height=640,
                width=960,
                data_types=["rgb"],
                renderer_cfg=IsaacRtxRendererCfg(),
                spawn=sim_utils.PinholeCameraCfg(
                    focal_length=22.0,
                    horizontal_aperture=24.0,
                    clipping_range=(0.05, 20.0),
                ),
            )
        )
        sim.reset()
        robot.initialize()
        cube.reset()
        camera.reset()
        camera.set_world_poses_from_view(
            eyes=torch.tensor([[2.0, 2.0, 2.0]], device=sim.device),
            targets=torch.tensor([[0.30, 0.0, 1.04]], device=sim.device),
        )

        def advance():
            for substep in range(deployment.decimation):
                robot.write_data_to_sim()
                sim.step(render=substep == deployment.decimation - 1)
                robot.update(deployment.physics_dt)
                cube.update(deployment.physics_dt)
            camera.update(deployment.control_dt, force_recompute=True)

        def settle(command):
            robot.reset(command)
            consecutive = 0
            for step in range(round(5.0 / deployment.control_dt)):
                advance()
                measured = robot.observe()
                errors, velocities, finger_errors, finger_velocities = [], [], [], []
                for side in ARMS:
                    arm = deployment.arms[side]
                    prefix = f"robot/{side}"
                    target = command[deployment.action_slices[f"{side}/arm"]]
                    fingers = (
                        np.asarray(arm.gripper.joint_map)
                        @ command[deployment.action_slices[f"{side}/gripper"]]
                        + arm.gripper.joint_offset
                    )
                    errors.append(
                        float(
                            np.max(
                                np.abs(measured[f"{prefix}/joint_position"] - target)
                            )
                        )
                    )
                    velocities.append(
                        float(np.max(np.abs(measured[f"{prefix}/joint_velocity"])))
                    )
                    finger_errors.append(
                        float(
                            np.max(
                                np.abs(measured[f"{prefix}/gripper_position"] - fingers)
                            )
                        )
                    )
                    finger_velocities.append(
                        float(np.max(np.abs(measured[f"{prefix}/gripper_velocity"])))
                    )
                stable = (
                    max(errors) < 0.003
                    and max(velocities) < 0.01
                    and max(finger_errors) < 0.0005
                    and max(finger_velocities) < 0.01
                )
                consecutive = consecutive + 1 if stable else 0
                if consecutive * deployment.control_dt >= 0.25:
                    return {
                        "settle_seconds": (step + 1) * deployment.control_dt,
                        "joint_error_rad": errors,
                        "joint_velocity_rad_s": velocities,
                        "finger_error": finger_errors,
                        "finger_velocity": finger_velocities,
                    }
            raise RuntimeError(
                f"Robot did not stabilize: errors={errors}, velocities={velocities}, finger_errors={finger_errors}"
            )

        reset_checks = [settle(source.home)]
        # Exercise a second reset after actual physics motion, before recording.
        perturb = source.home.copy()
        for side in ARMS:
            perturb[deployment.action_slices[f"{side}/arm"]] += (
                source.offsets[side] * 0.25
            )
        robot.submit(perturb)
        for _ in range(10):
            advance()
        reset_checks.append(settle(source.home))
        print("ROBOT_RESET_CHECKS " + json.dumps(reset_checks), flush=True)

        def capture(step, command):
            values = robot.observe()
            errors, grippers = [], []
            diagnostics = {}
            for side in ARMS:
                arm = deployment.arms[side]
                q = values[f"robot/{side}/joint_position"]
                errors.append(
                    float(
                        np.max(
                            np.abs(q - command[deployment.action_slices[f"{side}/arm"]])
                        )
                    )
                )
                diagnostics[f"diagnostic/{side}/joint_excursion"] = np.abs(
                    q - arm.initial_positions
                )
                grippers.append(
                    gripper_command(
                        arm.gripper, values[f"robot/{side}/gripper_position"]
                    )
                )
            values["cameras/front/rgb"] = (
                camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy()
            )
            values["cameras/front/timestamp"] = np.array(step * deployment.control_dt)
            values["cameras/front/valid"] = np.array(True)
            return Frame(
                Observation(step * deployment.control_dt, values),
                {
                    "cube/pose_world": cube.data.root_link_pose_w.torch[0]
                    .cpu()
                    .numpy(),
                    "diagnostic/time": np.array(step * deployment.control_dt),
                    "diagnostic/tracking_error": np.asarray(errors),
                    "diagnostic/gripper_command": np.asarray(grippers),
                    **diagnostics,
                },
            )

        class PreviewEnvironment:
            def reset_episode(self, spec):
                self.step_count = 0
                # This one-shot diagnostic was initialized and settled above.
                return capture(0, source.home)

            def step(self, action):
                robot.submit(action)
                advance()
                self.step_count += 1
                if self.step_count % 20 == 0:
                    print(
                        f"MOTION_FRAME {self.step_count}/{round(DURATION / deployment.control_dt)}",
                        flush=True,
                    )
                return Transition(capture(self.step_count, action), action.copy())

        camera_pose = np.r_[
            camera.data.pos_w.torch[0].cpu().numpy(),
            camera.data.quat_w_world.torch[0].cpu().numpy(),
        ]
        deployment = replace(
            deployment,
            id=deployment.id + "_motion_preview",
            cameras=(CameraSpec("front", 960, 640, 1, "world", camera_pose),),
        )
        initial = capture(0, source.home)
        collection = replace(
            collection,
            deployment=deployment,
            max_steps=round(DURATION / deployment.control_dt),
            task=TaskSpec(
                "dual_arm_joint_motion",
                "Run a dual-arm joint-motion diagnostic; no grasp execution.",
                collection.task.roles,
                collection.task.required_capabilities,
                {"duration_seconds": DURATION},
            ),
        )
        spec = EpisodeSpec(
            id=args.episode_id,
            collection=collection,
            seed=0,
            initial_state={
                "robot": {
                    key: value.tolist()
                    for key, value in initial.observation.values.items()
                    if key.startswith("robot/")
                },
                "cube_pose_world": initial.world_state["cube/pose_world"].tolist(),
                "camera_pose_world": camera_pose.tolist(),
                "camera_convention": "world(+X forward,+Z up)",
                "camera_intrinsics": camera.data.intrinsic_matrices.torch[0]
                .cpu()
                .tolist(),
            },
            sampled_parameters={
                "randomization": "none",
                "cube_position": [0.58, -0.06, 0.79],
                "container_position": [cx, cy, floor_z],
                "object_size": [0.04] * 3,
                "region_size": [sx, sy, sz],
                "controllers": robot.controller_parameters,
                "reset_checks": reset_checks,
                "gravity_compensation": True,
            },
            asset_versions={
                **robot.asset_versions,
                "primitive:cube": "preview-v1",
                "primitive:open_box": "preview-v1",
            },
            runtime_versions={
                name: importlib.metadata.version(name)
                for name in (
                    "loom-env",
                    "isaaclab",
                    "isaaclab-assets",
                    "isaacsim",
                    "torch",
                )
            },
            provenance={
                "source": "simulation",
                "purpose": "joint_motion_diagnostic",
                "planner": "none",
                "grasp_execution": False,
                "physics_backend": "PhysX",
                "runtime": "SimulationContext diagnostic",
                "resolved_robot_usd": robot.usd_paths,
                "asset_manifests": robot.asset_manifests,
                "model_checks": robot.model_checks,
            },
        )
        task = MotionCheck(deployment)
        result = EpisodeRunner(PreviewEnvironment(), task, source).run(
            spec, args.output_dir
        )
        report = {
            "episode": str(result.episode_path),
            "outcome": result.outcome.code,
            "reason": result.outcome.reason,
            "steps": result.steps,
            "measurements": getattr(task, "report", {}),
            "reset_checks": reset_checks,
            "deployment": deployment.id,
            "model_checks": robot.model_checks,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / f"{args.episode_id}-report.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print(json.dumps(report, indent=2), flush=True)
        if result.episode_path is None:
            raise RuntimeError(result.outcome.reason)
        print(f"MOTION_EPISODE_SAVED {result.episode_path}", flush=True)
        if result.outcome.code != "success":
            raise SystemExit(1)
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        app.close(exit_code=int(sys.exc_info()[0] is not None))


if __name__ == "__main__":
    main()
