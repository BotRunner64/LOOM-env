#!/usr/bin/env python3
"""Record a real dual-arm joint-motion diagnostic with the LOOM Runner.

This uses Isaac Lab's low-level SimulationContext, as in its articulation
tutorial. It is a motion/rendering diagnostic, separate from ManagerBasedEnv
pick-and-place collection. No cuRobo planner or grasp expert is used.
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

from loom_env.environments.tabletop import sample_objects, tabletop_geometry
from loom_env.runtime.runner import EpisodeRunner
from loom_env.runtime.motion import DURATION, MotionCheck, MotionSource, gripper_command
from loom_env.specs.config import (
    ARMS,
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
    parser.add_argument("--deployment", type=Path)
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    parser.add_argument(
        "--collection", type=Path, default=ROOT / "configs/collection/pick_place.yaml"
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
        parser.error(
            "Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's Omniverse EULA"
        )

    collection = load_collection(args.collection)
    if args.deployment:
        collection = replace(collection, deployment=load_deployment(args.deployment))
    geometry = tabletop_geometry(collection)
    candidates = sample_objects(collection, args.seed)
    deployment = collection.deployment
    source = MotionSource(deployment)

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, enable_cameras=True)
    app = launcher.app
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.assets import RigidObject
        from isaaclab.sensors import Camera
        from loom_env.embodiments.cameras import CameraMounts, camera_config
        from loom_env.environments.isaac_lab import tabletop_object_config
        from loom_env.embodiments.isaac_lab import DualArmArticulation
        from isaaclab_physx.physics import PhysxCfg

        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(
                dt=deployment.physics_dt,
                device="cuda:0",
                physics=PhysxCfg(),
                render=sim_utils.RenderCfg(ambient_light_intensity=0.3),
            )
        )

        def block(path, size, position, color):
            cfg = sim_utils.CuboidCfg(
                size=tuple(float(v) for v in size),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    contact_offset=0.002, rest_offset=0.0
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(float(v) for v in color)
                ),
            )
            cfg.func(path, cfg, translation=tuple(float(v) for v in position))

        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/Ground", ground)
        for name, box in geometry.items():
            block(f"/World/geometry_{name}", box["size"], box["position"], box["color"])
        light = sim_utils.DomeLightCfg(intensity=600)
        light.func("/World/Light", light)
        robot = DualArmArticulation(deployment, args.asset_root)
        objects = {
            name: RigidObject(
                tabletop_object_config(
                    collection.scene.objects[name], f"/World/object_{name}", position
                )
            )
            for name, position in candidates.items()
        }

        cameras = {
            spec.name: Camera(camera_config(spec)) for spec in deployment.cameras
        }
        camera_samples = {}
        sim.reset()
        robot.initialize()
        camera_mounts = CameraMounts(deployment.cameras, cameras, robot.robots)
        for obj in objects.values():
            obj.reset()
        for camera in cameras.values():
            camera.reset()

        def advance():
            for substep in range(deployment.decimation):
                robot.write_data_to_sim()
                sim.step(render=substep == deployment.decimation - 1)
                robot.update(deployment.physics_dt)
                for obj in objects.values():
                    obj.update(deployment.physics_dt)

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
                    and all(
                        error
                        < (
                            0.0005
                            if deployment.arms[side].gripper.unit == "m"
                            else 0.003
                        )
                        for side, error in zip(ARMS, finger_errors)
                    )
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
                        "finger_units": [
                            deployment.arms[side].gripper.unit for side in ARMS
                        ],
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
            camera_mounts.update(sim, warmup=step == 0)
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
            camera_poses = {}
            for camera_spec in deployment.cameras:
                name = camera_spec.name
                camera = cameras[name]
                if name not in camera_samples or step % camera_spec.period_steps == 0:
                    camera.update(deployment.control_dt, force_recompute=True)
                    camera_samples[name] = (
                        camera.data.output["rgb"]
                        .torch[0, ..., :3]
                        .cpu()
                        .numpy()
                        .copy(),
                        step * deployment.control_dt,
                        np.r_[
                            camera.data.pos_w.torch[0].cpu().numpy(),
                            camera.data.quat_w_opengl.torch[0].cpu().numpy(),
                        ],
                    )
                rgb, timestamp, camera_pose = camera_samples[name]
                values[f"cameras/{name}/rgb"] = rgb
                values[f"cameras/{name}/timestamp"] = np.array(timestamp)
                values[f"cameras/{name}/valid"] = np.array(True)
                camera_poses[f"cameras/{name}/pose_world"] = camera_pose
            return Frame(
                Observation(step * deployment.control_dt, values),
                {
                    **{
                        f"{name}/pose_world": obj.data.root_link_pose_w.torch[0]
                        .cpu()
                        .numpy()
                        for name, obj in objects.items()
                    },
                    "diagnostic/time": np.array(step * deployment.control_dt),
                    "diagnostic/tracking_error": np.asarray(errors),
                    "diagnostic/gripper_command": np.asarray(grippers),
                    **diagnostics,
                    **camera_poses,
                },
            )

        class PreviewEnvironment:
            def reset_episode(self, spec):
                self.step_count = 0
                camera_samples.clear()
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
            seed=args.seed,
            initial_state={
                "robot": {
                    key: value.tolist()
                    for key, value in initial.observation.values.items()
                    if key.startswith("robot/")
                },
                "object_pose_world": {
                    name: initial.world_state[f"{name}/pose_world"].tolist()
                    for name in objects
                },
                "camera_pose_world": {
                    name: initial.world_state[f"cameras/{name}/pose_world"].tolist()
                    for name in cameras
                },
                "camera_convention": "opengl(-Z forward,+Y up)",
                "camera_intrinsics": {
                    name: camera.data.intrinsic_matrices.torch[0].cpu().tolist()
                    for name, camera in cameras.items()
                },
            },
            sampled_parameters={
                "object_candidates": {
                    name: position.tolist() for name, position in candidates.items()
                },
                "controllers": robot.controller_parameters,
                "reset_checks": reset_checks,
                "gravity_compensation": True,
            },
            asset_versions={
                **robot.asset_versions,
                "primitive:cube": "tabletop-v2",
                "primitive:open_box": "tabletop-v2",
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
