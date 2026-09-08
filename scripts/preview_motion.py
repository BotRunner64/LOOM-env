#!/usr/bin/env python3
"""Record a real dual-Panda joint-motion diagnostic with the LOOM Runner.

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
import math
import os
from pathlib import Path

import numpy as np

from loom_env.runtime.runner import EpisodeRunner
from loom_env.specs.config import (
    ARMS,
    CameraSpec,
    EpisodeSpec,
    TaskSpec,
    load_collection,
)
from loom_env.specs.episode import (
    Action,
    Event,
    Frame,
    Observation,
    Outcome,
    TaskStatus,
    Transition,
)

ROOT = Path(__file__).resolve().parents[1]
# Explicit asset version: the default Isaac Lab 6.0 asset URL currently returns 404.
PANDA_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/IsaacLab/Robots/FrankaEmika/panda_instanceable.usd"
)
PHASES = (
    "Right arm moves / left holds",
    "Left arm moves / right holds",
    "Both grippers close and open",
    "Both arms hold home",
)


def motion_phase(time):
    return 0 if time < 4 else 1 if time < 8 else 2 if time < 10 else 3


class MotionSource:
    def __init__(self, deployment):
        self.deployment = deployment
        self.home = np.concatenate(
            [np.r_[deployment.arms[side].initial_positions, 0.08] for side in ARMS]
        )

    def reset(self, episode_input):
        self.previous_phase = None

    def act(self, observation):
        time = observation.timestamp
        phase = motion_phase(time)
        command = self.home.copy()
        if phase < 2:
            side = "right" if phase == 0 else "left"
            local_time = time - phase * 4
            amount = math.sin(math.pi * local_time / 4) ** 2
            part = self.deployment.action_slices[f"{side}/arm"]
            command[part.start] += (0.55 if side == "left" else -0.55) * amount
            command[part.start + 1] += 0.22 * amount
            command[part.start + 3] += 0.35 * amount
        elif phase == 2:
            width = 0.08 * (1 - math.sin(math.pi * (time - 8) / 2) ** 2)
            for side in ARMS:
                command[self.deployment.action_slices[f"{side}/gripper"]] = width
        events = ()
        if phase != self.previous_phase:
            events = (
                Event(
                    "skill",
                    PHASES[phase],
                    round(time / self.deployment.control_dt),
                    {
                        "diagnostic": True,
                        "phase": phase,
                    },
                ),
            )
            self.previous_phase = phase
        return Action(command, events)


class MotionCheck:
    """Verify actual motion, stationary-arm tracking and gripper travel."""

    def reset(self, initial_state):
        self.excursion = np.zeros(2)
        self.hold_error = np.zeros(2)
        self.min_width = np.full(2, np.inf)
        self.max_width = np.zeros(2)

    def update(self, world_state, dt):
        time = float(world_state["diagnostic/time"])
        error = world_state["diagnostic/tracking_error"]
        self.excursion = np.maximum(self.excursion, world_state["diagnostic/excursion"])
        width = world_state["diagnostic/gripper_width"]
        self.min_width = np.minimum(self.min_width, width)
        self.max_width = np.maximum(self.max_width, width)
        # Use the interval's source phase, not the next control instant's phase.
        phase = motion_phase(time - dt / 2)
        if phase < 2:
            held = 0 if phase == 0 else 1
            self.hold_error[held] = max(self.hold_error[held], error[held])
        if time < 12 - 1e-8:
            return TaskStatus()
        self.report = {
            "max_joint_excursion_rad": self.excursion.tolist(),
            "max_held_arm_error_rad": self.hold_error.tolist(),
            "final_tracking_error_rad": error.tolist(),
            "gripper_width_range_m": np.stack(
                [self.min_width, self.max_width], axis=1
            ).tolist(),
            "arm_order": list(ARMS),
            "criteria": {
                "min_joint_excursion_rad": 0.35,
                "max_held_arm_error_rad": 0.04,
                "max_final_tracking_error_rad": 0.04,
                "min_gripper_travel_m": 0.05,
            },
        }
        passed = bool(
            np.all(self.excursion > 0.35)
            and np.all(self.hold_error < 0.04)
            and np.all(error < 0.04)
            and np.all(self.max_width - self.min_width > 0.05)
        )
        return TaskStatus(
            Outcome(
                "success" if passed else "task_failure",
                "joint_motion_diagnostic_passed"
                if passed
                else "joint_motion_threshold_failed",
            )
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "preview")
    parser.add_argument(
        "--episode-id",
        default="motion-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"),
    )
    parser.add_argument("--robot-usd", default=PANDA_USD)
    args = parser.parse_args()
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
        parser.error(
            "Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's Omniverse EULA"
        )

    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True, enable_cameras=True)
    app = launcher.app
    try:
        import torch
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
        from isaaclab.sensors import Camera, CameraCfg
        from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG
        from isaaclab_physx.physics import PhysxCfg
        from isaaclab_physx.renderers import IsaacRtxRendererCfg

        collection = load_collection(ROOT / "configs/collection/pick_place.yaml")
        deployment = collection.deployment
        if any(len(deployment.arms[side].joint_names) != 7 for side in ARMS):
            raise ValueError("This diagnostic requires the dual Panda preset")
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
        robots, joint_ids, gripper_ids, tcp_ids = {}, {}, {}, {}
        usd_paths = {}
        for side, color in (("left", (0.04, 0.6, 0.8)), ("right", (0.95, 0.48, 0.06))):
            arm = deployment.arms[side]
            block(
                f"/World/{side}_marker",
                (0.25, 0.25, 0.008),
                (arm.base_pose[0], arm.base_pose[1], 0.754),
                color,
                collision=False,
            )
            cfg = FRANKA_PANDA_HIGH_PD_CFG.copy()
            cfg.prim_path = f"/World/{side}_robot"
            cfg.spawn.usd_path = args.robot_usd
            cfg.init_state.pos = arm.base_pose[:3]
            cfg.init_state.rot = arm.base_pose[3:]
            cfg.init_state.joint_pos = {
                **dict(zip(arm.joint_names, arm.initial_positions)),
                **dict.fromkeys(arm.gripper.joint_names, 0.04),
            }
            robots[side] = Articulation(cfg)
            usd_paths[side] = cfg.spawn.usd_path

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
        for side, robot in robots.items():
            robot.reset()
            joint_ids[side], actual_names = robot.find_joints(
                deployment.arms[side].joint_names, preserve_order=True
            )
            gripper_ids[side], _ = robot.find_joints(
                deployment.arms[side].gripper.joint_names, preserve_order=True
            )
            bodies, _ = robot.find_bodies(
                deployment.arms[side].tcp_frame, preserve_order=True
            )
            if (
                tuple(actual_names) != deployment.arms[side].joint_names
                or len(bodies) != 1
            ):
                raise RuntimeError(f"Unexpected {side} joint/TCP mapping")
            tcp_ids[side] = bodies[0]
            actual_limits = (
                robot.data.joint_pos_limits.torch[0, joint_ids[side]].cpu().numpy()
            )
            if not np.allclose(
                actual_limits, deployment.arms[side].joint_limits, atol=1e-4
            ):
                raise RuntimeError(
                    f"{side} USD joint limits disagree with the deployment"
                )
        cube.reset()
        camera.reset()
        camera.set_world_poses_from_view(
            eyes=torch.tensor([[2.0, 2.0, 2.0]], device=sim.device),
            targets=torch.tensor([[0.30, 0.0, 1.04]], device=sim.device),
        )
        source = MotionSource(deployment)

        def submit(action):
            for side, robot in robots.items():
                robot.set_joint_position_target_index(
                    target=torch.tensor(
                        action[deployment.action_slices[f"{side}/arm"]][None],
                        dtype=torch.float32,
                        device=sim.device,
                    ),
                    joint_ids=joint_ids[side],
                )
                grip = deployment.arms[side].gripper
                target = (
                    np.asarray(grip.joint_map)
                    @ action[deployment.action_slices[f"{side}/gripper"]]
                    + grip.joint_offset
                )
                robot.set_joint_position_target_index(
                    target=torch.tensor(
                        target[None], dtype=torch.float32, device=sim.device
                    ),
                    joint_ids=gripper_ids[side],
                )

        def advance():
            for substep in range(deployment.decimation):
                for robot in robots.values():
                    robot.write_data_to_sim()
                sim.step(render=substep == deployment.decimation - 1)
                for robot in robots.values():
                    robot.update(deployment.physics_dt)
                cube.update(deployment.physics_dt)
            camera.update(deployment.control_dt, force_recompute=True)

        submit(source.home)
        for _ in range(30):
            advance()

        def capture(step, command):
            values, errors, excursions, widths = {}, [], [], []
            for side, robot in robots.items():
                q = robot.data.joint_pos.torch[0].cpu().numpy()
                dq = robot.data.joint_vel.torch[0].cpu().numpy()
                arm = deployment.arms[side]
                for key, value in (
                    ("joint_position", q[joint_ids[side]]),
                    ("joint_velocity", dq[joint_ids[side]]),
                    ("gripper_position", q[gripper_ids[side]]),
                    ("gripper_velocity", dq[gripper_ids[side]]),
                    (
                        "tcp_pose_world",
                        robot.data.body_link_pose_w.torch[0, tcp_ids[side]]
                        .cpu()
                        .numpy(),
                    ),
                ):
                    values[f"robot/{side}/{key}"] = value
                errors.append(
                    np.max(
                        np.abs(
                            q[joint_ids[side]]
                            - command[deployment.action_slices[f"{side}/arm"]]
                        )
                    )
                )
                excursions.append(
                    np.max(np.abs(q[joint_ids[side]] - arm.initial_positions))
                )
                widths.append(q[gripper_ids[side]].sum())
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
                    "diagnostic/excursion": np.asarray(excursions),
                    "diagnostic/gripper_width": np.asarray(widths),
                },
            )

        class PreviewEnvironment:
            def reset_episode(self, spec):
                self.step_count = 0
                # This one-shot diagnostic was initialized and settled above.
                return capture(0, source.home)

            def step(self, action):
                submit(action)
                advance()
                self.step_count += 1
                if self.step_count % 20 == 0:
                    print(f"MOTION_FRAME {self.step_count}/240", flush=True)
                return Transition(capture(self.step_count, action), action.copy())

        camera_pose = np.r_[
            camera.data.pos_w.torch[0].cpu().numpy(),
            camera.data.quat_w_world.torch[0].cpu().numpy(),
        ]
        deployment = replace(
            deployment,
            id="dual_panda_motion_preview",
            arms={
                side: replace(arm, asset=usd_paths[side])
                for side, arm in deployment.arms.items()
            },
            cameras=(CameraSpec("front", 960, 640, 1, "world", camera_pose),),
        )
        initial = capture(0, source.home)
        collection = replace(
            collection,
            deployment=deployment,
            max_steps=240,
            task=TaskSpec(
                "dual_arm_joint_motion",
                "Run a dual-arm joint-motion diagnostic; no grasp execution.",
                collection.task.roles,
                collection.task.required_capabilities,
                {"duration_seconds": 12},
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
                "controller": "FRANKA_PANDA_HIGH_PD_CFG",
                "gravity_compensation": True,
            },
            asset_versions={
                args.robot_usd: "Isaac-assets-5.1",
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
            },
        )
        task = MotionCheck()
        result = EpisodeRunner(PreviewEnvironment(), task, source).run(
            spec, args.output_dir
        )
        report = {
            "episode": str(result.episode_path),
            "outcome": result.outcome.code,
            "reason": result.outcome.reason,
            "steps": result.steps,
            "measurements": getattr(task, "report", {}),
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / f"{args.episode_id}-report.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print(json.dumps(report, indent=2), flush=True)
        if result.episode_path is None:
            raise RuntimeError(result.outcome.reason)
        print(f"MOTION_EPISODE_SAVED {result.episode_path}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
