"""Shared RGB optics and rigid camera mounts. Import after AppLauncher."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg
from isaaclab.utils.math import combine_frame_transforms
from isaaclab_physx.renderers import IsaacRtxRendererCfg

from loom_env.embodiments.frames import resolve_camera_mount


def camera_config(spec):
    """Create a world-space optical prim; CameraMounts drives wrist extrinsics."""
    return CameraCfg(
        prim_path=f"/World/Camera_{spec.name}",
        update_latest_camera_pose=True,
        width=spec.width,
        height=spec.height,
        offset=CameraCfg.OffsetCfg(
            pos=spec.pose[:3] if spec.parent_frame == "world" else (0, 0, 0),
            rot=spec.pose[3:] if spec.parent_frame == "world" else (0, 0, 0, 1),
            convention="opengl",
        ),
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=spec.focal_length,
            horizontal_aperture=spec.horizontal_aperture,
            clipping_range=spec.clipping_range,
        ),
        data_types=["rgb"],
        renderer_cfg=IsaacRtxRendererCfg(),
    )


class CameraMounts:
    """Rigid mounts driven by measured link poses, before rendering any camera.

    World-space optical prims avoid stale descendant transforms in Fabric views
    and nested imported USD hierarchies. The link-to-camera transform stays fixed.
    """

    def __init__(self, deployment, cameras, robots, asset_root):
        self.cameras = cameras
        self.mounts = []
        self.resolved = {}
        for spec in deployment.cameras:
            if spec.parent_frame == "world":
                continue
            side, _ = spec.parent_frame.split("/")
            robot = robots[side]
            body, pose = resolve_camera_mount(
                spec, deployment.arms[side], robot.body_names, asset_root
            )
            body_id = robot.body_names.index(body)
            local = torch.tensor(pose[None], dtype=torch.float32, device=robot.device)
            self.mounts.append((cameras[spec.name], robot, body_id, local))
            self.resolved[spec.name] = {
                "parent_frame": spec.parent_frame,
                "body_frame": f"{side}/{body}",
                "optical_pose_in_body": pose.tolist(),
            }

    def update(self, sim, *, warmup=False):
        for camera, robot, body_id, local in self.mounts:
            parent = robot.data.body_link_pose_w.torch[:, body_id]
            position, orientation = combine_frame_transforms(
                parent[:, :3], parent[:, 3:], local[:, :3], local[:, 3:]
            )
            camera.set_world_poses(position, orientation, convention="opengl")
        # Warm temporal rendering history at episode initialization without
        # advancing physics or the recorded control clock.
        for _ in range(5 if warmup else 1):
            sim.render()
            if warmup:
                for camera in self.cameras.values():
                    camera.update(0.0, force_recompute=True)
                    _ = camera.data.output["rgb"]
