#!/usr/bin/env python3
"""Probe existing insertion pairs with bounded force and unchanged source physics."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PAIRS = {
    "nut_bolt": {
        "fixture": "Geometry/factory_bolt/00000",
        "insert": "Rigid/factory_nut/00000",
    },
    "coin_slot": {
        "fixture": "Geometry/vertical_coin_stand/00000",
        "insert": "Rigid/coin/00000",
    },
}


def prepare_pair(source_root, output, pair):
    # Import USD only after launching the application in main().
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    from loom_env.assets.prepare import mesh_arrays, physics_properties

    report = {}
    for name, relative in PAIRS[pair].items():
        definition = ROOT / "configs/assets/robodojo" / relative / "object.usda"
        raw = source_root / relative / "object.usdz"
        layer = Sdf.Layer.FindOrOpen(str(definition))
        with raw.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != layer.customLayerData["source_sha256"]:
            raise ValueError(f"Source geometry checksum mismatch: {raw}")
        destination = output / name
        destination.mkdir()
        shutil.copyfile(definition, destination / "object.usda")
        shutil.copyfile(raw, destination / "object.usdz")
        stage = Usd.Stage.Open(str(destination / "object.usda"))
        properties = physics_properties(stage)
        root = stage.GetDefaultPrim()
        original = np.asarray(UsdGeom.Xformable(root).GetLocalTransformation())
        # Measurements must be in the rigid body's local frame, not include
        # source placement that the USD spawner replaces at instantiation.
        colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
        vertices, faces = mesh_arrays(colliders)
        vertices = (np.c_[vertices, np.ones(len(vertices))] @ np.linalg.inv(original))[
            :, :3
        ]
        np.savez_compressed(
            destination / "collision.npz", vertices=vertices, faces=faces
        )
        report[name] = {
            "source": relative,
            "geometry_sha256": digest,
            "definition_sha256": hashlib.sha256(definition.read_bytes()).hexdigest(),
            "source_root_transform": original.tolist(),
            "local_bounds": [vertices.min(0).tolist(), vertices.max(0).tolist()],
            "physics_properties": properties,
        }
    return report


def probe(args, output):
    import torch
    from PIL import Image
    from scipy.spatial.transform import Rotation
    from pxr import UsdGeom, UsdPhysics
    import imageio.v2 as imageio
    import isaaclab.sim as sim
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_physx.physics import PhysxCfg
    from isaaclab_physx.renderers import IsaacRtxRendererCfg

    from loom_env.scenes.isaac_lab import render_config

    assets = prepare_pair(args.source_root.resolve(), output, args.pair)
    dt, seconds = 1 / 240, 6
    ctx = sim.SimulationContext(
        sim.SimulationCfg(
            dt=dt,
            device="cuda:0",
            physics=PhysxCfg(enable_external_forces_every_iteration=True),
            render=render_config(),
        )
    )
    bolt_bounds = np.asarray(assets["fixture"]["local_bounds"])
    nut_bounds = np.asarray(assets["insert"]["local_bounds"])
    bolt_z = -bolt_bounds[0, 2]
    bolt_top = bolt_z + bolt_bounds[1, 2]
    # Full 5 mm separation before physics; no initial nut/bolt overlap.
    insert_rotation = Rotation.from_euler(
        "x", 90 if args.pair == "coin_slot" else 0, degrees=True
    )
    with np.load(output / "insert/collision.npz") as mesh:
        rotated = insert_rotation.apply(mesh["vertices"])
    insert_low = float(rotated[:, 2].min())
    nut_z = bolt_top - insert_low + 0.005
    horizontal = np.zeros(2)
    if args.pair == "coin_slot":
        horizontal = bolt_bounds.mean(0)[:2] - (rotated.min(0) + rotated.max(0))[:2] / 2
    cfg = sim.UsdFileCfg(usd_path=str(output / "fixture/object.usda"))
    bolt = cfg.func("/World/Bolt", cfg, translation=(0, 0, float(bolt_z)))
    # Diagnostic fixed mounting only; preserve source collision and material.
    UsdPhysics.RigidBodyAPI(bolt).GetRigidBodyEnabledAttr().Set(False)
    cfg = sim.CuboidCfg(
        size=(0.3, 0.3, 0.01),
        collision_props=sim.CollisionPropertiesCfg(),
        visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
    )
    cfg.func("/World/Floor", cfg, translation=(0, 0, -0.005))
    cfg = sim.DomeLightCfg(intensity=1000)
    cfg.func("/World/Light", cfg)
    nut = RigidObject(
        RigidObjectCfg(
            prim_path="/World/Nut",
            spawn=sim.UsdFileCfg(usd_path=str(output / "insert/object.usda")),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(*horizontal, float(nut_z)), rot=tuple(insert_rotation.as_quat())
            ),
        )
    )
    camera = Camera(
        CameraCfg(
            prim_path="/World/Camera",
            width=640,
            height=480,
            data_types=["rgb"],
            renderer_cfg=IsaacRtxRendererCfg(),
            spawn=sim.PinholeCameraCfg(clipping_range=(0.001, 10)),
        )
    )
    ctx.reset()
    nut.reset()
    camera.reset()
    camera.set_world_poses_from_view(
        eyes=torch.tensor([[0.13, 0.13, 0.14]], device="cuda"),
        targets=torch.tensor([[0.0, 0.0, 0.03]], device="cuda"),
    )
    actual = (
        UsdGeom.BBoxCache(0, ["default", "render"])
        .ComputeWorldBound(bolt)
        .ComputeAlignedRange()
    )
    actual_bounds = np.array([actual.GetMin(), actual.GetMax()])
    expected_bounds = bolt_bounds + [0, 0, bolt_z]
    if not np.allclose(actual_bounds, expected_bounds, atol=1e-6):
        raise ValueError(f"Fixture spawn changed geometry: {actual_bounds}")
    rows = []
    force = torch.zeros((1, 1, 3), device="cuda")
    nut_vertices = np.load(output / "insert/collision.npz")["vertices"]
    with imageio.get_writer(output / "probe.mp4", fps=30, codec="libx264") as video:
        for step in range(round(seconds / dt)):
            time = step * dt
            force[0, 0, 2] = -args.force if 2 <= time < 4 else 0
            nut.instantaneous_wrench_composer.set_forces_and_torques_index(
                forces=force,
                is_global=True,
            )
            nut.write_data_to_sim()
            ctx.step(render=step % 8 == 0)
            nut.update(dt)
            if step % 8:
                continue
            camera.update(dt * 8)
            pose = nut.data.root_pose_w.torch[0].cpu().numpy().copy()
            rotation = Rotation.from_quat(pose[3:])  # Isaac Lab 3: xyzw
            points = rotation.apply(nut_vertices) + pose[:3]
            if args.pair == "coin_slot":
                # A circular coin is invariant to spin about its own normal.
                # Measure plane alignment, not rotation of a chosen radial axis.
                normal = rotation.apply([0, 0, 1])
                reference = insert_rotation.apply([0, 0, 1])
                tilt = np.degrees(np.arccos(np.clip(abs(normal @ reference), 0, 1)))
                center = rotation.apply(nut_bounds.mean(0)) + pose[:3]
                lateral = np.linalg.norm(center[:2] - bolt_bounds.mean(0)[:2])
            else:
                tilt = np.degrees(
                    np.arccos(np.clip(rotation.apply([0, 0, 1])[2], -1, 1))
                )
                lateral = np.linalg.norm(pose[:2])
            row = {
                "time_s": (step + 1) * dt,
                "pose_xyzw": pose.tolist(),
                "downward_force_n": float(-force[0, 0, 2]),
                "lowest_point_overlap_m": float(bolt_top - points[:, 2].min()),
                "axial_entry_m": float(bolt_top - (pose[2] + insert_low)),
                "lateral_error_m": float(lateral),
                "tilt_deg": float(tilt),
            }
            rows.append(row)
            rgb = camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy()
            video.append_data(rgb)
            if step in (0, 472, 952, 1432):
                Image.fromarray(rgb).save(output / f"step-{step:04d}.png")
    report = {
        "assets": assets,
        "bolt_world_bounds": actual_bounds.tolist(),
        "initial_gap_m": 0.005,
        "insert_axial_extent_m": float(np.ptp(rotated[:, 2])),
        "pair": args.pair,
        "force_n": args.force,
        "protocol": "0–2 s gravity; 2–4 s downward CoM force; 4–6 s gravity. No commanded rotation or pose writes.",
        "limitation": "Free-body feasibility probe, not an expert or a task success test. Axial entry assumes upright nut; inspect tilt and lateral error.",
        "samples": rows,
        "phase_endpoints": [rows[59], rows[119], rows[179]],
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        "INSERTION_PROBE "
        + json.dumps(
            {"output": str(output), "phase_endpoints": report["phase_endpoints"]}
        ),
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root", type=Path, default=ROOT / ".cache/assets/robodojo"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--force",
        type=float,
        default=1.0,
        help="Downward force in N, in addition to gravity (0–2 N)",
    )
    parser.add_argument("--pair", choices=tuple(PAIRS), default="nut_bolt")
    args = parser.parse_args()
    if not 0 <= args.force <= 2:
        parser.error("--force must be finite and in [0, 2] N")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    from loom_env.runtime.app import launch_app

    launcher = launch_app(headless=True, enable_cameras=True)
    try:
        probe(args, output)
    finally:
        launcher.app.close()


if __name__ == "__main__":
    main()
