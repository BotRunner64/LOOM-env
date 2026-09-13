#!/usr/bin/env python3
"""Fit cuRobo collision spheres to pinned URDF collision geometry (CUDA required)."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import trimesh

import numpy as np
import torch
from curobo.robot_builder import RobotBuilder

from loom_env.embodiments.assets import prepared_urdf, sha256, verify_asset


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "models",
        nargs="+",
        choices=[
            "yam",
            "x5",
            "ur5_wsg",
            "xarm6_robotiq",
        ],
    )
    parser.add_argument("--asset-root", type=Path, default=Path(".cache/assets"))
    args = parser.parse_args()
    for name in args.models:
        verify_asset(args.asset_root, name)
        urdf = prepared_urdf(args.asset_root, name)
        np.random.seed(0)
        torch.manual_seed(0)
        # RobotBuilder currently ignores URDF mesh scale. Bake non-unit scales
        # into cache meshes before fitting; keep the simulation URDF untouched.
        tree = ET.parse(urdf)
        cache = urdf.parent / "planning"
        cache.mkdir(exist_ok=True)
        for index, mesh in enumerate(tree.getroot().iter("mesh")):
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            if not np.allclose(scale, 1.0):
                geometry = trimesh.load(mesh.get("filename"), force="mesh")
                geometry.apply_scale(scale)
                baked = (cache / f"mesh_{index}.obj").resolve()
                geometry.export(baked)
                mesh.set("filename", str(baked))
                mesh.set("scale", "1 1 1")
        fitting_urdf = cache / "robot.urdf"
        tree.write(fitting_urdf)
        builder = RobotBuilder(str(fitting_urdf))
        settings = dict(
            sphere_density=2.0,
            protrusion_weight=100.0,
            use_collision_mesh=True,
            compute_metrics=True,
        )
        spheres = builder.fit_collision_spheres(**settings)
        output = urdf.parent / "planning.json"
        output.write_text(
            json.dumps(
                {
                    "urdf_sha256": sha256(urdf),
                    "settings": settings,
                    "collision_spheres": spheres,
                    "metrics": {
                        link: asdict(value)
                        for link, value in builder.link_metrics.items()
                    },
                },
                indent=2,
            )
            + "\n"
        )
        print(f"{name}: {builder.num_spheres} spheres -> {output}", flush=True)


if __name__ == "__main__":
    main()
