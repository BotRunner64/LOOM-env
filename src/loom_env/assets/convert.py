"""Run installed Isaac Lab converters in an isolated simulator process."""

import argparse
from pathlib import Path


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("urdf", "mesh"))
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fix-base", action="store_true")
    parser.add_argument("--merge-joints", action="store_true")
    parser.add_argument("--joint-stiffness", type=float, default=100.0)
    parser.add_argument("--joint-damping", type=float, default=1.0)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    source, destination = args.input.resolve(), args.output.resolve()
    if not source.is_file():
        parser.error(f"Input asset does not exist: {source}")

    app = AppLauncher(args).app
    try:
        from isaaclab.sim.converters import (
            MeshConverter,
            MeshConverterCfg,
            UrdfConverter,
            UrdfConverterCfg,
        )
        from isaaclab.sim.schemas import schemas_cfg

        if args.kind == "urdf":
            converter = UrdfConverter(
                UrdfConverterCfg(
                    asset_path=str(source),
                    usd_dir=str(destination),
                    fix_base=args.fix_base,
                    merge_fixed_joints=args.merge_joints,
                    force_usd_conversion=True,
                    joint_drive=UrdfConverterCfg.JointDriveCfg(
                        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                            stiffness=args.joint_stiffness,
                            damping=args.joint_damping,
                        ),
                        target_type="position",
                    ),
                )
            )
        else:
            converter = MeshConverter(
                MeshConverterCfg(
                    asset_path=str(source),
                    usd_dir=str(destination.parent),
                    usd_file_name=destination.name,
                    force_usd_conversion=True,
                    make_instanceable=False,
                    mass_props=None,
                    rigid_props=None,
                    collision_props=schemas_cfg.CollisionPropertiesCfg(
                        collision_enabled=True
                    ),
                    mesh_collision_props=schemas_cfg.TriangleMeshPropertiesCfg(),
                )
            )
        print(f"ASSET_CONVERTED {converter.usd_path}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
