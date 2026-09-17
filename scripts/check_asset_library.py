#!/usr/bin/env python3
"""Audit every local object entry directly from USD; no simulator or GPU required."""

import argparse
import json
from pathlib import Path

from loom_env.assets.catalog import ASSETS, is_articulated, prepared_directory
from loom_env.assets.isaacsim import ISAACSIM_PROPS
from loom_env.assets.prepare import articulation_inventory, physics_properties

ROOT = Path(__file__).resolve().parents[1]


def inspect_asset(path, dynamic, collision=True, definition=None):
    from pxr import Tf, Usd, UsdPhysics

    try:
        if not path.is_file():
            raise ValueError("Missing USD entry")
        stage = Usd.Stage.Open(str(path))
        root = stage.GetDefaultPrim()
        if not root:
            raise ValueError("Missing default prim")
        if definition is not None and is_articulated(definition):
            from loom_env.assets.articulation import describe

            return {
                "status": "ready",
                "physics": describe(stage, definition),
                "scope": "USD fixed-hinge checks; simulation requires scene validation",
            }
        if any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse()):
            return {
                "status": "pending",
                "reason": "Articulated candidate; task admission not validated",
                "articulation": articulation_inventory(stage),
            }
        bodies = [
            p
            for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
            if p.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if dynamic and (
            len(bodies) != 1
            or not UsdPhysics.RigidBodyAPI(bodies[0]).GetRigidBodyEnabledAttr().Get()
        ):
            raise ValueError("Object needs exactly one enabled rigid body")
        properties = physics_properties(stage, root=bodies[0] if dynamic else root)
        if collision and not properties["materials"]:
            raise ValueError("Object has no collision")
        return {"status": "ready", "physics": properties}
    except (ValueError, OSError, Tf.ErrorException) as error:
        return {"status": "pending", "reason": str(error)}


def audit(asset_root):
    rows = []
    definitions = ROOT / "configs/assets/robodojo"
    for path in sorted(definitions.glob("*/*/*/object.usda")):
        local = asset_root / "robodojo" / path.relative_to(definitions)
        rows.append(
            {"library": "robodojo", "path": str(local), **inspect_asset(local, True)}
        )
    for group, paths in ISAACSIM_PROPS.items():
        for path in paths:
            local = asset_root / "isaacsim-6.0/Isaac/Props" / path
            rows.append(
                {
                    "library": "isaacsim",
                    "path": str(local),
                    **inspect_asset(local, group != "workspaces"),
                }
            )
    for key, definition in ASSETS.items():
        local = prepared_directory(asset_root, key) / "asset.usda"
        rows.append(
            {
                "library": "scene",
                "asset_id": key,
                "path": str(local),
                **inspect_asset(
                    local,
                    definition.dynamic,
                    definition.category != "target_region"
                    or definition.name == "placemat",
                    definition=definition,
                ),
            }
        )
    return {
        "scope": "Rigid body mass and bound material checks; articulated source inventory remains pending task admission; simulation requires scene checks",
        "ready": sum(r["status"] == "ready" for r in rows),
        "pending": sum(r["status"] == "pending" for r in rows),
        "assets": rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    parser.add_argument(
        "--output", type=Path, default=ROOT / "outputs/asset-physics/library-audit.json"
    )
    args = parser.parse_args()
    report = audit(args.asset_root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(
        f"USD_READY {report['ready']} PENDING {report['pending']} REPORT {args.output.resolve()}"
    )
    return 1 if report["pending"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
