"""Copy USDZ assets unchanged; convert mesh-only workspaces with standard tools."""

from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from .catalog import ASSETS, PREPARATION_VERSION, prepared_directory, sha256


def source_physics(stage):
    """Read authored physics, relative to the default prim, for preservation checks."""
    from pxr import Usd

    root = stage.GetDefaultPrim()
    result = {}
    for prim in Usd.PrimRange(root):
        schemas = [
            s for s in prim.GetAppliedSchemas() if "Physics" in s or "Physx" in s
        ]
        attributes = {
            attr.GetName(): str(attr.Get())
            for attr in prim.GetAttributes()
            if attr.HasAuthoredValue()
            and attr.GetName().startswith(("physics:", "physx"))
        }
        if schemas or attributes:
            result[str(prim.GetPath().MakeRelativePath(root.GetPath()))] = {
                "schemas": schemas,
                "attributes": attributes,
            }
    return result


def mesh_arrays(prims):
    from pxr import Usd, UsdGeom

    vertices, triangles = [], []
    for prim in prims:
        mesh = UsdGeom.Mesh(prim)
        points = np.array(mesh.GetPointsAttr().Get(), dtype=float)
        matrix = np.array(
            UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        )
        points = (np.c_[points, np.ones(len(points))] @ matrix)[:, :3]
        indices = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
        start, offset = 0, len(vertices)
        for count in mesh.GetFaceVertexCountsAttr().Get():
            face = indices[start : start + count]
            triangles.extend(
                [offset + face[0], offset + face[i], offset + face[i + 1]]
                for i in range(1, count - 1)
            )
            start += count
        vertices.extend(points.tolist())
    vertices, triangles = np.array(vertices), np.array(triangles, dtype=np.int32)
    if not np.isfinite(vertices).all() or triangles.size == 0:
        raise ValueError("Invalid source collision mesh")
    return vertices, triangles


def table_collision_parts(vertices, triangles, tabletop_bounds):
    """Replace exactly the measured top component, retaining all lower geometry."""
    import trimesh

    source = trimesh.Trimesh(vertices, triangles, process=True)
    components = source.split(only_watertight=False)
    selected = [
        part
        for part in components
        if np.allclose(part.bounds, tabletop_bounds, atol=1e-5, rtol=0)
    ]
    if len(selected) != 1:
        raise ValueError("Expected one tabletop component matching the catalog bounds")
    lower = trimesh.util.concatenate(
        [part for part in components if part is not selected[0]]
    )
    bounds = np.asarray(tabletop_bounds)
    slab = trimesh.creation.box(extents=bounds[1] - bounds[0])
    slab.apply_translation(bounds.mean(axis=0))
    return lower, slab


def prepare_scene_assets(asset_root, source_root):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    for asset_id, definition in ASSETS.items():
        source = Path(source_root) / definition.path
        if not source.is_file() or sha256(source) != definition.sha256:
            raise ValueError(f"Source checksum mismatch or missing asset: {source}")
        directory = prepared_directory(asset_root, asset_id).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "asset.json").unlink(missing_ok=True)
        mesh_only = source.suffix == ".glb"
        if mesh_only:
            local = directory / "table.usd"
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "loom_env.assets.convert",
                    "mesh",
                    str(source),
                    str(local),
                    "--headless",
                ],
                check=True,
            )
        else:
            local = directory / "source.usdz"
            shutil.copyfile(source, local)
        source_stage = Usd.Stage.Open(str(local))
        if not mesh_only and (
            UsdGeom.GetStageUpAxis(source_stage) != "Z"
            or UsdGeom.GetStageMetersPerUnit(source_stage) != 1.0
        ):
            raise ValueError(f"Source needs an explicit unit/axis adapter: {asset_id}")
        destination = directory / "asset.usda"
        # Start with an empty reference layer; no stale physics overrides survive.
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
        stage.SetDefaultPrim(root)
        UsdGeom.SetStageUpAxis(stage, "Z")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        if mesh_only:
            model = UsdGeom.Xform.Define(stage, "/Asset/Model").GetPrim()
            model.GetReferences().AddReference(str(local))
            original = UsdGeom.Xformable(model).GetLocalTransformation()
            normalization = Gf.Matrix4d().SetScale(definition.source_scale)
            normalization.SetTranslateOnly(Gf.Vec3d(*definition.source_translation))
            UsdGeom.Xformable(model).MakeMatrixXform().Set(original * normalization)
        else:
            root.GetReferences().AddReference(str(local))
        collision_prims = [
            p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if not collision_prims or any(not p.IsA(UsdGeom.Mesh) for p in collision_prims):
            raise ValueError(f"Expected authored mesh collision: {asset_id}")
        actual_dynamic = (
            root.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(root).GetRigidBodyEnabledAttr().Get()
        )
        if bool(actual_dynamic) != definition.dynamic:
            raise ValueError(f"Source rigid body differs from inventory: {asset_id}")
        vertices, triangles = mesh_arrays(collision_prims)
        actual = np.array([vertices.min(axis=0), vertices.max(axis=0)])
        if not np.allclose(actual, definition.bounds, atol=0.001, rtol=0):
            raise ValueError(f"Source dimensions differ: {asset_id}: {actual}")
        source_triangle_count = len(triangles)
        if mesh_only:
            # GLB has no source physics. Use a flat primitive for the tabletop
            # and retain the source lower geometry after coordinate conversion.
            for prim in collision_prims:
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
            if definition.tabletop_bounds is None:
                raise ValueError(
                    f"Mesh workspace needs reviewed tabletop bounds: {asset_id}"
                )
            lower, slab = table_collision_parts(
                vertices, triangles, definition.tabletop_bounds
            )
            vertices, triangles = np.asarray(lower.vertices), np.asarray(lower.faces)
            bounds = np.asarray(definition.tabletop_bounds)
            top = UsdGeom.Cube.Define(stage, "/Asset/Tabletop")
            top.CreateSizeAttr(1.0)
            top.AddTranslateOp().Set(Gf.Vec3d(*bounds.mean(axis=0)))
            top.AddScaleOp().Set(Gf.Vec3f(*(bounds[1] - bounds[0])))
            top.CreateVisibilityAttr("invisible")
            UsdPhysics.CollisionAPI.Apply(top.GetPrim())
            collider = UsdGeom.Mesh.Define(stage, "/Asset/Collision")
            collider.CreatePointsAttr(vertices.tolist())
            collider.CreateFaceVertexCountsAttr([3] * len(triangles))
            collider.CreateFaceVertexIndicesAttr(triangles.reshape(-1).tolist())
            collider.CreateSubdivisionSchemeAttr("none")
            collider.CreateVisibilityAttr("invisible")
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(
                collider.GetPrim()
            ).CreateApproximationAttr("none")
            # The planner sees precisely the same slab and retained lower mesh.
            import trimesh

            planning = trimesh.util.concatenate([lower, slab])
            vertices, triangles = (
                np.asarray(planning.vertices),
                np.asarray(planning.faces),
            )
        elif source_physics(stage) != source_physics(source_stage):
            raise ValueError(f"Imported USDZ physics differs from source: {asset_id}")
        # References become relative only at export; source model bytes are intact.
        reference_prim = model if mesh_only else root
        reference_prim.GetReferences().ClearReferences()
        reference_prim.GetReferences().AddReference(local.name)
        stage.GetRootLayer().Export(str(destination))
        np.savez_compressed(
            directory / "collision.npz", vertices=vertices, faces=triangles
        )
        prepared = Usd.Stage.Open(str(destination))
        if not mesh_only and source_physics(prepared) != source_physics(source_stage):
            raise ValueError(f"Cached USDZ physics differs from source: {asset_id}")
        manifest = {
            "asset_id": asset_id,
            "definition": definition.fingerprint,
            "source": asdict(definition),
            "preparation_version": PREPARATION_VERSION,
            "bounds": actual.tolist(),
            "source_collision_triangles": source_triangle_count,
            "collision_representation": "box_top_and_source_legs"
            if mesh_only
            else "source",
            "physics": source_physics(prepared),
            "source_physics_preserved": not mesh_only,
            "versions": {
                name: importlib.metadata.version(name)
                for name in ("isaaclab", "isaacsim")
            },
            "files": {
                str(p.relative_to(directory)): sha256(p)
                for p in sorted(directory.rglob("*"))
                if p.is_file() and p.name != "asset.json"
            },
        }
        (directory / "asset.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"ASSET_READY {asset_id} {destination}", flush=True)
