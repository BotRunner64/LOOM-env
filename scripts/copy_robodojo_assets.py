#!/usr/bin/env python3
"""Copy RoboDojo geometry and install the versioned LOOM USD definitions."""

import argparse
import hashlib
import importlib.metadata
import json
import shutil
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path("/home/jw/projects/sim_projects/RoboDojo/Assets/Object/RoboDojo")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def inventory(source):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    from loom_env.assets.prepare import articulation_inventory, physics_properties

    selected, excluded = [], []
    for definition in sorted(
        (ROOT / "configs/assets/robodojo").glob("*/*/*/object.usda")
    ):
        relative = definition.parent.relative_to(ROOT / "configs/assets/robodojo")
        raw = source / relative / "object.usdz"
        row = {"path": str(relative), "usd": str(relative / "object.usda")}
        if not raw.is_file():
            excluded.append({**row, "reason": "missing_usdz"})
            continue
        layer = Sdf.Layer.CreateAnonymous()
        layer.ImportFromString(definition.read_text())
        row["revision"] = layer.customLayerData.get(
            "source_revision", "a14409d7fae673c00499e01fd88b4457df6351b1"
        )
        if digest(raw) != layer.customLayerData["source_sha256"]:
            raise ValueError(f"Source geometry checksum mismatch: {raw}")
        layer.GetPrimAtPath("/Asset").referenceList.prependedItems = [
            Sdf.Reference(str(raw.resolve()))
        ]
        stage = Usd.Stage.Open(layer)
        bounds = (
            UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
            .ComputeWorldBound(stage.GetDefaultPrim())
            .ComputeAlignedRange()
        )
        dims = [
            float(x) * UsdGeom.GetStageMetersPerUnit(stage) for x in bounds.GetSize()
        ]
        row["dimensions_m"] = dims
        if relative.parts[0] == "Articulation":
            row.update(
                physics_status="pending",
                physics_issue="Articulated candidate; task admission not validated",
                articulation=articulation_inventory(stage),
            )
            selected.append(row)
            continue
        try:
            root = stage.GetDefaultPrim()
            bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if (
                bodies != [root]
                or not UsdPhysics.RigidBodyAPI(root).GetRigidBodyEnabledAttr().Get()
            ):
                raise ValueError("Object needs one enabled root rigid body")
            properties = physics_properties(stage)
            if "mass_kg" not in properties or not properties["materials"]:
                raise ValueError("Object needs a root rigid body and collision")
            row["physics_status"] = "ready"
        except ValueError as error:
            row.update(physics_status="pending", physics_issue=str(error))
        selected.append(row)
    return selected, excluded


def copy_model(source, target, row):
    src, dst = source / row["path"], target / row["path"]
    dst.mkdir(parents=True, exist_ok=True)
    files = {}
    definition = ROOT / "configs/assets/robodojo" / row["path"] / "object.usda"
    for original in (src / "object.usdz", *sorted(definition.parent.glob("*.usda"))):
        destination = dst / original.name
        expected = digest(original)
        if destination.is_symlink():
            raise ValueError(f"Refusing symlink destination: {destination}")
        changed = destination.exists() and digest(destination) != expected
        if changed and original.suffix != ".usda":
            raise ValueError(f"Existing geometry differs from source: {destination}")
        if not destination.exists() or changed:
            temporary = destination.with_name(destination.name + ".partial")
            shutil.copyfile(original, temporary)
            if digest(temporary) != expected:
                raise ValueError(f"Copy checksum mismatch: {destination}")
            temporary.replace(destination)
        files[original.name] = {"sha256": expected, "size": original.stat().st_size}
    with zipfile.ZipFile(dst / "object.usdz") as package:
        bad = package.testzip()
        if bad:
            raise ValueError(f"Corrupt USDZ member: {bad}")
    return {**row, "files": files, "copy_check": "passed"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=SOURCE)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / ".cache/assets/robodojo"
    )
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    from pxr import Usd, UsdUtils

    source, target = args.source_root.resolve(), args.output_dir.resolve()
    if (
        source == target
        or target.is_relative_to(source)
        or source.is_relative_to(target)
    ):
        raise ValueError("Source and destination must be separate directories")
    selected, excluded = inventory(source)
    if not selected:
        raise ValueError("No object assets found")
    target.mkdir(parents=True, exist_ok=True)
    manifest = {
        "source": "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "source_root": str(source),
        "status": "copying",
        "selection": "configs/assets/robodojo USD definitions",
        "excluded": excluded,
        "assets": [],
        "failures": [],
    }
    manifest_path = target / "manifest.json"

    def save():
        temporary = target / "manifest.json.partial"
        temporary.write_text(json.dumps(manifest, indent=2) + "\n")
        temporary.replace(manifest_path)

    print(f"SELECTED {len(selected)} models; EXCLUDED {len(excluded)}", flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(copy_model, source, target, row): row for row in selected
        }
        for future in as_completed(futures):
            try:
                manifest["assets"].append(future.result())
            except Exception as error:
                manifest["failures"].append(
                    {"path": futures[future]["path"], "error": str(error)}
                )
            count = len(manifest["assets"]) + len(manifest["failures"])
            if count % 25 == 0 or count == len(selected):
                save()
                print(
                    f"COPIED {count}/{len(selected)}; failed={len(manifest['failures'])}",
                    flush=True,
                )
    sdk = Path(
        importlib.metadata.distribution("isaacsim").locate_file("isaacsim/kit/mdl")
    )
    runtime = [path.as_posix() for path in sdk.rglob("*.mdl")]
    manifest["status"] = "checking_dependencies"
    manifest["assets"].sort(key=lambda row: row["path"])
    for index, row in enumerate(manifest["assets"]):
        try:
            usd = target / row["usd"]
            stage = Usd.Stage.Open(str(usd))
            if not stage or not stage.GetDefaultPrim():
                raise ValueError("USD has no valid default prim")
            layers, resources, missing = UsdUtils.ComputeAllDependencies(str(usd))
            row["runtime_materials"] = sorted(
                {
                    name
                    for name in missing
                    if any(path.endswith("/" + name) for path in runtime)
                }
            )
            unresolved = set(missing) - set(row["runtime_materials"])
            outside = [
                ref
                for ref in [layer.identifier for layer in layers] + resources
                if not Path(ref.split("[")[0]).resolve().is_relative_to(target)
            ]
            row["unresolved_dependencies"] = sorted(unresolved)
            row["external_dependencies"] = outside
            row["dependency_check"] = "failed" if unresolved or outside else "passed"
            del stage, layers
        except Exception as error:
            row.update(dependency_check="failed", dependency_error=str(error))
        if (index + 1) % 25 == 0 or index + 1 == len(manifest["assets"]):
            save()
            print(f"CHECKED {index + 1}/{len(manifest['assets'])}", flush=True)
    issues = len(excluded) + sum(
        row["dependency_check"] != "passed" for row in manifest["assets"]
    )
    manifest["status"] = (
        "copied_with_issues"
        if issues or manifest["failures"]
        else "copied_and_dependencies_checked"
    )
    save()
    size = sum(f["size"] for row in manifest["assets"] for f in row["files"].values())
    print(
        f"RESULT models={len(manifest['assets'])} GiB={size / 1024**3:.2f} dependency_issues={issues} copy_failures={len(manifest['failures'])}",
        flush=True,
    )
    print(f"MANIFEST {manifest_path}", flush=True)
    return 1 if issues or manifest["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
