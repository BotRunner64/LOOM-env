#!/usr/bin/env python3
"""Copy the local RoboDojo tabletop model collection and retain source annotations."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(
    "/inspire/hdd/global_user/czxs253130598/projects/sim_projects/RoboDojo/Assets/Object/RoboDojo"
)
KINDS = ("Rigid", "Geometry", "Clutter")
MAX_EXTENT_M = 0.5
MAX_MASS_KG = 3.0


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def inventory(source):
    selected, excluded = [], []
    for kind in KINDS:
        for path in sorted((source / kind).glob("*/*/metadata.json")):
            relative = str(path.parent.relative_to(source))
            data = json.loads(path.read_text())
            geometry, physics = data.get("geometry", {}), data.get("physics", {})
            dims = (
                geometry.get("aligned_bbox", {}).get("extents")
                or geometry.get("bbox")
                or physics.get("size")
            )
            mass = physics.get("mass")
            reason = None
            if not (path.parent / "object.usdz").is_file():
                reason = "missing_usdz"
            elif (
                not isinstance(dims, list)
                or len(dims) != 3
                or any(
                    not isinstance(d, (int, float)) or not math.isfinite(d) or d < 0
                    for d in dims
                )
                or max(dims) <= 0
            ):
                reason = "invalid_dimensions"
            elif max(dims) > MAX_EXTENT_M:
                reason = "oversize"
            elif isinstance(mass, (int, float)) and (
                not math.isfinite(mass) or mass <= 0 or mass > MAX_MASS_KG
            ):
                reason = "invalid_or_excessive_mass"
            row = {
                "path": relative,
                "usd": relative + "/object.usdz",
                "dimensions_m": dims,
                "mass_kg": mass,
            }
            if reason:
                excluded.append({**row, "reason": reason})
            else:
                selected.append(row)
    return selected, excluded


def copy_model(source, target, row):
    src, dst = source / row["path"], target / row["path"]
    files = {}
    for original in sorted(src.rglob("*")):
        if not original.is_file():
            continue
        relative = original.relative_to(src)
        destination = dst / relative
        expected = digest(original)
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if destination.is_symlink() or digest(destination) != expected:
                raise ValueError(f"Existing file differs from source: {destination}")
        else:
            temporary = destination.with_name(destination.name + ".partial")
            shutil.copyfile(original, temporary)
            if digest(temporary) != expected:
                raise ValueError(f"Copy checksum mismatch: {destination}")
            temporary.replace(destination)
        files[str(relative)] = {"sha256": expected, "size": original.stat().st_size}
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
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    selected, excluded = inventory(source)
    if not selected:
        raise ValueError("No tabletop assets found")
    target.mkdir(parents=True, exist_ok=True)
    manifest = {
        "source": "https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo",
        "revision": revision,
        "source_root": str(source),
        "status": "copying",
        "selection": {
            "kinds": KINDS,
            "max_extent_m": MAX_EXTENT_M,
            "max_mass_kg_when_authored": MAX_MASS_KG,
        },
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
    issues = sum(row["dependency_check"] != "passed" for row in manifest["assets"])
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
