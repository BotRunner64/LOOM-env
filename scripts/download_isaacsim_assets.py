#!/usr/bin/env python3
"""Download the selected Isaac Sim 6.0 tabletop models and their dependencies."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.metadata
import json
from pathlib import Path
import re
import shutil
import time
from urllib.parse import quote
from urllib.request import Request, urlopen
import zipfile

from loom_env.assets.isaacsim import ISAACSIM_PROPS

ROOT = Path(__file__).resolve().parents[1]
SERVER = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
PREFIX = "Assets/Isaac/6.0/"
ASSET_PATHS = [
    "Isaac/Props/" + path for paths in ISAACSIM_PROPS.values() for path in paths
]


def digest(path, algorithm="sha256"):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, algorithm).hexdigest()


def inventory():
    rows = []
    for relative in ASSET_PATHS:
        request = Request(SERVER + quote(PREFIX + relative, safe="/"), method="HEAD")
        with urlopen(request, timeout=60) as response:
            rows.append(
                {
                    "path": relative,
                    "size": int(response.headers["Content-Length"]),
                    "etag": response.headers["ETag"].strip('"'),
                    "last_modified": response.headers.get("Last-Modified"),
                }
            )
    return rows


def download(root, item):
    target = (root / item["path"]).resolve()
    if not target.is_relative_to(root.resolve()):
        raise ValueError(f"Path escapes cache: {item['path']}")
    if target.is_file() and target.stat().st_size == item["size"]:
        if item.get("sha256") and digest(target) == item["sha256"]:
            return item
        if re.fullmatch(r"[0-9a-f]{32}", item["etag"]):
            if digest(target, "md5") == item["etag"]:
                return {**item, "sha256": digest(target)}
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + ".partial")
    for attempt in range(3):
        try:
            request = Request(
                SERVER + quote(PREFIX + item["path"], safe="/"),
                headers={"If-Match": '"' + item["etag"] + '"'},
            )
            with urlopen(request, timeout=90) as response, partial.open("wb") as stream:
                shutil.copyfileobj(response, stream)
            if partial.stat().st_size != item["size"]:
                raise ValueError(f"Size mismatch: {target}")
            if re.fullmatch(r"[0-9a-f]{32}", item["etag"]):
                if digest(partial, "md5") != item["etag"]:
                    raise ValueError(f"ETag checksum mismatch: {target}")
            checksum = digest(partial)
            if item.get("sha256") and checksum != item["sha256"]:
                raise ValueError(f"SHA-256 mismatch: {target}")
            partial.replace(target)
            return {**item, "sha256": checksum}
        except Exception:
            if attempt == 2:
                raise
            time.sleep(attempt + 1)


def check_dependencies(root, manifest):
    """Fetch relative USD/MDL file dependencies; report upstream broken references."""
    from pxr import UsdUtils

    root = root.resolve()
    known = {row["path"]: row for row in manifest["files"]}
    pending = list(known)
    checked, missing, runtime = [], {}, set()
    edges = {}
    sdk = Path(
        importlib.metadata.distribution("isaacsim").locate_file("isaacsim/kit/mdl")
    )
    runtime_files = {p.name for p in sdk.rglob("*.mdl")}
    while pending:
        relative = pending.pop()
        path = root / relative
        suffix = path.suffix.lower()
        if suffix in (".usd", ".usda", ".usdc", ".usdz"):
            groups = UsdUtils.ExtractExternalReferences(str(path))
            references = sorted({ref for group in groups for ref in group})
            checked.append(relative)
        elif suffix == ".mdl":
            source = path.read_text()
            references = sorted(
                set(re.findall(r'"([^"\n]+\.(?:png|jpg|jpeg|exr|dds|mdl))"', source))
            )
            if "::nvidia::core_definitions" in source:
                if "core_definitions.mdl" not in runtime_files:
                    raise ValueError("Isaac Sim core_definitions.mdl is unavailable")
                runtime.add("nvidia/core_definitions.mdl")
        else:
            continue
        for ref in references:
            if ref in runtime_files and "/" not in ref:
                runtime.add(ref)
                continue
            if suffix == ".usdz":
                with zipfile.ZipFile(path) as archive:
                    if ref.removeprefix("./") in archive.namelist():
                        continue
            outer, separator, inner = ref.partition("[")
            target = (path.parent / outer).resolve()
            if not target.is_relative_to(root):
                missing.setdefault(
                    ref, {"error": "Reference outside cache", "users": []}
                )["users"].append(relative)
                continue
            key = str(target.relative_to(root))
            edges.setdefault(relative, set()).add(key)
            if key in missing:
                missing[key]["users"].append(relative)
                continue
            if key not in known:
                try:
                    request = Request(
                        SERVER + quote(PREFIX + key, safe="/"), method="HEAD"
                    )
                    with urlopen(request, timeout=60) as response:
                        row = {
                            "path": key,
                            "size": int(response.headers["Content-Length"]),
                            "etag": response.headers["ETag"].strip('"'),
                            "last_modified": response.headers.get("Last-Modified"),
                        }
                    known[key] = download(root, row)
                    pending.append(key)
                    print(f"DEPENDENCY {key}", flush=True)
                except Exception as error:
                    missing[key] = {"error": str(error), "users": [relative]}
                    print(f"UNRESOLVED {key}: {error}", flush=True)
                    continue
            if separator:
                with zipfile.ZipFile(target) as archive:
                    if inner.removesuffix("]") not in archive.namelist():
                        missing.setdefault(
                            ref, {"error": "Missing package member", "users": []}
                        )["users"].append(relative)
    manifest["files"] = sorted(known.values(), key=lambda row: row["path"])
    affected = {user for item in missing.values() for user in item["users"]}
    while True:
        parents = {user for user, deps in edges.items() if deps & affected}
        if parents <= affected:
            break
        affected.update(parents)
    report = {
        "affected_usd_files": sorted(set(checked) & affected),
        "checked_usd_files": sorted(checked),
        "unresolved": missing,
        "runtime_materials": sorted(runtime),
        "scope": "USD asset references, USDZ members and MDL texture paths; no physics or render validation",
    }
    report_path = root / "dependency-report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    manifest["status"] = (
        "downloaded_with_upstream_missing_dependencies"
        if missing
        else "dependencies_checked"
    )
    print(f"DEPENDENCY_REPORT {report_path}: {len(missing)} unresolved", flush=True)
    return missing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / ".cache/assets/isaacsim-6.0"
    )
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    # Fail before downloading if the documented simulation environment is absent.
    from pxr import UsdUtils  # noqa: F401

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output_dir / "download-manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if (
            manifest["source"] != SERVER + PREFIX
            or manifest["asset_paths"] != ASSET_PATHS
        ):
            raise ValueError("Existing manifest has different source or selection")
        rows = manifest["files"]
    else:
        rows = inventory()
        manifest = {
            "source": SERVER + PREFIX,
            "asset_paths": ASSET_PATHS,
            "status": "pending",
            "files": rows,
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        f"Downloading/verifying {len(rows)} files, "
        f"{sum(r['size'] for r in rows) / 1024**2:.1f} MiB",
        flush=True,
    )
    completed, failed = [], []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(download, args.output_dir, row): row for row in rows}
        for future in as_completed(futures):
            row = futures[future]
            try:
                completed.append(future.result())
            except Exception as error:
                failed.append({"path": row["path"], "error": str(error)})
                print(f"FAILED {row['path']}: {error}", flush=True)
            if (len(completed) + len(failed)) % 25 == 0:
                print(
                    f"Progress {len(completed) + len(failed)}/{len(rows)}", flush=True
                )
    updated = {row["path"]: row for row in completed}
    manifest.update(
        status="failed" if failed else "downloaded",
        files=[updated.get(row["path"], row) for row in rows],
        failures=failed,
    )
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"MANIFEST {manifest_path.resolve()}", flush=True)
    if failed:
        raise SystemExit(1)
    missing = check_dependencies(args.output_dir, manifest)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    if missing:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
