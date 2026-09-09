#!/usr/bin/env python3
"""Download the pinned Piper asset, verify it, and convert it with Isaac Lab."""

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from urllib.request import urlopen
import zipfile

from loom_env.embodiments.assets import (
    PIPER_SOURCE,
    PIPER_CONVERSION_ARGS,
    piper_source_dir,
    piper_usd,
    sha256,
)

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    args = parser.parse_args()
    root = args.asset_root.resolve()
    if root.is_relative_to(ROOT) and not root.is_relative_to(ROOT / ".cache/assets"):
        parser.error("Repository-local assets must be under the ignored .cache/assets/")
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
        parser.error("Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's EULA")
    source = piper_source_dir(root)
    archive = source.parents[1] / PIPER_SOURCE["archive"]
    archive.parent.mkdir(parents=True, exist_ok=True)
    if not archive.is_file():
        partial = archive.with_suffix(".zip.partial")
        url = (
            f"{PIPER_SOURCE['repository']}/resolve/"
            f"{PIPER_SOURCE['revision']}/{PIPER_SOURCE['archive']}"
        )
        print(f"Downloading {url}", flush=True)
        with urlopen(url, timeout=60) as response, partial.open("wb") as output:
            shutil.copyfileobj(response, output)
        if sha256(partial) != PIPER_SOURCE["sha256"]:
            raise ValueError(f"Asset archive checksum mismatch: {partial}")
        partial.replace(archive)
    if sha256(archive) != PIPER_SOURCE["sha256"]:
        raise ValueError(f"Asset archive checksum mismatch: {archive}")
    with zipfile.ZipFile(archive) as stream:
        for member in stream.infolist():
            if not member.filename.startswith("embodiments/piper/"):
                continue
            if Path(member.filename).name.startswith("."):
                continue
            target = (archive.parent / member.filename).resolve()
            if not target.is_relative_to(source):
                raise ValueError(
                    f"Asset archive path escapes Piper directory: {member.filename}"
                )
            stream.extract(member, archive.parent)
    print(f"Verified source: {source}", flush=True)
    converter = ROOT / ".deps/IsaacLab/scripts/tools/convert_urdf.py"
    # A fresh output directory prevents the importer from silently creating
    # piper_1/ on repeat runs and leaving the old piper/ as the runtime asset.
    with tempfile.TemporaryDirectory(prefix="piper-convert-", dir=root) as scratch:
        converted = Path(scratch) / "usd"
        command = [
            sys.executable,
            str(converter),
            str(source / "piper.urdf"),
            str(converted),
            *PIPER_CONVERSION_ARGS,
        ]
        subprocess.run(command, check=True)
        if not (converted / "piper/piper.usda").is_file():
            raise RuntimeError("Converter did not produce the expected Piper USD")
        destination = root / "piper/usd"
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            shutil.rmtree(destination)
        converted.replace(destination)
    usd = piper_usd(root)
    manifest = {
        "source": PIPER_SOURCE,
        "urdf_sha256": sha256(source / "piper.urdf"),
        "usd": str(usd.relative_to(root)),
        "conversion": command[4:],
        "versions": {
            name: importlib.metadata.version(name) for name in ("isaaclab", "isaacsim")
        },
        "usd_files": {
            str(path.relative_to(root)): sha256(path)
            for path in sorted(usd.parent.rglob("*"))
            if path.is_file()
        },
    }
    path = root / "piper/asset.json"
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"PIPER_ASSET_READY {usd}", flush=True)


if __name__ == "__main__":
    main()
