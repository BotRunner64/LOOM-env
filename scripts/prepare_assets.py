#!/usr/bin/env python3
"""Prepare pinned robot models in the ignored cache with Isaac Lab's URDF converter."""

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
import xml.etree.ElementTree as ET
import zipfile

from loom_env.embodiments.assets import (
    SOURCES,
    MODELS,
    CONVERSION_ARGS,
    PREPARATION_VERSION,
    source_dir,
    source_tree,
    source_urdf,
    prepared_urdf,
    converted_usd,
    sha256,
    validate_visuals,
)

ROOT = Path(__file__).resolve().parents[1]


def extract_source(root, source, names):
    info = SOURCES[source]
    directory = source_dir(root, source)
    directory.mkdir(parents=True, exist_ok=True)
    archive = directory / info["archive"]
    if not archive.is_file():
        partial = archive.with_suffix(".zip.partial")
        if source == "robotwin":
            url = f"{info['repository']}/resolve/{info['revision']}/{info['archive']}"
        else:
            repo = info["repository"].removeprefix("https://github.com/")
            url = f"https://codeload.github.com/{repo}/zip/{info['revision']}"
        print(f"Downloading {url}", flush=True)
        with urlopen(url, timeout=60) as response, partial.open("wb") as output:
            shutil.copyfileobj(response, output)
        if sha256(partial) != info["sha256"]:
            raise ValueError(f"Asset archive checksum mismatch: {partial}")
        partial.replace(archive)
    if sha256(archive) != info["sha256"]:
        raise ValueError(f"Asset archive checksum mismatch: {archive}")
    prefixes = [str(Path(MODELS[name]["urdf"]).parent) + "/" for name in names]
    with zipfile.ZipFile(archive) as stream:
        for member in stream.infolist():
            if any(part.startswith(".") for part in Path(member.filename).parts):
                continue
            if source == "robotwin" and not any(
                member.filename.startswith(p) for p in prefixes
            ):
                continue
            target = (directory / member.filename).resolve()
            if not target.is_relative_to(directory.resolve()):
                raise ValueError(
                    f"Archive path escapes source directory: {member.filename}"
                )
            stream.extract(member, directory)


def normalize_urdf(root, name):
    """Keep source files intact; record all simulator-specific repairs in asset.json."""
    model = MODELS[name]
    original = source_urdf(root, name)
    robot = ET.parse(original).getroot()
    changes = []
    robot.set("name", name)
    if model["source"] == "openarm":
        # Official example contains torso + both arms. Keep the requested arm's
        # rooted subtree, preserving every local transform and its handed limits.
        links = {model["base"]}
        while True:
            children = {
                j.find("child").get("link")
                for j in robot.findall("joint")
                if j.find("parent").get("link") in links
            }
            if children <= links:
                break
            links |= children
        for child in list(robot):
            if child.tag == "link" and child.get("name") not in links:
                robot.remove(child)
            elif child.tag == "joint" and child.find("parent").get("link") not in links:
                robot.remove(child)
            elif child.tag not in {"link", "joint", "material"}:
                robot.remove(child)
        changes.append(
            f"Extract arm subtree rooted at {model['base']}; omit torso and opposite arm"
        )
    for joint in robot.findall("joint"):
        limits = joint.findall("limit")
        if len(limits) > 1:
            # UR5's second limit lacks position bounds. The first complete limit
            # is the source of truth; avoid parser-dependent duplicate handling.
            if not {"lower", "upper", "effort", "velocity"} <= limits[0].attrib.keys():
                raise ValueError(f"Incomplete first limit: {joint.get('name')}")
            for extra in limits[1:]:
                joint.remove(extra)
            changes.append(
                f"Remove duplicate limit from {joint.get('name')}; retain first complete limit"
            )
    if name == "xarm6_robotiq":
        # Replace SAPIEN's external four-bar point constraints with equivalent
        # ideal parallel-link angular coupling. The driving knuckle remains active.
        master = "left_outer_knuckle_joint"
        for joint in robot.findall("joint"):
            jname = joint.get("name")
            if jname != master and any(
                word in jname for word in ("knuckle_joint", "inner_finger_joint")
            ):
                ET.SubElement(
                    joint,
                    "mimic",
                    joint=master,
                    multiplier="-1" if "inner_finger" in jname else "1",
                    offset="0",
                )
        changes.append(
            "Add five Robotiq ideal parallel-link mimic relations to left_outer_knuckle_joint"
        )
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename.startswith("package://"):
            package, relative = filename.removeprefix("package://").split("/", 1)
            if package != "openarm_description":
                raise ValueError(f"Unknown mesh package: {package}")
            file = source_tree(root, model["source"]) / relative
        else:
            file = original.parent / filename
        if not file.is_file():
            raise FileNotFoundError(file)
        if file.suffix.lower() == ".glb":
            # Importer 3.0 silently drops GLB visuals. Flatten the glTF scene to
            # OBJ with the standard mesh library, preserving node transforms/materials.
            import trimesh

            destination = root / name / "meshes" / file.stem
            destination.mkdir(parents=True, exist_ok=True)
            obj, resources = trimesh.exchange.obj.export_obj(
                trimesh.load_scene(file, process=False),
                return_texture=True,
                mtl_name="materials.mtl",
            )
            for relative, data in resources.items():
                target = destination / relative
                if not target.resolve().is_relative_to(destination.resolve()):
                    raise ValueError(f"Mesh resource escapes output: {relative}")
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
            file = destination / "mesh.obj"
            file.write_text(obj)
            changes.append(
                f"Convert GLB visual to OBJ with materials: {mesh.get('filename')}"
            )
        mesh.set("filename", str(file.resolve()))
    changes.append("Resolve mesh references to verified local source files")
    destination = prepared_urdf(root, name)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(destination, encoding="utf-8", xml_declaration=True)
    return changes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="+", choices=[*MODELS, "openarm", "all"])
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    args = parser.parse_args()
    root = args.asset_root.resolve()
    if root.is_relative_to(ROOT) and not root.is_relative_to(ROOT / ".cache/assets"):
        parser.error("Repository-local assets must be under the ignored .cache/assets/")
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
        parser.error("Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's EULA")
    names = []
    for choice in args.models:
        if choice == "all":
            names.extend(MODELS)
        elif choice == "openarm":
            names.extend(("openarm_left", "openarm_right"))
        else:
            names.append(choice)
    names = list(dict.fromkeys(names))
    for source in dict.fromkeys(MODELS[name]["source"] for name in names):
        extract_source(
            root, source, [n for n in names if MODELS[n]["source"] == source]
        )
    for name in names:
        changes = normalize_urdf(root, name)
        converter = ROOT / ".deps/IsaacLab/scripts/tools/convert_urdf.py"
        with tempfile.TemporaryDirectory(
            prefix=f"{name}-convert-", dir=root
        ) as scratch:
            converted = Path(scratch) / "usd"
            subprocess.run(
                [
                    sys.executable,
                    str(converter),
                    str(prepared_urdf(root, name)),
                    str(converted),
                    *CONVERSION_ARGS,
                ],
                check=True,
            )
            if not (converted / name / f"{name}.usda").is_file():
                raise RuntimeError(f"Converter did not produce the expected {name} USD")
            from pxr import Usd

            stage = Usd.Stage.Open(str(converted / name / f"{name}.usda"))
            visual_counts = validate_visuals(stage.GetDefaultPrim())
            stage = None
            destination = root / name / "usd"
            if destination.exists():
                shutil.rmtree(destination)
            converted.replace(destination)
        manifest = {
            "model": name,
            "source": SOURCES[MODELS[name]["source"]],
            "source_urdf_sha256": sha256(source_urdf(root, name)),
            "urdf_sha256": sha256(prepared_urdf(root, name)),
            "preparation_version": PREPARATION_VERSION,
            "visual_meshes_per_body": visual_counts,
            "prepared_mesh_files": {
                str(path.relative_to(root)): sha256(path)
                for path in sorted((root / name / "meshes").rglob("*"))
                if path.is_file()
            },
            "changes": changes,
            "conversion": list(CONVERSION_ARGS),
            "versions": {
                name: importlib.metadata.version(name)
                for name in ("isaaclab", "isaacsim")
            },
            "usd_files": {
                str(path.relative_to(root)): sha256(path)
                for path in sorted((root / name / "usd").rglob("*"))
                if path.is_file()
            },
        }
        (root / name / "asset.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"ASSET_READY {converted_usd(root, name)}", flush=True)


if __name__ == "__main__":
    main()
