#!/usr/bin/env python3
"""Prepare pinned robot and scene models in the ignored asset cache."""

import argparse
from copy import deepcopy
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
    preparation_version,
    source_dir,
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
    original = source_urdf(root, name)
    robot = ET.parse(original).getroot()
    changes = []
    robot.set("name", name)
    if name == "yam":
        # The official Onshape URDF supplies visuals and inertias but no
        # collision geometry. Let the standard importer convexify the same meshes.
        for link in robot.findall("link"):
            for visual in link.findall("visual"):
                collision = ET.SubElement(link, "collision")
                for tag in ("origin", "geometry"):
                    child = visual.find(tag)
                    if child is not None:
                        collision.append(deepcopy(child))
        changes.append("Use YAM visual meshes as convex collision geometry")
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
    # PhysX misplaces reflected convex collision meshes. Bake the complete
    # scale into vertices before import; preserve the authored origin/shape.
    import numpy as np
    import trimesh

    for index, element in enumerate(robot.findall("link/collision/geometry/mesh")):
        scale = np.fromstring(element.get("scale", "1 1 1"), sep=" ")
        if np.any(scale < 0):
            geometry = trimesh.load(
                element.get("filename"), force="mesh", process=False
            )
            geometry.apply_scale(scale)
            destination = root / name / "meshes" / f"collision_scaled_{index}.stl"
            destination.parent.mkdir(parents=True, exist_ok=True)
            geometry.export(destination)
            element.set("filename", str(destination.resolve()))
            element.set("scale", "1 1 1")
            changes.append(
                f"Bake reflected collision mesh scale into vertices: {index}"
            )
    changes.append("Resolve mesh references to verified local source files")
    destination = prepared_urdf(root, name)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(destination, encoding="utf-8", xml_declaration=True)
    return changes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="+", choices=[*MODELS, "all", "scene"])
    parser.add_argument("--asset-root", type=Path, default=ROOT / ".cache/assets")
    parser.add_argument(
        "--source-root", type=Path, help="Shared sim_projects root for scene assets"
    )
    args = parser.parse_args()
    root = args.asset_root.resolve()
    if root.is_relative_to(ROOT) and not root.is_relative_to(ROOT / ".cache/assets"):
        parser.error("Repository-local assets must be under the ignored .cache/assets/")
    if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
        parser.error("Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's EULA")
    if "scene" in args.models:
        if args.source_root is None:
            parser.error("scene requires --source-root pointing to sim_projects")
        from loom_env.assets.prepare import prepare_scene_assets

        prepare_scene_assets(root, args.source_root)
    names = []
    for choice in args.models:
        if choice == "scene":
            continue
        if choice == "all":
            names.extend(MODELS)
        else:
            names.append(choice)
    names = list(dict.fromkeys(names))
    for source in dict.fromkeys(MODELS[name]["source"] for name in names):
        extract_source(
            root, source, [n for n in names if MODELS[n]["source"] == source]
        )
    for name in names:
        changes = normalize_urdf(root, name)
        with tempfile.TemporaryDirectory(
            prefix=f"{name}-convert-", dir=root
        ) as scratch:
            converted = Path(scratch) / "usd"
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "loom_env.assets.convert",
                    "urdf",
                    str(prepared_urdf(root, name)),
                    str(converted),
                    *CONVERSION_ARGS,
                ],
                check=True,
            )
            if not (converted / name / f"{name}.usda").is_file():
                raise RuntimeError(f"Converter did not produce the expected {name} USD")
            from pxr import Usd, UsdPhysics

            if name == "yam":
                # A single hull fills the interlocking fingers' recesses and
                # makes them collide even when fully open. Preserve their
                # external contact surfaces with PhysX's convex decomposition.
                instances = Usd.Stage.Open(
                    str(converted / name / "payloads" / "instances.usda")
                )
                fingers = set()
                for prim in instances.Traverse():
                    if prim.GetName() in {"tip_left", "tip_right"} and prim.HasAPI(
                        UsdPhysics.MeshCollisionAPI
                    ):
                        UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr(
                            "convexDecomposition"
                        )
                        fingers.add(prim.GetName())
                if fingers != {"tip_left", "tip_right"}:
                    raise ValueError("Missing YAM finger collision meshes")
                instances.GetRootLayer().Save()
                instances = None
                changes.append("Use PhysX convex decomposition for both YAM fingers")

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
            "preparation_version": preparation_version(name),
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
