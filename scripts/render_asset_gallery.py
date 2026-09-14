#!/usr/bin/env python3
"""Render local object assets into labelled contact sheets with Isaac Sim RTX."""

import argparse
import json
import math
from pathlib import Path
import textwrap

from loom_env.assets.isaacsim import ISAACSIM_PROPS

ROOT = Path(__file__).resolve().parents[1]


def select_assets(root, selections=ISAACSIM_PROPS):
    """Render exactly the source models retained by the downloader."""
    result = []
    for group, paths in selections.items():
        for relative in paths:
            path = root / relative
            if not path.is_file():
                raise FileNotFoundError(path)
            result.append(
                {
                    "group": group,
                    "asset": relative,
                    "label": f"{path.parent.parent.name}_{path.parent.name}"
                    if path.stem == "object"
                    else path.stem,
                    "source": str(path.resolve()),
                }
            )
    return result


def contact_sheets(rows, output):
    from PIL import Image, ImageDraw, ImageFont

    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 16)
    small = ImageFont.truetype(font_path, 14)
    title_font = ImageFont.truetype(font_path, 30)
    sheets = []
    for group in dict.fromkeys(row["group"] for row in rows):
        selected = [row for row in rows if row["group"] == group]
        columns = min(5, len(selected))
        width, height, pad, header = 384, 366, 16, 100
        sheet = Image.new(
            "RGB",
            (
                columns * (width + pad) + pad,
                math.ceil(len(selected) / columns) * (height + pad) + header,
            ),
            (23, 29, 39),
        )
        draw = ImageDraw.Draw(sheet)
        title = f"{selected[0].get('collection', 'Isaac Sim 6.0').upper()} / {group.upper()} / {len(selected)} assets"
        fitted_title = title_font.font_variant(
            size=min(
                30, int((sheet.width - 2 * pad) / title_font.getlength(title) * 30)
            )
        )
        draw.text(
            (pad, 16),
            title,
            font=fitted_title,
            fill=(242, 246, 250),
        )
        draw.text(
            (pad, 60),
            "RTX | source materials | fitted views | cm",
            font=small,
            fill=(177, 191, 207),
        )
        for index, row in enumerate(selected):
            x = pad + (index % columns) * (width + pad)
            y = header + (index // columns) * (height + pad)
            draw.rectangle((x, y, x + width, y + height), fill=(36, 45, 59))
            if row.get("image"):
                with Image.open(output / row["image"]) as source:
                    sheet.paste(source.convert("RGB").resize((width, 288)), (x, y))
            else:
                draw.text(
                    (x + 12, y + 120),
                    "RENDER FAILED - see manifest",
                    font=font,
                    fill=(255, 142, 142),
                )
            label = row["label"].replace("SM_", "").replace("sm_", "")
            for line_index, line in enumerate(textwrap.wrap(label, width=38)[:2]):
                draw.text(
                    (x + 10, y + 296 + line_index * 19),
                    line,
                    font=font,
                    fill=(239, 244, 251),
                )
            if "dimensions_m" in row:
                size = " x ".join(f"{d * 100:.1f}" for d in row["dimensions_m"]) + " cm"
                draw.text((x + 10, y + 342), size, font=small, fill=(167, 189, 211))
        path = output / f"{group}.jpg"
        sheet.save(path, quality=94)
        sheets.append(path)
    return sheets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--collection", choices=("isaacsim", "robodojo"), default="isaacsim"
    )
    parser.add_argument("--asset-root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--group")
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    if args.collection == "robodojo":
        selections = json.loads(
            (ROOT / "configs/assets/robodojo_preview.json").read_text()
        )
        args.asset_root = args.asset_root or ROOT / ".cache/assets/robodojo"
        args.output_dir = args.output_dir or ROOT / ".cache/previews/robodojo-sample"
    else:
        selections = ISAACSIM_PROPS
        args.asset_root = (
            args.asset_root or ROOT / ".cache/assets/isaacsim-6.0/Isaac/Props"
        )
        args.output_dir = args.output_dir or ROOT / ".cache/previews/isaacsim-assets"
    if args.group and args.group not in selections:
        parser.error(f"Unknown group {args.group}; choose from {', '.join(selections)}")
    rows = select_assets(args.asset_root, selections)
    for row in rows:
        row["collection"] = (
            "RoboDojo" if args.collection == "robodojo" else "Isaac Sim 6.0"
        )
    if args.group:
        rows = [row for row in rows if row["group"] == args.group]
    if args.limit:
        rows = rows[: args.limit]
    if not rows:
        raise ValueError(
            "No assets found; run scripts/download_isaacsim_assets.py first"
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "images").mkdir(exist_ok=True)
    from loom_env.runtime.app import launch_app

    launcher = launch_app(headless=True, enable_cameras=True)
    app = launcher.app
    try:
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdShade

        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, "Z")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        light = UsdLux.DomeLight.Define(stage, "/World/Dome")
        light.CreateIntensityAttr(700)
        sun = UsdLux.DistantLight.Define(stage, "/World/Key")
        sun.CreateIntensityAttr(1800)
        sun.CreateAngleAttr(12)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -30))
        floor = UsdGeom.Cube.Define(stage, "/World/Floor")
        floor.CreateSizeAttr(1)
        floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.026))
        floor.AddScaleOp().Set(Gf.Vec3f(200, 200, 0.05))
        material = UsdShade.Material.Define(stage, "/World/FloorMaterial")
        shader = UsdShade.Shader.Define(stage, "/World/FloorMaterial/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        from pxr import Sdf

        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.36)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material)
        camera = UsdGeom.Camera.Define(stage, "/World/Camera")
        camera.CreateFocalLengthAttr(40)
        camera.CreateHorizontalApertureAttr(36)
        camera.CreateVerticalApertureAttr(27)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, 1000))
        camera_op = camera.AddTransformOp()
        product = rep.create.render_product(str(camera.GetPath()), (640, 480))
        annotator = rep.AnnotatorRegistry.get_annotator("LdrColor")
        annotator.attach(product)
        manifest = args.output_dir / "render-manifest.json"
        for index, row in enumerate(rows):
            try:
                stage.RemovePrim("/World/Asset")
                source = Usd.Stage.Open(row["source"])
                unit = UsdGeom.GetStageMetersPerUnit(source)
                axis = str(UsdGeom.GetStageUpAxis(source))
                wrapper = UsdGeom.Xform.Define(stage, "/World/Asset")
                model = UsdGeom.Xform.Define(stage, "/World/Asset/Model")
                model.GetPrim().GetReferences().AddReference(row["source"])
                translate = wrapper.AddTranslateOp()
                wrapper.AddRotateXOp().Set(90 if axis == "Y" else 0)
                scale = wrapper.AddScaleOp()
                scale.Set(Gf.Vec3f(unit))
                cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=False
                )
                bound = cache.ComputeWorldBound(wrapper.GetPrim()).ComputeAlignedRange()
                low, high = np.array(bound.GetMin()), np.array(bound.GetMax())
                extents = high - low
                if not np.isfinite(extents).all() or extents.max() <= 0:
                    raise ValueError(f"Invalid render bounds: {extents}")
                row.update(
                    dimensions_m=extents.tolist(),
                    source_up_axis=axis,
                    source_meters_per_unit=unit,
                )
                factor = float(extents.max())
                translate.Set(
                    Gf.Vec3d(
                        *(
                            -np.array(
                                [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]]
                            )
                            / factor
                        )
                    )
                )
                scale.Set(Gf.Vec3f(unit / factor))
                size = extents / factor
                center = np.array([0.0, 0.0, size[2] / 2])
                direction = np.array([1.4, -2.0, 1.3])
                direction /= np.linalg.norm(direction)
                right = np.cross([0.0, 0.0, 1.0], direction)
                right /= np.linalg.norm(right)
                up = np.cross(direction, right)
                corners = np.array(
                    [
                        [x, y, z]
                        for x in (-size[0] / 2, size[0] / 2)
                        for y in (-size[1] / 2, size[1] / 2)
                        for z in (-size[2] / 2, size[2] / 2)
                    ]
                )
                distances = (
                    np.maximum(
                        abs(corners @ right) / (18 / 40),
                        abs(corners @ up) / (13.5 / 40),
                    )
                    + corners @ direction
                )
                distance = max(float(distances.max()) * 1.25, 0.3)
                eye = center + direction * distance
                camera_op.Set(
                    Gf.Matrix4d()
                    .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0, 0, 1))
                    .GetInverse()
                )
                # Static render updates leave the physics timeline stopped.
                for _ in range(40):
                    app.update()
                rgb = np.asarray(annotator.get_data())[..., :3].copy()
                if rgb.shape != (480, 640, 3) or rgb.std() < 1:
                    raise ValueError(f"Invalid RGB frame: {rgb.shape}")
                name = f"{index:03d}-{row['group']}-{row['label']}.png"
                Image.fromarray(rgb).save(args.output_dir / "images" / name)
                row["image"] = "images/" + name
                print(f"RENDERED {index + 1}/{len(rows)} {row['asset']}", flush=True)
            except Exception as error:
                row["error"] = str(error)
                print(f"FAILED {row['asset']}: {error}", flush=True)
            manifest.write_text(json.dumps(rows, indent=2) + "\n")
            if index + 1 == len(rows) or rows[index + 1]["group"] != row["group"]:
                for sheet in contact_sheets(rows[: index + 1], args.output_dir):
                    print(f"GALLERY {sheet.resolve()}", flush=True)
        annotator.detach(product)
        product.destroy()
    finally:
        app.close()
    return 1 if any("error" in row for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
