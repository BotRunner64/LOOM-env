#!/usr/bin/env python3
"""Export an Episode's recorded RGB stream as an annotated MP4, without simulation."""

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

from loom_env.data.episodes import EpisodeReader


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--camera", nargs="+", default=["front"])
    parser.add_argument("--label", default="Recorded simulation")
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.episode.resolve()):
        parser.error("Video output must be outside the immutable episode directory")
    if args.output.exists():
        parser.error(f"Output already exists: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(args.output.stem + ".partial.mp4")
    if temporary.exists():
        parser.error(f"Unfinished output already exists: {temporary}")
    with EpisodeReader(args.episode) as episode:
        deployment = episode.spec.collection.deployment
        available = {camera.name: camera for camera in deployment.cameras}
        if len(set(args.camera)) != len(args.camera) or any(
            name not in available for name in args.camera
        ):
            parser.error("Select distinct cameras present in the episode")
        selected = [available[name] for name in args.camera]
        width = sum(camera.width for camera in selected)
        height = max(camera.height for camera in selected) + 96
        events = [
            event for event in episode.manifest["events"] if event["kind"] == "skill"
        ]
        with imageio.get_writer(
            temporary,
            format="FFMPEG",
            fps=1 / deployment.control_dt,
            codec="libx264",
            quality=8,
            macro_block_size=2,
            ffmpeg_params=["-movflags", "+faststart"],
        ) as writer:
            last_rgb = {}
            for step, observation in enumerate(episode.observations()):
                frame = Image.new("RGB", (width, height), (17, 24, 35))
                draw = ImageDraw.Draw(frame)
                draw.text(
                    (12, 8), "LOOM-env | " + args.label, fill="white", font_size=22
                )
                active = [event for event in events if event["step"] <= step]
                phase = active[-1]["name"] if active else "Initial state"
                status = (
                    " | " + episode.manifest["outcome"]["code"]
                    if step == len(episode)
                    else ""
                )
                draw.text(
                    (12, 40),
                    f"{phase} | t={observation.timestamp:05.2f}s | {step}/{len(episode)}{status}",
                    fill=(137, 212, 236),
                    font_size=18,
                )
                x = 0
                for camera in selected:
                    prefix = f"cameras/{camera.name}"
                    if bool(observation.values[f"{prefix}/valid"]):
                        last_rgb[camera.name] = observation.values[f"{prefix}/rgb"]
                    if camera.name not in last_rgb:
                        raise ValueError(f"No valid initial image: {camera.name}")
                    frame.paste(Image.fromarray(last_rgb[camera.name]), (x, 96))
                    draw.text((x + 12, 72), camera.name, fill="white", font_size=18)
                    x += camera.width
                writer.append_data(np.asarray(frame))
                if step == 0:
                    frame.save(args.output.with_suffix(".png"))
        temporary.rename(args.output)
    print(args.output)


if __name__ == "__main__":
    main()
