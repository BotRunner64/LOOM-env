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
    parser.add_argument("--camera", default="front")
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
        if args.camera not in {camera.name for camera in deployment.cameras}:
            parser.error(f"Camera not present in episode: {args.camera}")
        prefix = f"cameras/{args.camera}"
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
            last_rgb = None
            for step, observation in enumerate(episode.observations()):
                if bool(observation.values[f"{prefix}/valid"]):
                    last_rgb = observation.values[f"{prefix}/rgb"]
                if last_rgb is None:
                    raise ValueError(
                        "No valid camera image available at the initial frame"
                    )
                frame = Image.fromarray(last_rgb.copy())
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 0, frame.width, 72), fill=(17, 24, 35))
                draw.text(
                    (20, 10), "LOOM-env  |  " + args.label, fill="white", font_size=22
                )
                active = [event for event in events if event["step"] <= step]
                phase = active[-1]["name"] if active else "Initial state"
                draw.text((20, 42), phase, fill=(137, 212, 236), font_size=18)
                draw.rectangle(
                    (0, frame.height - 42, frame.width, frame.height), fill=(17, 24, 35)
                )
                draw.text(
                    (20, frame.height - 30),
                    f"t = {observation.timestamp:05.2f} s   |   frame {step:03d}/{len(episode)}   |   {args.camera}",
                    fill="white",
                    font_size=17,
                )
                if step == len(episode):
                    draw.text(
                        (frame.width - 250, frame.height - 30),
                        "Outcome: " + episode.manifest["outcome"]["code"],
                        fill=(137, 212, 236),
                        font_size=17,
                    )
                writer.append_data(np.asarray(frame))
                if step == 0:
                    frame.save(args.output.with_suffix(".png"))
        temporary.rename(args.output)
    print(args.output)


if __name__ == "__main__":
    main()
