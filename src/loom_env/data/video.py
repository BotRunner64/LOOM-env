"""Encode recorded Episode images without starting simulation."""

from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw

from loom_env.data.episodes import EpisodeReader


def export_video(episode_path, output, *, cameras=None, label="Recorded simulation"):
    """Save an annotated MP4 and first-frame PNG; default to all episode cameras."""
    episode_path = Path(episode_path)
    output = Path(output)
    if output.resolve().is_relative_to(episode_path.resolve()):
        raise ValueError("Video output must be outside the immutable episode directory")
    if output.exists() or output.with_suffix(".png").exists():
        raise ValueError(f"Output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem + ".partial.mp4")
    if temporary.exists():
        raise ValueError(f"Unfinished output already exists: {temporary}")
    preview = output.with_suffix(".png")
    temporary_preview = output.with_name(output.stem + ".partial.png")
    if temporary_preview.exists():
        raise ValueError(f"Unfinished output already exists: {temporary_preview}")
    try:
        with EpisodeReader(episode_path) as episode:
            deployment = episode.spec.collection.deployment
            available = {camera.name: camera for camera in deployment.cameras}
            if cameras is None:
                cameras = list(available)
            if (
                not cameras
                or len(set(cameras)) != len(cameras)
                or any(name not in available for name in cameras)
            ):
                raise ValueError("Select distinct cameras present in the episode")
            selected = [available[name] for name in cameras]
            width = sum(camera.width for camera in selected)
            height = max(camera.height for camera in selected) + 96
            events = [
                event
                for event in episode.manifest["events"]
                if event["kind"] == "skill"
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
                        (12, 8), "LOOM-env | " + label, fill="white", font_size=22
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
                        frame.save(temporary_preview)
            temporary_preview.rename(preview)
            temporary.rename(output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        temporary_preview.unlink(missing_ok=True)
        raise
    return output
