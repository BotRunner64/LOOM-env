"""Checked streaming H.264 encoding for episode camera observations."""

from fractions import Fraction
import hashlib
from pathlib import Path
import subprocess
import tempfile

import imageio_ffmpeg


VIDEO_ENCODING = {
    "codec": "h264",
    "encoder": "libx264",
    "pixel_format": "yuv444p",
    "crf": 18,
    "preset": "medium",
    "gop": 20,
}


def video_path(name: str) -> str:
    return f"cameras/{name}.mp4"


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class CameraVideoWriter:
    """Pipe one RGB frame per control tick; encoder failure prevents publication."""

    def __init__(self, path, camera, control_dt):
        path.parent.mkdir(parents=True, exist_ok=True)
        self._errors = tempfile.TemporaryFile()
        self._closed = False
        rate = 1 / Fraction(str(control_dt))
        try:
            self._process = subprocess.Popen(
                [
                    imageio_ffmpeg.get_ffmpeg_exe(),
                    "-nostdin",
                    "-n",
                    "-loglevel",
                    "error",
                    "-f",
                    "rawvideo",
                    "-pixel_format",
                    "rgb24",
                    "-video_size",
                    f"{camera.width}x{camera.height}",
                    "-framerate",
                    str(rate),
                    "-i",
                    "pipe:0",
                    "-an",
                    "-c:v",
                    VIDEO_ENCODING["encoder"],
                    "-pix_fmt",
                    VIDEO_ENCODING["pixel_format"],
                    "-crf",
                    str(VIDEO_ENCODING["crf"]),
                    "-preset",
                    VIDEO_ENCODING["preset"],
                    "-g",
                    str(VIDEO_ENCODING["gop"]),
                    "-bf",
                    "0",
                    "-threads",
                    "2",
                    "-movflags",
                    "+faststart",
                    str(path),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._errors,
            )
        except BaseException:
            self._errors.close()
            raise

    def append(self, rgb):
        if self._closed:
            raise RuntimeError("Camera encoder is closed")
        self._process.stdin.write(rgb.tobytes())

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            try:
                self._process.stdin.close()
            except BrokenPipeError:
                pass
            try:
                code = self._process.wait(timeout=60)
            except subprocess.TimeoutExpired as error:
                self._process.kill()
                self._process.wait()
                raise OSError(
                    "Camera encoder did not finish within 60 seconds"
                ) from error
            self._errors.seek(0)
            detail = self._errors.read().decode(errors="replace")
            if code:
                raise OSError(f"Camera encoder failed ({code}): {detail.strip()}")
        except BaseException:
            if self._process.poll() is None:
                self._process.kill()
                self._process.wait()
            raise
        finally:
            self._errors.close()

    def abort(self):
        if self._closed:
            return
        self._closed = True
        self._process.kill()
        self._process.wait()
        try:
            self._process.stdin.close()
        except BrokenPipeError:
            pass
        self._errors.close()
