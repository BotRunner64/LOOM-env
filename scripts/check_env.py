#!/usr/bin/env python3
"""Check data/CUDA dependencies; optionally exercise cuRobo and RTX/PhysX."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def versions_check(packages):
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(f"Expected Python 3.12, got {sys.version_info[:2]}")
    missing = [name for name, version in packages.items() if version is None]
    if missing:
        raise RuntimeError(f"Missing packages: {missing}")
    if packages["isaacsim"] != "6.0.1.0":
        raise RuntimeError(f"Expected Isaac Sim 6.0.1.0, got {packages['isaacsim']}")
    if packages["torch"] != "2.11.0+cu128":
        raise RuntimeError(f"Unexpected PyTorch build: {packages['torch']}")
    return (
        "Python 3.12, Sim 6.0.1, PyTorch 2.11 CUDA 12.8 and required packages present"
    )


def data_check():
    import cv2
    import h5py
    import imageio
    import numpy as np
    import yaml

    with tempfile.TemporaryDirectory(prefix="loom-env-check-") as directory:
        path = Path(directory) / "episode.h5"
        expected = np.arange(12, dtype=np.float32).reshape(4, 3)
        with h5py.File(path, "w") as stream:
            stream.create_dataset("observations", data=expected)
        with h5py.File(path, "r") as stream:
            np.testing.assert_array_equal(stream["observations"][:], expected)
        assert yaml.safe_load(yaml.safe_dump({"steps": 3})) == {"steps": 3}
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        assert cv2.cvtColor(frame, cv2.COLOR_RGB2BGR).shape == frame.shape
        imageio.imwrite(Path(directory) / "frame.png", frame)
    return "NumPy, YAML, HDF5, OpenCV and image I/O passed"


def cuda_check():
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch cannot access a CUDA GPU")
    if torch.version.cuda != "12.8":
        raise RuntimeError(f"Expected CUDA 12.8 PyTorch, got {torch.version.cuda}")
    values = torch.arange(16, dtype=torch.float32, device="cuda").reshape(4, 4)
    actual = (values @ values.T).cpu()
    torch.testing.assert_close(actual, values.cpu() @ values.cpu().T)
    torch.cuda.synchronize()
    return {"device": torch.cuda.get_device_name(0), "torch_cuda": torch.version.cuda}


def child_check(name, command, timeout, directory, success_marker=None):
    log = directory / f"{name}.log"
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    env["LOOM_CHECK_OUTPUT_DIR"] = str(directory.resolve())
    started = time.monotonic()
    with log.open("w") as stream:
        result = subprocess.run(
            command,
            cwd=ROOT,
            env=env,
            stdout=stream,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    if result.returncode:
        raise RuntimeError(f"Exit {result.returncode}; see {log}")
    if success_marker is not None and success_marker not in log.read_text(
        errors="replace"
    ):
        raise RuntimeError(f"Missing completion marker {success_marker!r}; see {log}")
    return {"seconds": round(time.monotonic() - started, 2), "log": str(log)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--curobo", action="store_true", help="Run cuRobo GPU FK and gradients"
    )
    parser.add_argument(
        "--sim", action="store_true", help="Run Isaac Lab, PhysX and an RGB camera"
    )
    parser.add_argument("--output-dir", type=Path, default=ROOT / ".cache" / "checks")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {"python": sys.version, "prefix": sys.prefix, "packages": {}, "checks": {}}
    packages = [
        "numpy",
        "torch",
        "torchvision",
        "isaacsim",
        "isaaclab",
        "nvidia-curobo",
        "warp-lang",
        "cuda-core",
        "h5py",
        "isaaclab-physx",
        "isaaclab-ov",
        "isaaclab-assets",
        "isaaclab-visualizers",
    ]
    for name in packages:
        try:
            report["packages"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            report["packages"][name] = None
    checks = {
        "versions": lambda: versions_check(report["packages"]),
        "pip": lambda: child_check(
            "pip-check", [sys.executable, "-m", "pip", "check"], 120, args.output_dir
        ),
        "data": data_check,
        "cuda": cuda_check,
    }
    if args.curobo:
        checks["curobo"] = lambda: child_check(
            "curobo",
            [
                sys.executable,
                "-m",
                "curobo.examples.getting_started.forward_kinematics",
                "--test",
            ],
            600,
            args.output_dir,
            success_marker="Gradient w.r.t. joints:",
        )
    if args.sim:
        checks["simulation"] = lambda: child_check(
            "simulation",
            [sys.executable, str(ROOT / "scripts" / "smoke_sim.py")],
            300,
            args.output_dir,
            success_marker="ISAACLAB_PHYSX_RGB_PASS",
        )
    for name, check in checks.items():
        print(f"Checking {name}...", flush=True)
        try:
            report["checks"][name] = {"passed": True, "details": check()}
        except Exception as error:
            report["checks"][name] = {"passed": False, "error": str(error)}
        (args.output_dir / "report.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print(json.dumps(report["checks"][name]), flush=True)
    return 0 if all(item["passed"] for item in report["checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
