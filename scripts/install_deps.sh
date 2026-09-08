#!/usr/bin/env bash
set -Eeuo pipefail
LOOM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$LOOM_ROOT"
if (( $# > 1 )); then
    echo "Usage: bash scripts/install_deps.sh [--locked]" >&2
    exit 2
fi
case "${1:-}" in
    "") LOOM_INSTALL_ARGS=(-e ".[sim]" -r requirements/isaaclab.txt) ;;
    --locked) LOOM_INSTALL_ARGS=(-r requirements/pip-linux-64.lock) ;;
    *) echo "Usage: bash scripts/install_deps.sh [--locked]" >&2; exit 2 ;;
esac
python - <<'PY'
import os, platform, sys
if not os.environ.get("CONDA_PREFIX") or sys.prefix != os.environ["CONDA_PREFIX"]:
    raise SystemExit("Activate the loom-env Conda environment first.")
if os.environ.get("CONDA_DEFAULT_ENV") == "base":
    raise SystemExit("Refusing to install into the base Conda environment.")
if sys.version_info[:2] != (3, 12) or platform.system() != "Linux" or platform.machine() != "x86_64":
    raise SystemExit("This dependency set targets Linux x86_64 and Python 3.12.")
PY
export PYTHONNOUSERSITE=1 PIP_USER=0 PIP_CONFIG_FILE=/dev/null
export PIP_INDEX_URL=https://pypi.org/simple PIP_EXTRA_INDEX_URL=
export PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_NO_INPUT=1 PIP_PROGRESS_BAR=off
LOOM_LAB_REV=ffff603eafc6b74264a5261cc0183d6a65390d78
LOOM_LAB_DIR="$LOOM_ROOT/.deps/IsaacLab"
mkdir -p "$LOOM_ROOT/.deps"
if [[ ! -d "$LOOM_LAB_DIR/.git" ]]; then
    git clone --depth 1 --branch v3.0.0-beta2.patch1 https://github.com/isaac-sim/IsaacLab.git "$LOOM_LAB_DIR"
fi
if [[ "$(git -C "$LOOM_LAB_DIR" rev-parse HEAD)" != "$LOOM_LAB_REV" ]]; then
    echo "Isaac Lab checkout differs from the pinned release: $LOOM_LAB_REV" >&2
    exit 1
fi
LOOM_PATCH="$LOOM_ROOT/patches/isaaclab-sim601-coverage.patch"
if git -C "$LOOM_LAB_DIR" apply --reverse --check "$LOOM_PATCH" 2>/dev/null; then
    echo "Isaac Lab coverage compatibility patch is already applied."
else
    git -C "$LOOM_LAB_DIR" apply --check "$LOOM_PATCH"
    git -C "$LOOM_LAB_DIR" apply "$LOOM_PATCH"
fi
# Read build prerequisites and CUDA package versions from the project manifest.
python - <<'PY'
import subprocess
import sys
import tomllib
from pathlib import Path

config = tomllib.loads(Path("pyproject.toml").read_text())
bootstrap = config["build-system"]["requires"] + config["dependency-groups"]["bootstrap"]
subprocess.run([sys.executable, "-m", "pip", "install", *bootstrap], check=True)

from packaging.requirements import Requirement

torch_names = {"torch", "torchvision", "torchaudio"}
torch_requirements = [
    item for item in config["project"]["optional-dependencies"]["sim"]
    if Requirement(item).name in torch_names
]
if {Requirement(item).name for item in torch_requirements} != torch_names:
    raise SystemExit("The sim extra must declare torch, torchvision and torchaudio.")
subprocess.run(
    [sys.executable, "-m", "pip", "install", *torch_requirements,
     "--index-url", "https://download.pytorch.org/whl/cu128"],
    check=True,
)
PY
python -m pip install --no-build-isolation \
    "${LOOM_INSTALL_ARGS[@]}" --group dev --extra-index-url https://pypi.nvidia.com
python -m pip check
