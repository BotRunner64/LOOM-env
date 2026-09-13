"""Launch the installed Isaac Lab experience with project startup settings.

Import this module freely; simulator imports remain inside ``launch_app``.
See docs/environment.md for the settings and remaining upstream diagnostics.
"""

import atexit
import importlib.util
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory


def launch_app(launcher_args=None, **kwargs):
    """Return a native AppLauncher; the caller owns ``launcher.app.close()``."""
    import isaacsim
    from isaaclab.app import AppLauncher

    sim_root = Path(isaacsim.__file__).resolve().parent
    # Use the same source-relative root as AppLauncher's experience resolver.
    lab_root = (
        Path(importlib.util.find_spec("isaaclab.app.app_launcher").origin)
        .resolve()
        .parents[4]
    )
    folders = [
        sim_root / name
        for name in ("exts", "extscache", "extsPhysics", "kit/exts", "kit/extscore")
        if (sim_root / name).is_dir()
    ]
    paths = sorted((lab_root / "apps").glob("*.kit")) + sorted(
        path
        for path in (lab_root / "source").iterdir()
        if (path / "config/extension.toml").is_file()
    )
    mdl_folders = sorted((sim_root / "extscache").glob("omni.replicator.core-*/mdl"))
    if len(mdl_folders) != 1:
        raise RuntimeError(
            f"Expected one installed Replicator MDL directory, found {mdl_folders}"
        )
    mdl_folder = mdl_folders[0]
    custom_paths = [str(mdl_folder)]
    materials = sorted(path.stem for path in mdl_folder.glob("*.mdl"))
    temporary = TemporaryDirectory(prefix="loom-materials-")
    material_file = Path(temporary.name) / "material.toml"
    original_argv = sys.argv[:]
    try:
        # Replicator checks this file before registering its materials. Supply
        # the same values to live settings: specifying the file alone does not
        # load its contents into Carb's material configuration.
        material_file.write_text(
            "[searchPaths]\ncustom = "
            + json.dumps(custom_paths)
            + "\n[materialGraph]\nuserAllowList = "
            + json.dumps(materials)
            + "\n[options]\nnoStandardPath = false\n",
            encoding="utf-8",
        )
        settings = {
            # Disable experimental mesh processing; Sim 6 still reports its
            # shared streaming service active (see the remaining diagnostics).
            "/UJITSO/geometry": False,
            "/persistent/UJITSO/geometry": False,
            "/app/settings/persistent": False,
            "/persistent/app/usd/muteUsdDiagnostics": False,
            "/app/exts/folders": [str(path) for path in folders],
            "/app/exts/paths": [str(path) for path in paths],
            "/materialConfig/configFilePath": str(material_file),
            "/materialConfig/searchPaths/custom": custom_paths,
            "/materialConfig/materialGraph/userAllowList": materials,
            "/materialConfig/options/noStandardPath": False,
        }
        # AppLauncher's kit_args uses whitespace splitting. Keep each setting
        # as one argv item, including when installation paths contain spaces.
        sys.argv.extend(
            f"--{key}={value if isinstance(value, str) else json.dumps(value)}"
            for key, value in settings.items()
        )
        # Material library imports this optional dependency even in headless.
        sys.argv.extend(["--enable", "omni.kit.context_menu"])
        launcher = AppLauncher(launcher_args, **kwargs)
    except BaseException:
        temporary.cleanup()
        raise
    finally:
        sys.argv[:] = original_argv
    # Keep the per-process material file available until all extensions stop.
    atexit.register(temporary.cleanup)
    return launcher
