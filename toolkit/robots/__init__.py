# Per-robot settings. Pick one with the ROBOT environment variable (ROBOT=<name> blender -b ...): the module
# robots/r<name>.py. This folder holds the module/asset configs and an example (rexample.py); robot configs live in your
# own folders, listed in the config file (robot_config_dirs) or SIMPLIFY_ROBOT_DIRS, and are searched after this one, so
# `from robots.r<other> import *` works between them too.
import importlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths  # noqa: E402

for _d in paths.robot_config_dirs():
    if _d not in __path__:
        __path__.append(_d)


def load():
    name = os.environ.get("ROBOT")
    if not name:
        raise SystemExit("set ROBOT=<name>, e.g. ROBOT=example (configs are robots/r<name>.py in the toolkit or in a "
                         "folder listed in robot_config_dirs / SIMPLIFY_ROBOT_DIRS)")
    try:
        return importlib.import_module(f"robots.r{name}")
    except ModuleNotFoundError as e:
        if e.name != f"robots.r{name}":
            raise
        raise SystemExit(f"no config robots/r{name}.py in: {', '.join(__path__)}")
