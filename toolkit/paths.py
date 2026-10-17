# Where things live on this machine. Nothing in the toolkit hard-codes a user's paths: they come from, in order,
#   1. environment variables: SIMPLIFY_ASSET_LIBRARY (the asset library clone, the folder that holds Components/ and
#      Materials/) and SIMPLIFY_ROBOT_DIRS (extra folders with robots/r<name>.py configs, separated by os.pathsep);
#   2. the config file written by install.sh: $SIMPLIFY_CONFIG, else ~/.config/simplify-robot-model/config.json
#      {"asset_library": "...", "robot_config_dirs": ["..."], "robots_root": "..."}.
# Scripts import this module (it works in Blender's Python and in plain python3).
import json
import os

_CFG = None


def config():
    global _CFG
    if _CFG is None:
        p = os.environ.get("SIMPLIFY_CONFIG") or os.path.expanduser("~/.config/simplify-robot-model/config.json")
        try:
            with open(p) as f:
                _CFG = json.load(f)
        except FileNotFoundError:
            _CFG = {}
    return _CFG


def asset_library(required=True):
    """The asset library clone (holds Components/ and Materials/)."""
    p = os.environ.get("SIMPLIFY_ASSET_LIBRARY") or config().get("asset_library")
    if not p and required:
        raise SystemExit("asset library path not set: run install.sh, or set SIMPLIFY_ASSET_LIBRARY to the folder that "
                         "holds Components/ and Materials/")
    return os.path.expanduser(p) if p else None


def components(required=True):
    lib = asset_library(required)
    return os.path.join(lib, "Components") if lib else None


def robot_materials(required=True):
    lib = asset_library(required)
    return os.path.join(lib, "Materials", "RobotMaterials.blend") if lib else None


def robot_config_dirs():
    """Folders searched for robots/r<name>.py after the toolkit's own robots/ (module configs)."""
    dirs = []
    env = os.environ.get("SIMPLIFY_ROBOT_DIRS")
    if env:
        dirs += [d for d in env.split(os.pathsep) if d]
    dirs += config().get("robot_config_dirs", [])
    return [os.path.expanduser(d) for d in dirs if os.path.isdir(os.path.expanduser(d))]


def robots_root():
    """Folder that holds one sub-folder per robot (source CAD, _work/, deliverables); informational."""
    p = os.environ.get("SIMPLIFY_ROBOTS_ROOT") or config().get("robots_root")
    return os.path.expanduser(p) if p else None
