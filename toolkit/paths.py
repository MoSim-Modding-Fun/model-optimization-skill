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
            with open(p, encoding="utf-8") as f:
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


def _first(paths_):
    for p in paths_:
        if p and os.path.exists(p):
            return p
    return None


def blender():
    """The Blender executable: SIMPLIFY_BLENDER, config "blender", PATH, then the usual install folders."""
    import glob
    import shutil
    import sys
    p = os.environ.get("SIMPLIFY_BLENDER") or config().get("blender")
    if p:
        return os.path.expanduser(p)
    p = shutil.which("blender")
    if p:
        return p
    if sys.platform.startswith("win"):
        pf = os.environ.get("ProgramW6432") or os.environ.get("ProgramFiles", r"C:\Program Files")  # 64-bit folder
        cands = sorted(glob.glob(os.path.join(pf, "Blender Foundation", "Blender*", "blender.exe")),
                       key=lambda c: [int(x) for x in "".join(ch if ch.isdigit() else " " for ch in c).split()],
                       reverse=True)
        cands += [os.path.join(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
                               "Steam", "steamapps", "common", "Blender", "blender.exe")]
    elif sys.platform == "darwin":
        cands = ["/Applications/Blender.app/Contents/MacOS/Blender",
                 os.path.expanduser("~/Applications/Blender.app/Contents/MacOS/Blender")]
    else:
        cands = ["/usr/bin/blender", "/snap/bin/blender", "/opt/blender/blender"]
    p = _first(cands)
    if not p:
        raise SystemExit("Blender not found: set \"blender\" in the config (install.py) or SIMPLIFY_BLENDER")
    return p


def unity(required=True):
    """The Unity editor (drop-in checks only): SIMPLIFY_UNITY or config "unity"."""
    p = os.environ.get("SIMPLIFY_UNITY") or os.environ.get("UNITY") or config().get("unity")
    if not p and required:
        raise SystemExit("Unity editor not set: config \"unity\" or SIMPLIFY_UNITY (the editor binary, same version as "
                         "the game project)")
    return os.path.expanduser(p) if p else None


def font(bold=False):
    """A TrueType font file for ImageMagick labels (an explicit file: font names differ per system, and a missing one
    renders labels as boxes). Config "font" / "font_bold" wins."""
    import sys
    p = config().get("font_bold" if bold else "font")
    if p:
        return os.path.expanduser(p)
    if sys.platform.startswith("win"):
        d = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts")
        cands = [os.path.join(d, "arialbd.ttf" if bold else "arial.ttf"), os.path.join(d, "segoeuib.ttf" if bold else "segoeui.ttf")]
    elif sys.platform == "darwin":
        cands = ["/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold else "/System/Library/Fonts/Supplemental/Arial.ttf",
                 "/Library/Fonts/Arial Bold.ttf" if bold else "/Library/Fonts/Arial.ttf"]
    else:
        w = "Bold" if bold else "Regular"
        cands = [f"/usr/share/fonts/noto/NotoSans-{w}.ttf", f"/usr/share/fonts/truetype/noto/NotoSans-{w}.ttf",
                 f"/usr/share/fonts/TTF/DejaVuSans{'-Bold' if bold else ''}.ttf",
                 f"/usr/share/fonts/truetype/dejavu/DejaVuSans{'-Bold' if bold else ''}.ttf",
                 f"/usr/share/fonts/liberation/LiberationSans-{w}.ttf"]
    return _first(cands) or ("DejaVu-Sans-Bold" if bold else "DejaVu-Sans")


def safe_filename(s, limit=90):
    """A file-name-safe version of an object / part name (Windows forbids : / \\ < > | ? * and quotes, a trailing dot
    or space, and the device names CON, NUL, COM1...). Windows and macOS ignore case: names that differ only in case
    land in one file, so add the object's index or id when names may collide."""
    import re
    s = re.sub(r'[:/\\<>|?*"\x00-\x1f]', " ", s)
    s = re.sub(r"\s+", " ", s).strip()[:limit].rstrip(". ")
    if re.fullmatch(r"(?i)(con|prn|aux|nul|com\d|lpt\d)(\..*)?", s):
        s = "_" + s
    return s or "_"


if __name__ == "__main__":
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(newline="\n")  # $(...) in Git Bash must not keep a \r
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    if what == "blender":
        print(blender())
    elif what == "unity":
        print(unity())
    elif what == "library":
        print(asset_library())
    elif what == "font":
        print(font())
    else:
        print({"asset_library": asset_library(False), "robot_config_dirs": robot_config_dirs(),
               "robots_root": robots_root(), "blender": blender(), "font": font(), "config": config()})
