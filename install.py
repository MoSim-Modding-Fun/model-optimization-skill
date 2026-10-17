#!/usr/bin/env python3
# Install the simplify-robot-model skill for Claude Code on Linux, Windows or macOS (Python 3.8+, standard library).
#   python3 install.py                           asks for anything it needs (Windows: python install.py, or install.cmd)
#   python3 install.py --library <asset library clone> --robots <robot config folder> [--robots-root <folder>]
#                      [--blender <blender executable>] [--unity <Unity editor>] [--maintainer] [--clone-library]
#                      [--copy] [--yes]
# What it does:
#   1. links this folder to ~/.claude/skills/simplify-robot-model (a symlink; on Windows a directory junction when
#      symlinks aren't allowed; --copy copies instead), so Claude Code loads the skill in every session;
#   2. writes ~/.config/simplify-robot-model/config.json (asset library clone, robot config folder, robots folder,
#      Blender and Unity executables, whether you maintain the asset library); the toolkit reads its paths from there;
#   3. creates the robot config folder with a copy of the example config if it is empty;
#   4. checks the tools the skill uses (Blender, ImageMagick, git, git-lfs) and how the resource guard can cap memory.
# --clone-library clones https://github.com/MoSim-Modding-Fun/Asset-Library to the --library path (git-lfs needed).
# Re-run it any time to change the settings; `git pull` in this folder updates a linked install.
import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.realpath(__file__))  # the real folder, also when run through the installed link
WIN, MAC = sys.platform.startswith("win"), sys.platform == "darwin"
CFG = os.path.join(os.path.expanduser("~"), ".config", "simplify-robot-model", "config.json")
DEST = os.path.join(os.path.expanduser("~"), ".claude", "skills", "simplify-robot-model")
LIB_URL = "https://github.com/MoSim-Modding-Fun/Asset-Library.git"

ap = argparse.ArgumentParser(description="Install the simplify-robot-model skill")
for opt in ("library", "robots", "robots-root", "blender", "unity"):
    ap.add_argument(f"--{opt}")
ap.add_argument("--maintainer", action="store_true")
ap.add_argument("--clone-library", action="store_true")
ap.add_argument("--copy", action="store_true")
ap.add_argument("--yes", action="store_true", help="no questions: use flags, the existing config, or defaults")
A = ap.parse_args()

old = {}
if os.path.exists(CFG):
    with open(CFG, encoding="utf-8") as f:
        old = json.load(f)
home = os.path.expanduser("~")
defaults = {
    "library": old.get("asset_library") or os.path.join(home, "MoSim", "Asset-Library"),
    "robots": (old.get("robot_config_dirs") or [os.path.join(home, "MoSim", "simplify-robots")])[0],
    "robots_root": old.get("robots_root") or os.path.join(home, "MoSim", "robots"),
}


def ask(value, prompt, default):
    if value:
        return value
    if A.yes:
        return default
    ans = input(f"{prompt} [{default}]: ").strip()
    return ans or default


lib = os.path.expanduser(ask(A.library, "Asset library clone (the folder holding Components/ and Materials/)",
                             defaults["library"]))
robots = os.path.expanduser(ask(A.robots, "Folder for your robot configs (r<name>.py files)", defaults["robots"]))
root = os.path.expanduser(ask(A.robots_root, "Folder that holds one folder per robot (CAD, _work, deliverables)",
                              defaults["robots_root"]))
if A.maintainer:
    maint = True
elif A.yes:
    maint = bool(old.get("library_maintainer", False))
else:
    ans = input(f"Do you maintain the asset library repo (commit/push new assets)? y/N "
                f"[{'y' if old.get('library_maintainer') else 'N'}]: ").strip().lower()
    maint = ans in ("y", "yes") if ans else bool(old.get("library_maintainer", False))

# Blender: flag, existing config, or found by the toolkit's own lookup (PATH, usual install folders)
sys.path.insert(0, os.path.join(HERE, "toolkit"))
blender = A.blender or old.get("blender")
if not blender:
    try:
        import paths
        paths._CFG = {}
        blender = paths.blender()
    except SystemExit:
        blender = None
if not blender and not A.yes:
    blender = input("Blender executable (blender.exe / Blender.app/Contents/MacOS/Blender) []: ").strip() or None
unity = A.unity or old.get("unity")

if not os.path.isdir(os.path.join(lib, "Components")):
    if A.clone_library:
        if subprocess.run(["git", "lfs", "version"], capture_output=True).returncode != 0:
            sys.exit("git-lfs is needed to clone the asset library")
        subprocess.run(["git", "clone", LIB_URL, lib], check=True)
    else:
        print(f"note: {lib} has no Components/ folder yet; clone {LIB_URL} there (or re-run with --clone-library)")

os.makedirs(robots, exist_ok=True)
if not any(f.startswith("r") and f.endswith(".py") for f in os.listdir(robots)):
    shutil.copy(os.path.join(HERE, "toolkit", "robots", "rexample.py"), os.path.join(robots, "rexample_copy.py"))
    print(f"created {robots} (with rexample_copy.py to start from)")

cfg = dict(old)
cfg.update({"asset_library": lib, "robot_config_dirs": [robots], "robots_root": root, "library_maintainer": maint})
if blender:
    cfg["blender"] = blender
if unity:
    cfg["unity"] = unity
os.makedirs(os.path.dirname(CFG), exist_ok=True)
with open(CFG, "w", encoding="utf-8") as f:
    json.dump(cfg, f, indent=2)
print("wrote", CFG)



def is_link(p):
    """A symlink, or on Windows a directory junction (also a dangling one)."""
    if os.path.islink(p):
        return True
    if hasattr(os.path, "isjunction"):  # Python 3.12+
        return os.path.isjunction(p)
    if WIN:
        import stat
        try:
            st = os.lstat(p)
        except OSError:
            return False
        return getattr(st, "st_reparse_tag", 0) == getattr(stat, "IO_REPARSE_TAG_MOUNT_POINT", 0xA0000003)
    return False


# link (or copy) the skill into Claude Code's skills folder
os.makedirs(os.path.dirname(DEST), exist_ok=True)
already = os.path.exists(DEST) and os.path.samefile(DEST, HERE)  # linked before, or cloned right there
if is_link(DEST) and not already:
    os.unlink(DEST)  # an old link or junction: removes the link only, never its target
if already:
    print("the skill is installed at", DEST, "->", HERE)
elif os.path.exists(DEST):
    print(f"note: {DEST} exists and is not a link; left it alone (remove it and re-run to install this copy)")
elif A.copy:
    shutil.copytree(HERE, DEST, ignore=shutil.ignore_patterns(".git", "__pycache__"))
    print("copied the skill to", DEST)
else:
    try:
        os.symlink(HERE, DEST, target_is_directory=True)
        print("linked", DEST, "->", HERE)
    except OSError:
        if WIN and subprocess.run(["cmd", "/c", "mklink", "/J", DEST, HERE], capture_output=True).returncode == 0:
            print("linked (junction)", DEST, "->", HERE)
        else:
            shutil.copytree(HERE, DEST, ignore=shutil.ignore_patterns(".git", "__pycache__"))
            print("copied the skill to", DEST, "(links not allowed here: re-run install after `git pull`)")
if not WIN:
    for f in ("scripts/guard.py", "scripts/guard.sh", "scripts/bl.sh", "install.sh", "install.py"):
        p = os.path.join(HERE, f)
        if os.path.exists(p):
            os.chmod(p, os.stat(p).st_mode | 0o111)

print("checks:")
print(f"  {'ok     ' if sys.version_info >= (3, 8) else 'OLD    '} python {sys.version.split()[0]}")
print(f"  {'ok     ' if blender and os.path.exists(blender) else 'MISSING'} blender {blender or ''}")
for tool in ("magick", "git"):
    print(f"  {'ok     ' if shutil.which(tool) else 'MISSING'} {tool}")
lfs = shutil.which("git") and subprocess.run(["git", "lfs", "version"], capture_output=True).returncode == 0
print(f"  {'ok     ' if lfs else 'MISSING'} git-lfs (the asset library's .blend files)")
if WIN:
    print("  ok      memory cap: Windows job objects (scripts/guard.py)")
    if not shutil.which("bash"):
        print("  note    no bash on PATH: Claude Code on Windows runs commands in Git Bash; install Git for Windows")
elif MAC:
    print("  ok      memory cap: polling watchdog (macOS has no hard per-job cap; scripts/guard.py)")
else:
    ok = shutil.which("systemd-run") and subprocess.run(["systemd-run", "--user", "--scope", "-q", "--", "true"],
                                                        capture_output=True).returncode == 0
    print(f"  {'ok     ' if ok else 'note   '} memory cap: "
          + ("systemd user scopes" if ok else "no systemd user scopes: polling watchdog instead"))
tmp = os.environ.get("TMPDIR", "/tmp")
if not WIN and not MAC and subprocess.run(["df", "--output=fstype", tmp], capture_output=True, text=True).stdout.strip().endswith("tmpfs"):
    print(f"  note    {tmp} is in RAM here: keep big files in the robot's _work/ folder")
print("done. In Claude Code, ask e.g. \"simplify the robot in <folder>\".")
