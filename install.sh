#!/bin/bash
# Install the simplify-robot-model skill for Claude Code.
#   ./install.sh                                    asks for anything it needs
#   ./install.sh --library <asset library clone> --robots <robot config folder> [--robots-root <folder>]
#                [--maintainer] [--clone-library] [--copy] [--yes]
# What it does:
#   1. links this folder to ~/.claude/skills/simplify-robot-model (--copy copies it instead), so Claude Code loads the
#      skill in every session;
#   2. writes ~/.config/simplify-robot-model/config.json (asset library clone, robot config folder, robots folder,
#      whether you maintain the asset library); the toolkit reads its paths from there;
#   3. creates the robot config folder with a copy of the example config if it doesn't exist;
#   4. checks the tools the skill uses (blender, python3, magick, systemd-run --user, git-lfs).
# --clone-library clones https://github.com/MoSim-Modding-Fun/Asset-Library to the --library path (git-lfs needed).
# Re-run it any time to change the settings; `git pull` in this folder updates a linked install.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG_DIR="$HOME/.config/simplify-robot-model"; CFG="$CFG_DIR/config.json"
SKILLS="$HOME/.claude/skills"; DEST="$SKILLS/simplify-robot-model"
LIB=""; ROBOTS=""; ROOT=""; MAINT=""; CLONE=0; COPY=0; YES=0
while [ $# -gt 0 ]; do
  case "$1" in
    --library) LIB="$2"; shift 2 ;;
    --robots) ROBOTS="$2"; shift 2 ;;
    --robots-root) ROOT="$2"; shift 2 ;;
    --maintainer) MAINT=true; shift ;;
    --clone-library) CLONE=1; shift ;;
    --copy) COPY=1; shift ;;
    --yes) YES=1; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

old() { [ -f "$CFG" ] && python3 -c "import json,sys; print(json.load(open('$CFG')).get('$1', '') or '')" 2>/dev/null || true; }
ask() {  # ask <var> <prompt> <default>
  local cur="${!1}"; [ -n "$cur" ] && return
  if [ "$YES" = 1 ]; then printf -v "$1" '%s' "$3"; return; fi
  read -r -p "$2 [$3]: " ans; printf -v "$1" '%s' "${ans:-$3}"
}

ask LIB "Asset library clone (the folder holding Components/ and Materials/)" "$(old asset_library || true)"
[ -z "$LIB" ] && LIB="$HOME/MoSim/Asset-Library"
ask ROBOTS "Folder for your robot configs (r<name>.py files)" "$(old robot_config_dirs | tr -d "[]'\"" | cut -d, -f1)"
[ -z "$ROBOTS" ] && ROBOTS="$HOME/MoSim/simplify-robots"
ask ROOT "Folder that holds one folder per robot (CAD, _work, deliverables)" "$(old robots_root || true)"
[ -z "$ROOT" ] && ROOT="$HOME/MoSim/robots"
if [ -z "$MAINT" ]; then
  m="$(old library_maintainer)"
  if [ "$YES" = 1 ]; then MAINT="${m:-false}"
  else read -r -p "Do you maintain the asset library repo (commit/push new assets)? y/N [${m:-False}]: " ans
       case "$ans" in y|Y|yes) MAINT=true ;; n|N|no) MAINT=false ;; *) MAINT="$( [ "$m" = True ] && echo true || echo false )" ;; esac
  fi
fi
LIB="${LIB/#\~/$HOME}"; ROBOTS="${ROBOTS/#\~/$HOME}"; ROOT="${ROOT/#\~/$HOME}"

if [ ! -d "$LIB/Components" ]; then
  if [ "$CLONE" = 1 ]; then
    git lfs version >/dev/null 2>&1 || { echo "git-lfs is needed to clone the asset library" >&2; exit 1; }
    git clone https://github.com/MoSim-Modding-Fun/Asset-Library.git "$LIB"
  else
    echo "note: $LIB has no Components/ folder yet; clone the library there (or re-run with --clone-library)" >&2
  fi
fi

mkdir -p "$ROBOTS" "$CFG_DIR"
if [ ! -e "$ROBOTS/rexample.py" ] && ! ls "$ROBOTS"/r*.py >/dev/null 2>&1; then
  cp "$HERE/toolkit/robots/rexample.py" "$ROBOTS/rexample_copy.py"
  echo "created $ROBOTS (with rexample_copy.py to start from)"
fi
python3 - "$CFG" "$LIB" "$ROBOTS" "$ROOT" "$MAINT" <<'EOF'
import json, sys
cfg, lib, robots, root, maint = sys.argv[1:6]
json.dump({"asset_library": lib, "robot_config_dirs": [robots], "robots_root": root,
           "library_maintainer": maint == "true"}, open(cfg, "w"), indent=2)
print("wrote", cfg)
EOF

mkdir -p "$SKILLS"
if [ -L "$DEST" ] || [ ! -e "$DEST" ]; then
  rm -f "$DEST"
  if [ "$COPY" = 1 ]; then cp -r "$HERE" "$DEST"; echo "copied the skill to $DEST"
  else ln -s "$HERE" "$DEST"; echo "linked $DEST -> $HERE"; fi
else
  echo "note: $DEST exists and is not a link; left it alone (remove it and re-run to install this copy)" >&2
fi
chmod +x "$HERE/scripts/"*.sh "$HERE/toolkit/unity_probe/unity_probe.sh" 2>/dev/null || true

echo "checks:"
for t in blender python3 magick git git-lfs systemd-run; do
  if command -v "$t" >/dev/null 2>&1; then echo "  ok      $t"; else echo "  MISSING $t"; fi
done
systemd-run --user --scope -q -- true 2>/dev/null && echo "  ok      systemd user scopes (the RAM guard needs them)" \
  || echo "  MISSING systemd user scopes: scripts/guard.sh cannot cap memory on this machine"
if command -v blender >/dev/null 2>&1 && systemd-run --user --scope -q -- true 2>/dev/null; then
  echo "  $(LOCK=0 MEM=1G "$HERE/scripts/guard.sh" blender --version 2>/dev/null | head -1)"   # guarded, like every run
fi
case "$(df --output=fstype /tmp 2>/dev/null | tail -1)" in
  tmpfs) echo "  note    /tmp is in RAM here: keep big files in the robot's _work/ folder" ;;
esac
echo "done. In Claude Code, ask e.g. \"simplify the robot in <folder>\"."
