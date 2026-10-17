#!/bin/bash
# Drop-in proof in Unity itself: import one FBX together with the game's .meta into a throwaway Unity project and dump
# every sub-asset (type, name, fileID, submesh and vertex counts). Never touches the game's Unity project; the owner's
# editor can stay open. ~3 min and 2-3 GB per import; runs through guard.sh with the heavy-job lock.
#   UNITY=<path to the Unity editor binary> bash unity_probe.sh <work dir> <fbx> <game .meta> <out.json>
#   then: python3 unity_check.py <dump of the ORIGINAL game fbx> <out.json> <prefab file> <fbx guid from the .meta>
# Validate on the original game FBX first (0 problems), then every FBX you deliver. The editor version should match the
# game's (see the game project's ProjectSettings/ProjectVersion.txt). Unity often crashes on exit after writing the
# dump: check the JSON, not the exit code.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
W="$1"; FBX="$2"; META="$3"; OUT="$4"
: "${UNITY:?set UNITY to the Unity editor binary}"
P="$W/unityprobe"                      # one probe project per robot work dir: never share it between sessions
if [ ! -d "$P/Assets/Editor" ]; then
  mkdir -p "$P/Assets/Editor" "$P/Packages"
  cp "$HERE/FbxDump.cs" "$P/Assets/Editor/"
  echo '{"dependencies": {}}' > "$P/Packages/manifest.json"    # nothing to resolve, no network
fi
rm -f "$P"/Assets/*.fbx "$P"/Assets/*.fbx.meta "$P/fbx_dump.json"
N=$(basename "$META" .meta)
cp "$FBX" "$P/Assets/$N"
cp "$META" "$P/Assets/$N.meta"
MEM=5G CPU=800% LOCK=1 "$HERE/../../scripts/guard.sh" timeout 1200 "$UNITY" -batchmode -nographics -projectPath "$P" \
  -executeMethod FbxDump.Run -logFile "$P/unity.log" || true
# Unity's package manager child can outlive the editor: stop only the one started for THIS probe project
for pid in $(pgrep -x UnityPackageManager || true); do
  if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qF "$P"; then kill "$pid" 2>/dev/null || true; fi
done
cp "$P/fbx_dump.json" "$OUT"
grep -E "FBXDUMP|error|Error" "$P/unity.log" | head -5 || true
