# Drop-in proof in Unity itself: import one FBX together with the game's .meta into a throwaway Unity project and dump
# every sub-asset (type, name, fileID, submesh and vertex counts). Never touches the game's Unity project; the owner's
# editor can stay open. ~3 min and 2-3 GB per import; runs through the guard with the heavy-job lock and a 20 min
# timeout, and the guard ends anything Unity leaves running (its package manager). Linux, Windows and macOS.
#   python3 unity_probe.py <work dir> <fbx> <game .meta> <out.json>          (Unity editor: config "unity" / SIMPLIFY_UNITY)
#   then: python3 unity_check.py <dump of the ORIGINAL game fbx> <out.json> <prefab file> <fbx guid from the .meta>
# Validate on the original game FBX first (0 problems), then every FBX you deliver. The editor version should match the
# game's (the game project's ProjectSettings/ProjectVersion.txt). Unity often crashes on exit after writing the dump:
# check the JSON, not the exit code.
import glob
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import paths  # noqa: E402

W, FBX, META, OUT = sys.argv[1:5]
P = os.path.join(W, "unityprobe")  # one probe project per robot work dir: never share it between sessions
if not os.path.isdir(os.path.join(P, "Assets", "Editor")):
    os.makedirs(os.path.join(P, "Assets", "Editor"))
    os.makedirs(os.path.join(P, "Packages"), exist_ok=True)
    shutil.copy(os.path.join(HERE, "FbxDump.cs"), os.path.join(P, "Assets", "Editor"))
    with open(os.path.join(P, "Packages", "manifest.json"), "w") as f:
        f.write('{"dependencies": {}}\n')  # nothing to resolve, no network
for f in glob.glob(os.path.join(P, "Assets", "*.fbx")) + glob.glob(os.path.join(P, "Assets", "*.fbx.meta")) + \
        [os.path.join(P, "fbx_dump.json")]:
    if os.path.exists(f):
        os.remove(f)
name = os.path.basename(META)[:-len(".meta")]
shutil.copy(FBX, os.path.join(P, "Assets", name))
shutil.copy(META, os.path.join(P, "Assets", name + ".meta"))
guard = os.path.join(os.path.dirname(os.path.dirname(HERE)), "scripts", "guard.py")
log = os.path.join(P, "unity.log")
subprocess.run([sys.executable, guard, "--lock", "--mem", "5G", "--timeout", "1200", "--",
                paths.unity(), "-batchmode", "-nographics", "-projectPath", P, "-executeMethod", "FbxDump.Run",
                "-logFile", log])
dump = os.path.join(P, "fbx_dump.json")
if not os.path.exists(dump):
    sys.exit(f"no dump written: see {log}")
shutil.copy(dump, OUT)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")  # log lines outside the console code page (Windows)
with open(log, encoding="utf-8", errors="replace") as f:
    hits = [line.rstrip() for line in f if "FBXDUMP" in line or "rror" in line]
print("\n".join(hits[:5]))
print("PROBE wrote", OUT)
