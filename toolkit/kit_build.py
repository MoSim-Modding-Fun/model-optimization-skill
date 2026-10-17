# Build library OBJECT assets for kit parts (stock channels and plates, block mounts, motors, servos, wheels, batteries)
# from a robot CAD, one command per asset. Plain python3; run it under guard.sh so every Blender it starts shares the cap:
#   ROBOT=<config> LOCK=1 MEM=6G guard.sh python3 kit_build.py <robot .blend> <work dir> <asset key> [...]
#       [--from extract|vis|pipe|asset|prune|cmp]
# Specs: cfg.ASSETS {key: {"src": empty/object name, "key": regex of the piece whose frame the asset is built in
# (assemblies), "folder": library folder under Components/<cfg.LIB_SUBDIR>, "asset": object name (default = folder),
# "opts": kit_asset.py options}} (cfg.KIT_OPTS: defaults for every asset, e.g. tags and finish candidates). Steps per asset in <work dir>/<key>/: kit_extract.py -> vis.py -> pipeline.py
# (KIT_ASSET=1: the config simplifies the pieces one by one, no library swaps) -> kit_asset.py (stage.blend) ->
# asset_prune.py (hidden faces deleted, gap check) straight into the library -> kit_cmp.py (CAD vs asset renders).
# An existing library file is copied to <work dir>/<key>/lib_backup/ before it is replaced.
import os, sys, json, shutil, subprocess, time, importlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["KIT_ASSET"] = "1"
cfg = importlib.import_module(f"robots.r{os.environ['ROBOT']}")
import paths  # noqa: E402
LIB = paths.components()
STEPS = ["extract", "vis", "pipe", "asset", "prune", "cmp"]
args = [a for a in sys.argv[1:] if not a.startswith("--")]
start = STEPS.index(sys.argv[sys.argv.index("--from") + 1]) if "--from" in sys.argv else 0
if "--from" in sys.argv:
    args.remove(sys.argv[sys.argv.index("--from") + 1])
SRC, WORK, keys = args[0], args[1], args[2:]


def run(cmd, logf):
    t = time.time()
    with open(logf, "w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=HERE)
    txt = open(logf, encoding="utf-8", errors="replace").read()
    if r.returncode != 0 or "Traceback" in txt:
        print(txt[-3000:])
        raise SystemExit(f"FAILED {' '.join(cmd[:4])} ... see {logf}")
    return time.time() - t, txt


for key in keys:
    spec = cfg.ASSETS[key]
    A = os.path.join(WORK, key)
    os.makedirs(A, exist_ok=True)
    folder = spec["folder"]
    name = spec.get("asset", folder)
    libf = os.path.join(LIB, getattr(cfg, "LIB_SUBDIR", ""), folder, folder + ".blend")
    B = [paths.blender(), "-b"]
    if start <= 0:
        dt, out = run(B + [SRC, "--python", "kit_extract.py", "--", f"{A}/work.blend", spec["src"]] +
                      ([spec["key"]] if spec.get("key") else []), f"{A}/extract.log")
        print(key, "extract", round(dt), "s:", [l for l in out.splitlines() if l.startswith("EXTRACT")][0], flush=True)
    if start <= 1:
        dt, _ = run(B + [f"{A}/work.blend", "--python", "vis.py", "--", f"{A}/vis.json", "80", "1400"], f"{A}/vis.log")
        print(key, "vis", round(dt), "s", flush=True)
    if start <= 2:
        dt, out = run(B + [f"{A}/work.blend", "--python", "pipeline.py", "--", f"{A}/vis.json", f"{A}/simplified.blend",
                           f"{A}/report.json"], f"{A}/pipeline.log")
        print(key, "pipeline", round(dt), "s:", [l for l in out.splitlines() if "tris before" in l][0].split("]")[-1], flush=True)
    if start <= 3:
        dt, out = run(B + ["--factory-startup", "--python", "kit_asset.py", "--", A, f"{A}/stage.blend", name,
                           json.dumps({**getattr(cfg, "KIT_OPTS", {}), **spec.get("opts", {})})], f"{A}/asset.log")
        print(key, "asset", round(dt), "s:", [l for l in out.splitlines() if l.startswith("ASSET")][0], flush=True)
    if start <= 4:
        if os.path.exists(libf):
            os.makedirs(f"{A}/lib_backup", exist_ok=True)
            shutil.copy2(libf, f"{A}/lib_backup/{time.strftime('%Y%m%d-%H%M%S')}.blend")
        os.makedirs(os.path.dirname(libf), exist_ok=True)
        dt, out = run(B + [f"{A}/stage.blend", "--python", "asset_prune.py", "--", libf, "1000", "Asset"], f"{A}/prune.log")
        print(key, "prune", round(dt), "s:", [l for l in out.splitlines() if l.startswith("PRUNE Asset")][0][-120:], flush=True)
    if start <= 5:
        os.makedirs(f"{A}/cmp", exist_ok=True)
        run(B + [libf, "--python", "kit_cmp.py", "--", f"{A}/cmp/v"] + ([json.dumps(spec["views"])] if "views" in spec else []),
            f"{A}/cmp.log")
        n = len(spec.get("views", [0] * 4))
        subprocess.run(["magick", "montage"] + [f"{A}/cmp/v_{i}_cad.png" for i in range(n)] +
                       [f"{A}/cmp/v_{i}_asset.png" for i in range(n)] +
                       ["-tile", f"{n}x2", "-geometry", "340x340+2+2", f"{A}/cmp/cmp.jpg"], check=True)
        print(key, "cmp ->", f"{A}/cmp/cmp.jpg", flush=True)
