# Phase 1 for very large CADs: Merge by Distance -> Limited Dissolve in batches, one Blender process per batch,
# resumable (progress.json + checkpoints in <dir>/_work). Plain python3.
#   python3 cleanup/run_phase1.py "<dir>/<name>.blend" [--cap 1000000] [--alone 500000]  ->  "<dir>/<name> - cleaned.blend"
import json, os, subprocess, sys, time, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("src"); ap.add_argument("--cap", type=int, default=1_000_000); ap.add_argument("--alone", type=int, default=500_000)
A = ap.parse_args()
SRC = os.path.abspath(A.src)
D, NAME = os.path.dirname(SRC), os.path.splitext(os.path.basename(SRC))[0]
WORK = os.path.join(D, "_work")
CLEANED = os.path.join(D, f"{NAME} - cleaned.blend")
PLAN, PROG = os.path.join(WORK, "plan.json"), os.path.join(WORK, "progress.json")
os.makedirs(os.path.join(WORK, "logs"), exist_ok=True)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def mem_avail_gb():
    for l in open("/proc/meminfo"):
        if l.startswith("MemAvailable"):
            return int(l.split()[1]) / 1048576


if not os.path.exists(PLAN):
    units_json = os.path.join(WORK, "units.json")
    subprocess.run(["blender", "-b", SRC, "--python", os.path.join(HERE, "plan.py"), "--", units_json], check=True,
                   stdout=open(os.path.join(WORK, "logs", "plan.log"), "w"), stderr=subprocess.STDOUT)
    units = sorted(json.load(open(units_json)), key=lambda u: -u["tris"])
    batches = []
    for u in units:  # first-fit decreasing; anything above --alone gets its own batch
        if u["tris"] > A.alone:
            batches.append([u]); continue
        for b in batches:
            if b[0]["tris"] <= A.alone and sum(x["tris"] for x in b) + u["tris"] <= A.cap:
                b.append(u); break
        else:
            batches.append([u])
    plan = [{"batch": i, "units": b, "unique_tris": sum(x["tris"] for x in b),
             "scene_tris": sum(x["tris"] * (1 + len(x["copies"])) for x in b)} for i, b in enumerate(batches)]
    json.dump(plan, open(PLAN, "w"))
    for p in plan:
        log(f"plan B{p['batch']:02d}: {len(p['units'])} geometries, {p['unique_tris']:,} unique / {p['scene_tris']:,} scene tris")
plan = json.load(open(PLAN))
prog = json.load(open(PROG)) if os.path.exists(PROG) else {"done_batches": [], "done_meshes": [], "last_ckpt": SRC, "batches": {}}
for b in plan:
    bi = b["batch"]
    if bi in prog["done_batches"]:
        continue
    need = 4.5 if b["unique_tris"] > A.cap else 3.5
    while mem_avail_gb() < need + 2:
        log(f"B{bi:02d} waiting: only {mem_avail_gb():.1f} GB available"); time.sleep(60)
    ck, res = os.path.join(WORK, f"ckpt_b{bi:02d}.blend"), os.path.join(WORK, f"b{bi:02d}.json")
    log(f"B{bi:02d} start: {len(b['units'])} geometries, {b['unique_tris']:,} unique tris, from {os.path.basename(prog['last_ckpt'])}")
    t = time.time()
    with open(os.path.join(WORK, "logs", f"b{bi:02d}.log"), "w") as lf:
        r = subprocess.run(["blender", "-b", prog["last_ckpt"], "--python", os.path.join(HERE, "phase1_batch.py"), "--",
                            PLAN, str(bi), ck, res, os.path.join(WORK, "p1_renders")], stdout=lf, stderr=subprocess.STDOUT,
                           timeout=5400 if b["unique_tris"] > A.cap else 1800)
    if r.returncode != 0 or not os.path.exists(res) or not os.path.exists(ck):
        log(f"B{bi:02d} FAILED rc={r.returncode}; see _work/logs/b{bi:02d}.log; rerun to resume"); sys.exit(1)
    out = json.load(open(res))
    prev = prog["last_ckpt"]
    prog["done_batches"].append(bi)
    prog["done_meshes"] += [m for u in out["units"] for m in u["meshes"]]
    prog["last_ckpt"] = ck
    prog["batches"][str(bi)] = {k: out[k] for k in ("tris_before", "tris_after", "t_process", "t_save", "t_total", "peak_rss_mb")}
    json.dump(prog, open(PROG, "w"), indent=1)
    if prev != SRC:  # keep the newest checkpoint + the one before it
        for f in os.listdir(WORK):
            if f.startswith("ckpt_") and os.path.join(WORK, f) not in (prev, ck):
                os.remove(os.path.join(WORK, f))
    log(f"B{bi:02d} done in {time.time() - t:.0f}s: {out['tris_before']:,} -> {out['tris_after']:,} tris, peak {out['peak_rss_mb']} MB, "
        f"max bbox change {max(u['bbox_dmm'] for u in out['units'])} mm")
if len(prog["done_batches"]) == len(plan):
    subprocess.run(["blender", "-b", prog["last_ckpt"], "--python-expr",
                    f"import bpy; bpy.ops.wm.save_as_mainfile(filepath={CLEANED!r}, copy=True, compress=True)"],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    log("ALL DONE ->", CLEANED, f"({sum(v['tris_after'] for v in prog['batches'].values()):,} tris)")
