# Labelled before/after contact sheets: python3 sheets.py pairs.json report.json pairs_dir out_dir [visible_only]
import json, subprocess, os, sys
PJ, RJ, PD, OUT = sys.argv[1:5]
vis_only = len(sys.argv) > 5 and sys.argv[5] == "1"
E = json.load(open(PJ))
R = {x["object"]: x for x in json.load(open(RJ))["rows"]}
for i, e in enumerate(E):
    r = R.get(e["before"][0], {})
    e["idx"] = i; e["method"] = r.get("method", "?"); e["review"] = r.get("review")
order = sorted([e for e in E if e["tris_before"] > 0], key=lambda e: (e["review"] is not None, -e["tris_before"] * e["count"]))
if vis_only:
    order = [e for e in order if not e["review"]]
os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    if f.startswith("sheet_"): os.remove(os.path.join(OUT, f))
short = {"cylinder rebuild": "cyl", "plate rebuild": "plate", "tube rebuild": "tube", "decimate": "decim", "library": "LIB",
         "belt rebuild": "belt", "chain link rebuild": "link", "hex prism (6 sides)": "hex", "unchanged (already light)": "same",
         "unchanged (already simplified)": "same", "profile extrusion": "profile"}
PER = 24
for s in range(0, len(order), PER):
    args = ["montage"]
    for e in order[s:s + PER]:
        tag = "R " if e["review"] else ""
        args += ["-label", f"{e['idx']} {e['part'][-22:]}", os.path.join(PD, f"{e['idx']:03d}_before.png"),
                 "-label", f"{tag}{short.get(e['method'], e['method'][:6])} x{e['count']} {e['tris_before']}>{e.get('tris_after')}",
                 os.path.join(PD, f"{e['idx']:03d}_after.png")]
    args += ["-font", "DejaVu-Sans", "-pointsize", "11", "-tile", "8x6", "-geometry", "190x190+2+2", os.path.join(OUT, f"sheet_{s // PER:02d}.png")]
    subprocess.run(args, check=True)
print(len(order), "pairs ->", len([f for f in os.listdir(OUT) if f.startswith('sheet_')]), "sheets")
