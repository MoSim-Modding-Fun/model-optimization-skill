# Build labelled before/after contact sheets from pairs.json + report.json (plain python3, needs ImageMagick).
#   python3 make_sheets.py [workdir]   (workdir holds pairs.json, report.json, pairs/; sheets/ is written there)
import json, subprocess, os, sys
HERE = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))  # work dir
E = json.load(open(os.path.join(HERE, "pairs.json")))
R = {x["object"]: x for x in json.load(open(os.path.join(HERE, "report.json")))["rows"]}
for i, e in enumerate(E):
    r = R.get(e["before"][0], {})
    e["idx"] = i
    e["method"] = "library" if e["key"].startswith("asm:") else r.get("method", "?")
    e["review"] = r.get("review")
order = sorted(E, key=lambda e: (e["review"] is not None, -e["tris_before"] * e["count"]))
json.dump(order, open(os.path.join(HERE, "pairs_order.json"), "w"))
out = os.path.join(HERE, "sheets")
os.makedirs(out, exist_ok=True)
for f in os.listdir(out):
    os.remove(os.path.join(out, f))
short = {"cylinder rebuild": "cyl", "plate rebuild": "plate", "tube rebuild": "tube", "decimate": "decim", "library": "LIB",
         "keep (your modifiers)": "yours", "belt rebuild": "belt", "box (chain link)": "box", "chain link rebuild": "link", "hex prism (6 sides)": "hex",
         "unchanged (already light)": "same", "unchanged (already simplified)": "same", "profile extrusion": "profile",
         "spring rebuild": "spring", "eyebolt rebuild": "eyebolt", "wheel section": "wheel", "kraken cap (custom)": "cap"}
PER = 24
for s in range(0, len(order), PER):
    args = ["magick", "montage"]
    for e in order[s:s + PER]:
        tag = "R " if e["review"] else ""
        args += ["-label", f"{e['idx']} {e['part'][:22]}", os.path.join(HERE, f"pairs/{e['idx']:03d}_before.png"),
                 "-label", f"{tag}{short.get(e['method'], e['method'][:6])} x{e['count']} {e['tris_before']}>{e['tris_after']}",
                 os.path.join(HERE, f"pairs/{e['idx']:03d}_after.png")]
    args += ["-font", "DejaVu-Sans", "-pointsize", "10", "-tile", "8x6", "-geometry", "190x190+2+2", os.path.join(out, f"sheet_{s // PER:02d}.png")]
    subprocess.run(args, check=True)
print(len(order), "pairs ->", len(os.listdir(out)), "sheets")
