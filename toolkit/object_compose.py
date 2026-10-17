# Compose object_shots.py renders into one labelled image per unique part and view (plain python3, needs ImageMagick).
#   python3 object_compose.py <shots dir> <final dir> [jobs]
# <final dir>/"O001 - <name> - view 1.jpg" ...: CAD (original) and simplified with the same framing, side by side, or
# stacked for wide parts; ids number the visible parts first (most CAD triangles placed first), then the REVIEW ones.
# Writes <final dir>/index.csv (id, name, objects, method, copies, tris, REVIEW collection, files).
import json, os, re, sys, csv, subprocess
from concurrent.futures import ThreadPoolExecutor
SH, FIN = sys.argv[1], sys.argv[2]
JOBS = int(sys.argv[3]) if len(sys.argv) > 3 else 12
os.makedirs(FIN, exist_ok=True)
for f in os.listdir(FIN):
    if f.endswith((".png", ".jpg")) or f == "index.csv":
        os.remove(os.path.join(FIN, f))
meta = json.load(open(os.path.join(SH, "meta.json")))
# placed copies of each module asset ({asset: n}, counted in the simplified file): module pairs have one object each
LC = os.path.join(SH, "libcount.json")
for m in meta:
    if m["key"].startswith("asm:") and os.path.exists(LC):
        m["count"] = json.load(open(LC)).get(m["asset"], m["count"])
order = sorted(meta, key=lambda m: (bool(m["review"]), -m["tris_before"] * m["count"]))
NV = len([f for f in os.listdir(os.path.join(SH, "renders")) if f.startswith(f"{order[0]['i']:03d}_after_")])
VIEW_NAME = {1: "front-top-right", 2: "back-bottom-left (opposite side)"}
METHOD = {"cylinder rebuild": "lathe (revolved)", "decimate": "decimated", "remodel (sections + cutters)": "remodel",
          "library": "library asset", "hex prism (6 sides)": "hex prism", "plate rebuild": "plate rebuild",
          "unchanged (already light)": "unchanged"}
FONT, BOLD = "/usr/share/fonts/noto/NotoSans-Regular.ttf", "/usr/share/fonts/noto/NotoSans-Bold.ttf"
BG, BAR = "#5a5e68", "#202228"


def safe(s):
    s = re.sub(r"\s*\(whole module\)", "", s)
    s = re.sub(r'[:/\\<>|?*"]', " ", s)
    return re.sub(r"\s+", " ", s).strip()[:90]


def size(p):
    return [int(x) for x in subprocess.run(["magick", "identify", "-format", "%w %h", p], capture_output=True,
                                           text=True, check=True).stdout.split()]


jobs, rows = [], []
for n, m in enumerate(order, 1):
    oid = f"O{n:03d}"
    name = m["asset"] or m["part"]
    if m["asset"] and not m["key"].startswith("asm:"):
        name = f"{m['asset']} [{m['part']}]"
    files = []
    for k in range(1, NV + 1):
        b = os.path.join(SH, "renders", f"{m['i']:03d}_before_{k}.png")
        a = os.path.join(SH, "renders", f"{m['i']:03d}_after_{k}.png")
        out = os.path.join(FIN, f"{oid} - {safe(name)} - view {k}.jpg")
        files.append(os.path.basename(out))
        jobs.append((oid, name, m, k, b, a, out))
    rows.append({"id": oid, "name": name, "objects": " | ".join(m["objects"]), "method": METHOD.get(m["method"], m["method"]),
                 "copies": m["count"], "tris_cad_each": m["tris_before"], "tris_simplified_each": m["tris_after"],
                 "tris_simplified_all_copies": m["tris_after"] * m["count"],
                 "review": "; ".join(m["review"]), "files": " | ".join(files)})


def compose(j):
    oid, name, m, k, b, a, out = j
    w, h = size(b)
    stack = w >= 1.25 * h  # wide part: CAD above the simplified, else side by side
    fs = max(18, min(30, (w if stack else 2 * w) // 60))
    rev = f"   ·   REVIEW (hidden in the robot): {', '.join(m['review'])}" if m["review"] else ""
    copies = f"{m['count']} copies" if m["count"] != 1 else "1 copy"
    l1 = f"{oid}   {name}".replace("%", "%%")
    l2 = (f"{METHOD.get(m['method'], m['method'])}   ·   {copies}   ·   CAD {m['tris_before']:,} tris  ->  {m['tris_after']:,} tris"
          f" each   ·   view {k} of {NV}: {VIEW_NAME.get(k, k)}{rev}").replace("%", "%%")

    def panel(src, label):
        return ["(", src, "-background", BG, "-flatten", "-gravity", "NorthWest", "-fill", "#f0f0f0", "-undercolor", "#00000080",
                "-font", BOLD, "-pointsize", str(fs), "-annotate", "+12+10", f" {label} ", ")"]
    gap = ["(", "-size", f"{w}x6" if stack else f"6x{h}", "xc:#e8e8e8", ")"]
    body = panel(b, f"CAD (original)  {m['tris_before']:,} tris") + gap + panel(a, f"Simplified  {m['tris_after']:,} tris")
    W = w if stack else 2 * w + 6
    bar_h = int(fs * 3.6)
    cmd = ["magick", "(", "-size", f"{W}x{bar_h}", f"xc:{BAR}", "-gravity", "NorthWest", "-fill", "#ffffff", "-font", BOLD,
           "-pointsize", str(int(fs * 1.25)), "-annotate", f"+16+{int(fs * 0.35)}", l1,
           "-fill", "#ffb347" if m["review"] else "#c8c8c8", "-font", FONT, "-pointsize", str(int(fs * 0.85)),
           "-annotate", f"+16+{int(fs * 2.1)}", l2, ")",
           "(", *body, ("-append" if stack else "+append"), ")", "-append",
           "-quality", "92", "-sampling-factor", "4:4:4", out]
    subprocess.run(cmd, check=True)
    return out


LIMIT = int(os.environ.get("LIMIT", 0))  # test: compose only the first N images
with ThreadPoolExecutor(JOBS) as ex:
    done = list(ex.map(compose, jobs[:LIMIT] if LIMIT else jobs))
with open(os.path.join(FIN, "index.csv"), "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0]))
    wr.writeheader(); wr.writerows(rows)
print(len(rows), "objects ->", len(done), "images in", FIN)
