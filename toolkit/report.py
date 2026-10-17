# report.csv + totals for a finished robot file: per object the CAD tris, the method (pipeline report;
# library asset objects by their asset), the REVIEW collection and the final evaluated tris; totals by method, with and
# without the REVIEW collections. Plain Blender run, read-only:
#   ROBOT=<robot> blender -b "<robot> - simplified.blend" --python report.py -- <pipeline report.json> <out.csv> <totals.json>
import bpy, sys, json, csv, collections
REP, CSV, TOT = sys.argv[sys.argv.index("--") + 1:][:3]
rep = json.load(open(REP))
rows_rep = {r["object"]: r for r in rep["rows"]}
dg = bpy.context.evaluated_depsgraph_get()
rows = []
by = collections.defaultdict(lambda: {"objects": 0, "tris_before": 0, "tris_after": 0})
by_rev = collections.defaultdict(lambda: {"objects": 0, "tris_before": 0, "tris_after": 0})
tot = {"visible": 0, "review": 0}
for o in bpy.context.scene.objects:
    if o.type != 'MESH':
        continue
    ev = o.evaluated_get(dg); m = ev.to_mesh(); n = sum(len(p.vertices) - 2 for p in m.polygons); ev.to_mesh_clear()
    review = next((c.name for c in o.users_collection if c.name.startswith("REVIEW")), "")
    r = rows_rep.get(o.name, {})
    method = r.get("method") or ("library: " + o.data.name.split(" [library")[0] if "[library" in o.data.name else "?")
    if method == "library" or o.name.endswith(" [library]"):
        method = "library: " + o.data.name.split(" [library")[0]
    before = r.get("before", 0) or 0
    rows.append({"object": o.name, "parent": o.parent.name if o.parent else "", "method": method, "tris_cad": before,
                 "tris_final": n, "review": review, "modifiers": " > ".join(md.type for md in o.modifiers),
                 "note": r.get("note", "") or ""})
    key = "library assets" if method.startswith("library") else method
    D = by_rev if review else by
    D[key]["objects"] += 1; D[key]["tris_before"] += before; D[key]["tris_after"] += n
    tot["review" if review else "visible"] += n
lib = collections.defaultdict(lambda: [0, 0])
for r in rows:
    if r["method"].startswith("library") and not r["review"]:
        lib[r["method"][9:]][0] += 1; lib[r["method"][9:]][1] += r["tris_final"]
with open(CSV, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(sorted(rows, key=lambda r: (r["review"], -r["tris_final"])))
tot.update(total=tot["visible"] + tot["review"], source=rep.get("before"), by_method=dict(by),
           review_by_method=dict(by_rev), library=dict(lib),
           review_collections={c.name: len(c.objects) for c in bpy.data.collections if c.name.startswith("REVIEW")})
json.dump(tot, open(TOT, "w"), indent=1)
print("REPORT visible", tot["visible"], "review", tot["review"], "total", tot["total"], "source", tot["source"])
for k, v in sorted(by.items(), key=lambda kv: -kv[1]["tris_after"]):
    print(f"  {k:42} {v['objects']:>5} {v['tris_before']:>11,} -> {v['tris_after']:>8,}")
for k, v in sorted(lib.items(), key=lambda kv: -kv[1][1]):
    print(f"    lib {k:60} x{v[0]:<3} {v[1]:>8,}")
