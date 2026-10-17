# usage: bl.sh <run>/placed.blend --python review_sync.py -- <report.json in> <report.json out>
# Sheet labels take the REVIEW status from the file, not the pipeline report: post steps (library module placement)
# move pieces into REVIEW after report.json was written. Rewrites each row's "review" from the object's collections.
import bpy, sys, json
src, out = sys.argv[sys.argv.index("--") + 1:][:2]
rep = json.load(open(src))
n = 0
for r in rep["rows"]:
    o = bpy.data.objects.get(r["object"])
    if o is None:
        continue
    rev = next((c.name for c in o.users_collection if c.name.startswith("REVIEW")), None)
    if rev != r.get("review"):
        r["review"] = rev; n += 1
json.dump(rep, open(out, "w"))
print("REVIEW_SYNC rows changed", n, flush=True)
