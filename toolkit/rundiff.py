# What changed between two runs (or two saves of a deliverable), per object. Run it after every re-run or toolkit change,
# before looking at anything else, and review every changed object.
#   blender -b runA/final.blend --python rundiff.py -- dump a.json      (per mesh object: evaluated tris, a hash of its
#   blender -b runB/final.blend --python rundiff.py -- dump b.json       sorted rounded vertices, mesh, REVIEW collection)
#   python3 rundiff.py compare a.json b.json [reportA.json reportB.json]   (plain python; reports add the method)
# compare prints objects only in one file, then the changed objects grouped by mesh, largest tri change first.
import sys, json, collections

a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
if a[0] == "dump":
    import bpy, hashlib
    import numpy as np
    dg = bpy.context.evaluated_depsgraph_get()
    res, cache = {}, {}
    for o in bpy.context.scene.objects:
        if o.type != 'MESH':
            continue
        key = (o.data.name, tuple((m.type, getattr(m, "ratio", 0), m.show_viewport) for m in o.modifiers))
        if key not in cache:
            me = o.evaluated_get(dg).to_mesh()
            co = np.empty(len(me.vertices) * 3)
            me.vertices.foreach_get("co", co)
            h = hashlib.sha1(np.round(np.sort(co.reshape(-1, 3), axis=0), 5).tobytes()).hexdigest()[:12]
            cache[key] = (sum(len(p.vertices) - 2 for p in me.polygons), h)
            o.evaluated_get(dg).to_mesh_clear()
        review = next((c.name for c in o.users_collection if c.name.startswith("REVIEW")), "")
        res[o.name] = {"tris": cache[key][0], "hash": cache[key][1], "mesh": o.data.name, "review": review}
    json.dump(res, open(a[1], "w"))
    print("RUNDIFF dumped", len(res), "mesh objects")
else:
    A, B = json.load(open(a[1])), json.load(open(a[2]))
    meth = [{}, {}]
    for i, p in enumerate(a[3:5]):
        meth[i] = {r["object"]: r.get("method") for r in json.load(open(p)).get("rows", [])}
    print("only in A:", sorted(set(A) - set(B))[:30])
    print("only in B:", sorted(set(B) - set(A))[:30])
    groups = collections.defaultdict(list)
    for o in sorted(set(A) & set(B)):
        x, y = A[o], B[o]
        if x["hash"] != y["hash"] or x["review"] != y["review"]:
            groups[(x["mesh"], meth[0].get(o), meth[1].get(o), x["review"], y["review"])].append((o, x["tris"], y["tris"]))
    tot = 0
    for k, v in sorted(groups.items(), key=lambda kv: -abs(sum(t[2] - t[1] for t in kv[1]))):
        d = sum(t[2] - t[1] for t in v)
        tot += d
        line = f"{d:+8d} x{len(v):<3d} {k[0][:40]:40s} {v[0][0][:40]:40s}"
        if k[1] or k[2]:
            line += f" {k[1]} -> {k[2]}"
        if k[3] != k[4]:
            line += f"  review '{k[3]}' -> '{k[4]}'"
        print(line)
    print("RUNDIFF changed objects", sum(len(v) for v in groups.values()), "tris", f"{tot:+d}",
          "A total", sum(x["tris"] for x in A.values()), "B total", sum(x["tris"] for x in B.values()))
