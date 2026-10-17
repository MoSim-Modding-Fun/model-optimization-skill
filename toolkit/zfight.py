# Point-sampled coplanar overlap (z-fighting) between different objects (qa.md: never plane offsets). Area-weighted samples
# on every visible object's faces (REVIEW collections and render-hidden objects skipped); a sample counts when ANOTHER
# object's face lies within 0.05 mm of it, facing the same way within 1.8 deg. Run it on the CAD once (baseline) and on
# every result: pairs that already overlap in the CAD are the CAD's own z-fights, not the simplification's.
#   blender -b "<source>.blend" --python zfight.py -- <cad.json> [samples per object]
#   blender -b <run>/final.blend --python zfight.py -- <run.json> [samples per object] [cad.json]
# Output: [[object A, object B, overlap area mm2 (estimate), samples], ...] largest first; with a baseline it prints the
# pairs that are new or at least twice the CAD's overlap ("ZF new"). A pair of 1-4 samples is a few mm2: look at it in
# context before fixing. Fixes seen: LATHE_INSET for lathe end faces, "keep" for parts the rebuild made coplanar with a
# shaft, EXTRA_DUPLICATES for coincident CAD twins, a 0.3 mm shift of a library mesh off a mounting face.
import bpy, sys, os, json, math, collections, random
import numpy as np
from mathutils.bvhtree import BVHTree
a = sys.argv[sys.argv.index("--") + 1:]
out = a[0]; NS = int(a[1]) if len(a) > 1 else 400; base = a[2] if len(a) > 2 else None
dg = bpy.context.evaluated_depsgraph_get()
review = {c.name for c in bpy.data.collections if c.name.startswith("REVIEW")}
V, F, owner, objs = [], [], [], []
for o in bpy.data.objects:
    if o.type != 'MESH' or o.hide_render or any(c.name in review for c in o.users_collection):
        continue
    ev = o.evaluated_get(dg); m = ev.to_mesh(); m.calc_loop_triangles()
    if not len(m.loop_triangles):
        ev.to_mesh_clear(); continue
    M = o.matrix_world
    b0 = len(V)
    V += [tuple(M @ v.co) for v in m.vertices]
    tris = []
    for t in m.loop_triangles:
        F.append(tuple(b0 + i for i in t.vertices)); owner.append(len(objs))
        tris.append(([M @ m.vertices[i].co for i in t.vertices], t.area))
    objs.append((o.name, tris))
    ev.to_mesh_clear()
bvh = BVHTree.FromPolygons(V, F)
area, n_hit = collections.Counter(), collections.Counter()
rng = random.Random(0); cos_lim = math.cos(math.radians(1.8))
for oi, (name, tris) in enumerate(objs):
    ar = np.array([t[1] for t in tris]); tot = ar.sum()
    if tot <= 0:
        continue
    k = min(NS, max(20, int(tot * 1e6 / 4)))  # ~1 sample per 4 mm2, capped
    for ti in np.random.default_rng(oi).choice(len(tris), size=k, p=ar / tot):
        (p0, p1, p2), _ = tris[ti]
        n = (p1 - p0).cross(p2 - p0)
        if n.length < 1e-15:
            continue
        n.normalize()
        u, v = rng.random(), rng.random()
        if u + v > 1:
            u, v = 1 - u, 1 - v
        p = p0 + (p1 - p0) * u + (p2 - p0) * v
        for co, nrm, fi, d in bvh.find_nearest_range(p, 5e-5):
            if owner[fi] != oi and nrm.dot(n) > cos_lim:
                key = tuple(sorted((name, objs[owner[fi]][0])))
                area[key] += tot / k; n_hit[key] += 1
                break
res = sorted(([*k, round(v * 1e6, 1), n_hit[k]] for k, v in area.items()), key=lambda r: -r[2])
json.dump(res, open(out, "w"), indent=0)
print("ZF objects", len(objs), "pairs", len(res), flush=True)
if base and os.path.exists(base):
    cad = {tuple(r[:2]): r[2] for r in json.load(open(base))}
    for r in res:
        if cad.get(tuple(r[:2]), 0.0) < 0.5 * r[2]:
            print("ZF new", r, "CAD", round(cad.get(tuple(r[:2]), 0.0), 1), flush=True)
else:
    for r in res[:40]:
        print("ZF", r, flush=True)
