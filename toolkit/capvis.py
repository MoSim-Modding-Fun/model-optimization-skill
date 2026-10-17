# Are the closed holes of simplified parts visible in the finished robot? Answers an impartial review's "hole X was
# plugged" without guessing: points on the simplified surface more than 0.8 mm from the CAD surface (plugged holes, filled
# pockets, a mitred tube's square slab end) are clustered (2 mm grid); from up to 24 points per cluster, 96 hemisphere rays
# (REVIEW collections excluded, `Transparent` panels passed through) count how many escape the robot. vis 0 = never seen.
# Cluster ext_mm ~ hole size; maxdev_mm ~ hole radius (a mitred slab end shows ~ wall x tan45 = 1.6 mm).
#   blender -b "robot - simplified.blend" --python capvis.py -- <CAD.blend> '<json [object names]>' <out.json>
import bpy, sys, json, math, random
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
a = sys.argv[sys.argv.index("--") + 1:]
CAD, names, OUT = a[0], json.loads(a[1]), a[2]
GAP, NPTS, NDIR = 0.0008, 24, 96
random.seed(1)
sc = bpy.context.scene; vl = bpy.context.view_layer
def walk(lc):
    yield lc
    for c in lc.children:
        yield from walk(c)
for lc in walk(vl.layer_collection):
    if lc.collection.name.startswith("REVIEW"):
        lc.exclude = True
clear = {o.name for o in sc.objects if o.type == 'MESH' and o.material_slots and
         all(s.material and "Transparent" in s.material.name for s in o.material_slots)}
# CAPVIS_CLEAR: JSON list of object names that are clear only in the game (prefab materials), e.g. the config's CLEAR
import os as _os, json as _json
clear |= set(_json.loads(_os.environ.get("CAPVIS_CLEAR", "[]")))
dg = bpy.context.evaluated_depsgraph_get()
want = {bpy.data.objects[n].name: n for n in names}
# CAD meshes of the subjects: same object names in the source file
with bpy.data.libraries.load(CAD, link=False) as (src, dst):
    req = [n for n in names if n in src.objects]
    dst.objects = list(req)
# appended names collide with the simplified objects and get renumbered: map by request order
cadme = {n: ob.data for n, ob in zip(req, dst.objects) if ob is not None}
dirs = []
gold = math.pi * (3 - math.sqrt(5))
for i in range(NDIR):
    z = 1 - (i + 0.5) / NDIR; r = math.sqrt(1 - z * z); t = gold * i
    dirs.append(Vector((r * math.cos(t), r * math.sin(t), z)))
# CAPVIS_FLOOR=1 counts the floor plane as an occluder, as the tube/plate hole tests do: without it, holes on a
# frame rail's side faces read 10-20 % visible through rays that pass under the bumper into the floor
FLOOR = _os.environ.get("CAPVIS_FLOOR") == "1"


def escapes(p, d):
    o = p
    for _ in range(12):
        hit, loc, nrm, idx, hob, _ = sc.ray_cast(dg, o, d, distance=6.0)
        if not hit:
            return not (FLOOR and d.z < 0)  # CAPVIS_FLOOR=1: a downward ray leaving the robot hits the floor
        if hob.name not in clear:
            return False
        o = loc + d * 1e-5
    return False
res = {}
for n in names:
    ob = bpy.data.objects[n]
    ev = ob.evaluated_get(dg); me = ev.to_mesh()
    cm = cadme[n]
    bvh = BVHTree.FromPolygons([v.co for v in cm.vertices], [p.vertices for p in cm.polygons], epsilon=0.0)
    me.calc_loop_triangles()
    tris = me.loop_triangles
    areas = np.array([t.area for t in tris]); tot = areas.sum()
    nsamp = int(min(150000, max(6000, tot * 1e6 / 1.5)))  # ~1 sample per 1.5 mm^2: a 5 mm hole cap gets ~13
    pick = np.random.default_rng(0).choice(len(tris), nsamp, p=areas / tot)
    rng = np.random.default_rng(1)
    pts = []
    for ti in pick:
        t = tris[ti]; v = [me.vertices[k].co for k in t.vertices]
        r1, r2 = rng.random(), rng.random()
        if r1 + r2 > 1: r1, r2 = 1 - r1, 1 - r2
        p = v[0] + (v[1] - v[0]) * r1 + (v[2] - v[0]) * r2
        q, _, _, dist = bvh.find_nearest(p)
        if dist is not None and dist > GAP:
            pts.append((p.copy(), t.normal.copy(), dist))
    # cluster on a 2 mm grid (union of neighbouring cells)
    cells = {}
    for i, (p, nm, dd) in enumerate(pts):
        cells.setdefault(tuple(int(math.floor(c / 0.002)) for c in p), []).append(i)
    parent = {c: c for c in cells}
    def f(c):
        while parent[c] != c:
            parent[c] = parent[parent[c]]; c = parent[c]
        return c
    for c in cells:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    nb = (c[0] + dx, c[1] + dy, c[2] + dz)
                    if nb in cells:
                        parent[f(c)] = f(nb)
    groups = {}
    for c, idx in cells.items():
        groups.setdefault(f(c), []).extend(idx)
    M = ob.matrix_world; R3 = M.to_3x3()
    out = []
    for g in sorted(groups.values(), key=len, reverse=True):
        P = np.array([pts[i][0] for i in g]); ext = float(np.ptp(P, axis=0).max())
        sel = random.sample(g, min(NPTS, len(g)))
        esc = tot_r = 0
        for i in sel:
            p, nm, dd = pts[i]
            wn = (R3 @ nm).normalized(); wp = M @ p + wn * 2e-5
            for d in dirs:
                wd = d if d.dot(wn) > 0 else -d
                tot_r += 1; esc += escapes(wp, wd)
        cen = M @ Vector(P.mean(0))
        if len(g) < 3:
            continue
        out.append({"n": len(g), "ext_mm": round(ext * 1000, 1), "maxdev_mm": round(max(pts[i][2] for i in g) * 1000, 2),
                    "vis": round(esc / tot_r, 4), "centre": [round(x, 4) for x in cen]})
    ev.to_mesh_clear()
    res[n] = out
    print("CAPVIS", n, len(pts), "pts", len(out), "clusters; visible:",
          [(c["ext_mm"], c["vis"]) for c in out if c["vis"] > 0.005][:12], flush=True)
json.dump(res, open(OUT, "w"), indent=1)
